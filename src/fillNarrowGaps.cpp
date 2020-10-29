//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include "Application.h"
#include "FffGcodeWriter.h"
#include "sliceDataStorage.h"
#include "utils/linearAlg2D.h"
#include "LayerPlan.h"

namespace cura
{

void FffGcodeWriter::fillNarrowGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const Polygons& gaps, const GCodePathConfig& gap_config, const bool is_outline, bool& added_something) const
{
    if (!gaps.polyLineLength())
    {
        return;
    }

    const Ratio min_flow = std::max(Ratio(0.2), mesh.settings.get<Ratio>("wall_min_flow"));

    const double min_gap_area = mesh.settings.get<bool>("filter_out_tiny_gaps") ? mesh.settings.get<double>("min_gap_area") * 1000 * 1000 : 1; // mm^2 -> microns^2

    Polygons simplified_gaps(gaps);

    // two benefits to reducing the resolution of the outline
    // 1 - very small line segments get removed
    // 2 - should take less time to process
    simplified_gaps.simplify(100, 50);

    // split gaps into parts
    std::vector<PolygonsPart> gap_parts(simplified_gaps.splitIntoParts(false));

    while (gap_parts.size())
    {
        // find the part that is closest to the current location
        Polygons part_outlines;
        for (const PolygonsPart& part : gap_parts)
        {
            part_outlines.add(part.outerPolygon());
        }

        ClosestPolygonPoint cpp = PolygonUtils::findClosest(gcode_layer.getLastPlannedPositionOrStartingPosition(), part_outlines);
        if (!cpp.isValid())
        {
            // what else can we do?
            return;
        }

        Polygons gap_part(gap_parts[cpp.poly_idx]);

        // remove from vector so loop terminates!
        gap_parts.erase(gap_parts.begin() + cpp.poly_idx);

        const coord_t avg_width = 2 * gap_part.area() / gap_part.polygonLength();

        if ((float)avg_width / gap_config.getLineWidth() < min_flow)
        {
            continue;
        }

        Polygons gap_polygons(gap_part);
        Polygons all_filled_areas;

        bool check_overlap = true; // by default we check each line to be drawn to see if it overlaps with lines already drawn

        // we can possibly optimize the scenario where the gap consists of an outline with a single hole in it (i.e. a tube)
        if (gap_polygons.size() == 2 && gap_polygons[0].size() > 50)
        {
            // create a polygon that should mask just the core of the gap
            // any spikes in either the outer or inner boundaries should not be included in the mask
            Polygons core_mask;
            // convert the hole into an anti-hole and expand it so that it should be outside of the core region
            Polygon hole(gap_polygons[1]);
            hole.reverse();
            core_mask.add(hole.offset(avg_width * 2));
            // now shrink the original outline so that it is inside of the core region and subtract it
            // to leave a polygon
            core_mask = core_mask.difference(gap_polygons[0].offset(-avg_width * 2));
            // if there's nothing left when the core mask is subtracted from the gap polygons it
            // means that the inner and outer boundaries must have a similar shape and so it is OK
            // to only print the lines for the outer boundary and there is no need to check for overlaps
            if (gap_polygons.difference(core_mask).size() == 0)
            {
                gap_polygons.remove(1);
                check_overlap = false;
            }
        }

        // print the lines from each polygon in the gap part

        bool is_outer_poly = is_outline;

        unsigned next_poly_index = 0; // start with outer polygon

        while (gap_polygons.size() > 0)
        {
            PolygonRef poly = gap_polygons[next_poly_index];

            // fill areas not less than min_gap_area
            if (std::abs(poly.area()) >= min_gap_area)
            {
                // remove some points that cause problems
                for (unsigned n = 0; n < poly.size(); ++n)
                {
                    const Point& prev_point = poly[(n + poly.size() - 1) % poly.size()];
                    const Point& next_point = poly[(n + 1) % poly.size()];
                    const double prev_len = vSize(poly[n] - prev_point);
                    const double next_len = vSize(poly[n] - next_point);
                    const double corner_rads = LinearAlg2D::getAngleLeft(prev_point, poly[n], next_point);

                    // remove points that are at the apex of short narrow spikes - these can be created where the thin wall meets normal wall
                    // if the corner at point 2 is sharp and that point is either preceded or followed by a relatively long line, remove point 2
                    // real spike will be sharper than shown in the AA below
                    //
                    // -------1--------------------------------------2
                    //                                              /
                    //                                             /
                    //                                            3
                    //                                            |
                    //                                            |
                    //                                            |
                    //                                            |
                    //                                            4

                    if (corner_rads < 0.3 || corner_rads > (M_PI * 2 - 0.3))
                    {
                        if (prev_len < gap_config.getLineWidth() * 2 || next_len < gap_config.getLineWidth() * 2)
                        {
                            //std::cerr << gcode_layer.getLayerNr() << ": at " << poly[n] << " angle " << corner_rads << " prev_len " << prev_len << " next_len " << next_len << "\n";
                            poly.remove(n);
                            // hack alert - adjust n
                            --n;
                            continue;
                        }
                    }

                    // remove point 2 when you get a little wiggle like this
                    // detect by vSize(2-3) being small compared to vSize(1-2) and vSize(3-4) and the angles close to 90deg with opposite signs
                    //
                    // ---4------------------------3
                    //                             |
                    //                             2-------------------------1----
                    //

                    if (next_len < prev_len / 10 && next_len < vSize(poly[(n + 2) % poly.size()] - next_point) / 10)
                    {
                        const double sin1 = std::sin(corner_rads);
                        const double sin2 = std::sin(LinearAlg2D::getAngleLeft(poly[n], next_point, poly[(n + 2) % poly.size()]));
                        if (std::abs(sin1) > 0.8 && std::abs(sin2) > 0.8 && (sin1 < 0) != (sin2 < 0))
                        {
                            //std::cerr << gcode_layer.getLayerNr() << ": at " << poly[n] << " sin1 = " << sin1 << " sin2 = " << sin2 << " len ratio = " << prev_len/next_len << "\n";
                            poly.remove(n);
                            // hack alert - adjust n
                            --n;
                        }
                    }
                }

                // now calculate the values that define each line segment and the area it covers
                std::vector<Point> begin_points;    // corner of segment's area that is the vertex on the gap outline
                std::vector<Point> end_points;      // corner of segment's area that is orthogonal to begin_point
                std::vector<Point> mid_points;      // point that the line segment is drawn through
                std::vector<coord_t> widths;        // width of the line at this point
                for (unsigned n = 0; n < poly.size(); ++n)
                {
                    const Point& prev_point = poly[(n + poly.size() - 1) % poly.size()];
                    const Point& next_point = poly[(n + 1) % poly.size()];
                    const double corner_rads = LinearAlg2D::getAngleLeft(prev_point, poly[n], next_point);

#if 0
                    // diagnostic - print gap outline
                    gcode_layer.addTravel(prev_point);
                    gcode_layer.addExtrusionMove(poly[n], gap_config, SpaceFillType::Lines, 0.1);
#endif

                    Polygons lines;
                    Point bisector(poly[n] + rotate(normal(next_point - poly[n], avg_width * 5), corner_rads/2));

                    // adjust the width when bisector isn't normal to the direction of the next line segment
                    // if we don't do this, the resulting line width is too big where the gap polygon has sharp(ish) corners
                    lines.addLine(poly[n], bisector);
                    lines = gap_part.intersectionPolyLines(lines);
                    if (lines.size() > 0)
                    {
                        // find clipped line segment that starts/ends close to poly[n]
                        unsigned ln = 0;
                        while (ln < (lines.size() - 1) && vSize2(lines[ln][0] - poly[n]) > 100 && vSize2(lines[ln][1] - poly[n]) > 100)
                        {
                            ++ln;
                        }
                        Point clipped(lines[ln][(vSize2(lines[ln][0] - poly[n]) > 100) ? 0 : 1]);
                        coord_t line_len = vSize(lines[ln][1] - lines[ln][0]) * std::abs(std::sin(corner_rads / 2));
#if 0
                        // diagnostic - print vertex bisector lines
                        gcode_layer.addTravel(poly[n]);
                        gcode_layer.addExtrusionMove(clipped, gap_config, SpaceFillType::Lines, 0.1);
                        continue;
#endif

                        // if the bisector angle is > 90 deg we possibly want to add some points to improve the accuracy of the line width
                        const bool possibly_add_points = corner_rads > M_PI;
                        const double split_dist = 0.03;
                        if (possibly_add_points)
                        {
                            // measure the width just before the corner and if it is quite different from
                            // line_len, add another point before this one
                            const coord_t prev_dist = vSize(prev_point - poly[n]);
                            if (prev_dist >= 10)
                            {
                                const Point split(poly[n] + normal(prev_point - poly[n], std::max(3.0, prev_dist * split_dist)));
                                Polygons lines;
                                lines.addLine(split, split + normal(turn90CCW(poly[n] - split), avg_width * 5));
                                lines = gap_part.intersectionPolyLines(lines);
                                if (lines.size() > 0)
                                {
                                    unsigned ln = 0;
                                    while (ln < (lines.size() - 1) && vSize2(lines[ln][0] - split) > 100 && vSize2(lines[ln][1] - split) > 100)
                                    {
                                        ++ln;
                                    }
                                    const coord_t width = vSize(lines[ln][1] - lines[ln][0]);
                                    if (width < 2 * avg_width && std::abs(width - line_len) > line_len * 0.1)
                                    {
                                        widths.push_back(width);
                                        begin_points.emplace_back(split);
                                        end_points.emplace_back(split + normal(turn90CCW(poly[n] - split), width));
                                        mid_points.emplace_back((begin_points.back() + end_points.back()) / 2);
                                    }
                                }
                            }
                        }

                        widths.push_back(line_len);
                        begin_points.emplace_back(poly[n]);
                        end_points.emplace_back(poly[n] + normal(turn90CCW(next_point - poly[n]), line_len));
                        mid_points.emplace_back((poly[n] + clipped) / 2);

                        if (possibly_add_points)
                        {
                            // measure the width just after the corner and if it is quite different from
                            // line_len, add another point after this one
                            const coord_t next_dist = vSize(next_point - poly[n]);
                            if (next_dist >= 10)
                            {
                                const Point split(poly[n] + normal(next_point - poly[n], std::max(3.0, next_dist * split_dist)));
                                Polygons lines;
                                lines.addLine(split, split + normal(turn90CCW(next_point - split), avg_width * 5));
                                lines = gap_part.intersectionPolyLines(lines);
                                if (lines.size() > 0)
                                {
                                    unsigned ln = 0;
                                    while (ln < (lines.size() - 1) && vSize2(lines[ln][0] - split) > 100 && vSize2(lines[ln][1] - split) > 100)
                                    {
                                        ++ln;
                                    }
                                    const coord_t width = vSize(lines[ln][1] - lines[ln][0]);
                                    if (width < 2 * avg_width && std::abs(width - line_len) > line_len * 0.1)
                                    {
                                        widths.push_back(width);
                                        begin_points.emplace_back(split);
                                        end_points.emplace_back(split + normal(turn90CCW(next_point - split), width));
                                        mid_points.emplace_back((begin_points.back() + end_points.back()) / 2);
                                    }
                                }
                            }
                        }
                    }
                }

                if (widths.size() > 1)
                {
                    // filter out spikes that can occur when a point is positioned opposite to a wide area
                    // an example of this is when the walls are shaped like a T. A point on the bar of the T that lies
                    // directly above the stem will have a much bigger width than points on either side.
                    // So here we try and determine what would be a sensible width to use instead
                    for (unsigned n = 0; n < widths.size(); ++n)
                    {
                        if (widths[n] > 2 * avg_width)
                        {
                            // first we determine what we think are the widths at the next and previous points so we can compare the current
                            // point's width
                            const unsigned prev_index = ((n + widths.size()) - 1) % widths.size();
                            const unsigned next_index = (n + 1) % widths.size();
                            const coord_t prev_dist = vSize(begin_points[n] - begin_points[prev_index]);
                            const coord_t next_dist = vSize(begin_points[n] - begin_points[next_index]);
                            coord_t prev_width = widths[prev_index];
                            coord_t next_width = widths[next_index];
                            if (prev_width > 1.1 * avg_width || prev_dist > 10 * avg_width)
                            {
                                // get a new width for a point between this point and the previous
                                const Point split(begin_points[n] + (begin_points[prev_index] - begin_points[n]) * ((prev_dist > 10 * avg_width) ? 0.1 : 0.5));
                                Polygons lines;
                                lines.addLine(split, split + normal(turn90CCW(begin_points[n] - begin_points[prev_index]), prev_width));
                                lines = gap_part.intersectionPolyLines(lines);
                                if (lines.size() > 0)
                                {
                                    unsigned ln = 0;
                                    while (ln < (lines.size() - 1) && vSize2(lines[ln][0] - split) > 100 && vSize2(lines[ln][1] - split) > 100)
                                    {
                                        ++ln;
                                    }
                                    const coord_t width = vSize(lines[ln][1] - lines[ln][0]);
                                    // if the width is more reasonable, use it instead
                                    if (width < prev_width)
                                    {
                                        prev_width = width;
                                    }
                                }
                            }
                            if (next_width > 1.1 * avg_width || next_dist > 10 * avg_width)
                            {
                                // get a new width for a point between this point and the next
                                const Point split(begin_points[n] + (begin_points[next_index] - begin_points[n]) * ((next_dist > 10 * avg_width) ? 0.1 : 0.5));
                                Polygons lines;
                                lines.addLine(split, split + normal(turn90CCW(begin_points[next_index] - begin_points[n]), next_width));
                                lines = gap_part.intersectionPolyLines(lines);
                                if (lines.size() > 0)
                                {
                                    unsigned ln = 0;
                                    while (ln < (lines.size() - 1) && vSize2(lines[ln][0] - split) > 100 && vSize2(lines[ln][1] - split) > 100)
                                    {
                                        ++ln;
                                    }
                                    const coord_t width = vSize(lines[ln][1] - lines[ln][0]);
                                    // if the width is more reasonable, use it instead
                                    if (width < next_width)
                                    {
                                        next_width = width;
                                    }
                                }
                            }
                            // now see if the current width is much different to the neighbouring points' widths and
                            // if it is, use the neighbours' widths and recalculate the end and mid points
                            const coord_t new_width = (prev_width < 2 * avg_width && next_width < 2 * avg_width) ? (prev_width + next_width) / 2 : std::min(prev_width, next_width);
                            if (widths[n] > new_width)
                            {
                                const double len_scale = std::abs(std::sin(LinearAlg2D::getAngleLeft(begin_points[prev_index], begin_points[n], begin_points[next_index]) / 2));
                                if (len_scale > 0.1)
                                {
                                    widths[n] = new_width;
                                    end_points[n] = begin_points[n] + normal(turn90CCW(begin_points[next_index] - begin_points[n]), widths[n]);
                                    mid_points[n] = begin_points[n] + normal(mid_points[n] - begin_points[n], widths[n] / (2 * len_scale));
                                }
                            }
                        }
                    }
                }

#if 0
                // diagnostic - print middle of gap lines
                gcode_layer.addTravel(mid_points[0]);
                for (unsigned n = 0; n <= mid_points.size(); ++n)
                {
                    gcode_layer.addExtrusionMove(mid_points[(n + 1) % mid_points.size()], gap_config, SpaceFillType::Lines, 0.1);
                }
#endif

                if (mid_points.size() > 1)
                {
                    added_something = true;
                    setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                    gcode_layer.setIsInside(true); // going to print stuff inside print object

                    // create an area polygon for each segment, it's a hull like shape formed from the begin, end and mid points at each end of the segment
                    std::vector<Polygon> areas;
                    if (check_overlap)
                    {
                        for (unsigned point_index = 0; point_index < begin_points.size(); ++point_index)
                        {
                            const unsigned next_point_index = (point_index + 1) % begin_points.size();
                            areas.emplace_back();
                            areas.back().add(begin_points[point_index]);
                            areas.back().add(begin_points[next_point_index]);
                            // add the next mid point if it makes the area bigger
                            if (LinearAlg2D::getAngleLeft(begin_points[point_index], begin_points[next_point_index], mid_points[next_point_index]) > M_PI * 0.55)
                            {
                                areas.back().add(mid_points[next_point_index]);
                            }
                            // make the width constant based on the maximum width of the two ends
                            const coord_t hull_width = std::max(widths[point_index], widths[next_point_index]);
                            const Point seg_width_vec(normal(end_points[point_index] - begin_points[point_index], hull_width));
                            const Point next_end_point(begin_points[next_point_index] + seg_width_vec);
                            areas.back().add(next_end_point);
                            const Point end_point(begin_points[point_index] + seg_width_vec);
                            areas.back().add(end_point);
                            // add the current mid point if it makes the area bigger
                            if (LinearAlg2D::getAngleLeft(next_end_point, end_point, mid_points[point_index]) > M_PI * 0.55)
                            {
                                areas.back().add(mid_points[point_index]);
                            }
                        }
#if 0
                        // diagnostic - print area outlines
                        for (unsigned n = 0; n < areas.size(); ++n)
                        {
                            gcode_layer.addTravel(areas[n][0]);
                            for (unsigned p = 1; p < areas[n].size(); ++p)
                            {
                                gcode_layer.addExtrusionMove(areas[n][p], gap_config, SpaceFillType::Lines, 0.1);
                            }
                            gcode_layer.addExtrusionMove(areas[n][0], gap_config, SpaceFillType::Lines, 0.1);
                        }
#endif
                    }

                    // detect the mid points that we want to avoid using
                    std::vector<bool> ignore_points(mid_points.size(), false);

                    for (unsigned n = 0; n < mid_points.size(); ++n)
                    {
                        // in this scenario, we want to skip the very short line that would occur at *
                        // we do this because (a) the direction of that short line is pretty random
                        // and (b) the overlap area for that line will tend to overlap other lines' areas
                        // which will stop them being output
                        //
                        //   2--------------1------ <<
                        //   |\     /       |
                        //   | \   /        |
                        //   |  \ /         |
                        //   |   *##########@###
                        //   |  / \         |
                        //   | /   \        |
                        //   |/     \       |
                        //   3--------------------- >>
                        //
                        // the distance between the mid points for vertex bisectors 2 and 3 should be much smaller
                        // than the width of the gap at vertex 2 in this situation

                        if (vSize(mid_points[n] - mid_points[(n + 1) % mid_points.size()]) < widths[n] / 10)
                        {
                            ignore_points[n] = true;
                        }
                    }

                    Point origin;
                    if (is_outer_poly && mesh.settings.get<EZSeamType>("z_seam_type") == EZSeamType::USER_SPECIFIED)
                    {
                        // start at the mid point that is closest to the z-seam location
                        origin = Point(mesh.settings.get<coord_t>("z_seam_x"), mesh.settings.get<coord_t>("z_seam_y"));
                    }
                    else
                    {
                        // start at the mid point that is closest to the current location
                        origin = gcode_layer.getLastPlannedPositionOrStartingPosition();
                    }

                    unsigned start_point_index = 0;
                    while (start_point_index < mid_points.size() && ignore_points[start_point_index])
                    {
                        ++start_point_index;
                    }
                    if (start_point_index < mid_points.size())
                    {
                        coord_t min_dist2 = vSize2(origin - mid_points[start_point_index]);
                        for (unsigned n = start_point_index + 1; n < mid_points.size(); ++n)
                        {
                            if (!ignore_points[n])
                            {
                                coord_t dist2 = vSize2(origin - mid_points[n]);
                                if (dist2 < min_dist2)
                                {
                                    min_dist2 = dist2;
                                    start_point_index = n;
                                }
                            }
                        }
                    }
                    else
                    {
                        // avoid array bounds error in (impossible?) situation where all points are ignored
                        start_point_index = 0;
                    }

                    // if the line before the chosen start point is short, start with that one instead as this
                    // reduces the chance of having to return back to it later
                    unsigned last_point_index = (start_point_index + mid_points.size() - 1) % mid_points.size();
                    if (!ignore_points[last_point_index] && vSize(mid_points[last_point_index] - mid_points[start_point_index]) < 2 * avg_width)
                    {
                        start_point_index = last_point_index;
                    }

                    bool travel_needed = true;

                    // helper function that adds a line between two points - if the line width at each end alters appreciably, the line is sub-divided
                    std::function<void(const Point&, const Point&, const coord_t, const coord_t)> addLine = [&](const Point& start, const Point& end, const coord_t start_width, const coord_t end_width) -> void
                    {
                        const coord_t estimated_width = (start_width + end_width) / 2;
                        // split the line if it is longer than a min length and the flow required at each end differs appreciably
                        const coord_t min_len = 500;
                        const float max_flow_ratio = 1.2;
                        const float flow_ratio = (float)std::max(start_width, end_width) / std::min(start_width, end_width);
                        if (flow_ratio >= max_flow_ratio && vSize2(end - start) >= min_len * min_len)
                        {
                            const Point split_point(start + (end - start) / 2);
                            coord_t split_width = estimated_width;
                            if (!is_outline)
                            {
                                // measure the gap width at split_point and use that rather than estimated_width
                                const Point half_line(normal(turn90CCW(end - start), estimated_width * 2));
                                Polygons lines;
                                lines.addLine(split_point + half_line, split_point - half_line);
                                lines = gap_part.intersectionPolyLines(lines);
                                if (lines.size() > 0)
                                {
                                    // Limit amount the line width can grow when the line is split.
                                    // If we don't do this then it is possible that the line width could become too large because
                                    // it was measured in a place that appeared to be wide (like at a T junction).
                                    split_width = std::min(vSize(lines[0][1] - lines[0][0]), (coord_t)(estimated_width * (max_flow_ratio / 2 + 0.5f)));
                                }
                            }
                            addLine(start, split_point, start_width, split_width);
                            addLine(split_point, end, split_width, end_width);
                        }
                        else
                        {
                            const float flow = (float)estimated_width / gap_config.getLineWidth();
                            if (flow > min_flow)
                            {
                                if (travel_needed)
                                {
                                    gcode_layer.addTravel(start);
                                    travel_needed = false;
                                }
                                gcode_layer.addExtrusionMove(end, gap_config, SpaceFillType::Lines, flow);
                            }
                            else
                            {
                                travel_needed = true;
                            }
                        }
                    };

                    Point start_mid_point(mid_points[start_point_index]);

                    // output the lines between the mid points
                    for (unsigned n = 0; n < mid_points.size(); ++n)
                    {
                        const unsigned point_index = (start_point_index + n) % mid_points.size();
                        const unsigned next_point_index = (point_index + 1) % mid_points.size();
                        const Point& next_mid_point(mid_points[next_point_index]);

                        if (ignore_points[point_index])
                        {
                            start_mid_point = next_mid_point;
                            travel_needed = true;
                            continue;
                        }

                        Polygons filled; // area filled by this line
                        Polygons overlap;
                        if (check_overlap)
                        {
                            filled.add(areas[point_index]);
                            overlap = filled.intersection(all_filled_areas);
                        }

#if 0
                        if (gcode_layer.getLayerNr() == 0)
                        {
                            std::cerr << point_index << ": overlap % = " << 100 * overlap.area() / filled.area() << " is_outline = " << is_outline << "\n";
                        }
#endif
                        bool add_to_filled_areas = check_overlap;

                        if (overlap.size() > 0)
                        {
                            double overlap_area = overlap.area();
                            double filled_area = filled.area();
                            if (overlap_area > filled_area * 0.9)
                            {
                                // don't draw any of this line
                                travel_needed = true;
                                add_to_filled_areas = false;
                            }
                            else if (overlap_area > filled_area * 0.2)
                            {
                                const double seg_len = vSize(start_mid_point - next_mid_point);
                                Polygons lines;
                                lines.addLine(start_mid_point, next_mid_point);
                                lines = filled.difference(overlap).intersectionPolyLines(lines);
                                if (lines.size())
                                {
                                    // assume current position is the start of the line to ensure that the segment lines are drawn in the correct order and direction
                                    Point cur_pos(start_mid_point);

                                    while (lines.size() > 0)
                                    {
                                        unsigned closest_seg = 0;
                                        unsigned closest_end = 0;
                                        coord_t min_dist2 = vSize2(cur_pos - lines[0][0]);
                                        for (unsigned ln = 0; ln < lines.size(); ++ln)
                                        {
                                            for (unsigned en = 0; en < 2; ++en)
                                            {
                                                coord_t dist2 = vSize2(cur_pos - lines[ln][en]);
                                                if (dist2 < min_dist2)
                                                {
                                                    closest_seg = ln;
                                                    closest_end = en;
                                                    min_dist2 = dist2;
                                                }
                                            }
                                        }
                                        if (vSize(lines[closest_seg][1] - lines[closest_seg][0]) > widths[point_index] * 0.2)
                                        {
                                            const coord_t w0 = widths[point_index] + ((int)widths[next_point_index] - (int)widths[point_index]) * vSize(lines[closest_seg][0] - start_mid_point) / seg_len;
                                            const coord_t w1 = widths[point_index] + ((int)widths[next_point_index] - (int)widths[point_index]) * vSize(lines[closest_seg][1] - start_mid_point) / seg_len;
                                            if (closest_end == 0)
                                            {
                                                addLine(lines[closest_seg][0], lines[closest_seg][1], w0, w1);
                                            }
                                            else
                                            {
                                                addLine(lines[closest_seg][1], lines[closest_seg][0], w1, w0);
                                            }
                                        }
                                        else
                                        {
                                            travel_needed = true;
                                        }
                                        lines.remove(closest_seg);
                                        cur_pos = gcode_layer.getLastPlannedPositionOrStartingPosition();
                                    }
                                }
                                else
                                {
                                    travel_needed = true;
                                }
                            }
                            else
                            {
                                addLine(start_mid_point, next_mid_point, widths[point_index], widths[next_point_index]);
                            }
                        }
                        else
                        {
                            addLine(start_mid_point, next_mid_point, widths[point_index], widths[next_point_index]);
                        }

                        if (add_to_filled_areas)
                        {
                            all_filled_areas = all_filled_areas.unionPolygons(filled);
                        }

                        start_mid_point = next_mid_point;
                    }
                }
            }

            gap_polygons.remove(next_poly_index);
            if (gap_polygons.size())
            {
                // find the closest inner polygon (hole)
                ClosestPolygonPoint cpp = PolygonUtils::findClosest(gcode_layer.getLastPlannedPositionOrStartingPosition(), gap_polygons);
                if (cpp.isValid())
                {
                    next_poly_index = cpp.poly_idx;
                }
            }
            is_outer_poly = false;
        }
    }
}

}