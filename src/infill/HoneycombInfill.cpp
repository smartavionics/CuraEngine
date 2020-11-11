//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "HoneycombInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

Point HoneycombInfill::rotate_around_origin(const Point& point, const double rads)
{
    return (rads != 0) ? infill_origin + rotate(point - infill_origin, rads) : point;
}

void HoneycombInfill::generate(Polygons& result_lines, const Polygons& outline)
{
    Polygons rotated_outline = outline;
    if (fill_angle_rads != 0)
    {
        for (PolygonRef poly : rotated_outline)
        {
            for (Point& point : poly)
            {
                point = rotate_around_origin(point, -fill_angle_rads);
            }
        }
    }
    const AABB aabb(rotated_outline);

    // scale pitch so that total amount of filament used matches the amount used by the "line" infill pattern
    const int pitch = line_distance * 4.42;

    const coord_t height = pitch/3 * sqrt(3);

    x_min = infill_origin.X - std::ceil((float)(infill_origin.X - aabb.min.X) / pitch + 1) * pitch;
    y_min = infill_origin.Y - std::ceil((float)(infill_origin.Y - aabb.min.Y) / height + 1) * height;
    x_max = infill_origin.X + std::ceil((float)(aabb.max.X - infill_origin.X) / pitch + 1) * pitch;
    y_max = infill_origin.Y + std::ceil((float)(aabb.max.Y - infill_origin.Y) / height + 1) * height;

    generateCoordinates(result_lines, outline, pitch, height);

    if (zig_zaggify)
    {
        generateConnections(result_lines, outline);
    }
}

void HoneycombInfill::generateCoordinates(Polygons& result, const Polygons& outline, const coord_t pitch, const coord_t height)
{
    unsigned num_rows = 0;
    const coord_t line_width_adj_y = infill_line_width * 0.5;
    const coord_t line_width_adj_x = infill_line_width * 0.275;
    // when testing to see if a line's ends are both inside the outline, use an outline that has been shrunk to ensure we
    // catch the situation where both ends are inside the area but between the ends the line hits/crosses the boundary
    const Polygons shrunk_outline = outline.offset(-pitch / 3);

    for (coord_t y = y_min; y < y_max; y += height)
    {
        for (int sign_i = 0; sign_i < 2; ++sign_i)
        {
            const int sign = sign_i * 2 - 1;
            bool is_first_point = true;
            Point last(x_min, y + sign * line_width_adj_y);
            bool last_inside = false;
            unsigned chain_end_index = 0;
            Point chain_end[2];
            for (coord_t x = x_min; x < x_max; x += pitch)
            {
                for (int segment = 0; segment < 4; ++segment)
                {
                    Point current;
                    switch (segment)
                    {
                        case 0:
                            current = Point(x + pitch/3+line_width_adj_x, y + sign * line_width_adj_y);
                            break;
                        case 1:
                            current = Point(x + pitch/2-line_width_adj_x, y + sign * (height/2 - line_width_adj_y));
                            break;
                        case 2:
                            current = Point(x + pitch*5/6+line_width_adj_x, y + sign * (height/2 - line_width_adj_y));
                            break;
                        case 3:
                            current = Point(x + pitch-line_width_adj_x, y + sign * line_width_adj_y);
                            break;
                    }
                    current = rotate_around_origin(current, fill_angle_rads);
                    const bool current_inside = shrunk_outline.inside(current, false);
                    if (!is_first_point)
                    {
                        if (last_inside && current_inside)
                        {
                            // line doesn't hit the boundary, add the whole line
                            result.addLine(last, current);
                        }
                        else
                        {
                            // add the parts of the line that are inside the boundary
                            Polygons line;
                            line.addLine(last, current);
                            for (ConstPolygonRef line_seg : outline.intersectionPolyLines(line))
                            {
                                result.addLine(line_seg[0], line_seg[1]);

                                if (zig_zaggify)
                                {
                                    for (const Point& pt : line_seg)
                                    {
                                        if ((pt != last && pt != current) || !outline.inside(pt, false))
                                        {
                                            chain_end[chain_end_index] = pt;
                                            if (++chain_end_index == 2)
                                            {
                                                chains[0].push_back(chain_end[0]);
                                                chains[1].push_back(chain_end[1]);
                                                chain_end_index = 0;
                                                connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                                connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                                line_numbers.push_back(num_rows);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    last = current;
                    last_inside = current_inside;
                    is_first_point = false;
                }
            }
            ++num_rows;
        }
    }
}

void HoneycombInfill::generateConnections(Polygons& result, const Polygons& outline)
{
    // zig-zaggification consists of joining alternate chain ends to make a chain of chains
    // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
    // whenever we come across the end of a chain we toggle the connector drawing state
    // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
    // of the indentity of the first chain in a connected sequence

    int chain_ends_remaining = chains[0].size() * 2;

    if (chain_ends_remaining == 0)
    {
        return;
    }

    for (ConstPolygonRef outline_poly : outline)
    {
        std::vector<Point> connector_points; // the points that make up a connector line

        // we need to remember the first chain processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last chain processed
        unsigned first_chain_chain_index = std::numeric_limits<unsigned>::max();
        std::vector<Point> path_to_first_chain;

        bool drawing = false; // true when a connector line is being (potentially) created

        // keep track of the chain+point that a connector line started at
        unsigned connector_start_chain_index = std::numeric_limits<unsigned>::max();
        unsigned connector_start_point_index = std::numeric_limits<unsigned>::max();

        Point cur_point; // current point of interest - either an outline point or a chain end

        // go round all of the region's outline and find the chain ends that meet it
        // quit the loop early if we have seen all the chain ends and are not currently drawing a connector
        for (unsigned outline_point_index = 0; (chain_ends_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
        {
            Point op0 = outline_poly[outline_point_index];
            Point op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
            std::vector<unsigned> points_on_outline_chain_index;
            std::vector<unsigned> points_on_outline_point_index;

            // collect the chain ends that meet this segment of the outline
            for (unsigned chain_index = 0; chain_index < chains[0].size(); ++chain_index)
            {
                for (unsigned point_index = 0; point_index < 2; ++point_index)
                {
                    // don't include chain ends that are close to the segment but are beyond the segment ends
                    short beyond = 0;
                    if (LinearAlg2D::getDist2FromLineSegment(op0, chains[point_index][chain_index], op1, &beyond) < 10 && !beyond)
                    {
                        points_on_outline_point_index.push_back(point_index);
                        points_on_outline_chain_index.push_back(chain_index);
                    }
                }
            }

            if (outline_point_index == 0 || vSize2(op0 - cur_point) > 100)
            {
                // this is either the first outline point or it is another outline point that is not too close to cur_point

                if (first_chain_chain_index == std::numeric_limits<unsigned>::max())
                {
                    // include the outline point in the path to the first chain
                    path_to_first_chain.push_back(op0);
                }

                cur_point = op0;
                if (drawing)
                {
                    // include the start point of this outline segment in the connector
                    connector_points.push_back(op0);
                }
            }

            // iterate through each of the chain ends that meet the current outline segment
            while (points_on_outline_chain_index.size() > 0)
            {
                // find the nearest chain end to the current point
                unsigned nearest_point_index = 0;
                float nearest_point_dist2 = std::numeric_limits<float>::infinity();
                for (unsigned pi = 0; pi < points_on_outline_chain_index.size(); ++pi)
                {
                    float dist2 = vSize2f(chains[points_on_outline_point_index[pi]][points_on_outline_chain_index[pi]] - cur_point);
                    if (dist2 < nearest_point_dist2)
                    {
                        nearest_point_dist2 = dist2;
                        nearest_point_index = pi;
                    }
                }
                const unsigned point_index = points_on_outline_point_index[nearest_point_index];
                const unsigned chain_index = points_on_outline_chain_index[nearest_point_index];

                // make the chain end the current point and add it to the connector line
                cur_point = chains[point_index][chain_index];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < 100)
                {
                    // this chain end will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                if (first_chain_chain_index == std::numeric_limits<unsigned>::max())
                {
                    // this is the first chain to be processed, remember it
                    first_chain_chain_index = chain_index;
                    path_to_first_chain.push_back(cur_point);
                }

                if (drawing)
                {
                    // add the connector line segments but only if
                    //  1 - the start/end points are not the opposite ends of the same chain
                    //  2 - the other end of the current chain is not connected to the chain the connector line is coming from

                    if (chain_index != connector_start_chain_index && connected_to[(point_index + 1) % 2][chain_index] != connector_start_chain_index)
                    {
                        for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                        {
                            result.addLine(connector_points[pi - 1], connector_points[pi]);
                        }
                        drawing = false;
                        connector_points.clear();
                        // remember the connection
                        connected_to[point_index][chain_index] = connector_start_chain_index;
                        connected_to[connector_start_point_index][connector_start_chain_index] = chain_index;
                    }
                    else
                    {
                        // start a new connector from the current location
                        connector_points.clear();
                        connector_points.push_back(cur_point);

                        // remember the chain+point that the connector started from
                        connector_start_chain_index = chain_index;
                        connector_start_point_index = point_index;
                    }
                }
                else
                {
                    // we have just jumped a gap so now we want to start drawing again
                    drawing = true;

                    // if this connector is the first to be created or we are not connecting chains from the same row/column,
                    // remember the chain+point that this connector is starting from
                    if (connector_start_chain_index == std::numeric_limits<unsigned>::max() || line_numbers[chain_index] != line_numbers[connector_start_chain_index])
                    {
                        connector_start_chain_index = chain_index;
                        connector_start_point_index = point_index;
                    }
                }

                // done with this chain end
                points_on_outline_chain_index.erase(points_on_outline_chain_index.begin() + nearest_point_index);
                points_on_outline_point_index.erase(points_on_outline_point_index.begin() + nearest_point_index);

                // decrement total amount of work to do
                --chain_ends_remaining;
            }
        }

        // we have now visited all the points in the outline, if a connector was (potentially) being drawn
        // check whether the first chain is already connected to the last chain and, if not, draw the
        // connector between
        if (drawing && first_chain_chain_index != std::numeric_limits<unsigned>::max()
            && first_chain_chain_index != connector_start_chain_index
            && connected_to[0][first_chain_chain_index] != connector_start_chain_index
            && connected_to[1][first_chain_chain_index] != connector_start_chain_index)
        {
            // output the connector line segments from the last chain to the first point in the outline
            connector_points.push_back(outline_poly[0]);
            for (unsigned pi = 1; pi < connector_points.size(); ++pi)
            {
                result.addLine(connector_points[pi - 1], connector_points[pi]);
            }
            // output the connector line segments from the first point in the outline to the first chain
            for (unsigned pi = 1; pi < path_to_first_chain.size(); ++pi)
            {
                result.addLine(path_to_first_chain[pi - 1], path_to_first_chain[pi]);
            }
        }

        if (chain_ends_remaining < 1)
        {
            break;
        }
    }
}

} // namespace cura