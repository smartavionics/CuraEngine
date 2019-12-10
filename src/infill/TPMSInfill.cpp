//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

// Implement a few TPMS (Triply Periodic Minimal Surfaces) infill patterns

#include "TPMSInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

TPMSInfill::TPMSInfill(const bool zig_zaggify, const coord_t line_distance, const coord_t z, const EFillMethod pattern, const EFillResolution resolution, const Point& infill_origin, const AngleDegrees fill_angle)
    : zig_zaggify(zig_zaggify)
    , line_distance(line_distance)
    , z(z)
    , pattern(pattern)
    , resolution(resolution)
    , infill_origin(infill_origin)
    , fill_angle_rads(fill_angle / (180 / M_PI))
{
}

TPMSInfill::~TPMSInfill() {
}

Point TPMSInfill::rotate_around_origin(const Point& point, const double rads)
{
    return (rads != 0) ? infill_origin + rotate(point - infill_origin, rads) : point;
}

void TPMSInfill::generate(Polygons& result_lines, const Polygons& outline)
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

    int pitch = line_distance;
    // scale pitch so that total amount of filament used matches the amount used by the "line" infill pattern
    if (pattern == EFillMethod::GYROID)
    {
        pitch *= 2.41;
    }
    else if (pattern == EFillMethod::SCHWARZ_P)
    {
        pitch *= 1.94;
    }
    else if (pattern == EFillMethod::SCHWARZ_D)
    {
        pitch *= 1.5;
    }

    int num_steps = 4;
    int step = pitch / num_steps;
    const int max_steps = (resolution == EFillResolution::LOW_RESOLUTION) ? 4 : (resolution == EFillResolution::MEDIUM_RESOLUTION) ? 8 : 16;
    while (step > 500 && num_steps < max_steps)
    {
        num_steps *= 2;
        step = pitch / num_steps;
    }
    pitch = step * num_steps; // recalculate to avoid precision errors

    x_min = infill_origin.X - std::ceil((float)(infill_origin.X - aabb.min.X) / pitch) * pitch;
    y_min = infill_origin.Y - std::ceil((float)(infill_origin.Y - aabb.min.Y) / pitch + 0.25) * pitch;
    x_max = infill_origin.X + std::ceil((float)(aabb.max.X - infill_origin.X) / pitch) * pitch;
    y_max = infill_origin.Y + std::ceil((float)(aabb.max.Y - infill_origin.Y) / pitch + 0.25) * pitch;

    if (pattern == EFillMethod::GYROID)
    {
        generateGyroidCoordinates(result_lines, outline, pitch, step);

        if (zig_zaggify && chains[0].size() > 0)
        {
            generateGyroidConnections(result_lines, outline);
        }
    }
    else if (pattern == EFillMethod::SCHWARZ_P)
    {
        generateSchwarzPCoordinates(result_lines, outline, pitch, step);

        if (zig_zaggify)
        {
            generateSchwarzPConnections(result_lines, outline);
        }
    }
    else if (pattern == EFillMethod::SCHWARZ_D)
    {
        generateSchwarzDCoordinates(result_lines, outline, pitch, step);

        if (zig_zaggify)
        {
            generateSchwarzPConnections(result_lines, outline);
        }
    }
}

void TPMSInfill::generateGyroidCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
{
    // generate infill based on the gyroid equation: sin_x * cos_y + sin_y * cos_z + sin_z * cos_x = 0
    // kudos to the author of the Slic3r implementation equation code, the equation code here is based on that

    const double z_rads = 2 * M_PI * z / pitch;
    const double cos_z = std::cos(z_rads);
    const double sin_z = std::sin(z_rads);

    std::vector<coord_t> odd_line_coords;
    std::vector<coord_t> even_line_coords;

    if (std::abs(sin_z) <= std::abs(cos_z))
    {
        // "vertical" lines
        const double phase_offset = ((cos_z < 0) ? M_PI : 0) + M_PI;
        for (coord_t y = 0; y < pitch; y += step)
        {
            const double y_rads = 2 * M_PI * y / pitch;
            const double a = cos_z;
            const double b = std::sin(y_rads + phase_offset);
            const double odd_c = sin_z * std::cos(y_rads + phase_offset);
            const double even_c = sin_z * std::cos(y_rads + phase_offset + M_PI);
            const double h = std::sqrt(a * a + b * b);
            const double odd_x_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) - M_PI/2;
            const double even_x_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) - M_PI/2;
            odd_line_coords.push_back(odd_x_rads / M_PI * pitch);
            even_line_coords.push_back(even_x_rads / M_PI * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_columns = 0;
        for (coord_t x = x_min - 1.25 * pitch; x < x_max; x += pitch/2)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            unsigned chain_end_index = 0;
            Point chain_end[2];
            for (coord_t y = y_min; y < y_max; y += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    Point current(x + ((num_columns & 1) ? odd_line_coords[i] : even_line_coords[i])/2 + pitch, y + (coord_t)(i * step));
                    current = rotate_around_origin(current, fill_angle_rads);
                    bool current_inside = outline.inside(current, true);
                    if (!is_first_point)
                    {
                        if (last_inside && current_inside)
                        {
                            // line doesn't hit the boundary, add the whole line
                            result.addLine(last, current);
                        }
                        else if (last_inside != current_inside)
                        {
                            // line hits the boundary, add the part that's inside the boundary
                            Polygons line;
                            line.addLine(last, current);
                            line = outline.intersectionPolyLines(line);
                            if (line.size() > 0)
                            {
                                // some of the line is inside the boundary
                                result.addLine(line[0][0], line[0][1]);
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
                                    if (++chain_end_index == 2)
                                    {
                                        chains[0].push_back(chain_end[0]);
                                        chains[1].push_back(chain_end[1]);
                                        chain_end_index = 0;
                                        connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                        connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                        line_numbers.push_back(num_columns);
                                    }
                                }
                            }
                            else
                            {
                                // none of the line is inside the boundary so the point that's actually on the boundary
                                // is the chain end
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = (last_inside) ? last : current;
                                    if (++chain_end_index == 2)
                                    {
                                        chains[0].push_back(chain_end[0]);
                                        chains[1].push_back(chain_end[1]);
                                        chain_end_index = 0;
                                        connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                        connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                        line_numbers.push_back(num_columns);
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
            ++num_columns;
        }
    }
    else
    {
        // "horizontal" lines
        const double phase_offset = (sin_z < 0) ? M_PI : 0;
        for (coord_t x = 0; x < pitch; x += step)
        {
            const double x_rads = 2 * M_PI * x / pitch;
            const double a = sin_z;
            const double b = std::cos(x_rads + phase_offset);
            const double odd_c = cos_z * std::sin(x_rads + phase_offset + M_PI);
            const double even_c = cos_z * std::sin(x_rads + phase_offset);
            const double h = std::sqrt(a * a + b * b);
            const double odd_y_rads = ((h != 0) ? std::asin(odd_c / h) + std::asin(b / h) : 0) + M_PI/2;
            const double even_y_rads = ((h != 0) ? std::asin(even_c / h) + std::asin(b / h) : 0) + M_PI/2;
            odd_line_coords.push_back(odd_y_rads / M_PI * pitch);
            even_line_coords.push_back(even_y_rads / M_PI * pitch);
        }
        const unsigned num_coords = odd_line_coords.size();
        unsigned num_rows = 0;
        for (coord_t y = y_min; y < y_max; y += pitch/2)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            unsigned chain_end_index = 0;
            Point chain_end[2];
            for (coord_t x = x_min; x < x_max; x += pitch)
            {
                for (unsigned i = 0; i < num_coords; ++i)
                {
                    Point current(x + (coord_t)(i * step), y + ((num_rows & 1) ? odd_line_coords[i] : even_line_coords[i])/2);
                    current = rotate_around_origin(current, fill_angle_rads);
                    bool current_inside = outline.inside(current, true);
                    if (!is_first_point)
                    {
                        if (last_inside && current_inside)
                        {
                            // line doesn't hit the boundary, add the whole line
                            result.addLine(last, current);
                        }
                        else if (last_inside != current_inside)
                        {
                            // line hits the boundary, add the part that's inside the boundary
                            Polygons line;
                            line.addLine(last, current);
                            line = outline.intersectionPolyLines(line);
                            if (line.size() > 0)
                            {
                                // some of the line is inside the boundary
                                result.addLine(line[0][0], line[0][1]);
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
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
                            else
                            {
                                // none of the line is inside the boundary so the point that's actually on the boundary
                                // is the chain end
                                if (zig_zaggify)
                                {
                                    chain_end[chain_end_index] = (last_inside) ? last : current;
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
                    last = current;
                    last_inside = current_inside;
                    is_first_point = false;
                }
            }
            ++num_rows;
        }
    }
}


void TPMSInfill::generateSchwarzPCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
{
    // generate Schwarz P "Primitive" surface defined by equation: cos(x) + cos(y) + cos(z) = 0
    // see https://en.wikipedia.org/wiki/Schwarz_minimal_surface

    const double cos_z = std::cos(2 * M_PI * z / pitch);

    std::vector<bool> gaps;
    std::vector<coord_t> x_coords;
    std::vector<coord_t> y_coords;

    bool at_start = true;
    double large_y_inc = step / 4.0;
    double small_y_inc = step / 16.0;
    double y_inc = small_y_inc;

    // generate coordinates for first half of the steps in the y direction
    for (double y = 0; y < pitch/2; y += y_inc)
    {
        const double y_rads = 2 * M_PI * y / pitch;
        double x_val = std::cos(y_rads) + cos_z;
        // if x_val is not in range, we have a gap in the line
        bool gap = (x_val < -1.0 || x_val > 1.0);
        if (at_start)
        {
            if (gap)
            {
                if (gaps.empty())
                {
                    // mark the gap
                    gaps.push_back(true);
                    x_coords.push_back(0);
                    y_coords.push_back(0);
                }
                continue;
            }
            if (!gaps.empty())
            {
                // insert extra point that lies exactly on the surface
                gaps.push_back(false);
                x_coords.push_back(std::acos((x_val < 0) ? -1.0 : 1.0) / M_PI / 2 * pitch + pitch/4);
                y_coords.push_back(y);
            }
            at_start = false;
            y_inc = large_y_inc;
        }
        else if (gap)
        {
            if (y_inc != small_y_inc)
            {
                // we have gone too far, go back and use smaller increments
                y -= y_inc;
                y_inc = small_y_inc;
                continue;
            }
            else
            {
                // insert extra point that lies exactly on the surface
                gaps.push_back(false);
                x_coords.push_back(std::acos((x_val < 0) ? -1.0 : 1.0) / M_PI / 2 * pitch + pitch/4);
                y_coords.push_back(y);
            }
        }

        if (gap)
        {
            if(gaps.empty() || !gaps.back())
            {
                // mark the gap
                gaps.push_back(true);
                x_coords.push_back(0);
                y_coords.push_back(0);
            }
        }
        else
        {
            // a point on the surface
            gaps.push_back(false);
            x_coords.push_back(std::acos(x_val) / M_PI / 2 * pitch + pitch/4);
            y_coords.push_back(y);
        }
    }

    // value for y == pitch/2
    double x_val = cos_z - 1;
    if (x_val < -1.0)
    {
        gaps.push_back(true);
        x_coords.push_back(0);
        y_coords.push_back(0);
    }
    else
    {
        gaps.push_back(false);
        x_coords.push_back(std::acos(x_val) / M_PI / 2 * pitch + pitch/4);
        y_coords.push_back(pitch/2);
    }

    // mirror the values below pitch/2

    for (int i = x_coords.size() - 2; i > 0; --i)
    {
        gaps.push_back(gaps[i]);
        x_coords.push_back(x_coords[i]);
        y_coords.push_back(pitch - y_coords[i]);
    }

    // repeat the lines over the infill area
    const unsigned num_coords = x_coords.size();
    unsigned num_columns = 0;
    for (coord_t x = x_min - 1.25 * pitch; x < x_max; x += pitch/2)
    {
        bool is_first_point = true;
        Point last;
        bool last_inside = false;
        unsigned num_rows = 0;
        for (coord_t y = y_min; y < y_max; y += pitch)
        {
            for (unsigned i = 0; i < num_coords; ++i)
            {
                if (gaps[i])
                {
                    is_first_point = true;
                    continue;
                }

                Point current(x + ((num_columns & 1) ? -x_coords[i] : x_coords[i]) + pitch, y + y_coords[i]);
                current = rotate_around_origin(current, fill_angle_rads);
                bool current_inside = outline.inside(current, true);
                if (!is_first_point)
                {
                    if (last_inside && current_inside)
                    {
                        // line doesn't hit the boundary, add the whole line
                        result.addLine(last, current);
                    }
                    else if (last_inside != current_inside)
                    {
                        // line hits the boundary, add the part that's inside the boundary
                        Polygons line;
                        line.addLine(last, current);
                        line = outline.intersectionPolyLines(line);
                        unsigned connection_id = 0;
                        if (cos_z >= 0)
                        {
                            connection_id = (num_columns/2 << 16) | num_rows;
                        }
                        else
                        {
                            connection_id = (((num_columns+1)/2) << 16) | (num_rows + (i > num_coords/2));
                        }
                        if (line.size() > 0)
                        {
                            // some of the line is inside the boundary
                            result.addLine(line[0][0], line[0][1]);
                            if (zig_zaggify)
                            {
                                connection_points.push_back(line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1]);
                                connection_ids.push_back(connection_id);
                            }
                        }
                        else
                        {
                            // none of the line is inside the boundary so the point that's actually on the boundary
                            // is the chain end
                            if (zig_zaggify)
                            {
                                connection_points.push_back((last_inside) ? last : current);
                                connection_ids.push_back(connection_id);
                            }
                        }
                    }
                }
                last = current;
                last_inside = current_inside;
                is_first_point = false;
            }
            ++num_rows;
        }
        ++num_columns;
    }
}

void TPMSInfill::generateSchwarzDCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
{
    // generate Schwarz D "Primitive" surface defined by equation: sin(x) sin(y) sin(z) + sin(x) cos(y) cos(z) + cos(x) sin(y) cos(z) + cos(x) cos(y) sin(z) = 0
    // see https://en.wikipedia.org/wiki/Schwarz_minimal_surface

    const double cos_z = std::cos(M_PI * z / pitch);
    const double sin_z = std::sin(M_PI * z / pitch);

    std::vector<coord_t> x_coords;
    std::vector<coord_t> y_coords;

    double x_inc = step / 4.0;

    for (double x = 0; x <= pitch; x += x_inc)
    {
        const double x_rads = M_PI * x / pitch;
        const double sin_x = std::sin(x_rads);
        const double cos_x = std::cos(x_rads);

        const double A = sin_x * sin_z;
        const double B = cos_x * cos_z;
        const double C = sin_x * cos_z;
        const double D = cos_x * sin_z;
        const double y_rads = std::atan(-(C + D) / (A + B));

        x_coords.push_back(x);
        y_coords.push_back(y_rads / M_PI * pitch);
    }

    // repeat the lines over the infill area
    const unsigned num_coords = x_coords.size();
    unsigned num_cols = 0;
    for (coord_t x = x_min - 1.25 * pitch; x < x_max; x += pitch)
    {
        unsigned chain_end_index = 0;
        Point chain_end[2];
        for (coord_t y = y_min; y < y_max; y += pitch)
        {
            bool is_first_point = true;
            Point last;
            bool last_inside = false;
            for (unsigned i = 0; i < num_coords; ++i)
            {
                coord_t y_coord = y_coords[i];
                unsigned last_i = (i + num_coords - 1) % num_coords;
                bool discontinuity = false;
                if (std::abs(y_coords[last_i] - y_coords[i]) > pitch/2)
                {
                    y_coord += (y_coords[last_i] < y_coords[i]) ? -pitch : pitch;
                    discontinuity = true;
                }
                Point current(x + x_coords[i], y + y_coord);
                current = rotate_around_origin(current, fill_angle_rads);
                bool current_inside = outline.inside(current, true);
                if (!is_first_point)
                {
                    if (last_inside && current_inside)
                    {
                        // line doesn't hit the boundary, add the whole line
                        result.addLine(last, current);
                    }
                    else if (last_inside != current_inside)
                    {
                        // line hits the boundary, add the part that's inside the boundary
                        Polygons line;
                        line.addLine(last, current);
                        line = outline.intersectionPolyLines(line);
                        if (line.size() > 0)
                        {
                            // some of the line is inside the boundary
                            result.addLine(line[0][0], line[0][1]);
                            if (zig_zaggify)
                            {
                                chain_end[chain_end_index] = line[0][(line[0][0] != last && line[0][0] != current) ? 0 : 1];
                                if (++chain_end_index == 2)
                                {
                                    chains[0].push_back(chain_end[0]);
                                    chains[1].push_back(chain_end[1]);
                                    chain_end_index = 0;
                                    connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                    connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                    line_numbers.push_back(num_cols);
                                }
                            }
                        }
                        else
                        {
                            // none of the line is inside the boundary so the point that's actually on the boundary
                            // is the chain end
                            if (zig_zaggify)
                            {
                                chain_end[chain_end_index] = (last_inside) ? last : current;
                                if (++chain_end_index == 2)
                                {
                                    chains[0].push_back(chain_end[0]);
                                    chains[1].push_back(chain_end[1]);
                                    chain_end_index = 0;
                                    connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                                    connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                                    line_numbers.push_back(num_cols);
                                }
                            }
                        }
                    }
                }
                if (discontinuity)
                {
                    last = rotate_around_origin(Point(x + x_coords[i], y + y_coords[i]), fill_angle_rads);
                    last_inside = outline.inside(last, true);
                }
                else
                {
                    last = current;
                    last_inside = current_inside;
                }
                is_first_point = false;
            }
        }
        ++num_cols;
    }
}

void TPMSInfill::generateGyroidConnections(Polygons& result, const Polygons& outline)
{
    // zig-zaggification consists of joining alternate chain ends to make a chain of chains
    // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
    // whenever we come across the end of a chain we toggle the connector drawing state
    // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
    // of the indentity of the first chain in a connected sequence

    int chain_ends_remaining = chains[0].size() * 2;

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

void TPMSInfill::generateSchwarzPConnections(Polygons& result, const Polygons& outline)
{
    // zig-zaggification consists of joining alternate chain ends to make a chain of chains
    // the basic algorithm is that we follow the infill area boundary and as we progress we are either drawing a connector or not
    // whenever we come across the end of a chain we toggle the connector drawing state
    // things are made more complicated by the fact that we want to avoid generating loops and so we need to keep track
    // of the indentity of the first chain in a connected sequence

    int connections_remaining = connection_points.size();

    unsigned last_connection_point_id = std::numeric_limits<unsigned>::max();

    for (ConstPolygonRef outline_poly : outline)
    {
        std::vector<Point> connector_points; // the points that make up a connector line

        // we need to remember the first chain processed and the path to it from the first outline point
        // so that later we can possibly connect to it from the last chain processed
        unsigned first_connection_index = std::numeric_limits<unsigned>::max();
        std::vector<Point> path_to_first_connection;

        bool drawing = false; // true when a connector line is being (potentially) created

        // keep track of the chain+point that a connector line started at
        unsigned connector_start_chain_index = std::numeric_limits<unsigned>::max();

        Point cur_point; // current point of interest - either an outline point or a chain end

        // go round all of the region's outline and find the chain ends that meet it
        // quit the loop early if we have seen all the chain ends and are not currently drawing a connector
        for (unsigned outline_point_index = 0; (connections_remaining > 0 || drawing) && outline_point_index < outline_poly.size(); ++outline_point_index)
        {
            Point op0 = outline_poly[outline_point_index];
            Point op1 = outline_poly[(outline_point_index + 1) % outline_poly.size()];
            std::vector<unsigned> points_on_outline_connection_point_index;

            // collect the connections that meet this segment of the outline
            for (unsigned connection_points_index = 0; connection_points_index < connection_points.size(); ++connection_points_index)
            {
                // don't include chain ends that are close to the segment but are beyond the segment ends
                short beyond = 0;
                if (LinearAlg2D::getDist2FromLineSegment(op0, connection_points[connection_points_index], op1, &beyond) < 10 && !beyond)
                {
                    points_on_outline_connection_point_index.push_back(connection_points_index);
                }
            }

            if (outline_point_index == 0 || vSize2(op0 - cur_point) > 100)
            {
                // this is either the first outline point or it is another outline point that is not too close to cur_point

                if (first_connection_index == std::numeric_limits<unsigned>::max())
                {
                    // include the outline point in the path to the first chain
                    path_to_first_connection.push_back(op0);
                }

                cur_point = op0;
                if (drawing)
                {
                    // include the start point of this outline segment in the connector
                    connector_points.push_back(op0);
                }
            }

            // iterate through each of the connection points that meet the current outline segment
            while (points_on_outline_connection_point_index.size() > 0)
            {
                // find the nearest connection point to the current point
                unsigned nearest_connection_point_index = 0;
                float nearest_point_dist2 = std::numeric_limits<float>::infinity();
                for (unsigned pi = 0; pi < points_on_outline_connection_point_index.size(); ++pi)
                {
                    float dist2 = vSize2f(connection_points[points_on_outline_connection_point_index[pi]] - cur_point);
                    if (dist2 < nearest_point_dist2)
                    {
                        nearest_point_dist2 = dist2;
                        nearest_connection_point_index = pi;
                    }
                }

                // make the chain end the current point and add it to the connector line
                cur_point = connection_points[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (drawing && connector_points.size() > 0 && vSize2(cur_point - connector_points.back()) < 100)
                {
                    // this chain end will be too close to the last connector point so throw away the last connector point
                    connector_points.pop_back();
                }
                connector_points.push_back(cur_point);

                if (first_connection_index == std::numeric_limits<unsigned>::max())
                {
                    // this is the first chain to be processed, remember it
                    first_connection_index = nearest_connection_point_index;
                    path_to_first_connection.push_back(cur_point);
                }

                const unsigned connection_point_id = connection_ids[points_on_outline_connection_point_index[nearest_connection_point_index]];

                if (drawing)
                {
                    // add the connector line segments but only if
                    //  1 - the start/end points are not the opposite ends of the same chain
                    //  2 - the other end of the current chain is not connected to the chain the connector line is coming from

                    if (last_connection_point_id != connection_point_id)
                    {
                        for (unsigned pi = 1; pi < connector_points.size(); ++pi)
                        {
                            result.addLine(connector_points[pi - 1], connector_points[pi]);
                        }
                        drawing = false;
                        connector_points.clear();
                        last_connection_point_id = std::numeric_limits<unsigned>::max();
                    }
                    else
                    {
                        // start a new connector from the current location
                        connector_points.clear();
                        connector_points.push_back(cur_point);

                        // remember the connection point that the connector started from
                        last_connection_point_id = connection_point_id;
                    }
                }
                else
                {
                    // we have just jumped a gap so now we want to start drawing again
                    drawing = true;

                    // remember the connection point that the connector started from
                    last_connection_point_id = connection_point_id;
/*
                    // if this connector is the first to be created or we are not connecting chains from the same row/column,
                    // remember the chain+point that this connector is starting from
                    if (connector_start_chain_index == std::numeric_limits<unsigned>::max())
                    {
                        connector_start_chain_index = points_on_outline_connection_point_index[nearest_connection_point_index];
                    }
*/
                }

                // done with this chain end
                points_on_outline_connection_point_index.erase(points_on_outline_connection_point_index.begin() + nearest_connection_point_index);

                // decrement total amount of work to do
                --connections_remaining;
            }
        }

        // we have now visited all the points in the outline, if a connector was (potentially) being drawn
        // check whether the first chain is already connected to the last chain and, if not, draw the
        // connector between
        if (drawing && first_connection_index != std::numeric_limits<unsigned>::max())
        // && first_connection_index != connector_start_chain_index)
        {
            // output the connector line segments from the last chain to the first point in the outline
            connector_points.push_back(outline_poly[0]);
            for (unsigned pi = 1; pi < connector_points.size(); ++pi)
            {
                result.addLine(connector_points[pi - 1], connector_points[pi]);
            }
            // output the connector line segments from the first point in the outline to the first chain
            for (unsigned pi = 1; pi < path_to_first_connection.size(); ++pi)
            {
                result.addLine(path_to_first_connection[pi - 1], path_to_first_connection[pi]);
            }
        }

        if (connections_remaining < 1)
        {
            break;
        }
    }
}

void TPMSInfill::generateSchwarzDConnections(Polygons& result, const Polygons& outline)
{
}

} // namespace cura