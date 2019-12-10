//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TPMSInfill.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

void TPMSInfillSchwarzD::generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step)
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

void TPMSInfillSchwarzD::generateConnections(Polygons& result, const Polygons& outline)
{
}

} // namespace cura