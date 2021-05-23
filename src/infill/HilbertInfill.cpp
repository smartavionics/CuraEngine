//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "HilbertInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

Point HilbertInfill::rotate_around_origin(const Point& point, const double rads)
{
    return (rads != 0) ? infill_origin + rotate(point - infill_origin, rads) : point;
}

void HilbertInfill::generate(Polygons& result_lines, const Polygons& outline, const coord_t mesh_max_size)
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

    const coord_t outline_max_size = std::max(aabb.max.X - aabb.min.X, aabb.max.Y - aabb.min.Y);

    if (outline_max_size < 10)
    {
        // don't know why but some really small areas can get here which cause crashes so ignore them
        return;
    }

    // determine hilbert recursion depth and size of the square area to be filled with the curve

    // the number of lines required to cover the mesh is rounded up to an integer power of 2
    const int depth = std::ceil(std::log2((double)mesh_max_size / line_distance));

    const coord_t size = std::exp2(depth) * line_distance;

    //std::cerr << "max_size = " << max_size << ", size = " << size << ", depth = " << depth << ", line_distance = " << line_distance << "\n";

    x_min = aabb.min.X;
    y_min = aabb.min.Y;
    x_max = aabb.max.X;
    y_max = aabb.max.Y;

    // when testing to see if a line's ends are both inside the outline, use an outline that has been shrunk to ensure we
    // catch the situation where both ends are inside the area but between the ends the line hits/crosses the boundary
    generateCoordinates(result_lines, outline, outline.offset(-line_distance / 2), size, depth);

    if (zig_zaggify)
    {
        generateConnections(result_lines, outline);
    }
}

void HilbertInfill::generateCoordinates(Polygons& result, const Polygons& outline, const Polygons& shrunk_outline, const coord_t size, const int depth)
{
    bool is_first_point = true;
    bool last_inside = false;
    Point last(x_min, y_min);
    unsigned chain_end_index = 0;
    Point chain_end[2];

    std::function<void(coord_t, coord_t, coord_t, coord_t, coord_t, coord_t, int)> hilbert = [&, this](coord_t x0, coord_t y0, coord_t xi, coord_t xj, coord_t yi, coord_t yj, int n) {
        if (n <= 0)
        {
            const coord_t x = x0 + (xi + yi)/2;
            const coord_t y = y0 + (xj + yj)/2;
            Point current(x, y);
            current = rotate_around_origin(current, fill_angle_rads);
            if ((current.X < x_min && last.X < x_min) || (current.X > x_max && last.X > x_max) ||
                (current.Y < y_min && last.Y < y_min) || (current.Y > y_max && last.Y > y_max))
            {
                last = current;
                last_inside = false;
                return;
            }
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
        else
        {
            hilbert(x0,               y0,               yi/2, yj/2, xi/2, xj/2, n - 1);
            hilbert(x0 + xi/2,        y0 + xj/2,        xi/2, xj/2, yi/2, yj/2, n - 1);
            hilbert(x0 + xi/2 + yi/2, y0 + xj/2 + yj/2, xi/2, xj/2, yi/2, yj/2, n - 1);
            hilbert(x0 + xi/2 + yi,   y0 + xj/2 + yj,  -yi/2,-yj/2,-xi/2,-xj/2, n - 1);
        }
    };

    hilbert(infill_origin.X - size/2, infill_origin.Y - size/2, size, 0, 0, size, depth);
}

void HilbertInfill::generateConnections(Polygons& result, const Polygons& outline)
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

                    // if this connector is the first to be created remember the chain+point that this connector is starting from
                    if (connector_start_chain_index == std::numeric_limits<unsigned>::max())
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