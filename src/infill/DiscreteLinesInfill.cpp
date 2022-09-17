//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "DiscreteLinesInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/logoutput.h"
#include "../utils/polygon.h"

#include <fstream>
#include <rapidjson/error/en.h> //Loading JSON documents to get settings from them.
#include <rapidjson/filereadstream.h>
#include <rapidjson/memorystream.h>

namespace cura {

static std::map<std::string, rapidjson::Document*> definitions;

DiscreteLinesInfill::DiscreteLinesInfill(const coord_t z, const Point& infill_origin, const coord_t infill_line_width, const SliceMeshStorage* mesh)
    : z(z)
    , infill_origin(infill_origin)
    , infill_line_width(infill_line_width)
    , mesh(mesh)
{
    if (!mesh)
    {
        if (definitions.find("no mesh") == definitions.end())
        {
            logError("DiscreteLinesInfill: Cannot work without a mesh\n");
            definitions["no mesh"] = nullptr;
        }
        return;
    }
    const std::string definition = mesh->settings.get<std::string>("discrete_lines_infill_definition");
    const std::size_t start = definition.find_first_not_of(" \t\r\n");
    if (start != std::string::npos)
    {
        if (definitions.find(definition) != definitions.end())
        {
            json_document = definitions.at(definition);
        }
        else
        {
            json_document = new rapidjson::Document();
            definitions[definition] = json_document;

            if (definition.at(start) == '@')
            {
                const char *json_filename = definition.c_str() + start + 1;
                while (isspace(*json_filename))
                {
                    ++json_filename;
                }

#if defined(__linux__) || (defined(__APPLE__) && defined(__MACH__))
                const char *slash = "/";
#else
                const char *slash = "\\";
#endif
                std::string pathname(json_filename);

                FILE* file = fopen(pathname.c_str(), "rb");
                if (!file)
                {
                    std::string dir = mesh->settings.get<std::string>("project_file_dir");
                    if (dir.size() > 0)
                    {
                        pathname = dir + slash + json_filename;
                        file = fopen(pathname.c_str(), "rb");
                    }
                }

                if (!file)
                {
                    std::string dir = mesh->settings.get<std::string>("home_dir");
                    if (dir.size() > 0)
                    {
                        pathname = dir + slash + json_filename;
                        file = fopen(pathname.c_str(), "rb");
                    }
                }

                if (!file)
                {
                    logError("DiscreteLinesInfill: Couldn't open JSON file: %s\n", json_filename);
                    return;
                }
                logAlways("DiscreteLinesInfill: Opened JSON file: %s\n", pathname.c_str());
                char read_buffer[4096];
                rapidjson::FileReadStream reader_stream(file, read_buffer, sizeof(read_buffer));
                json_document->ParseStream(reader_stream);
                fclose(file);
            }
            else
            {
                rapidjson::MemoryStream string_stream(definition.c_str(), strlen(definition.c_str()));
                json_document->ParseStream(string_stream);
            }
            if (json_document->HasParseError())
            {
                logError("DiscreteLinesInfill: Error parsing JSON (offset %u): %s\n", static_cast<unsigned int>(json_document->GetErrorOffset()), GetParseError_En(json_document->GetParseError()));
                return;
            }
            if (json_document->IsArray())
            {
                for (rapidjson::Value::ValueIterator def_iter = json_document->Begin(); def_iter != json_document->End(); def_iter++)
                {
                    if (!def_iter->IsObject())
                    {
                        logError("DiscreteLinesInfill: JSON definition must be a single object or an array of objects");
                        return;
                    }
                }
            }
            else if (!json_document->IsObject())
            {
                logError("DiscreteLinesInfill: JSON definition must be a single object or an array of objects");
            }
        }
    }
    else
    {
        logError("Empty Discrete Lines Infill Definition");
        json_document = new rapidjson::Document();
        definitions[definition] = json_document;
    }
}

DiscreteLinesInfill::~DiscreteLinesInfill() {
}

Point DiscreteLinesInfill::rotate_around_origin(const Point& point, const double rads)
{
    return (rads != 0) ? infill_origin + rotate(point - infill_origin, rads) : point;
}

void DiscreteLinesInfill::generate(Polygons& result_lines, const Polygons& outline)
{
    Polygons clipped_outline(outline);

    if (json_document->IsArray())
    {
        for (rapidjson::Value::ValueIterator def_iter = json_document->Begin(); def_iter != json_document->End(); def_iter++)
        {
            generateCoordinates(result_lines, outline, def_iter, clipped_outline);
        }
    }
    else
    {
        generateCoordinates(result_lines, outline, json_document, clipped_outline);
    }

    generateConnections(result_lines, outline);
}

void DiscreteLinesInfill::generateCoordinates(Polygons& result, const Polygons& outline, rapidjson::Value* one_def, Polygons& clipped_outline)
{
    rapidjson::Value::MemberIterator mi;

    mi = one_def->FindMember("enable");
    if (mi != one_def->MemberEnd() && !mi->value.GetBool())
    {
        // "enable" is false so no point in doing anything else
        return;
    }

    const size_t bottom_layers = mesh->settings.get<size_t>("initial_bottom_layers");
    coord_t bottom_skin_depth = 0;
    if (bottom_layers > 0)
    {
        bottom_skin_depth += mesh->settings.get<coord_t>("layer_height_0");
        if (bottom_layers > 1)
        {
            bottom_skin_depth += mesh->settings.get<coord_t>("layer_height") * (bottom_layers - 1);
        }
    }
    const coord_t top_skin_depth = mesh->settings.get<coord_t>("layer_height") * mesh->settings.get<size_t>("top_layers");

    coord_t bottom = mesh->bounding_box.min.z + bottom_skin_depth;
    coord_t top = mesh->bounding_box.max.z - top_skin_depth + 10;

    std::vector<coord_t> x_vals;
    std::vector<coord_t> y_vals;

    auto interpolateValue = [&](rapidjson::Value& value) {
        double result = 0;
        if (value.IsArray())
        {
            if (value.Size() > 0)
            {
                result = value[0].GetDouble();
                if (value.Size() > 1)
                {
                    double valt = value[1].GetDouble();
                    result += (valt - result) * (z - bottom) / (top - bottom);
                }
            }
        }
        else if(value.IsNumber())
        {
            result = value.GetDouble();
        }
        return result;
    };

    mi = one_def->FindMember("zmin");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        coord_t zmin = MM2INT(val);
        if (z < zmin)
        {
            return;
        }
        if (zmin > bottom)
        {
            bottom = zmin;
        }
    }

    mi = one_def->FindMember("zmax");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        coord_t zmax = MM2INT(val);
        if (z >= zmax)
        {
            return;
        }
        if (zmax < top)
        {
            top = zmax;
        }
    }

    double rot_rads = 0;
    mi = one_def->FindMember("angle");
    if (mi != one_def->MemberEnd())
    {
        rot_rads = mi->value.GetDouble() / (180 / M_PI);
    }

    Polygons rotated_outline = outline;
    if (rot_rads != 0)
    {
        for (PolygonRef poly : rotated_outline)
        {
            for (Point& point : poly)
            {
                point = rotate_around_origin(point, -rot_rads);
            }
        }
    }
    const AABB aabb(rotated_outline);

    coord_t clip_x_min = aabb.min.X - 1;
    coord_t clip_x_max = aabb.max.X + 1;
    coord_t clip_y_min = aabb.min.Y - 1;
    coord_t clip_y_max = aabb.max.Y + 1;

    mi = one_def->FindMember("xmin");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        clip_x_min = infill_origin.X + MM2INT(val);
    }

    mi = one_def->FindMember("xmax");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        clip_x_max = infill_origin.X + MM2INT(val);
    }

    mi = one_def->FindMember("ymin");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        clip_y_min = infill_origin.Y + MM2INT(val);
    }

    mi = one_def->FindMember("ymax");
    if (mi != one_def->MemberEnd())
    {
        double val = mi->value.GetDouble();
        clip_y_max = infill_origin.Y + MM2INT(val);
    }

    mi = one_def->FindMember("xpitch");
    if (mi != one_def->MemberEnd())
    {
        double val = interpolateValue(mi->value);
        coord_t xpitch = MM2INT(val);

        if (xpitch > 0)
        {
            coord_t x_min = infill_origin.X + std::ceil((float)(infill_origin.X - aabb.min.X) / xpitch + 1) * -xpitch;
            coord_t x_max = infill_origin.X + std::ceil((float)(aabb.max.X - infill_origin.X) / xpitch + 1) * xpitch;

            for (coord_t x = x_min; x < x_max; x += xpitch)
            {
                if (x >= clip_x_min && x <= clip_x_max)
                {
                    x_vals.push_back(x);
                }
            }
        }
    }

    mi = one_def->FindMember("x");
    if (mi != one_def->MemberEnd())
    {
        rapidjson::Value& x_array = mi->value;

        if (x_array.IsArray())
        {
            for (rapidjson::Value::ConstValueIterator x_iter = x_array.Begin(); x_iter != x_array.End(); x_iter++)
            {
                double x = x_iter->GetDouble();
                x = infill_origin.X + MM2INT(x);
                if (x >= clip_x_min && x <= clip_x_max)
                {
                    x_vals.push_back(x);
                }
            }
        }
    }

    mi = one_def->FindMember("ypitch");
    if (mi != one_def->MemberEnd())
    {
        double val = interpolateValue(mi->value);
        coord_t ypitch = MM2INT(val);

        if (ypitch > 0)
        {
            coord_t y_min = infill_origin.Y + std::ceil((float)(infill_origin.Y - aabb.min.Y) / ypitch + 1) * -ypitch;
            coord_t y_max = infill_origin.Y + std::ceil((float)(aabb.max.Y - infill_origin.Y) / ypitch + 1) * ypitch;

            for (coord_t y = y_min; y < y_max; y += ypitch)
            {
                if (y >= clip_y_min && y <= clip_y_max)
                {
                    y_vals.push_back(y);
                }
            }
        }
    }

    mi = one_def->FindMember("y");
    if (mi != one_def->MemberEnd())
    {
        rapidjson::Value& y_array = mi->value;

        if (y_array.IsArray())
        {
            for (rapidjson::Value::ConstValueIterator y_iter = y_array.Begin(); y_iter != y_array.End(); y_iter++)
            {
                double y = y_iter->GetDouble();
                y = infill_origin.Y + MM2INT(y);
                if (y >= clip_y_min && y <= clip_y_max)
                {
                    y_vals.push_back(y);
                }
            }
        }
    }

    mi = one_def->FindMember("zigzag");
    bool zig_zaggify = (mi != one_def->MemberEnd() && mi->value.GetBool());

    unsigned num_lines = 0;
    unsigned chain_end_index = 0;
    Point chain_end[2];

    auto addClippedLine = [&](const Point& p0, const Point& p1, unsigned line_index)
    {
        Polygons lines;
        lines.addLine(p0, p1);
        for (ConstPolygonRef line_seg : clipped_outline.intersectionPolyLines(lines))
        {
            // some of the line is inside the clipped outline, add it if it's not too small
            if (vSize2(line_seg[0] - line_seg[1]) >= min_line_len2)
            {
                result.addLine(line_seg[0], line_seg[1]);

                if (zig_zaggify)
                {
                    for (const Point& pt : line_seg)
                    {
                        chain_end[chain_end_index] = pt;
                        if (++chain_end_index == 2)
                        {
                            chains[0].push_back(chain_end[0]);
                            chains[1].push_back(chain_end[1]);
                            chain_end_index = 0;
                            connected_to[0].push_back(std::numeric_limits<unsigned>::max());
                            connected_to[1].push_back(std::numeric_limits<unsigned>::max());
                            line_numbers.push_back(line_index);
                        }
                    }
                }
            }
        }
    };

    coord_t wavelength = 0;
    mi = one_def->FindMember("wavelength");
    if (mi != one_def->MemberEnd())
    {
        double val = interpolateValue(mi->value);
        wavelength = MM2INT(val);
    }
    coord_t amplitude = 0;
    mi = one_def->FindMember("amplitude");
    if (mi != one_def->MemberEnd())
    {
        double val = interpolateValue(mi->value);
        amplitude = MM2INT(val);
    }

    auto genWaveform = [&](std::vector<double>& amplitudes, std::vector<double>& phases)
    {
        int num_segs = amplitudes.size();
        if (num_segs < 1)
        {
            return;
        }
        Polygons shrunk_outline(clipped_outline.offset(-std::max(wavelength / 2, amplitude)));

        coord_t y_min = infill_origin.Y + std::ceil((float)(infill_origin.Y - aabb.min.Y) / wavelength + 1) * -wavelength;
        coord_t y_max = infill_origin.Y + std::ceil((float)(aabb.max.Y - infill_origin.Y) / wavelength + 1) * wavelength;

        for (coord_t x : x_vals)
        {
            bool line_start_inside = false;
            for (coord_t y = y_min; y < y_max; y += wavelength)
            {
                Point line_start = rotate_around_origin(Point(x + amplitudes.back() * amplitude, y), rot_rads);
                //bool line_start_inside = shrunk_outline.inside(line_start, true);
                for (int seg = 1; seg <= num_segs; ++seg)
                {
                    Point line_end = rotate_around_origin(Point(x + amplitudes[seg - 1] * amplitude, y + wavelength * phases[seg - 1]), rot_rads);
                    bool line_end_inside = shrunk_outline.inside(line_end, true);
                    if (line_start_inside && line_end_inside)
                    {
                        result.addLine(line_start, line_end);
                    }
                    else
                    {
                        addClippedLine(line_start, line_end, num_lines);
                    }
                    line_start_inside = line_end_inside;
                    line_start = line_end;
                }
            }
            ++num_lines;
        }

        coord_t x_min = infill_origin.X + std::ceil((float)(infill_origin.X - aabb.min.X) / wavelength + 1) * -wavelength;
        coord_t x_max = infill_origin.X + std::ceil((float)(aabb.max.X - infill_origin.X) / wavelength + 1) * wavelength;

        for (coord_t y : y_vals)
        {
            for (coord_t x = x_min; x < x_max; x += wavelength)
            {
                Point line_start = rotate_around_origin(Point(x, y + amplitudes.back() * amplitude), rot_rads);
                bool line_start_inside = shrunk_outline.inside(line_start, true);
                for (int seg = 1; seg <= num_segs; ++seg)
                {
                    Point line_end = rotate_around_origin(Point(x + wavelength * phases[seg - 1], y + amplitudes[seg - 1] * amplitude), rot_rads);
                    bool line_end_inside = shrunk_outline.inside(line_end, true);
                    if (line_start_inside && line_end_inside)
                    {
                        result.addLine(line_start, line_end);
                    }
                    else
                    {
                        addClippedLine(line_start, line_end, num_lines);
                    }
                    line_start_inside = line_end_inside;
                    line_start = line_end;
                }
            }
            ++num_lines;
        }
    };

    mi = one_def->FindMember("waveform");
    if (mi != one_def->MemberEnd() && amplitude && wavelength)
    {
        if (mi->value.IsString())
        {
            if (mi->value.GetString() == std::string("triangle"))
            {
                std::vector<double> amplitudes{-1.0, 1.0};
                std::vector<double> phases{0.5, 1.0};
                genWaveform(amplitudes, phases);
            }
            if (mi->value.GetString() == std::string("square"))
            {
                std::vector<double> amplitudes{-1.0, -1.0, 1.0, 1.0};
                std::vector<double> phases{0.0, 0.5, 0.5, 1.0};
                genWaveform(amplitudes, phases);
            }
            else if (mi->value.GetString() == std::string("sine"))
            {
                int num_segs = 16;
                while (num_segs > 4 && amplitude / num_segs < 100)
                {
                    num_segs /= 2;
                }

                std::vector<double> amplitudes;
                std::vector<double> phases;
                for (int seg = 1; seg <= num_segs; ++seg)
                {
                    amplitudes.push_back(std::sin(M_PI * 2 * seg / num_segs + M_PI/2));
                    phases.push_back((double)seg / num_segs);
                }
                genWaveform(amplitudes, phases);
            }
        }
        else if (mi->value.IsArray())
        {
            std::vector<double> amplitudes;
            std::vector<double> phases;
            // changes in amplitude that occur with zero rise/fall time are prefixed by an array element that is a string, e.g. ""
            int njumps = 0;
            for (rapidjson::Value::ConstValueIterator iter = mi->value.Begin(); iter != mi->value.End(); iter++)
            {
                if (iter->IsNumber())
                {
                    amplitudes.push_back(iter->GetDouble());
                }
                else if (iter->IsString())
                {
                    ++njumps;
                }
            }
            int nperiods = amplitudes.size() - njumps;
            if (nperiods > 0)
            {
                int i = 0;
                for (rapidjson::Value::ConstValueIterator iter = mi->value.Begin(); iter != mi->value.End(); iter++)
                {
                    if (iter->IsNumber())
                    {
                        phases.push_back((double)i / nperiods);
                        ++i;
                    }
                    else if (iter->IsString() && i > 0)
                    {
                        --i;
                    }
                }
            }
            // 3 consecutive occurences of the same amplitude can be shortened by removing the middle value
            for (unsigned i = 2; i < amplitudes.size();)
            {
                if (amplitudes[i] == amplitudes[i - 1] && amplitudes[i] == amplitudes[i - 2])
                {
                    amplitudes.erase(amplitudes.begin() + i - 1);
                    phases.erase(phases.begin() + i - 1);
                }
                else
                {
                    ++i;
                }
            }
            genWaveform(amplitudes, phases);
        }
    }
    else
    {
        // straight lines
        for (coord_t x : x_vals)
        {
            Point line_start = rotate_around_origin(Point(x, clip_y_min), rot_rads);
            Point line_end = rotate_around_origin(Point(x, clip_y_max), rot_rads);
            addClippedLine(line_start, line_end, num_lines++);
        }

        for (coord_t y : y_vals)
        {
            Point line_start = rotate_around_origin(Point(clip_x_min, y), rot_rads);
            Point line_end = rotate_around_origin(Point(clip_x_max, y), rot_rads);
            addClippedLine(line_start, line_end, num_lines++);
        }
    }

    if (!x_vals.size() && !y_vals.size())
    {
        return;
    }

    mi = one_def->FindMember("clip");
    if (mi == one_def->MemberEnd() || mi->value.GetBool())
    {
        Polygon infilled_area;
        infilled_area.add(Point(clip_x_min, clip_y_max));
        infilled_area.add(Point(clip_x_max, clip_y_max));
        infilled_area.add(Point(clip_x_max, clip_y_min));
        infilled_area.add(Point(clip_x_min, clip_y_min));
        Polygons infilled_areas;
        infilled_areas.add(infilled_area);
        clipped_outline = clipped_outline.difference(infilled_areas);
    }
}

void DiscreteLinesInfill::generateConnections(Polygons& result, const Polygons& outline)
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