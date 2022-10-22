//Copyright (c) 2022 Mark Burton
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../sliceDataStorage.h"

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/Angle.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>

namespace cura
{

class Polygons;

class DiscreteLinesInfill
{
public:
    DiscreteLinesInfill(const coord_t z, const Point& infill_origin, const AngleDegrees fill_angle, const coord_t infill_line_width, const SliceMeshStorage* mesh);

    ~DiscreteLinesInfill();

    void generate(Polygons& result_lines, const Polygons& outline);
    
protected:
    const coord_t z;                   //!< height of the current layer
    const Point& infill_origin;        //!< point the infill is rotated around
    const double fill_angle_rads;      //!< infill rotation angle
    const coord_t infill_line_width;   //!< width of infill lines
    const SliceMeshStorage* mesh;      //!< mesh being filled
    const coord_t min_line_len2 = 100; //!< minimum squared length of generated lines, don't output any shorter than 10um
    rapidjson::Document* json_document;

    Point rotate_around(const Point& point, const Point& origin, const double rads)
    {
        return (rads != 0) ? origin + rotate(point - origin, rads) : point;
    }

    Point rotate_around_origin(const Point& point, const double rads)
    {
        return rotate_around(point, infill_origin, rads);
    }

private:
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part of

    Polygons clipped_outline; // next infill pattern (including connecting wall lines if enabled) is clipped to this outline
    Polygons connections_outline; // if connections are enabled, the connection line segments follow this outline

    void generateCoordinates(Polygons& result, const Polygons& outline, rapidjson::Value* one_def);

    void generateConnections(Polygons& result, const Polygons& outline);

};

} // namespace cura

