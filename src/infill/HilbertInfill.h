//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/Angle.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

namespace cura
{

class Polygons;

class HilbertInfill
{
public:
    HilbertInfill(const bool zig_zaggify, const coord_t line_distance, const Point& infill_origin, const AngleDegrees fill_angle)
    : zig_zaggify(zig_zaggify)
    , line_distance(line_distance)
    , infill_origin(infill_origin)
    , fill_angle_rads(fill_angle / (180 / M_PI))
{
}

    void generate(Polygons& result_lines, const Polygons& outline, const coord_t mesh_max_size);
    
protected:
    const bool zig_zaggify;            //!< true if infill lines are to be connected where they meet the infill area walls
    const coord_t line_distance;       //!< distance between infill lines
    const Point& infill_origin;        //!< point the infill is rotated around
    const double fill_angle_rads;      //!< infill rotation angle

    coord_t x_min; //!< min X coordinate of generated infill
    coord_t x_max; //!< max X coordinate of generated infill
    coord_t y_min; //!< min Y coordinate of generated infill
    coord_t y_max; //!< max Y coordinate of generated infill

    Point rotate_around_origin(const Point& point, const double rads);

private:
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]

    void generateCoordinates(Polygons& result, const Polygons& outline, const Polygons& shrunk_outline, const coord_t size, const int depth);

    void generateConnections(Polygons& result, const Polygons& outline);
};

} // namespace cura

