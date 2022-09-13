//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../sliceDataStorage.h"

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/Angle.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

namespace cura
{

class Polygons;

class DiscreteLinesInfill
{
public:
    DiscreteLinesInfill(const bool zig_zaggify, const coord_t z, const Point& infill_origin, const AngleDegrees fill_angle, const coord_t infill_line_width, const SliceMeshStorage* mesh);

    ~DiscreteLinesInfill();

    void generate(Polygons& result_lines, const Polygons& outline);
    
protected:
    const bool zig_zaggify;            //!< true if infill lines are to be connected where they meet the infill area walls
    const coord_t z;                   //!< height of the current layer
    const Point& infill_origin;        //!< point the infill is rotated around
    const double fill_angle_rads;      //!< infill rotation angle
    const coord_t min_line_len2 = 100; //!< minimum squared length of generated lines, don't output any shorter than 10um
    const coord_t infill_line_width;   //!< width of infill lines
    const SliceMeshStorage* mesh;      //!< mesh being filled

    coord_t x_min; //!< min X coordinate of generated infill
    coord_t x_max; //!< max X coordinate of generated infill
    coord_t y_min; //!< min Y coordinate of generated infill
    coord_t y_max; //!< max Y coordinate of generated infill

    Point rotate_around_origin(const Point& point, const double rads);

private:
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part of

    void generateCoordinates(Polygons& result, const Polygons& outline, const coord_t pitch, const coord_t height);

    void generateConnections(Polygons& result, const Polygons& outline);

};

} // namespace cura

