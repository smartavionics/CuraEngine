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

class TPMSInfill
{
public:
    TPMSInfill(const bool zig_zaggify, const coord_t line_distance, const coord_t z, const EFillResolution resolution, const Point& infill_origin, const AngleDegrees fill_angle, const SliceMeshStorage* mesh);

    ~TPMSInfill();

    void generate(Polygons& result_lines, const Polygons& outline);
    
protected:
    const bool zig_zaggify;            //!< true if infill lines are to be connected where they meet the infill area walls
    const coord_t line_distance;       //!< distance between infill lines (pitch of TPMS curves is calculated from this to yield infill with similar material amount)
    const coord_t z;                   //!< height of the current layer
    const EFillResolution resolution;  //!< infill resolution to use
    const Point& infill_origin;        //!< point the infill is rotated around
    const double fill_angle_rads;      //!< infill rotation angle
    const SliceMeshStorage* mesh;      //!< mesh being filled
    const coord_t min_line_len2 = 100; //!< minimum squared length of generated lines, don't output any shorter than 10um

    coord_t x_min; //!< min X coordinate of generated infill
    coord_t x_max; //!< max X coordinate of generated infill
    coord_t y_min; //!< min Y coordinate of generated infill
    coord_t y_max; //!< max Y coordinate of generated infill

    Point rotate_around_origin(const Point& point, const double rads);

private:

    virtual void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const double step) = 0;

    virtual void generateConnections(Polygons& result, const Polygons& outline) = 0;

    virtual double pitchScaling() = 0;

    virtual coord_t constrainPitch(coord_t pitch) {
        return pitch;
    }
};

class TPMSInfillGyroid : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part of

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const double step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 2.41; }

    coord_t constrainPitch(coord_t pitch) {
        if (mesh && mesh->settings.get<bool>("infill_constrain_gyroid_pitch") && !mesh->settings.get<bool>("adaptive_layer_height_enabled"))
        {
            const coord_t layer_height = mesh->settings.get<coord_t>("layer_height");
            pitch = pitch / (4 * layer_height) * (4 * layer_height);
        }
        return pitch;
    }
};

class TPMSInfillSchwarzP : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    std::vector<Point> connection_points;
    std::vector<unsigned> connection_ids;

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const double step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 1.94; }
};

class TPMSInfillSchwarzD : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    typedef int ConnectionId;

    std::vector<Point> connection_points;
    std::vector<ConnectionId> connection_ids;

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const double step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 1.5; }
};

} // namespace cura

