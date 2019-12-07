//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/AngleDegrees.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

namespace cura
{

class Polygons;

class TPMSInfill
{
public:
    TPMSInfill();

    ~TPMSInfill();

    void generate(Polygons& result_lines, const bool zig_zaggify, const coord_t outline_offset, const coord_t infill_line_width, const coord_t line_distance, const Polygons& in_outline, const coord_t z, const EFillMethod pattern, const EFillResolution resolution, const Point& infill_origin, const AngleDegrees fill_angle);
    
private:

};

} // namespace cura

