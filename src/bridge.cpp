//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge.h"
#include "sliceDataStorage.h"
#include "settings/types/Ratio.h"
#include "utils/AABB.h"
#include "utils/polygon.h"

namespace cura
{

int bridgeAngle(const Settings& settings, const Polygons& skin_outline, const SliceDataStorage& storage, const unsigned layer_nr)
{
    AABB boundary_box(skin_outline);

    //To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    // This gives us the islands that the layer rests on.
    Polygons islands;

    Polygons prev_layer_outline; // we also want the complete outline of the previous layer

    const Ratio sparse_infill_max_density = settings.get<Ratio>("bridge_sparse_infill_max_density");

    // include parts from all meshes
    for (const SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.isPrinted())
        {
            const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
            const coord_t infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
            const bool part_has_sparse_infill = (infill_line_distance == 0) || ((float)infill_line_width / infill_line_distance) <= sparse_infill_max_density;

            for (const SliceLayerPart& prev_layer_part : mesh.layers[layer_nr - 1].parts)
            {
                Polygons solid_below(prev_layer_part.outline);
                if (part_has_sparse_infill)
                {
                    solid_below = solid_below.difference(prev_layer_part.getOwnInfillArea());
                }
                prev_layer_outline.add(solid_below); // not intersected with skin

                if (!boundary_box.hit(prev_layer_part.boundaryBox))
                    continue;

                islands.add(skin_outline.intersection(solid_below));
            }
        }
    }

    if (islands.size() > 5 || islands.size() < 1)
    {
        return -1;
    }

    //Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    int idx1 = -1;
    int idx2 = -1;
    for(unsigned int n=0; n<islands.size(); n++)
    {
        //Skip internal holes
        if (!islands[n].orientation())
            continue;
        double area = fabs(islands[n].area());
        if (area > area1)
        {
            if (area1 > area2)
            {
                area2 = area1;
                idx2 = idx1;
            }
            area1 = area;
            idx1 = n;
        }
        else if (area > area2)
        {
            area2 = area;
            idx2 = n;
        }
    }
    
    if (idx1 < 0 || idx2 < 0)
        return -1;
    
    Point center1 = islands[idx1].centerOfMass();
    Point center2 = islands[idx2].centerOfMass();

    return angle(center2 - center1);
}

}//namespace cura

