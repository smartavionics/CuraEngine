//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtruderTrain.h"
#include "Slice.h"
#include "utils/logoutput.h"

#include <algorithm>

namespace cura
{

Slice::Slice(const size_t num_mesh_groups)
: scene(num_mesh_groups)
{}

void Slice::compute()
{
    logWarning("%s", scene.getAllSettingsString().c_str());

    std::vector<std::pair<int,std::vector<MeshGroup>::iterator>> prioritised_meshes;

    int max_priority = 0;
    for (std::vector<MeshGroup>::iterator mesh_group = scene.mesh_groups.begin(); mesh_group != scene.mesh_groups.end(); mesh_group++)
    {
        int priority = 0;
        for (Mesh& mesh : mesh_group->meshes)
        {
            int pri = mesh.settings.get<int>("mesh_priority");
            if (pri > priority)
            {
                priority = pri;
            }
        }
        prioritised_meshes.push_back(std::make_pair(priority, mesh_group));
        if (priority > max_priority)
        {
            max_priority = priority;
        }
    }

    if (max_priority > 0)
    {
        auto sort_by_increasing_priority = [] (std::pair<int,std::vector<MeshGroup>::iterator>& a, std::pair<int,std::vector<MeshGroup>::iterator>& b)
        {
            return a.first > b.first;
        };
        std::sort(prioritised_meshes.begin(), prioritised_meshes.end(), sort_by_increasing_priority);
    }

    for (unsigned n = 0; n < prioritised_meshes.size(); ++n)
    {
        std::vector<MeshGroup>::iterator mesh_group = prioritised_meshes[n].second;
        scene.current_mesh_group = mesh_group;
        for (ExtruderTrain& extruder : scene.extruders)
        {
            extruder.settings.setParent(&scene.current_mesh_group->settings);
        }
        scene.processMeshGroup(*mesh_group);
    }
}

void Slice::reset()
{
    scene.extruders.clear();
    scene.mesh_groups.clear();
    scene.settings = Settings();
}

}