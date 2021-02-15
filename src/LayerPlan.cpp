//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cstring>

#include "Application.h" //To communicate layer view data.
#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "MergeInfillLines.h"
#include "raft.h" // getTotalExtraLayers
#include "Slice.h"
#include "sliceDataStorage.h"
#include "wallOverlap.h"
#include "communication/Communication.h"
#include "pathPlanning/Comb.h"
#include "pathPlanning/CombPaths.h"
#include "settings/types/Ratio.h"
#include "utils/logoutput.h"
#include "utils/polygonUtils.h"
#include "WipeScriptConfig.h"

namespace cura {

constexpr int MINIMUM_LINE_LENGTH = 5; // in uM. Generated lines shorter than this may be discarded
constexpr int MINIMUM_SQUARED_LINE_LENGTH = MINIMUM_LINE_LENGTH * MINIMUM_LINE_LENGTH;

ExtruderPlan::ExtruderPlan(const size_t extruder, const LayerIndex layer_nr, const bool is_initial_layer, const bool is_raft_layer, const coord_t layer_thickness, const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config)
: heated_pre_travel_time(0)
, required_start_temperature(-1)
, extruder_nr(extruder)
, layer_nr(layer_nr)
, is_initial_layer(is_initial_layer)
, is_raft_layer(is_raft_layer)
, layer_thickness(layer_thickness)
, fan_speed_layer_time_settings(fan_speed_layer_time_settings)
, retraction_config(retraction_config)
, extrudeSpeedFactor(1.0)
, extraTime(0.0)
, totalPrintTime(0)
{
}

void ExtruderPlan::setExtrudeSpeedFactor(const Ratio speed_factor)
{
    extrudeSpeedFactor = speed_factor;
}

double ExtruderPlan::getExtrudeSpeedFactor()
{
    return extrudeSpeedFactor;
}

void ExtruderPlan::setFanSpeed(double _fan_speed)
{
    fan_speed = _fan_speed;
}
double ExtruderPlan::getFanSpeed()
{
    return fan_speed;
}

double ExtruderPlan::getMaterial(std::vector<double>* amounts) const
{
    double total = 0;

    if (amounts)
    {
        amounts->assign((unsigned)PrintFeatureType::NumPrintFeatureTypes, 0.0);
    }

    for (const GCodePath& path : paths)
    {
        double amount = path.estimates.getMaterial();
        total +=  amount;
        if (amounts)
        {
            unsigned feature_type = (unsigned)path.config->type;
            if (feature_type < (unsigned)PrintFeatureType::NumPrintFeatureTypes)
            {
                (*amounts)[feature_type] += amount;
            }
        }
    }

    return total;
}

GCodePath* LayerPlan::getLatestPathWithConfig(const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio flow, bool spiralize, const Ratio speed_factor, const double fan_speed)
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0 && paths.back().config == &config && !paths.back().done && paths.back().flow == flow && paths.back().speed_factor == speed_factor && paths.back().mesh_id == current_mesh && paths.back().fan_speed == fan_speed) // spiralize can only change when a travel path is in between
    {
        return &paths.back();
    }
    paths.emplace_back(config, current_mesh, space_fill_type, flow, spiralize, speed_factor, fan_speed);
    GCodePath* ret = &paths.back();
    ret->skip_agressive_merge_hint = mode_skip_agressive_merge;
    return ret;
}

void LayerPlan::forceNewPathStart()
{
    std::vector<GCodePath>& paths = extruder_plans.back().paths;
    if (paths.size() > 0)
        paths[paths.size()-1].done = true;
}

LayerPlan::LayerPlan(const SliceDataStorage& storage, LayerIndex layer_nr, coord_t z, coord_t layer_thickness, size_t start_extruder, const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, coord_t comb_boundary_offset, coord_t comb_move_inside_distance, coord_t travel_avoid_distance)
: storage(storage)
, configs_storage(storage, layer_nr, layer_thickness)
, z(z)
, final_travel_z(z)
, mode_skip_agressive_merge(false)
, layer_nr(layer_nr)
, is_initial_layer(layer_nr == 0 - static_cast<LayerIndex>(Raft::getTotalExtraLayers()))
, is_raft_layer(layer_nr < 0 - static_cast<LayerIndex>(Raft::getFillerLayerCount()))
, layer_thickness(layer_thickness)
, has_prime_tower_planned_per_extruder(Application::getInstance().current_slice->scene.extruders.size(), false)
, current_mesh("NONMESH")
, max_path_time(0)
, last_extruder_previous_layer(start_extruder)
, last_planned_extruder(&Application::getInstance().current_slice->scene.extruders[start_extruder])
, first_travel_destination_is_inside(false) // set properly when addTravel is called for the first time (otherwise not set properly)
, comb_boundary_inside1(computeCombBoundaryInside(1))
, comb_boundary_inside2(computeCombBoundaryInside(2))
, comb_move_inside_distance(comb_move_inside_distance)
, fan_speed_layer_time_settings_per_extruder(fan_speed_layer_time_settings_per_extruder)
{
    size_t current_extruder = start_extruder;
    was_inside = true; // not used, because the first travel move is bogus
    is_inside = false; // assumes the next move will not be to inside a layer part (overwritten just before going into a layer part)
    if (Application::getInstance().current_slice->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing") != CombingMode::OFF)
    {
        comb = new Comb(storage, layer_nr, comb_boundary_inside1, comb_boundary_inside2, comb_boundary_offset, travel_avoid_distance, comb_move_inside_distance);
    }
    else
    {
        comb = nullptr;
    }
    for (const ExtruderTrain& extruder : Application::getInstance().current_slice->scene.extruders)
    {
        layer_start_pos_per_extruder.emplace_back(extruder.settings.get<coord_t>("layer_start_x"), extruder.settings.get<coord_t>("layer_start_y"));
    }
    extruder_plans.reserve(Application::getInstance().current_slice->scene.extruders.size());
    extruder_plans.emplace_back(current_extruder, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[current_extruder], storage.retraction_config_per_extruder[current_extruder]);

    for (size_t extruder_nr = 0; extruder_nr < Application::getInstance().current_slice->scene.extruders.size(); extruder_nr++)
    { //Skirt and brim.
        skirt_brim_is_processed[extruder_nr] = false;
    }
}

LayerPlan::~LayerPlan()
{
    if (comb)
        delete comb;
}

ExtruderTrain* LayerPlan::getLastPlannedExtruderTrain()
{
    return last_planned_extruder;
}


Polygons LayerPlan::computeCombBoundaryInside(const size_t max_inset)
{
    const CombingMode combing_mode = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<CombingMode>("retraction_combing");
    if (combing_mode == CombingMode::OFF)
    {
        return Polygons();
    }
    if (layer_nr < 0)
    { // when a raft is present
        if (combing_mode == CombingMode::NO_SKIN)
        {
            return Polygons();
        }
        else
        {
            return storage.raftOutline.offset(MM2INT(0.1));
        }
    }
    else 
    {
        Polygons comb_boundary;
        for (const SliceMeshStorage& mesh : storage.meshes)
        {
            const SliceLayer& layer = mesh.layers[layer_nr];
            if (mesh.settings.get<bool>("infill_mesh")) {
                continue;
            }
            const coord_t skin_avoid_distance = mesh.settings.get<coord_t>("machine_nozzle_tip_outer_diameter") * 0.75;
            const CombingMode combing_mode = mesh.settings.get<CombingMode>("retraction_combing");
            if (combing_mode == CombingMode::NO_SKIN)
            {
                // we need to include the walls in the comb boundary otherwise it's not possible to tell if a travel move crosses a skin region

                const coord_t line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0");

                for (const SliceLayerPart& part : layer.parts)
                {
                    const size_t num_insets = part.insets.size();
                    Polygons outer = part.outline; // outer boundary of wall combing region
                    coord_t outer_to_outline_dist = 0; // distance from outer to the part's outline

                    if (num_insets > 1 && part.insets[1].size() == part.outline.size())
                    {
                        // part's wall has multiple lines and the 2nd wall line is complete
                        // set the outer boundary to the inside edge of the outer wall

                        outer = part.insets[0].offset(-line_width_0/2);
                        outer_to_outline_dist = line_width_0;
                    }
                    else if (num_insets > 0)
                    {
                        // set the outer boundary to be 1/4 line width inside the centre line of the outer wall

                        outer = part.insets[0].offset(-line_width_0/4);
                        outer_to_outline_dist = line_width_0*3/4;
                    }

                    // finally, check that outer actually has the same number of polygons as the part's outline
                    // if it doesn't it means that the outer wall is missing where the part narrows so in those
                    // regions we need to use the part outline for the outer boundary of the combing region
                    // (expect poor results due to the nozzle being allowed to go right to the part's edge)

                    if (outer.size() != part.outline.size())
                    {
                        // first we calculate the part outline for those portions of the part where outer is missing
                        // this is done by shrinking the part outline so that it is very slightly smaller than outer, then expanding it again so it is very
                        // slightly larger than its original size and subtracting that from the original part outline
                        // NOTE - the additional small shrink/expands are required to ensure that the polygons overlap a little so we do not rely on exact results

                        Polygons outline_where_outer_is_missing(part.outline.difference(part.outline.offset(-(outer_to_outline_dist+5)).offset(outer_to_outline_dist+10)));

                        // merge outer with the portions of the part outline we just calculated
                        // the trick here is to expand the outlines sufficiently so that they overlap when unioned and then the result is shrunk back to the correct size

                        outer = outer.offset(outer_to_outline_dist/2+10).unionPolygons(outline_where_outer_is_missing.offset(outer_to_outline_dist/2+10)).offset(-(outer_to_outline_dist/2+10)).intersection(part.outline);
                    }

                    if (num_insets == 0)
                    {
                        comb_boundary.add(outer);
                    }
                    else
                    {
                        Polygons inner; // inner boundary of wall combing region

                        // the inside of the wall combing region is just inside the wall's inner edge so it can meet up with the infill (if any)

                        if (num_insets == 1)
                        {
                            inner = part.insets[0].offset(-10-line_width_0/2);
                        }
                        else
                        {
                            inner = part.insets[num_insets - 1].offset(-10 - mesh.settings.get<coord_t>("wall_line_width_x") / 2);
                        }

                        Polygons infill(part.infill_area);
                        Polygons skin_parts;
                        for (const SkinPart& skin_part : part.skin_parts)
                        {
                            skin_parts.add(skin_part.outline);
                        }
                        infill = infill.difference(skin_parts.unionPolygons().offset(skin_avoid_distance));

                        if (part.perimeter_gaps.size() > 0)
                        {
                            infill = infill.unionPolygons(part.perimeter_gaps.offset(10)); // ensure polygons overlap slightly
                        }

                        // combine the wall combing region (outer - inner) with the infill (if any)
                        comb_boundary.add(infill.unionPolygons(outer.difference(inner)));
                    }
                }
            }
            else if (combing_mode == CombingMode::INFILL)
            {
                for (const SliceLayerPart& part : layer.parts)
                {
                    Polygons skin_parts;
                    for (const SkinPart& skin_part : part.skin_parts)
                    {
                        skin_parts.add(skin_part.outline);
                    }
                    comb_boundary.add(part.infill_area.difference(skin_parts.unionPolygons().offset(skin_avoid_distance)));
                }
            }
            else
            {
                layer.getInnermostWalls(comb_boundary, max_inset, mesh);
            }
        }
        return comb_boundary;
    }
}

void LayerPlan::setIsInside(bool _is_inside)
{
    is_inside = _is_inside;
}

bool LayerPlan::setExtruder(const size_t extruder_nr)
{
    if (extruder_nr == getExtruder())
    {
        return false;
    }
    setIsInside(false);
    { // handle end position of the prev extruder
        ExtruderTrain* extruder = getLastPlannedExtruderTrain();
        const bool end_pos_absolute = extruder->settings.get<bool>("machine_extruder_end_pos_abs");
        Point end_pos(extruder->settings.get<coord_t>("machine_extruder_end_pos_x"), extruder->settings.get<coord_t>("machine_extruder_end_pos_y"));
        if (!end_pos_absolute)
        {
            end_pos += getLastPlannedPositionOrStartingPosition();
        }
        else 
        {
            const Point extruder_offset(extruder->settings.get<coord_t>("machine_nozzle_offset_x"), extruder->settings.get<coord_t>("machine_nozzle_offset_y"));
            end_pos += extruder_offset; // absolute end pos is given as a head position
        }
        if (end_pos_absolute || last_planned_position)
        {
            addTravel(end_pos); //  + extruder_offset cause it
        }
    }
    if (extruder_plans.back().paths.empty() && extruder_plans.back().inserts.empty())
    { // first extruder plan in a layer might be empty, cause it is made with the last extruder planned in the previous layer
        extruder_plans.back().extruder_nr = extruder_nr;
    }
    else 
    {
        extruder_plans.emplace_back(extruder_nr, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[extruder_nr], storage.retraction_config_per_extruder[extruder_nr]);
        assert(extruder_plans.size() <= Application::getInstance().current_slice->scene.extruders.size() && "Never use the same extruder twice on one layer!");
    }
    last_planned_extruder = &Application::getInstance().current_slice->scene.extruders[extruder_nr];

    { // handle starting pos of the new extruder
        ExtruderTrain* extruder = getLastPlannedExtruderTrain();
        const bool start_pos_absolute = extruder->settings.get<bool>("machine_extruder_start_pos_abs");
        Point start_pos(extruder->settings.get<coord_t>("machine_extruder_start_pos_x"), extruder->settings.get<coord_t>("machine_extruder_start_pos_y"));
        if (!start_pos_absolute)
        {
            start_pos += getLastPlannedPositionOrStartingPosition();
        }
        else 
        {
            Point extruder_offset(extruder->settings.get<coord_t>("machine_nozzle_offset_x"), extruder->settings.get<coord_t>("machine_nozzle_offset_y"));
            start_pos += extruder_offset; // absolute start pos is given as a head position
        }
        if (start_pos_absolute || last_planned_position)
        {
            last_planned_position = start_pos;
        }
    }
    return true;
}
void LayerPlan::setMesh(const std::string mesh_id)
{
    current_mesh = mesh_id;
}

void LayerPlan::setMaxPathTime(const double time)
{
    max_path_time = time;
}

void LayerPlan::moveInsideCombBoundary(const coord_t distance)
{
    constexpr coord_t max_dist2 = MM2INT(2.0) * MM2INT(2.0); // if we are further than this distance, we conclude we are not inside even though we thought we were.
    // this function is to be used to move from the boudary of a part to inside the part
    Point p = getLastPlannedPositionOrStartingPosition(); // copy, since we are going to move p
    if (PolygonUtils::moveInside(comb_boundary_inside2, p, distance, max_dist2) != NO_INDEX)
    {
        //Move inside again, so we move out of tight 90deg corners
        PolygonUtils::moveInside(comb_boundary_inside2, p, distance, max_dist2);
        if (comb_boundary_inside2.inside(p))
        {
            addTravel_simple(p);
            //Make sure the that any retraction happens after this move, not before it by starting a new move path.
            forceNewPathStart();
        }
    }
}

bool LayerPlan::getPrimeTowerIsPlanned(unsigned int extruder_nr) const
{
    return has_prime_tower_planned_per_extruder[extruder_nr];
}

void LayerPlan::setPrimeTowerIsPlanned(unsigned int extruder_nr)
{
    has_prime_tower_planned_per_extruder[extruder_nr] = true;
}

std::optional<std::pair<Point, bool>> LayerPlan::getFirstTravelDestinationState() const
{
    std::optional<std::pair<Point, bool>> ret;
    if (first_travel_destination)
    {
        ret = std::make_pair(*first_travel_destination, first_travel_destination_is_inside);
    }
    return ret;
}

GCodePath& LayerPlan::addTravel(const Point p, const bool force_retract, const coord_t min_comb_distance)
{
    const GCodePathConfig& travel_config = configs_storage.travel_config_per_extruder[getExtruder()];
    const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];

    GCodePath* path = getLatestPathWithConfig(travel_config, SpaceFillType::None);

    bool combed = false;

    const ExtruderTrain* extruder = getLastPlannedExtruderTrain();

    const bool retraction_hop_enabled = extruder->settings.get<bool>("retraction_hop_enabled") || (layer_nr >= 0 && (size_t)layer_nr < extruder->settings.get<size_t>("retraction_hop_initial_layers"));

    const bool is_first_travel_of_extruder_after_switch = extruder_plans.back().paths.size() == 1 && (extruder_plans.size() > 1 || last_extruder_previous_layer != getExtruder());
    bool bypass_combing = is_first_travel_of_extruder_after_switch && extruder->settings.get<bool>("retraction_hop_after_extruder_switch");

    const bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
    const bool retraction_enable = extruder->settings.get<bool>("retraction_enable");
    if (is_first_travel_of_layer)
    {
        bypass_combing = true; // first travel move is bogus; it is added after this and the previous layer have been planned in LayerPlanBuffer::addConnectingTravelMove
        first_travel_destination = p;
        first_travel_destination_is_inside = is_inside;
        if (layer_nr == 0 && retraction_enable && retraction_hop_enabled)
        {
            path->retract = true;
            path->perform_z_hop = true;
        }
        forceNewPathStart(); // force a new travel path after this first bogus move
    }
    else if (force_retract && last_planned_position && !shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        // path is not shorter than min travel distance, force a retraction
        path->retract = true;
        if (comb == nullptr)
        {
            path->perform_z_hop = retraction_hop_enabled;
        }
    }

    if (layer_nr == 0 && comb != nullptr && extruder->settings.get<bool>("travel_no_combing_on_initial_layer"))
    {
        bypass_combing = true;
    }

    if (comb != nullptr && !bypass_combing)
    {
        CombPaths combPaths;

        // Divide by 2 to get the radius
        // Multiply by 2 because if two lines start and end points places very close then will be applied combing with retractions. (Ex: for brim)
        const coord_t max_distance_ignored = extruder->settings.get<coord_t>("machine_nozzle_tip_outer_diameter") / 2 * 2;

        combed = comb->calc(*extruder, *last_planned_position, p, combPaths, was_inside, is_inside, std::max(min_comb_distance, max_distance_ignored));
        if (combed)
        {
            bool retract = path->retract || (combPaths.size() > 1 && retraction_enable);
            if (!retract)
            { // check whether we want to retract
                if (combPaths.throughAir)
                {
                    retract = retraction_enable;
                }
                else
                {
                    for (CombPath& combPath : combPaths)
                    { // retract when path moves through a boundary
                        if (combPath.cross_boundary)
                        {
                            retract = retraction_enable;
                            break;
                        }
                    }
                }
                if (combPaths.size() == 1)
                {
                    CombPath comb_path = combPaths[0];
                    if (extruder->settings.get<bool>("limit_support_retractions") &&
                        combPaths.throughAir && !comb_path.cross_boundary && comb_path.size() == 2 && comb_path[0] == *last_planned_position && comb_path[1] == p)
                    { // limit the retractions from support to support, which didn't cross anything
                        retract = false;
                    }
                }
            }

            const coord_t maximum_travel_resolution = extruder->settings.get<coord_t>("meshfix_maximum_travel_resolution");
            coord_t distance = 0;
            Point last_point((last_planned_position) ? *last_planned_position : Point(0, 0));
            const Point start_point(last_point);
            for (CombPath& combPath : combPaths)
            { // add all comb paths (don't do anything special for paths which are moving through air)
                if (combPath.size() == 0)
                {
                    continue;
                }
                for (Point& comb_point : combPath)
                {
                    if (path->points.empty() || vSize2(path->points.back() - comb_point) > maximum_travel_resolution * maximum_travel_resolution)
                    {
                        path->points.push_back(comb_point);
                        distance += vSize(last_point - comb_point);
                        last_point = comb_point;
                    }
                }
                distance += vSize(last_point - p);
                const coord_t retract_threshold = extruder->settings.get<coord_t>("retraction_combing_max_distance");
                path->retract = retract || (retract_threshold > 0 && distance > retract_threshold && retraction_enable);
                // don't perform a z-hop

                const coord_t max_extra_distance = extruder->settings.get<coord_t>("retraction_combing_max_extra_distance");
                if (max_extra_distance > 0)
                {
                    const coord_t direct_distance = vSize(start_point - p);
                    if (distance > (direct_distance + max_extra_distance))
                    {
                        // the combed travel distance is too long, use a direct line travel move instead
                        path->points.clear();
                        // retract and z-hop as per a normal non-combed travel move
                        path->retract = retraction_enable && (direct_distance >= retraction_config.retraction_min_travel_distance);
                        if (path->retract)
                        {
                            path->perform_z_hop = retraction_hop_enabled;
                        }
                    }
                }
            }
        }
    }

    // CURA-6675:
    // Retraction Minimal Travel Distance should work for all travel moves. If the travel move is shorter than the
    // Retraction Minimal Travel Distance, retraction should be disabled.
    if (!is_first_travel_of_layer && last_planned_position && shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        path->retract = false;
        path->perform_z_hop = false;
    }

    // no combing? retract only when path is not shorter than minimum travel distance
    if (!combed && !is_first_travel_of_layer && last_planned_position && !shorterThen(*last_planned_position - p, retraction_config.retraction_min_travel_distance))
    {
        if (was_inside) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
        {               // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
            assert (extruder != nullptr);
            coord_t innermost_wall_line_width = extruder->settings.get<coord_t>((extruder->settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
            if (layer_nr == 0)
            {
                innermost_wall_line_width *= extruder->settings.get<Ratio>("initial_layer_line_width_factor");
            }
            moveInsideCombBoundary(innermost_wall_line_width);
        }
        path->retract = retraction_enable;
        path->perform_z_hop = retraction_enable && retraction_hop_enabled;
    }

    // must start new travel path as retraction can be enabled or not depending on path length, etc.
    forceNewPathStart();

    GCodePath& ret = addTravel_simple(p, path);
    was_inside = is_inside;
    return ret;
}

GCodePath& LayerPlan::addTravel_simple(Point p, GCodePath* path)
{
    bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
    if (is_first_travel_of_layer)
    { // spiralize calls addTravel_simple directly as the first travel move in a layer
        first_travel_destination = p;
        first_travel_destination_is_inside = is_inside;
    }
    if (path == nullptr)
    {
        path = getLatestPathWithConfig(configs_storage.travel_config_per_extruder[getExtruder()], SpaceFillType::None);
    }
    path->points.push_back(p);
    last_planned_position = p;
    return *path;
}

void LayerPlan::planPrime(const float& prime_blob_wipe_length)
{
    forceNewPathStart();
    GCodePath& prime_travel = addTravel_simple(getLastPlannedPositionOrStartingPosition() + Point(0, MM2INT(prime_blob_wipe_length)));
    prime_travel.retract = false;
    prime_travel.perform_z_hop = false;
    prime_travel.perform_prime = true;
    forceNewPathStart();
}

void LayerPlan::addExtrusionMove(Point p, const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio& flow, bool spiralize, Ratio speed_factor, double fan_speed)
{
    // ignore extrusions less than 5uM long
    if(vSize2(p - *last_planned_position) >= 25)
    {
        GCodePath* path = getLatestPathWithConfig(config, space_fill_type, flow, spiralize, speed_factor, fan_speed);
        path->points.push_back(p);
        if (max_path_time > 0)
        {
            // start a new path if the current path print time is at least max_path_time
            path->length += vSize(p - *last_planned_position);
            if (INT2MM(path->length) >= max_path_time * config.getSpeed() * speed_factor)
            {
                forceNewPathStart();
            }
        }
        last_planned_position = p;
    }
}

void LayerPlan::addPolygon(ConstPolygonRef polygon, int start_idx, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation, coord_t wall_0_wipe_dist, bool spiralize, const Ratio& flow_ratio, bool always_retract)
{
    Point p0 = polygon[start_idx];
    addTravel(p0, always_retract);
    for (unsigned int point_idx = 1; point_idx < polygon.size(); point_idx++)
    {
        Point p1 = polygon[(start_idx + point_idx) % polygon.size()];
        const Ratio flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow, spiralize);
        p0 = p1;
    }
    if (polygon.size() > 2)
    {
        const Point& p1 = polygon[start_idx];
        const Ratio flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
        addExtrusionMove(p1, config, SpaceFillType::Polygons, flow, spiralize);

        if (wall_0_wipe_dist > 0)
        { // apply outer wall wipe
            p0 = polygon[start_idx];
            int distance_traversed = 0;
            for (unsigned int point_idx = 1; ; point_idx++)
            {
                Point p1 = polygon[(start_idx + point_idx) % polygon.size()];
                int p0p1_dist = vSize(p1 - p0);
                if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
                {
                    Point vector = p1 - p0;
                    Point half_way = p0 + normal(vector, wall_0_wipe_dist - distance_traversed);
                    addTravel_simple(half_way);
                    break;
                }
                else
                {
                    addTravel_simple(p1);
                    distance_traversed += p0p1_dist;
                }
                p0 = p1;
            }
            forceNewPathStart();
        }
    }
    else 
    {
        logWarning("WARNING: line added as polygon! (LayerPlan)\n");
    }
}

void LayerPlan::addPolygonsByOptimizer(const Polygons& polygons, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation, const ZSeamConfig& z_seam_config, coord_t wall_0_wipe_dist, bool spiralize, const Ratio flow_ratio, bool always_retract, bool reverse_order, std::optional<Point> start_near_location)
{
    if (polygons.size() == 0)
    {
        return;
    }
    PathOrderOptimizer orderOptimizer(start_near_location ? start_near_location.value() : getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(polygons[poly_idx]);
    }
    orderOptimizer.optimize();
    
    if(reverse_order == false)
    {
        for (unsigned int poly_idx : orderOptimizer.polyOrder)
        {
            addPolygon(polygons[poly_idx], orderOptimizer.polyStart[poly_idx], config, wall_overlap_computation, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
    else
    {
        for(int index = orderOptimizer.polyOrder.size() - 1; index >= 0; --index)
        {
            int poly_idx = orderOptimizer.polyOrder[index];
            addPolygon(polygons[poly_idx], orderOptimizer.polyStart[poly_idx], config, wall_overlap_computation, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
        }
    }
}

static constexpr float max_non_bridge_line_volume = MM2INT(100); // limit to accumulated "volume" of non-bridge lines which is proportional to distance x extrusion rate

void LayerPlan::addWallLine(const Point& p0, const Point& p1, const SliceMeshStorage& mesh, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, float flow, float& non_bridge_line_volume, Ratio speed_factor, coord_t& distance_to_bridge_start)
{
    const coord_t min_line_len = 5; // we ignore lines less than 5um long
    const double acceleration_segment_len = MM2INT(1); // accelerate using segments of this length
    const double acceleration_factor = 0.75; // must be < 1, the larger the value, the slower the acceleration
    const bool spiralize = false;

    const coord_t min_bridge_line_len = mesh.settings.get<coord_t>("bridge_wall_min_length");
    const Ratio bridge_wall_coast = mesh.settings.get<Ratio>("bridge_wall_coast");
    const Ratio bridge_wall_end_boost = mesh.settings.get<Ratio>("bridge_wall_end_boost");
    Ratio overhang_speed_factor = mesh.settings.get<Ratio>("wall_overhang_speed_factor");
    const Point mid(p0 + (p1 - p0)/2);
    const bool is_overhang = (!overhang_mask.empty() && overhang_mask.inside(p0, true) && overhang_mask.inside(p1, true) && overhang_mask.inside(mid, true));
    double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;

    if (is_overhang)
    {
        if (layer_nr > 1)
        {
            fan_speed = (double)mesh.settings.get<Ratio>("wall_overhang_fan_speed") * 100.0;
        }
        // use the distance from the mid point of the line segment to the inside edge of the overhang mask to modify the overhang speed factor
        // the closer the line segment is to the inside edge of the overhang mask, the closer the overhang speed factor is to 1.0
        // the effect of this is to smooth the speed transition
        const coord_t wall_line_width_0 = mesh.settings.get<coord_t>("wall_line_width_0");
        // end is normal to the line mid point spaced a wall line width to the inside of the polygon
        const Point end(mid + normal(turn90CCW(p1 - mid), wall_line_width_0));
        if (!overhang_mask.inside(end, true))
        {
            // end is not inside the overhang mask area so unless the outline polygon is a very weird shape end must be
            // located nearer to the inside edge of the mask than the outer edge
            ClosestPolygonPoint cpp = PolygonUtils::findClosest(end, overhang_mask);
            if (cpp.isValid())
            {
                // cpp.location is now the point on the inside edge of the overhang mask that is nearest to mid
                const coord_t overhang_width = layer_thickness * std::tan(mesh.settings.get<AngleDegrees>("wall_overhang_angle") / (180 / M_PI));
                const coord_t dist = vSize(cpp.location - mid);
                if (dist < overhang_width)
                {
                    overhang_speed_factor = 1.0 + (overhang_speed_factor - 1.0) * dist / overhang_width;
                }
            }
        }
        // ensure the speed factor does not produce a speed less than the minimum speed allowed
        const Velocity min_speed = fan_speed_layer_time_settings_per_extruder[getExtruder()].cool_min_speed;
        overhang_speed_factor = std::max((double)overhang_speed_factor, (double)(min_speed / non_bridge_config.getSpeed()));
    }

    Point cur_point = p0;

    // helper function to add a single non-bridge line

    // If the line precedes a bridge line, it may be coasted to reduce the nozzle pressure before the bridge is reached

    // alternatively, if the line follows a bridge line, it may be segmented and the print speed gradually increased to reduce under-extrusion

    auto addNonBridgeLine = [&](const Point& line_end)
    {
        coord_t distance_to_line_end = vSize(cur_point - line_end);

        while (distance_to_line_end > min_line_len)
        {
            // if we are accelerating after a bridge line, the segment length is less than the whole line length
            Point segment_end = (speed_factor == 1 || distance_to_line_end < acceleration_segment_len) ? line_end : cur_point + (line_end - cur_point) * acceleration_segment_len / distance_to_line_end;

            // flow required for the next line segment - when accelerating after a bridge segment, the flow is increased in inverse proportion to the speed_factor
            // so the slower the feedrate, the greater the flow - the idea is to get the extruder back to normal pressure as quickly as possible
            const float segment_flow = (speed_factor < 1) ? flow * (1 / speed_factor) : flow;

            // if a bridge is present in this wall, this particular segment may need to be partially or wholely coasted
            if (distance_to_bridge_start > 0)
            {
                // speed_flow_factor approximates how the extrusion rate alters between the non-bridge wall line and the following bridge wall line
                // if the extrusion rates are the same, its value will be 1, if the bridge config extrusion rate is < the non-bridge config extrusion rate, the value is < 1

                const Ratio speed_flow_factor((bridge_config.getSpeed() * bridge_config.getFlowRatio()) / (non_bridge_config.getSpeed() * non_bridge_config.getFlowRatio()));

                // coast distance is proportional to distance, speed and flow of non-bridge segments just printed and is throttled by speed_flow_factor
                const double coast_dist = std::min(non_bridge_line_volume, max_non_bridge_line_volume) * (1 - speed_flow_factor) * bridge_wall_coast / 40;
                const double distance_to_coast_start = std::max(distance_to_bridge_start - coast_dist, 0.0);

                if (coast_dist > 0 && distance_to_coast_start <= distance_to_line_end)
                {
                    segment_end = line_end;
                    if ((distance_to_line_end - distance_to_coast_start) > min_line_len)
                    {
                        // segment is longer than coast distance so extrude using non-bridge config to start of coast
                        addExtrusionMove(cur_point + (line_end - cur_point) * distance_to_coast_start / distance_to_line_end, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize, speed_factor);
                    }
                    // then coast to end of line
                    addExtrusionMove(line_end, non_bridge_config, SpaceFillType::Polygons, 0, spiralize, speed_factor);
                    distance_to_bridge_start -= distance_to_line_end;
                    if (distance_to_bridge_start < 0)
                    {
                        distance_to_bridge_start = 0;
                    }
                }
                else
                {
                    // no coasting required, just normal segment using non-bridge config
                    addExtrusionMove(segment_end, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize, (is_overhang) ? overhang_speed_factor : speed_factor, fan_speed);
                    distance_to_bridge_start -= vSize(cur_point - segment_end);
                    if (distance_to_bridge_start < 0)
                    {
                        distance_to_bridge_start = 0;
                    }
                }
            }
            else
            {
                // no coasting required, just normal segment using non-bridge config
                addExtrusionMove(segment_end, non_bridge_config, SpaceFillType::Polygons, segment_flow, spiralize, (is_overhang) ? overhang_speed_factor : speed_factor, fan_speed);
            }
            non_bridge_line_volume += vSize(cur_point - segment_end) * segment_flow * speed_factor * non_bridge_config.getSpeed();
            cur_point = segment_end;
            speed_factor = 1 - (1 - speed_factor) * acceleration_factor;
            if (speed_factor >= 0.9)
            {
                speed_factor = 1;
            }
            distance_to_line_end = vSize(cur_point - line_end);
        }
    };

    if (bridge_wall_mask.empty())
    {
        // no bridges required
        addExtrusionMove(p1, non_bridge_config, SpaceFillType::Polygons, flow, spiralize, (is_overhang) ? overhang_speed_factor : 1.0_r, fan_speed);
    }
    else
    {
        // bridges may be required
        if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0, p1))
        {
            // the line crosses the boundary between supported and non-supported regions so one or more bridges are required

            // determine which segments of the line are bridges

            Polygon line_poly;
            line_poly.add(p0);
            line_poly.add(p1);
            Polygons line_polys;
            line_polys.add(line_poly);
            line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

            // line_polys now contains the wall lines that need to be printed using bridge_config

            while (line_polys.size() > 0)
            {
                // find the bridge line segment that's nearest to the current point
                int nearest = 0;
                float smallest_dist2 = vSize2f(cur_point - line_polys[0][0]);
                for(unsigned i = 1; i < line_polys.size(); ++i)
                {
                    float dist2 = vSize2f(cur_point - line_polys[i][0]);
                    if (dist2 < smallest_dist2)
                    {
                        nearest = i;
                        smallest_dist2 = dist2;
                    }
                }
                ConstPolygonRef bridge = line_polys[nearest];

                // set b0 to the nearest vertex and b1 the furthest
                Point b0 = bridge[0];
                Point b1 = bridge[1];

                if (vSize2f(cur_point - b1) < vSize2f(cur_point - b0))
                {
                    // swap vertex order
                    b0 = bridge[1];
                    b1 = bridge[0];
                }

                const double bridge_line_len = vSize(b1 - b0);

                if (bridge_line_len >= min_bridge_line_len)
                {
                    // extrude using non_bridge_config to the start of the next bridge segment

                    distance_to_bridge_start = vSize(cur_point - b0);
                    addNonBridgeLine(b0);

                    // extrude using bridge_config to the end of the next bridge segment

                    if (bridge_line_len > min_line_len)
                    {
                        addExtrusionMove(b1, bridge_config, SpaceFillType::Polygons, flow);
                        non_bridge_line_volume = 0;
                        cur_point = b1;
                        // after a bridge segment, start slow and accelerate to avoid under-extrusion due to extruder lag
                        speed_factor = std::max(std::min(Ratio(bridge_config.getSpeed() / non_bridge_config.getSpeed()), 1.0_r), 0.5_r);
                        // user can control amount of boost
                        speed_factor = 1 - (1 - speed_factor) * bridge_wall_end_boost;
                    }
                    distance_to_bridge_start = 0;
                }
                else
                {
                    // treat the short bridge line just like a normal line

                    addNonBridgeLine(b1);
                }

                // finished with this segment
                line_polys.remove(nearest);
            }

            // if we haven't yet reached p1, fill the gap with non_bridge_config line
            addNonBridgeLine(p1);
        }
        else if (bridge_wall_mask.inside(p0, true) && vSize(p0 - p1) >= min_bridge_line_len)
        {
            // both p0 and p1 must be above air (the result will be ugly!)
            addExtrusionMove(p1, bridge_config, SpaceFillType::Polygons, flow);
            non_bridge_line_volume = 0;
        }
        else
        {
            // no part of the line is above air or the line is too short to print as a bridge line
            addNonBridgeLine(p1);
        }
    }
}

void LayerPlan::addWall(ConstPolygonRef wall, int start_idx, const SliceMeshStorage& mesh, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
    Point z_seam_point = wall[start_idx];

    // when the z-seam location is user-specified and no corner preference has been specified, calculate an "exact" z-seam point that does not have to
    // be located at a wall vertex, i.e. it can fall between vertices. This should eliminate the z-seam wobble that occurs due to the non-alignment of vertices
    // between layers.

    if (mesh.settings.get<EZSeamType>("z_seam_type") == EZSeamType::USER_SPECIFIED && mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner") == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
    {
        // create a "hint line" that extends all the way across the mesh and passes through the z-seam hint point and the middle of the mesh
        // then find the point on the wall outline that lies on the hint line and is closest to the z-seam hint point and use that for the wall's z_seam_point
        // if no such point can be found, just use the point that has been previously calculated to be nearest the z-seam hint as the wall's z_seam_point
        const Point abs_z_seam_hint = mesh.getZSeamHint();
        const Point mesh_middle = mesh.bounding_box.flatten().getMiddle();
        // we need the z-seam hint point to be at least a little distance from the middle of the mesh to be able to create a reliable hint line
        if (vSize(abs_z_seam_hint - mesh_middle) > 1000)
        {
            const coord_t approx_max_len = std::max(mesh.settings.get<coord_t>("machine_width"), mesh.settings.get<coord_t>("machine_depth")) * 1.414;
            // hint_vec needs to be long enough so that its end points are further away from abs_z_seam_hint than approx_max_len
            const Point hint_vec = normal(abs_z_seam_hint - mesh_middle, approx_max_len * 4);
            // intersect that line with the wall polygon and find the resulting line end point that is closest to the z-seam hint point
            Polygons lines;
            lines.addLine(mesh_middle - hint_vec, mesh_middle + hint_vec);
            Polygons wall_polys;
            wall_polys.add(wall);
            lines = wall_polys.intersectionPolyLines(lines);
            Point closest = z_seam_point;
            coord_t min_dist2 = approx_max_len * approx_max_len;
            for (ConstPolygonRef line : lines)
            {
                for (unsigned i = 0; i < 2; ++i)
                {
                    coord_t dist2 = vSize2(abs_z_seam_hint - line[i]);
                    if (dist2 < min_dist2)
                    {
                        min_dist2 = dist2;
                        closest = line[i];
                    }
                }
            }
            if (vSize2(closest - z_seam_point) > 25)
            {
                // find the wall line that closest lies on and set start_idx to the first point on that line
                // the line is likely to be close to wall[start_idx] so test the nearest lines first
                for (unsigned i = 0; i < (wall.size() + 1) / 2; ++i)
                {
                    // test lines forwards of wall[start_idx]
                    int16_t beyond = 0;
                    if (LinearAlg2D::getDist2FromLineSegment(wall[(start_idx + i) % wall.size()], closest, wall[(start_idx + i + 1) % wall.size()], &beyond) < 25 && !beyond)
                    {
                        z_seam_point = closest;
                        start_idx = (start_idx + i) % wall.size();
                        break;
                    }
                    // test lines backwards of wall[start_idx]
                    beyond = 0;
                    if (LinearAlg2D::getDist2FromLineSegment(wall[(start_idx + wall.size() - i - 1) % wall.size()], closest, wall[(start_idx + wall.size() - i) % wall.size()], &beyond) < 25 && !beyond)
                    {
                        z_seam_point = closest;
                        start_idx = (start_idx + wall.size() - i - 1) % wall.size();
                        break;
                    }
                }
            }
        }
    }

    // make sure wall start point is not above air!
    int supported_start_idx = locateFirstSupportedVertex(wall, start_idx);

    if (supported_start_idx != start_idx)
    {
        start_idx = supported_start_idx;
        z_seam_point = wall[supported_start_idx];
    }

    float non_bridge_line_volume = max_non_bridge_line_volume; // assume extruder is fully pressurised before first non-bridge line is output
    double speed_factor = 1.0; // start first line at normal speed
    coord_t distance_to_bridge_start = 0; // will be updated before each line is processed

    const coord_t min_bridge_line_len = mesh.settings.get<coord_t>("bridge_wall_min_length");
    const Ratio wall_min_flow = mesh.settings.get<Ratio>("wall_min_flow");
    const bool wall_min_flow_retract = mesh.settings.get<bool>("wall_min_flow_retract");
    const coord_t small_feature_max_length = mesh.settings.get<coord_t>("small_feature_max_length");
    const bool is_small_feature = (small_feature_max_length > 0) && wall.shorterThan(small_feature_max_length);
    Ratio small_feature_speed_factor = mesh.settings.get<Ratio>((layer_nr == 0) ? "small_feature_speed_factor_0" : "small_feature_speed_factor");
    const Velocity min_speed = fan_speed_layer_time_settings_per_extruder[getLastPlannedExtruderTrain()->extruder_nr].cool_min_speed;
    small_feature_speed_factor = std::max((double)small_feature_speed_factor, (double)(min_speed / non_bridge_config.getSpeed()));

    // helper function to calculate the distance from the start of the current wall line to the first bridge segment

    auto computeDistanceToBridgeStart = [&](const unsigned current_line_index, const unsigned num_lines)
    {
        if (!bridge_wall_mask.empty())
        {
            // there is air below the part so iterate through the lines that have not yet been output accumulating the total distance to the first bridge segment
            for (unsigned line_idx = 0; line_idx < num_lines; ++line_idx)
            {
                const Point& p0 = wall[(current_line_index + line_idx) % wall.size()];
                const Point& p1 = wall[(current_line_index + line_idx + 1) % wall.size()];

                if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0, p1))
                {
                    // the line crosses the boundary between supported and non-supported regions so it will contain one or more bridge segments

                    // determine which segments of the line are bridges

                    Polygon line_poly;
                    line_poly.add(p0);
                    line_poly.add(p1);
                    Polygons line_polys;
                    line_polys.add(line_poly);
                    line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

                    while (line_polys.size() > 0)
                    {
                        // find the bridge line segment that's nearest to p0
                        int nearest = 0;
                        float smallest_dist2 = vSize2f(p0 - line_polys[0][0]);
                        for(unsigned i = 1; i < line_polys.size(); ++i)
                        {
                            float dist2 = vSize2f(p0 - line_polys[i][0]);
                            if (dist2 < smallest_dist2)
                            {
                                nearest = i;
                                smallest_dist2 = dist2;
                            }
                        }
                        ConstPolygonRef bridge = line_polys[nearest];

                        // set b0 to the nearest vertex and b1 the furthest
                        Point b0 = bridge[0];
                        Point b1 = bridge[1];

                        if (vSize2f(p0 - b1) < vSize2f(p0 - b0))
                        {
                            // swap vertex order
                            b0 = bridge[1];
                            b1 = bridge[0];
                        }

                        if (vSize(b1 - b0) >= min_bridge_line_len)
                        {
                            // job done, we have found the first bridge line
                            distance_to_bridge_start += vSize(b0 - p0);
                            return;
                        }

                        // finished with this segment
                        line_polys.remove(nearest);
                    }
                    // none of the unsupported line segments were long enough to bridge
                    distance_to_bridge_start += vSize(p1 - p0);
                }
                else if (!bridge_wall_mask.inside(p0, true))
                {
                    // none of the line is over air
                    distance_to_bridge_start += vSize(p1 - p0);
                }
            }

            // we have got all the way to the end of the wall without finding a bridge segment so disable coasting by setting distance_to_bridge_start to -1

            distance_to_bridge_start = -1;
        }
    };

    bool travel_required = false; // true when a wall has been omitted due to its flow being less than the minimum required

    bool first_line = true;

    Point p0 = z_seam_point;

    const coord_t max_spurious_fat_segment_length = std::max((coord_t)50, mesh.settings.get<coord_t>("meshfix_maximum_resolution"));
    const coord_t max_spurious_fat_segment_length2 = max_spurious_fat_segment_length * max_spurious_fat_segment_length;

    const bool extend_floating_bridge_walls = !bridge_wall_mask.empty() && mesh.settings.get<bool>("bridge_extend_floating_walls");
    Polygons line_extension_mask;
    bool use_retraction = false;
    const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];
    if (extend_floating_bridge_walls)
    {
        const coord_t line_width = non_bridge_config.getLineWidth();
        line_extension_mask = mesh.layers[layer_nr].getOutlines().offset(-line_width).intersection(bridge_wall_mask).offset(line_width);
        use_retraction = getLastPlannedExtruderTrain()->settings.get<bool>("retraction_enable");
    }

    for (unsigned int point_idx = 1; point_idx <= wall.size(); point_idx++)
    {
        const Point& p1 = wall[(start_idx + point_idx) % wall.size()];
        const float flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;

        if (!bridge_wall_mask.empty() && distance_to_bridge_start == 0)
        {
            computeDistanceToBridgeStart((start_idx + point_idx - 1) % wall.size(), wall.size() - point_idx + 1);
        }

        // HACK ALERT
        // the overlap compensation is not perfect, it can produce short non-flow reduced line segments within a sequence of flow reduced
        // line segments and so to try and avoid printing the spurious fat line segments we require that their lengths are above a threshold


        if (flow >= wall_min_flow && (first_line || !travel_required || !wall_overlap_computation || vSize2f(p0 - p1) > max_spurious_fat_segment_length2))
        {
            if (first_line || travel_required)
            {
                addTravel(p0, (first_line) ? always_retract : wall_min_flow_retract);
                first_line = false;
                travel_required = false;
            }
            if (is_small_feature)
            {
                constexpr bool spiralize = false;
                addExtrusionMove(p1, non_bridge_config, SpaceFillType::Polygons, flow, spiralize, small_feature_speed_factor);
            }
            else if (extend_floating_bridge_walls)
            {
                auto extendLineThatEndsOnAir = [&](const Point& start, const Point& end)
                {
                    // extend wall line to a maximum of twice its original length
                    Point remote(end + normal(end - start, vSize(end - start)));
                    Polygons lines;
                    lines.addLine(remote, end);
                    lines = line_extension_mask.intersectionPolyLines(lines);
                    for (unsigned n = 0; n < lines.size(); ++n)
                    {
                        if (lines[n][0] == end)
                        {
                            return (lines[n][1] == remote || bridge_wall_mask.inside(lines[n][1])) ? end : lines[n][1];
                        }
                        if (lines[n][1] == end)
                        {
                            return (lines[n][0] == remote || bridge_wall_mask.inside(lines[n][0])) ? end : lines[n][0];
                        }
                    }
                    return end;
                };
                const Point _p0(bridge_wall_mask.inside(p0) ? extendLineThatEndsOnAir(p1, p0) : p0);
                const Point _p1(bridge_wall_mask.inside(p1) ? extendLineThatEndsOnAir(p0, p1) : p1);
                if (_p0 != p0)
                {
                    addTravel_simple(_p0).retract = use_retraction && (vSize(p0 - _p0) >= retraction_config.retraction_min_travel_distance);
                }
                addWallLine(_p0, _p1, mesh, non_bridge_config, bridge_config, flow, non_bridge_line_volume, speed_factor, distance_to_bridge_start);
                if (_p1 != p1)
                {
                    addTravel_simple(p1).retract = use_retraction && (vSize(p1 - _p1) >= retraction_config.retraction_min_travel_distance);
                }
            }
            else
            {
                addWallLine(p0, p1, mesh, non_bridge_config, bridge_config, flow, non_bridge_line_volume, speed_factor, distance_to_bridge_start);
            }
        }
        else
        {
            travel_required = true;
        }

        p0 = p1;
    }

    if (z_seam_point != wall[start_idx])
    {
        // add final line from wall[start_idx] to z_seam_point
        const float flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(wall[start_idx], z_seam_point) : flow_ratio;

        if (flow >= wall_min_flow)
        {
            if (travel_required)
            {
                addTravel(wall[start_idx], wall_min_flow_retract);
                travel_required = false;
            }
            if (is_small_feature)
            {
                constexpr bool spiralize = false;
                addExtrusionMove(z_seam_point, non_bridge_config, SpaceFillType::Polygons, flow, spiralize, small_feature_speed_factor);
            }
            else
            {
                addWallLine(wall[start_idx], z_seam_point, mesh, non_bridge_config, bridge_config, flow, non_bridge_line_volume, speed_factor, distance_to_bridge_start);
            }
        }
        else
        {
            travel_required = true;
        }
    }

    if (travel_required)
    {
        addTravel(z_seam_point, wall_min_flow_retract);
    }

    if (wall_0_wipe_dist > 0)
    { // apply outer wall wipe
        p0 = z_seam_point;
        int distance_traversed = 0;
        for (unsigned int point_idx = 1; ; point_idx++)
        {
            Point p1 = wall[(start_idx + point_idx) % wall.size()];
            int p0p1_dist = vSize(p1 - p0);
            if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
            {
                Point vector = p1 - p0;
                Point half_way = p0 + normal(vector, wall_0_wipe_dist - distance_traversed);
                addTravel_simple(half_way);
                break;
            }
            else
            {
                addTravel_simple(p1);
                distance_traversed += p0p1_dist;
            }
            p0 = p1;
        }
        forceNewPathStart();
    }
}

void LayerPlan::addWalls(const Polygons& walls, const SliceMeshStorage& mesh, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, const ZSeamConfig& z_seam_config, coord_t wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
    PathOrderOptimizer orderOptimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config);
    for (unsigned int poly_idx = 0; poly_idx < walls.size(); poly_idx++)
    {
        orderOptimizer.addPolygon(walls[poly_idx]);
    }
    orderOptimizer.optimize();
    for (unsigned int poly_idx : orderOptimizer.polyOrder)
    {
        addWall(walls[poly_idx], orderOptimizer.polyStart[poly_idx], mesh, non_bridge_config, bridge_config, wall_overlap_computation, wall_0_wipe_dist, flow_ratio, always_retract);
    }
}

unsigned LayerPlan::locateFirstSupportedVertex(ConstPolygonRef wall, const unsigned start_idx) const
{
    if (bridge_wall_mask.empty() && overhang_mask.empty())
    {
        return start_idx;
    }

    Polygons air_below(bridge_wall_mask.unionPolygons(overhang_mask));

    unsigned curr_idx = start_idx;

    while(true)
    {
        const Point& vertex = wall[curr_idx];
        if (!air_below.inside(vertex, true))
        {
            // vertex isn't above air so it's OK to use
            return curr_idx;
        }

        if (++curr_idx >= wall.size())
        {
            curr_idx = 0;
        }

        if (curr_idx == start_idx)
        {
            // no vertices are supported so just return the original index
            return start_idx;
        }
    }
}

void LayerPlan::addGradientInfillLine(const Point& p0, const Point& p1, const float flow, const GCodePathConfig& config, SpaceFillType space_fill_type, const double speed_factor, const double fan_speed, const SliceMeshStorage* mesh, float& gradient_infill_last_flow)
{
    const coord_t infill_line_distance = mesh->settings.get<coord_t>("infill_line_distance");
    const float gradient_infill_min_flow = mesh->settings.get<Ratio>("gradient_infill_min_flow");
    const float gradient_infill_max_flow = mesh->settings.get<Ratio>("gradient_infill_max_flow");
    const float gradient_infill_speed_factor = mesh->settings.get<Ratio>("gradient_infill_speed_factor");
    const EGradientInfillType gradient_infill_type = mesh->settings.get<EGradientInfillType>("gradient_infill_type");
    const EGradientInfillSpeedScheme gradient_infill_speed_scheme = mesh->settings.get<EGradientInfillSpeedScheme>("gradient_infill_speed_scheme");
    const Velocity min_speed = fan_speed_layer_time_settings_per_extruder[getExtruder()].cool_min_speed;
    const coord_t line_len = vSize(p1 - p0);

    const Polygons outline = mesh->layers[layer_nr].getOutlines();
    const coord_t gradient_infill_dist = mesh->settings.get<coord_t>("gradient_infill_dist");

    auto addLine = [&](const Point& from, const Point& to) {
        Point mid = (from + to) / 2;
        coord_t dist = 0;
        if (gradient_infill_type == EGradientInfillType::LINEAR_2D || gradient_infill_type == EGradientInfillType::LINEAR_2D_PLUS || gradient_infill_type == EGradientInfillType::LINEAR_3D)
        {
            dist = std::min(vSize(PolygonUtils::findClosest(mid, outline).location - mid), gradient_infill_dist);

            if (gradient_infill_type == EGradientInfillType::LINEAR_2D_PLUS)
            {
                if (z < dist)
                {
                    dist = z;
                }
                coord_t top_dist = mesh->layers.back().printZ - mesh->layers[layer_nr].printZ;
                if (top_dist < dist)
                {
                    dist = top_dist;
                }
            }
            else if (gradient_infill_type == EGradientInfillType::LINEAR_3D)
            {
                coord_t top_dist = mesh->layers[layer_nr].thickness;
                coord_t bot_dist = 0;
                for (int ln = 1; ln < layer_nr && (layer_nr + ln) < (int)mesh->layers.size(); ++ln)
                {
                    if (!mesh->layers[layer_nr + ln].getOutlines().inside(mid))
                    {
                        // found air above
                        break;
                    }
                    top_dist += mesh->layers[layer_nr + ln].thickness;
                    if (ln < layer_nr)
                    {
                        if (!mesh->layers[layer_nr - ln].getOutlines().inside(mid))
                        {
                            // found air below
                            break;
                        }
                        bot_dist += mesh->layers[layer_nr - ln].thickness;
                    }
                    if (top_dist >= dist && bot_dist >= dist)
                    {
                        // no point in checking any more layers
                        break;
                    }
                }
                if (top_dist < dist)
                {
                    dist = top_dist;
                }
                if (bot_dist < dist)
                {
                    dist = bot_dist;
                }
            }
        }
        else
        {
            dist = std::min(vSize(mid - p0), vSize(mid - p1));
        }
        const float fl = std::max(gradient_infill_max_flow - (gradient_infill_max_flow - gradient_infill_min_flow) * dist / gradient_infill_dist, gradient_infill_min_flow);
        float sf = 1;
        if (gradient_infill_speed_scheme == EGradientInfillSpeedScheme::FLOW_CHANGE)
        {
            const float last_fl = gradient_infill_last_flow;
            if (fl != last_fl)
            {
                sf = ((fl < last_fl) ? (fl / last_fl) : last_fl / fl);
            }
        }
        else if(gradient_infill_speed_scheme == EGradientInfillSpeedScheme::CONSTANT_FAST)
        {
            sf = 1.0f / fl;
        }
        else if(gradient_infill_speed_scheme == EGradientInfillSpeedScheme::CONSTANT_SLOW)
        {
            sf = 1.0f / (fl + 1.0 - gradient_infill_min_flow);
        }
        sf = 1.0f + (gradient_infill_speed_factor * (sf - 1.0f));

        if (sf < 1.0f)
        {
            // don't produce a speed less than the minimum speed allowed
            sf = std::max(sf, (float)(min_speed / (config.getSpeed() * speed_factor)));
        }
        addExtrusionMove(to, config, space_fill_type, fl * flow, false, sf * speed_factor, fan_speed);
        gradient_infill_last_flow = fl;
    };

    const EFillMethod infill_pattern = mesh->settings.get<EFillMethod>("infill_pattern");

    bool infill_is_short_lines = false;

    switch (infill_pattern)
    {
        case EFillMethod::GYROID:
        case EFillMethod::SCHWARZ_P:
        case EFillMethod::SCHWARZ_D:
        case EFillMethod::HONEYCOMB:
            infill_is_short_lines = true;
            break;
        default:
            break;
    }

    if (infill_is_short_lines && gradient_infill_type != EGradientInfillType::LINEAR_1D)
    {
        addLine(p0, p1);
    }
    else if (line_len >= infill_line_distance)
    {
        if(true)
        {
            // divide infill line up into 3 sections: start, middle, end
            // the start and end sections are sub-divided into small segments and the middle section is subdivided into large segments
            const int max_small_segs = 4;
            const coord_t small_seg_len = gradient_infill_dist / max_small_segs;
            const coord_t large_seg_len = std::min(small_seg_len * 2, infill_line_distance);
            const int num_small_segs = std::min((int)(line_len / (small_seg_len * 2)), max_small_segs);

            Point last(p0);
            for (int i = 1; i <= num_small_segs; ++i)
            {
                Point next = p0 + normal(p1 - p0, small_seg_len * i);
                addLine(last, next);
                last = next;
            }

            Point limit(p1 - normal(p1 - p0, small_seg_len * num_small_segs));

            if (gradient_infill_type != EGradientInfillType::LINEAR_1D)
            {
                while (vSize(limit - last) >= 1.1f * large_seg_len)
                {
                    Point next = last + normal(p1 - p0, large_seg_len);
                    addLine(last, next);
                    last = next;
                }
            }
            else
            {
                addLine(last, limit);
                last = limit;
            }

            for (int i = num_small_segs; i >= 0; --i)
            {
                Point next = p1 - normal(p1 - p0, small_seg_len * i);
                addLine(last, next);
                last = next;
            }
        }
#if 0
        if(gradient_infill_type == EGradientInfillType::EXPONENTIAL)
        {
            const float gradient_infill_exp_factor = mesh->settings.get<double>("gradient_infill_exp_factor");
            const float pos_inc = 0.1f;
            float pos = pos_inc;
            float fl = gradient_infill_max_flow;
            Point pa;
            while (fl > gradient_infill_min_flow && pos < 0.5f)
            {
                pa = p0 + normal(p1 - p0, dist * pos);
                addExtrusionMove(pa, config, space_fill_type, fl * flow, false, speed_factor, fan_speed);
                pos += pos_inc;
                fl /= gradient_infill_exp_factor;
            }
            pos -= pos_inc;
            if (pos >= 0.5f)
            {
                pos = 0.5f;
            }
            else
            {
                pos = 1.0f - pos;
            }
            if (fl < gradient_infill_min_flow)
            {
                fl = gradient_infill_min_flow;
            }
            pa = (p1 - (pa - p0));
            addExtrusionMove(pa, config, space_fill_type, fl * flow, false, speed_factor, fan_speed);
            fl *= gradient_infill_exp_factor;
            pos += pos_inc;
            while (fl <= gradient_infill_max_flow && pos <= 1.0f)
            {
                pa = p0 + normal(p1 - p0, dist * pos);
                addExtrusionMove(pa, config, space_fill_type, fl * flow, false, speed_factor, fan_speed);
                pos += pos_inc;
                fl *= gradient_infill_exp_factor;
            }
            addExtrusionMove(p1, config, space_fill_type, gradient_infill_max_flow * flow, false, speed_factor, fan_speed);
        }
#endif
    }
    else
    {
        addExtrusionMove(p1, config, space_fill_type, flow, false, speed_factor, fan_speed);
    }
}

void LayerPlan::addLinesByOptimizer(const Polygons& polygons, const GCodePathConfig& config, SpaceFillType space_fill_type, bool enable_travel_optimization, int wipe_dist, float flow_ratio, std::optional<Point> near_start_location, double fan_speed, const float avoid_freq, const SliceMeshStorage* mesh, const EFillMethod pattern)
{
    Polygons boundary;
    if (enable_travel_optimization && comb_boundary_inside2.size() > 0)
    {
        // use the combing boundary inflated so that all infill lines are inside the boundary
        int dist = 0;
        if (layer_nr >= 0)
        {
            // determine how much the skin/infill lines overlap the combing boundary
            for (const SliceMeshStorage& mesh : storage.meshes)
            {
                const coord_t overlap = std::max(mesh.settings.get<coord_t>("skin_overlap_mm"), mesh.settings.get<coord_t>("infill_overlap_mm"));
                if (overlap > dist)
                {
                    dist = overlap;
                }
            }
            dist += 100; // ensure boundary is slightly outside all skin/infill lines
        }
        boundary.add(comb_boundary_inside2.offset(dist));
        // simplify boundary to cut down processing time
        boundary.simplify(MM2INT(0.1), MM2INT(0.1));
    }
    LineOrderOptimizer orderOptimizer(near_start_location.value_or(getLastPlannedPositionOrStartingPosition()), &boundary);
    for (unsigned int line_idx = 0; line_idx < polygons.size(); line_idx++)
    {
        orderOptimizer.addPolygon(polygons[line_idx]);
    }

    if (config.type == PrintFeatureType::Skin && pattern == EFillMethod::LINES && mesh != nullptr && mesh->settings.get<bool>("monotonic_skin_lines"))
    {
        orderOptimizer.monotonicallyOrder(config.getLineWidth());
    }
    else
    {
        orderOptimizer.optimize();
    }

    coord_t min_avoid_len = 0;
    coord_t max_avoid_len = 0;
    if (avoid_freq != 0)
    {
        // to avoid resonance, all lines whose length is between min_avoid_len and max_avoid_len are printed slower
        const float avoid_len = MM2INT(config.getSpeed()) / avoid_freq / 2;
        min_avoid_len = avoid_len / 1.2f;
        max_avoid_len = avoid_len * 1.2f;
    }

    float gradient_infill_last_flow = 1;

    float max_flow_boost = 1.0;

    if(config.type == PrintFeatureType::Skin && mesh != nullptr && mesh->settings.get<bool>("skin_lines_boost_flow"))
    {
        max_flow_boost = mesh->settings.get<Ratio>((layer_nr == 0) ? "skin_lines_max_flow_boost_0" : "skin_lines_max_flow_boost");
    }

    Point last_position;
    for (unsigned int order_idx = 0; order_idx < orderOptimizer.polyOrder.size(); order_idx++)
    {
        const unsigned int poly_idx = orderOptimizer.polyOrder[order_idx];
        ConstPolygonRef polygon = polygons[poly_idx];
        const size_t start = orderOptimizer.polyStart[poly_idx];
        const size_t end = 1 - start;
        const Point& p0 = polygon[start];
        const Point& p1 = polygon[end];
        // ignore line segments that are less than 5uM long
        if(vSize2(p1 - p0) < MINIMUM_SQUARED_LINE_LENGTH)
        {
            continue;
        }
        coord_t travel_len = 0;
        coord_t unretracted_travel_len = 0;
        // travel to start point if at least MINIMUM_LINE_LENGTH away from the current position
        if (order_idx == 0 || vSize2(p0 - last_position) >= MINIMUM_SQUARED_LINE_LENGTH)
        {
            // try to avoid using combing when printing lines skin pattern
            const coord_t min_comb_distance = (config.type == PrintFeatureType::Skin) ?  config.getLineWidth() * 3 : 0;
            const GCodePath& travel_path = addTravel(p0, false, min_comb_distance);
            travel_len = vSize(p0 - last_position);
            if (!travel_path.retract)
            {
                unretracted_travel_len = travel_len;
            }
        }
        double speed_factor = 1.0;
        if (avoid_freq != 0)
        {
            // use the combined length of the line and the preceding travel move (if any)
            // I don't think it matters that the travel speed is going to be greater than the
            // print speed because the distances of interest are short and so it is unlikely that the
            // travel speed (or even the print speed) will be achieved
            coord_t len = vSize(p1 - p0) + travel_len;
            if (len >= min_avoid_len && len <= max_avoid_len)
            {
                // ensure the speed factor does not produce a speed less than the minimum speed allowed
                const Velocity min_speed = fan_speed_layer_time_settings_per_extruder[getExtruder()].cool_min_speed;
                speed_factor = std::max((double)min_avoid_len / len, (double)(min_speed / config.getSpeed()));
            }
        }
        if(config.type == PrintFeatureType::Infill && mesh != nullptr && mesh->settings.get<EGradientInfillType>("gradient_infill_type") != EGradientInfillType::NONE)
        {
            addGradientInfillLine(p0, p1, flow_ratio, config, space_fill_type, speed_factor, fan_speed, mesh, gradient_infill_last_flow);
        }
        else
        {
            float flow_ratio_here = flow_ratio;

            if (unretracted_travel_len > 0 && max_flow_boost > 1.0f)
            {
                const coord_t max_compensated_travel_len = config.getLineWidth() * 3;
                const coord_t travel_len = std::min(unretracted_travel_len, max_compensated_travel_len);
                const coord_t len = vSize(p1 - p0);
                flow_ratio_here *= std::min(max_flow_boost, (float)(len + travel_len) / len);
            }
            addExtrusionMove(p1, config, space_fill_type, flow_ratio_here, false, speed_factor, fan_speed);
        }
        last_position = p1;

        // Wipe
        if (wipe_dist != 0)
        {
            bool wipe = true;
            int line_width = config.getLineWidth();

            // Don't wipe is current extrusion is too small
            if (vSize2(p1 - p0) <= line_width * line_width * 4)
            {
                wipe = false;
            }

            // Don't wipe if next starting point is very near
            if (wipe && (order_idx < orderOptimizer.polyOrder.size() - 1))
            {
                const unsigned int next_poly_idx = orderOptimizer.polyOrder[order_idx + 1];
                ConstPolygonRef next_polygon = polygons[next_poly_idx];
                const size_t next_start = orderOptimizer.polyStart[next_poly_idx];
                const Point& next_p0 = next_polygon[next_start];
                if (vSize2(next_p0 - p1) <= line_width * line_width * 4)
                {
                    wipe = false;
                }
            }

            if (wipe)
            {
                addExtrusionMove(p1 + normal(p1-p0, wipe_dist), config, space_fill_type, 0.0, false, speed_factor, fan_speed);
            }
        }
    }
}

void LayerPlan::spiralizeWallSlice(const GCodePathConfig& config, ConstPolygonRef wall, ConstPolygonRef last_wall, const int seam_vertex_idx, const int last_seam_vertex_idx, const bool is_top_layer, const bool is_bottom_layer, const std::vector<float>& flows, const std::vector<Point>& shifts)
{
    const bool smooth_contours = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("smooth_spiralized_contours");

    // once we are into the spiral we always start at the end point of the last layer (if any)
    // NOTE: this travel move is now ignored when the g-code is being generated but we can't actually remove it
    const Point origin = (last_seam_vertex_idx >= 0 && !is_bottom_layer) ? last_wall[last_seam_vertex_idx] : wall[seam_vertex_idx];
    addTravel_simple(origin);

    if (!smooth_contours && last_seam_vertex_idx >= 0) {
        // when not smoothing, we get to the (unchanged) outline for this layer as quickly as possible so that the remainder of the
        // outline wall has the correct direction - although this creates a little step, the end result is generally better because when the first
        // outline wall has the wrong direction (due to it starting from the finish point of the last layer) the visual effect is very noticeable
        const Point p0 = wall[seam_vertex_idx] + shifts[seam_vertex_idx];
        const Point p1 = wall[(seam_vertex_idx + 1) % wall.size()] + shifts[(seam_vertex_idx + 1) % wall.size()];
        const Point join_first_wall_at = LinearAlg2D::getClosestOnLineSegment(origin + shifts[seam_vertex_idx], p0, p1);
        if (vSize(join_first_wall_at - (origin + shifts[seam_vertex_idx])) > 10)
        {
            addExtrusionMove(join_first_wall_at, config, SpaceFillType::Polygons, flows[seam_vertex_idx], true);
        }
    }

    const int n_points = wall.size();
    Polygons last_wall_polygons;
    last_wall_polygons.add(last_wall);
    const int max_dist2 = config.getLineWidth() * config.getLineWidth() * 4; // (2 * lineWidth)^2;

    double total_length = 0.0; // determine the length of the complete wall
    Point p0 = origin;
    for (int wall_point_idx = 1; wall_point_idx <= n_points; ++wall_point_idx)
    {
        const Point& p1 = wall[(seam_vertex_idx + wall_point_idx) % n_points];
        total_length += vSizeMM(p1 - p0);
        p0 = p1;
    }

    if (total_length == 0.0)
    {
        // nothing to do
        return;
    }

    // if this is the bottom layer, avoid creating a big elephants foot by starting with a reduced flow and increasing the flow
    // so that by the time the end of the first spiral is complete the flow is 100% - note that immediately before the spiral
    // is output, the extruder will be printing a normal wall line and so will be fully pressurised so that will tend to keep the
    // flow going

    // if this is the top layer, avoid an abrupt end by printing the same outline again but this time taper the spiral by reducing
    // the flow whilst keeping the same height - once the flow is down to a minimum allowed value, coast a little further

    const double min_bottom_layer_flow = 0.25; // start the bottom spiral at this flow rate
    const double min_top_layer_flow = 0.25; // lowest allowed flow while tapering the last spiral

    double speed_factor = 1; // may be reduced when printing the top layer so as to avoid a jump in extrusion rate as the layer starts

    if (is_top_layer)
    {
        // HACK ALERT - the last layer is approx 50% longer than the previous layer so it should take longer to print but the
        // normal slow down for quick layers mechanism can kick in and speed this layer up (because it is longer) but we prefer
        // the layer to be printed at a similar speed to the previous layer to avoid abrupt changes in extrusion rate so we slow it down

        const FanSpeedLayerTimeSettings& layer_time_settings = extruder_plans.back().fan_speed_layer_time_settings;
        const double min_time = layer_time_settings.cool_min_layer_time;
        const double normal_layer_time = total_length / config.getSpeed();

        // would this layer's speed normally get reduced to satisfy the min layer time?
        if (normal_layer_time < min_time)
        {
            // yes, so the extended version will not get slowed down so much and we want to compensate for that
            const double extended_layer_time = (total_length * (2 - min_top_layer_flow)) / config.getSpeed();

            // modify the speed factor to cancel out the speed increase that would normally happen due to the longer layer time
            speed_factor = normal_layer_time / std::min(extended_layer_time, min_time);
        }
    }

    // extrude to the points following the seam vertex
    // the last point is the seam vertex as the polygon is a loop
    double wall_length = 0.0;
    p0 = origin;
    for (int wall_point_idx = 1; wall_point_idx <= n_points; ++wall_point_idx)
    {
        // p is a point from the current wall polygon
        const size_t idx = (seam_vertex_idx + wall_point_idx) % n_points;
        const Point& p = wall[idx];
        wall_length += vSizeMM(p - p0);
        p0 = p;

        const double flow = (is_bottom_layer) ? (min_bottom_layer_flow + ((std::max((double)flows[idx], min_bottom_layer_flow) - min_bottom_layer_flow) * wall_length / total_length)) : flows[idx];

        // if required, use interpolation to smooth the x/y coordinates between layers but not for the first spiralized layer
        // as that lies directly on top of a non-spiralized wall with exactly the same outline and not for the last point in each layer
        // because we want that to be numerically exactly the same as the starting point on the next layer (not subject to any rounding)
        if (smooth_contours && !is_bottom_layer && wall_point_idx < n_points)
        {
            // now find the point on the last wall that is closest to p
            ClosestPolygonPoint cpp = PolygonUtils::findClosest(p, last_wall_polygons);

            // if we found a point and it's not further away than max_dist2, use it
            if (cpp.isValid() && vSize2(cpp.location - p) <= max_dist2)
            {
                // interpolate between cpp.location and p depending on how far we have progressed along wall
                addExtrusionMove(shifts[idx] + cpp.location + (p - cpp.location) * (wall_length / total_length), config, SpaceFillType::Polygons, flow, true, speed_factor);
            }
            else
            {
                // no point in the last wall was found close enough to the current wall point so don't interpolate
                addExtrusionMove(shifts[idx] + p, config, SpaceFillType::Polygons, flow, true, speed_factor);
            }
        }
        else
        {
            // no smoothing, use point verbatim
            addExtrusionMove(shifts[idx] + p, config, SpaceFillType::Polygons, flow, true, speed_factor);
        }
    }

    if (is_top_layer)
    {
        // add the tapering spiral
        const double min_spiral_coast_dist = 10; // mm
        double distance_coasted = 0;
        wall_length = 0;
        for (int wall_point_idx = 1; wall_point_idx <= n_points && distance_coasted < min_spiral_coast_dist; wall_point_idx++)
        {
            const size_t idx = (seam_vertex_idx + wall_point_idx) % n_points;
            const Point& p = wall[idx];
            const double seg_length = vSizeMM(p - p0);
            wall_length += seg_length;
            p0 = p;
            // flow is reduced in step with the distance travelled so the wall width should remain roughly constant
            double flow = flows[idx] * (1 - (wall_length / total_length));
            if (flow < min_top_layer_flow)
            {
                flow = 0;
                distance_coasted += seg_length;
            }
            // reduce number of paths created when polygon has many points by limiting precision of flow
            addExtrusionMove(shifts[idx] + p, config, SpaceFillType::Polygons, ((int)(flow * 20)) / 20.0, false, speed_factor);
        }
    }
}

void ExtruderPlan::forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrudeTime)
{
    double totalTime = travelTime + extrudeTime; 
    if (totalTime < minTime && extrudeTime > 0.0)
    {
        double minExtrudeTime = minTime - travelTime;
        if (minExtrudeTime < 1)
            minExtrudeTime = 1;
        double factor = extrudeTime / minExtrudeTime;
        for (GCodePath& path : paths)
        {
            if (path.isTravelPath())
                continue;
            double speed = path.config->getSpeed() * path.speed_factor * factor;
            if (speed < minimalSpeed)
                factor = minimalSpeed / (path.config->getSpeed() * path.speed_factor);
        }

        //Only slow down for the minimal time if that will be slower.
        assert(getExtrudeSpeedFactor() == 1.0); // The extrude speed factor is assumed not to be changed yet
        if (factor < 1.0)
        {
            setExtrudeSpeedFactor(factor);
        }
        else 
        {
            factor = 1.0;
        }
        
        double inv_factor = 1.0 / factor; // cause multiplication is faster than division
        
        // Adjust stored naive time estimates
        estimates.extrude_time *= inv_factor;
        for (GCodePath& path : paths)
        {
            path.estimates.extrude_time *= inv_factor;
        }

        if (minTime - (extrudeTime * inv_factor) - travelTime > 0.1)
        {
            extraTime = minTime - (extrudeTime * inv_factor) - travelTime;
        }
        totalPrintTime = (extrudeTime * inv_factor) + travelTime;
    }
}
TimeMaterialEstimates ExtruderPlan::computeNaiveTimeEstimates(Point starting_position)
{
    Point p0 = starting_position;

    bool was_retracted = false; // wrong assumption; won't matter that much. (TODO)
    for (GCodePath& path : paths)
    {
        bool is_extrusion_path = false;
        double* path_time_estimate;
        double& material_estimate = path.estimates.material;
        if (!path.isTravelPath())
        {
            is_extrusion_path = true;
            path_time_estimate = &path.estimates.extrude_time;
        }
        else 
        {
            if (path.retract)
            {
                path_time_estimate = &path.estimates.retracted_travel_time;
            }
            else 
            {
                path_time_estimate = &path.estimates.unretracted_travel_time;
            }
            if (path.retract != was_retracted)
            { // handle retraction times
                double retract_unretract_time;
                if (path.retract)
                {
                    retract_unretract_time = retraction_config.distance / retraction_config.speed;
                }
                else 
                {
                    retract_unretract_time = retraction_config.distance / retraction_config.primeSpeed;
                }
                path.estimates.retracted_travel_time += 0.5 * retract_unretract_time;
                path.estimates.unretracted_travel_time += 0.5 * retract_unretract_time;
            }
        }
        for(Point& p1 : path.points)
        {
            double length = vSizeMM(p0 - p1);
            if (is_extrusion_path)
            {
                material_estimate += length * INT2MM(layer_thickness) * INT2MM(path.config->getLineWidth());
            }
            double thisTime = length / (path.config->getSpeed() * path.speed_factor);
            *path_time_estimate += thisTime;
            p0 = p1;
        }
        estimates += path.estimates;
    }
    return estimates;
}

void ExtruderPlan::processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, Point starting_position)
{
    computeNaiveTimeEstimates(starting_position);
    totalPrintTime = estimates.getTotalTime();
    if (force_minimal_layer_time)
    {
        forceMinimalLayerTime(fan_speed_layer_time_settings.cool_min_layer_time, fan_speed_layer_time_settings.cool_min_speed, estimates.getTravelTime(), estimates.getExtrudeTime());
    }

    /*
                   min layer time
                   :
                   :  min layer time fan speed min
                |  :  :
      ^    max..|__:  :
                |  \  :
     fan        |   \ :
    speed  min..|... \:___________
                |________________
                  layer time >


    */
    // interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
    if (layer_nr < fan_speed_layer_time_settings.cool_fan_min_layer)
    {
        // fan is normally off for layers below cool_fan_min_layer
        fan_speed = 0;
    }
    else
    {
        // once cool_fan_min_layer has been reached, the fan runs no slower than cool_fan_speed_min
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
    }
    const double totalLayerTime = totalPrintTime;
    if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time)
    {
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max;
    }
    else if (fan_speed_layer_time_settings.cool_min_layer_time >= fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
    {
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
    }
    else if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max &&
             fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max > fan_speed_layer_time_settings.cool_min_layer_time)
    { 
        // when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
        double fan_speed_diff = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_layer_time_settings.cool_fan_speed_min;
        double layer_time_diff = fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max - fan_speed_layer_time_settings.cool_min_layer_time;
        double fraction_of_slope = (totalLayerTime - fan_speed_layer_time_settings.cool_min_layer_time) / layer_time_diff;
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_diff * fraction_of_slope;
    }
    /*
    Supposing no influence of minimal layer time;
    i.e. layer time > min layer time fan speed min:


              max..       fan 'full' on layer
                   |      :
                   |      :
      ^       min..|    ..:________________
     fan           |      /
    speed          |     /
          speed_0..|    /
                   |    |
                0__|____|_________________
                        :         layer nr >
                  fan 'min' layer
    */
    if (layer_nr < fan_speed_layer_time_settings.cool_fan_full_layer
        && layer_nr >= fan_speed_layer_time_settings.cool_fan_min_layer
        && (!force_minimal_layer_time || totalLayerTime > fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
        && fan_speed_layer_time_settings.cool_fan_full_layer > 0 // don't apply initial layer fan speed speedup if disabled.
        && !is_raft_layer // don't apply initial layer fan speed speedup to raft, but to model layers
    )
    {
        // Slow down the fan on the layers below the [cool_fan_full_layer], where layer cool_fan_min_layer is speed cool_fan_speed_0.
        fan_speed = fan_speed_layer_time_settings.cool_fan_speed_0 + (fan_speed - fan_speed_layer_time_settings.cool_fan_speed_0) * std::max(LayerIndex(0), layer_nr - fan_speed_layer_time_settings.cool_fan_min_layer) / (fan_speed_layer_time_settings.cool_fan_full_layer - fan_speed_layer_time_settings.cool_fan_min_layer);
    }
}

void LayerPlan::processFanSpeedAndMinimalLayerTime(Point starting_position)
{
    for (unsigned int extr_plan_idx = 0; extr_plan_idx < extruder_plans.size(); extr_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extr_plan_idx];
        bool force_minimal_layer_time = extr_plan_idx == extruder_plans.size() - 1;
        extruder_plan.processFanSpeedAndMinimalLayerTime(force_minimal_layer_time, starting_position);
        if (!extruder_plan.paths.empty() && !extruder_plan.paths.back().points.empty())
        {
            starting_position = extruder_plan.paths.back().points.back();
        }
    }
}



void LayerPlan::writeGCode(GCodeExport& gcode)
{
    Communication* communication = Application::getInstance().communication;
    communication->setLayerForSend(layer_nr);
    communication->sendCurrentPosition(gcode.getPositionXY());
    gcode.setLayerNr(layer_nr);
    
    gcode.writeLayerComment(layer_nr);

    // flow-rate compensation
    const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
    gcode.setFlowRateExtrusionSettings(mesh_group_settings.get<double>("flow_rate_max_extrusion_offset"), mesh_group_settings.get<Ratio>("flow_rate_extrusion_offset_factor")); //Offset is in mm.

    static LayerIndex layer_1 {1 - static_cast<LayerIndex>(Raft::getTotalExtraLayers())};
    if (layer_nr == layer_1 && mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature") != mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0"))
    {
        constexpr bool wait = false;
        gcode.writeBedTemperatureCommand(mesh_group_settings.get<Temperature>("material_bed_temperature"), wait);
    }

    gcode.setZ(z);

    const GCodePathConfig* last_extrusion_config = nullptr; // used to check whether we need to insert a TYPE comment in the gcode.

    size_t extruder_nr = gcode.getExtruderNr();
    const bool acceleration_enabled = mesh_group_settings.get<bool>("acceleration_enabled");
    const bool jerk_enabled = mesh_group_settings.get<bool>("jerk_enabled");
    std::string current_mesh = "NONMESH";
    size_t next_prime_tower_path = 0;

    for(size_t extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
    {
        ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
        const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_plan.extruder_nr];
        coord_t z_hop_height = retraction_config.zHop;
        const bool extruder_switched = (extruder_nr != extruder_plan.extruder_nr);

        if (extruder_switched)
        {
            int prev_extruder = extruder_nr;
            extruder_nr = extruder_plan.extruder_nr;

            last_extrusion_config = nullptr;

            gcode.ResetLastEValueAfterWipe(prev_extruder);

            const ExtruderTrain& prev_extruder_train = Application::getInstance().current_slice->scene.extruders[prev_extruder];
            if (prev_extruder_train.settings.get<bool>("retraction_hop_after_extruder_switch"))
            {
                z_hop_height = storage.extruder_switch_retraction_config_per_extruder[prev_extruder].zHop;
                gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder], z_hop_height);
            }
            else
            {
                gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder]);
            }

            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

            { // require printing temperature to be met
                constexpr bool wait = true;
                gcode.writeTemperatureCommand(extruder_nr, extruder_plan.required_start_temperature, wait);
            }

            if (extruder_plan.prev_extruder_standby_temp)
            { // turn off previous extruder
                constexpr bool wait = false;
                Temperature prev_extruder_temp = *extruder_plan.prev_extruder_standby_temp;
                const LayerIndex prev_layer_nr = (extruder_plan_idx == 0) ? layer_nr - 1 : layer_nr;
                if (prev_layer_nr == storage.max_print_height_per_extruder[prev_extruder])
                {
                    prev_extruder_temp = 0; // TODO ? should there be a setting for extruder_off_temperature ?
                }
                gcode.writeTemperatureCommand(prev_extruder, prev_extruder_temp, wait);
            }

            const double extra_prime_amount = extruder.settings.get<bool>("retraction_enable") ? extruder.settings.get<double>("switch_extruder_extra_prime_amount") : 0;
            gcode.addExtraPrimeAmount(extra_prime_amount);
        }
        else if (extruder_plan_idx == 0)
        {
            const WipeScriptConfig& wipe_config = storage.wipe_config_per_extruder[extruder_plan.extruder_nr];
            if (wipe_config.clean_between_layers && gcode.getExtrudedVolumeAfterLastWipe(extruder_nr) > wipe_config.max_extrusion_mm3)
            {
                gcode.insertWipeScript(wipe_config);
                gcode.ResetLastEValueAfterWipe(extruder_nr);
            }
            else if (layer_nr != 0 && Application::getInstance().current_slice->scene.extruders[extruder_nr].settings.get<bool>("retract_at_layer_change"))
            {
                // only do the retract if the paths are not spiralized
                if (!mesh_group_settings.get<bool>("magic_spiralize"))
                {
                    gcode.writeRetraction(retraction_config);
                }
            }
        }
        gcode.writeFanCommand(extruder_plan.getFanSpeed());
        std::vector<GCodePath>& paths = extruder_plan.paths;

        extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool
            {
                return  a.path_idx < b.path_idx; 
            });

        const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

        bool update_extrusion_offset = true;

        std::vector<std::pair<double, double>> fan_override_events;

        const Duration max_cool_fan_lag = extruder.settings.get<Duration>("cool_fan_lag"); // time taken for fan speed to change from 0 to 100%

        const double plan_fan_speed = extruder_plan.getFanSpeed();

        if (max_cool_fan_lag > 0)
        {
            double elapsed_time = 0;

            double current_fan_speed = plan_fan_speed;

            for (const GCodePath& path : extruder_plan.paths)
            {
                if (!path.config->isTravelPath())
                {
                    const double path_fan_speed = path.getFanSpeed();

                    if (path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT)
                    {
                        if (path_fan_speed != current_fan_speed)
                        {
                            double lag = max_cool_fan_lag * std::abs(path_fan_speed - current_fan_speed) / 100;
                            if (lag >= 0.1)
                            {
                                fan_override_events.emplace_back(elapsed_time, path_fan_speed);
                                current_fan_speed = path_fan_speed;
                            }
                        }
                    }
                    else
                    {
                        // add event to revert fan speed to default
                        if (current_fan_speed != plan_fan_speed)
                        {
                            fan_override_events.emplace_back(elapsed_time, plan_fan_speed);
                        }
                        current_fan_speed = plan_fan_speed;
                    }
                }
                double path_time = path.estimates.getTotalTime() / path.speed_factor;
                if(!path.config->isTravelPath())
                {
                    path_time /= extruder_plan.getExtrudeSpeedFactor();
                }
                elapsed_time += path_time;
            }
            // if last path overrode fan speed, add event to revert speed back to default
            // so that next layer will start with the right fan speed
            if (current_fan_speed != plan_fan_speed)
            {
                fan_override_events.emplace_back(elapsed_time, plan_fan_speed);
            }
        }

        double current_fan_speed = plan_fan_speed;

        bool fan_speed_hold = false; // when true, supress normal fan speed updates

        double elapsed_time = 0;

        double prime_tower_total_volume = 0; // the volume of all of the prime tower paths
        double prime_tower_used_volume = 0;  // the volume of the prime tower paths used so far
        double prime_tower_min_volume = extruder.settings.get<double>("prime_tower_min_volume");
        double prime_tower_max_volume = extruder.settings.get<double>("extruder_min_volume");
        bool prime_tower_compact = extruder.settings.get<bool>("prime_tower_compact");
        bool prime_tower_coasting = false; // true when ignoring prime tower lines
        bool suppress_accel_jerk = false;
        size_t prime_tower_paths_seen = 0;

        for(unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
        {
            extruder_plan.handleInserts(path_idx, gcode);
            
            GCodePath& path = paths[path_idx];

            if (fan_override_events.size() && elapsed_time > fan_override_events[0].first)
            {
                // now past the fan event time, remove the event and allow fan speed updates
                fan_override_events.erase(fan_override_events.begin());
                fan_speed_hold = false;
            }

            double fan_speed_override_at = -1; // when >= 0, this is the time offset within path when fan speed override should occur

            if (fan_override_events.size())
            {
                double path_time = path.estimates.getTotalTime() / path.speed_factor;
                if(!path.config->isTravelPath())
                {
                    path_time /= extruder_plan.getExtrudeSpeedFactor();
                }

                double fan_lag = max_cool_fan_lag * std::abs(fan_override_events[0].second - current_fan_speed) / 100;

                const double fan_speed_override_time = fan_override_events[0].first - fan_lag;

                const double path_end_time = elapsed_time + path_time;

                if (!path.config->isTravelPath() && path_end_time >= fan_speed_override_time)
                {
                    // path ends within the spool up/down period of the next fan event so schedule fan speed override
                    fan_speed_override_at = std::max(fan_speed_override_time - elapsed_time, 0.0);
                }

                elapsed_time = path_end_time;
            }

            if (path.perform_prime)
            {
                gcode.writePrimeTrain(extruder.settings.get<Velocity>("speed_travel"));
                gcode.writeRetraction(retraction_config);
            }

            if (!path.retract && path.config->isTravelPath() && path.points.size() == 1 && path.points[0] == gcode.getPositionXY() && z == gcode.getPositionZ())
            {
                // ignore travel moves to the current location to avoid needless change of acceleration/jerk
                continue;
            }

            if (prime_tower_coasting && path.config->isTravelPath())
            {
                // ignore unwanted travel between prime tower lines that won't be printed
                continue;
            }

            if (acceleration_enabled && !suppress_accel_jerk)
            {
                if (path.config->isTravelPath())
                {
                    gcode.writeTravelAcceleration(path.config->getAcceleration());
                }
                else
                {
                    gcode.writePrintAcceleration(path.config->getAcceleration());
                }
            }
            if (jerk_enabled && !suppress_accel_jerk)
            {
                gcode.writeJerk(path.config->getJerk());
            }

            if (path.retract)
            {
                gcode.writeRetraction(retraction_config);
                if (path.perform_z_hop)
                {
                    gcode.writeZhopStart(z_hop_height);
                    z_hop_height = retraction_config.zHop; // back to normal z hop
                }
                else
                {
                    gcode.writeZhopEnd();
                }
            }
            if (!path.config->isTravelPath() && last_extrusion_config != path.config)
            {
                if (path.config->type == PrintFeatureType::PrimeTower)
                {
                    if (layer_nr == 0)
                    {
                        // need to fill prime tower inside using the last extruder to be primed
                        if (extruder_plan_idx == (extruder_plans.size() - 1) || !extruder.settings.get<bool>("prime_all_extruders_on_layer_0"))
                        {
                            prime_tower_min_volume = std::numeric_limits<double>::max();
                        }
                    }
                    // if extruder_min_volume is greater than prime_tower_min_volume, see if prime_tower_min_volume needs to be increased
                    const double extruder_min_volume = extruder.settings.get<double>("extruder_min_volume");
                    std::vector<double> amounts;
                    double total_volume = extruder_plan.getMaterial(&amounts);
                    prime_tower_max_volume = amounts[(unsigned)PrintFeatureType::PrimeTower]; // volume required for all the lines in the prime tower for this extruder
                    if (extruder_min_volume > prime_tower_min_volume)
                    {
                        double model_volume = total_volume - prime_tower_max_volume; // volume required for everything other than the prime tower
                        // if extruder_min_volume is > (model_volume + prime_tower_min_volume), the prime tower will need to soak up the extra
                        prime_tower_min_volume = std::max(extruder_min_volume - model_volume, prime_tower_min_volume);
                    }
                }
                gcode.writeTypeComment(path.config->type);
                if (path.config->isBridgePath())
                {
                    gcode.writeComment("BRIDGE");
                }
                last_extrusion_config = path.config;
                update_extrusion_offset = true;
            }
            else
            {
                update_extrusion_offset = false;
            }

            double speed = path.config->getSpeed();

            // for some movements such as prime tower purge, the speed may get changed by this factor
            speed *= path.speed_factor;

            //Apply the extrusion speed factor if it's an extrusion move.
            if (!path.config->isTravelPath())
            {
                speed *= extruder_plan.getExtrudeSpeedFactor();
            }
            //This seems to be the best location to place this, but still not ideal.
            if (path.mesh_id != current_mesh)
            {
                current_mesh = path.mesh_id;
                std::stringstream ss;
                ss << "MESH:" << current_mesh;
                gcode.writeComment(ss.str());
            }
            if (path.config->isTravelPath())
            { // early comp for travel paths, which are handled more simply
                const bool is_final_travel = (extruder_plan_idx == (extruder_plans.size() - 1) && path_idx == (paths.size() - 1));
                if (is_final_travel)
                {
                    if (path_idx > 0 && paths[path_idx - 1].spiralize)
                    {
                        // this is a final travel following a spiralized wall, ignore it
                        continue;
                    }
                    if (!path.perform_z_hop && final_travel_z != z)
                    {
                        // Before the final travel, move up to the next layer height, on the current spot, with a sensible speed.
                        Point3 current_position = gcode.getPosition();
                        current_position.z = final_travel_z;
                        gcode.writeTravel(current_position, extruder.settings.get<Velocity>("speed_z_hop"));

                        // Prevent the final travel(s) from resetting to the 'previous' layer height.
                        gcode.setZ(final_travel_z);
                    }
                }
                else if (extruder_plan_idx == 0 && path_idx == 0 && paths.size() > 1 && paths[1].spiralize)
                {
                    // this is the first travel on the layer and it precedes a spiralized wall, ignore it
                    continue;
                }
                for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                {
                    gcode.writeTravel(path.points[point_idx], speed);
                }
                continue;
            }

            bool spiralize = path.spiralize;
            if (!spiralize) // normal (extrusion) move (with coasting)
            {
                bool coasting = extruder.settings.get<bool>("coasting_enable") && path.config->type != PrintFeatureType::PrimeTower;
                // if path provides a valid (in range 0-100) fan speed, use it
                const double path_fan_speed = path.getFanSpeed();
                if (coasting)
                {
                    if (fan_speed_override_at >= 0)
                    {
                        // start the override now
                        current_fan_speed = fan_override_events[0].second;
                        gcode.writeFanCommand(current_fan_speed);
                        fan_speed_override_at = -1;
                        fan_speed_hold = true;
                    }
                    if (!fan_speed_hold)
                    {
                        current_fan_speed = (path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT) ? path_fan_speed : plan_fan_speed;
                        gcode.writeFanCommand(current_fan_speed);
                    }
                    coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layer_thickness);
                }
                if (!coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
                { // normal path to gcode algorithm
                    double path_time = 0;
                    Point last_point = gcode.getPositionXY();
                    if (!fan_speed_hold)
                    {
                        current_fan_speed = (path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT) ? path_fan_speed : plan_fan_speed;
                        gcode.writeFanCommand(current_fan_speed);
                    }
                    if (path.config->type == PrintFeatureType::PrimeTower)
                    {
                        prime_tower_total_volume += path.estimates.getMaterial();
                        if (next_prime_tower_path > 0)
                        {
                            if (prime_tower_paths_seen < next_prime_tower_path)
                            {
                                ++prime_tower_paths_seen;
                                suppress_accel_jerk = true;
                                continue;
                            }
                            else if(prime_tower_paths_seen == next_prime_tower_path)
                            {
                                suppress_accel_jerk = false;
                            }
                        }

                        if ((layer_nr > 0 || extruder.settings.get<bool>("prime_all_extruders_on_layer_0")) && prime_tower_used_volume >= prime_tower_min_volume)
                        {
                            if (prime_tower_coasting)
                            {
                                if (prime_tower_total_volume >= prime_tower_max_volume)
                                {
                                    // no longer coasting
                                    prime_tower_coasting = false;

                                    // reset flag that suppresses travel + accel/jerk changes
                                    suppress_accel_jerk = false;
                                }
                            }
                            else
                            {
                                if (prime_tower_total_volume < prime_tower_max_volume)
                                {
                                    // now coasting through the remainder of the prime tower
                                    prime_tower_coasting = true;

                                    // set flag so that the travel moves and accel/jerk changes that would be output
                                    // for the remaining prime tower lines are suppressed
                                    suppress_accel_jerk = true;
                                }

                                if (prime_tower_compact || extruder.settings.get<bool>("prime_all_extruders_on_layer_0"))
                                {
                                    next_prime_tower_path = prime_tower_paths_seen;
                                }
                            }

                            ++prime_tower_paths_seen;

                            // don't need any more prime tower so ignore this path
                            continue;
                        }

                        ++prime_tower_paths_seen;
                        prime_tower_used_volume += path.estimates.getMaterial();
                    }
                    for(unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
                        if (fan_speed_override_at >= 0)
                        {
                            path_time += vSizeMM(path.points[point_idx] - last_point) / speed;
                            last_point = path.points[point_idx];
                            if (path_time >= fan_speed_override_at)
                            {
                                // time for the fan override to kick in
                                current_fan_speed = fan_override_events[0].second;
                                gcode.writeFanCommand(current_fan_speed);
                                fan_speed_override_at = -1;
                                fan_speed_hold = true;
                            }
                        }
                        communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), speed);
                        gcode.writeExtrusion(path.points[point_idx], speed, path.getExtrusionMM3perMM(), path.config->type, update_extrusion_offset);
                    }
                }
            }
            else
            { // SPIRALIZE
                //If we need to spiralize then raise the head slowly by 1 layer as this path progresses.
                float totalLength = 0.0;
                Point p0 = gcode.getPositionXY();
                for (unsigned int _path_idx = path_idx; _path_idx < paths.size() && paths[_path_idx].spiralize; _path_idx++)
                {
                    GCodePath& _path = paths[_path_idx];
                    for (unsigned int point_idx = 0; point_idx < _path.points.size(); point_idx++)
                    {
                        Point p1 = _path.points[point_idx];
                        totalLength += vSizeMM(p0 - p1);
                        p0 = p1;
                    }
                }

                float length = 0.0;
                p0 = gcode.getPositionXY();
                Point spiral_start = p0;
                for (; path_idx < paths.size() && paths[path_idx].spiralize; path_idx++)
                { // handle all consecutive spiralized paths > CHANGES path_idx!
                    GCodePath& path = paths[path_idx];

                    for (unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
                    {
                        Point p1 = path.points[point_idx];
                        length += vSizeMM(p0 - p1);
                        p0 = p1;
                        gcode.setZ(std::round(z + layer_thickness * length / totalLength));
                        communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), speed);
                        gcode.writeExtrusion(path.points[point_idx], speed, path.getExtrusionMM3perMM(), path.config->type, update_extrusion_offset);
                    }
                    // for layer display only - the loop finished at the seam vertex but as we started from
                    // the location of the previous layer's seam vertex the loop may have a gap if this layer's
                    // seam vertex is "behind" the previous layer's seam vertex. So output another line segment
                    // that joins this layer's seam vertex to the following vertex. If the layers have been blended
                    // then this can cause a visible ridge (on the screen, not on the print) because the first vertex
                    // would have been shifted in x/y to make it nearer to the previous layer outline but the seam
                    // vertex would not be shifted (as it's the last vertex in the sequence). The smoother the model,
                    // the less the vertices are shifted and the less obvious is the ridge. If the layer display
                    // really displayed a spiral rather than slices of a spiral, this would not be required.
                    if (path_idx + 1 >= paths.size() || !paths[path_idx + 1].spiralize)
                    {
                        communication->sendLineTo(path.config->type, spiral_start, path.getLineWidthForLayerView(), path.config->getLayerThickness(), speed);
                    }
                }
                path_idx--; // the last path_idx didnt spiralize, so it's not part of the current spiralize path
            }
        } // paths for this extruder /\  .

        if (extruder.settings.get<bool>("cool_lift_head") && extruder_plan.extraTime > 0.0)
        {
            gcode.writeComment("Small layer, adding delay");
            const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[gcode.getExtruderNr()];
            gcode.writeRetraction(retraction_config);
            if (extruder_plan_idx == extruder_plans.size() - 1 || !extruder.settings.get<bool>("machine_extruder_end_pos_abs"))
            { // only move the head if it's the last extruder plan; otherwise it's already at the switching bay area 
                // or do it anyway when we switch extruder in-place
                gcode.setZ(gcode.getPositionZ() + MM2INT(3.0));
                gcode.writeTravel(gcode.getPositionXY(), configs_storage.travel_config_per_extruder[extruder_nr].getSpeed());

                const Point current_pos = gcode.getPositionXY();
                const Point machine_middle = storage.machine_size.flatten().getMiddle();
                const Point toward_middle_of_bed = current_pos - normal(current_pos - machine_middle, MM2INT(20.0));
                gcode.writeTravel(toward_middle_of_bed, configs_storage.travel_config_per_extruder[extruder_nr].getSpeed());
            }
            gcode.writeDelay(extruder_plan.extraTime);
        }

        extruder_plan.handleAllRemainingInserts(gcode);
    } // extruder plans /\  .
    
    gcode.updateTotalPrintTime();
}

void LayerPlan::overrideFanSpeeds(double speed)
{
    for (ExtruderPlan& extruder_plan : extruder_plans)
    {
        extruder_plan.setFanSpeed(speed);
    }
}


bool LayerPlan::makeRetractSwitchRetract(unsigned int extruder_plan_idx, unsigned int path_idx)
{
    std::vector<GCodePath>& paths = extruder_plans[extruder_plan_idx].paths;
    for (unsigned int path_idx2 = path_idx + 1; path_idx2 < paths.size(); path_idx2++)
    {
        if (paths[path_idx2].getExtrusionMM3perMM() > 0) 
        {
            return false; 
        }
    }
    
    if (extruder_plans.size() <= extruder_plan_idx+1)
    {
        return false; // TODO: check first extruder of the next layer! (generally only on the last layer of the second extruder)
    }
        
    if (extruder_plans[extruder_plan_idx + 1].extruder_nr != extruder_plans[extruder_plan_idx].extruder_nr)
    {
        return true;
    }
    else 
    {
        return false;
    }
}
    
bool LayerPlan::writePathWithCoasting(GCodeExport& gcode, const size_t extruder_plan_idx, const size_t path_idx, const coord_t layer_thickness)
{
    ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
    const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_plan.extruder_nr];
    const double coasting_volume = extruder.settings.get<double>("coasting_volume");
    if (coasting_volume <= 0)
    { 
        return false; 
    }
    const std::vector<GCodePath>& paths = extruder_plan.paths;
    const GCodePath& path = paths[path_idx];
    if (path_idx + 1 >= paths.size()
        || (path.isTravelPath() || !paths[path_idx + 1].config->isTravelPath()) 
        || path.points.size() < 2)
    {
        return false;
    }

    coord_t coasting_min_dist_considered = MM2INT(0.1); // hardcoded setting for when to not perform coasting

    
    double extrude_speed = path.config->getSpeed() * extruder_plan.getExtrudeSpeedFactor() * path.speed_factor; // travel speed
    
    const coord_t coasting_dist = MM2INT(MM2_2INT(coasting_volume) / layer_thickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    const double coasting_min_volume = extruder.settings.get<double>("coasting_min_volume");
    const coord_t coasting_min_dist = MM2INT(MM2_2INT(coasting_min_volume + coasting_volume) / layer_thickness) / path.config->getLineWidth(); // closing brackets of MM2INT at weird places for precision issues
    //           /\ the minimal distance when coasting will coast the full coasting volume instead of linearly less with linearly smaller paths


    std::vector<coord_t> accumulated_dist_per_point; // the first accumulated dist is that of the last point! (that of the last point is always zero...)
    accumulated_dist_per_point.push_back(0);

    coord_t accumulated_dist = 0;

    bool length_is_less_than_min_dist = true;

    unsigned int acc_dist_idx_gt_coast_dist = NO_INDEX; // the index of the first point with accumulated_dist more than coasting_dist (= index into accumulated_dist_per_point)
     // == the point printed BEFORE the start point for coasting


    const Point* last = &path.points[path.points.size() - 1];
    for (unsigned int backward_point_idx = 1; backward_point_idx < path.points.size(); backward_point_idx++)
    {
        const Point& point = path.points[path.points.size() - 1 - backward_point_idx];
        const coord_t distance = vSize(point - *last);
        accumulated_dist += distance;
        accumulated_dist_per_point.push_back(accumulated_dist);
        
        if (acc_dist_idx_gt_coast_dist == NO_INDEX && accumulated_dist >= coasting_dist)
        {
            acc_dist_idx_gt_coast_dist = backward_point_idx; // the newly added point
        }
        
        if (accumulated_dist >= coasting_min_dist)
        {
            length_is_less_than_min_dist = false;
            break;
        }
        
        last = &point;
    }
    
    if (accumulated_dist < coasting_min_dist_considered)
    {
        return false;
    }
    coord_t actual_coasting_dist = coasting_dist;
    if (length_is_less_than_min_dist)
    {
        // in this case accumulated_dist is the length of the whole path
        actual_coasting_dist = accumulated_dist * coasting_dist / coasting_min_dist;
        for (acc_dist_idx_gt_coast_dist = 0 ; acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size() ; acc_dist_idx_gt_coast_dist++)
        { // search for the correct coast_dist_idx
            if (accumulated_dist_per_point[acc_dist_idx_gt_coast_dist] >= actual_coasting_dist)
            {
                break;
            }
        }
    }

    assert(acc_dist_idx_gt_coast_dist < accumulated_dist_per_point.size()); // something has gone wrong; coasting_min_dist < coasting_dist ?

    const size_t point_idx_before_start = path.points.size() - 1 - acc_dist_idx_gt_coast_dist;

    Point start;
    { // computation of begin point of coasting
        const coord_t residual_dist = actual_coasting_dist - accumulated_dist_per_point[acc_dist_idx_gt_coast_dist - 1];
        const Point& a = path.points[point_idx_before_start];
        const Point& b = path.points[point_idx_before_start + 1];
        start = b + normal(a - b, residual_dist);
    }

    { // write normal extrude path:
        Communication* communication = Application::getInstance().communication;
        for(size_t point_idx = 0; point_idx <= point_idx_before_start; point_idx++)
        {
            communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
            gcode.writeExtrusion(path.points[point_idx], extrude_speed, path.getExtrusionMM3perMM(), path.config->type);
        }
        communication->sendLineTo(path.config->type, start, path.getLineWidthForLayerView(), path.config->getLayerThickness(), extrude_speed);
        gcode.writeExtrusion(start, extrude_speed, path.getExtrusionMM3perMM(), path.config->type);
    }

    // write coasting path
    for (size_t point_idx = point_idx_before_start + 1; point_idx < path.points.size(); point_idx++)
    {
        const Ratio coasting_speed_modifier = extruder.settings.get<Ratio>("coasting_speed");
        const Velocity speed = Velocity(coasting_speed_modifier * path.config->getSpeed() * extruder_plan.getExtrudeSpeedFactor());
        gcode.writeTravel(path.points[point_idx], speed);
    }
    return true;
}

void LayerPlan::optimizePaths(const Point& starting_position)
{
    for (ExtruderPlan& extr_plan : extruder_plans)
    {
        //Merge paths whose endpoints are very close together into one line.
        MergeInfillLines merger(extr_plan);
        merger.mergeInfillLines(extr_plan.paths, starting_position);
    }
}

void LayerPlan::handleMeshTemperatureOverride(const SliceMeshStorage& mesh)
{
    // asumes that this mesh was the last mesh sliced and so its paths are the last ones in the current extruder plan
    ExtruderPlan& plan = extruder_plans.back();
    const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[plan.extruder_nr];

    if (plan.paths.size() > 1)
    {
        const Temperature plan_temperature = extruder.settings.get<Temperature>("material_print_temperature");
        double mesh_temperature_dawdle_time = mesh.settings.get<double>("mesh_temperature_dawdle_time");
        Velocity mesh_temperature_dawdle_speed = mesh.settings.get<Velocity>("mesh_temperature_dawdle_speed");

        double time_to_end = 0;
        int revert_temp_idx = -1;
        int path_idx;
        for (path_idx = plan.paths.size() - 1; path_idx >= 0; --path_idx)
        {
            GCodePath& path = plan.paths[path_idx];
            if (path.mesh_id != mesh.mesh_name)
            {
                break;
            }
            Velocity path_speed = path.config->getSpeed() * path.speed_factor;
            double path_time = INT2MM(path.length) / path_speed;
            if ((time_to_end + path_time) < mesh_temperature_dawdle_time)
            {
                if (!path.isTravelPath())
                {
                    path_time = INT2MM(path.length) / mesh_temperature_dawdle_speed;
                }
            }
            else if (revert_temp_idx < 0)
            {
                revert_temp_idx = path_idx;
            }
            time_to_end += path_time;
        }
        if (revert_temp_idx >= 0)
        {
            // path_idx is currently indexing the last path in the previous mesh
            // insert temperature override at the first path of this mesh
            plan.insertCommand(++path_idx, plan.extruder_nr, plan_temperature + mesh.settings.get<Temperature>("mesh_temperature_delta"), false);

            // modify the speed factors for the paths that follow the override command and the paths that precede the command to revert back to plan temperature
            double time_from_start = 0;
            while (path_idx < (int)plan.paths.size())
            {
                GCodePath& path = plan.paths[path_idx++];
                Velocity path_speed = path.config->getSpeed() * path.speed_factor;
                double path_time = INT2MM(path.length) / path_speed;
                if ((time_from_start + path_time) < mesh_temperature_dawdle_time || path_idx > revert_temp_idx)
                {
                    if (!path.isTravelPath())
                    {
                        path.speed_factor *= mesh_temperature_dawdle_speed / path_speed;
                        path_time = INT2MM(path.length) / mesh_temperature_dawdle_speed;
                    }
                }
                time_from_start += path_time;
            }

            // insert command to revert temperature to the normal plan temperature
            plan.insertCommand(revert_temp_idx, plan.extruder_nr, plan_temperature, false);
        }
    }
}

}//namespace cura
