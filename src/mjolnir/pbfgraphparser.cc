#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"

#include "graph_lua_proc.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmaccess.h"

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <future>
#include <thread>
#include <utility>

#include "baldr/complexrestriction.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/sequence.h"
#include "midgard/tiles.h"
#include "mjolnir/timeparsing.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// Convenience method to get a number from a string. Uses try/catch in case
// stoi throws an exception
int get_number(const std::string& tag, const std::string& value) { // NOLINT
  int num = -1;
  try {
    num = stoi(value);
  } catch (const std::invalid_argument& arg) {
    LOG_DEBUG("invalid_argument thrown for " + tag + " value: " + value);
  } catch (const std::out_of_range& oor) {
    LOG_DEBUG("out_of_range exception thrown for " + tag + " value: " + value);
  }
  return num;
}

// This class helps to set "culdesac" labels to loop roads correctly.
// How does it work?
// First needs to add loop roads (that are candidates to be a "culdesac" roads) using add_candidate
// method. After that, needs to call clarify_and_fix. It clarifies types of roads and marks roads as
// "culdesac" correctly. This call requires sets of OSMWayNode and OSMWay. clarify_and_fix must to
// call after finishing collecting these sets.
class culdesac_processor {
public:
  // Adds a loop road as a candidate to be a "culdesac" road.
  void add_candidate(uint64_t osm_way_id,
                     size_t osm_way_index,
                     const std::vector<uint64_t>& osm_node_ids) {
    loops_meta_.emplace(osm_way_id, loop_meta(osm_way_index));
    for (auto osm_node_id : osm_node_ids)
      node_to_loop_way_.emplace(osm_node_id, osm_way_id);
  }

  // Clarifies types of loop roads and saves fixed ways.
  void clarify_and_fix(sequence<OSMWayNode>& osm_way_node_seq, sequence<OSMWay>& osm_way_seq) {
    osm_way_node_seq.flush();
    osm_way_seq.flush();

    size_t number_of_nodes = 0;
    size_t count_node = 0;
    OSMWay osm_way;
    for (const auto& osm_way_node : osm_way_node_seq) {
      // Reads a new way only after its nodes are read.
      if (number_of_nodes == count_node) {
        osm_way = *osm_way_seq[osm_way_node.way_index];
        number_of_nodes = osm_way.node_count();
        count_node = 0;
      }

      const auto node_to_loop_way_range = node_to_loop_way_.equal_range(osm_way_node.node.osmid_);
      for (auto it = node_to_loop_way_range.first; it != node_to_loop_way_range.second; ++it) {
        if (osm_way.way_id() != it->second && osm_way.use() == Use::kRoad) {
          loops_meta_.at(it->second).add_id_of_intersection(osm_way_node.node.osmid_);
        }
      }

      ++count_node;
    }

    fix(osm_way_seq);
  }

private:
  // loop_meta is a helper class that stores loop info.
  class loop_meta {
  public:
    explicit loop_meta(size_t way_index) : way_index_(way_index) {
    }

    size_t get_way_index() const {
      return way_index_;
    }

    bool is_culdesac() const {
      return node_ids_of_intersections_.size() <= 1;
    }

    void add_id_of_intersection(uint64_t node_id) {
      node_ids_of_intersections_.insert(node_id);
    }

  private:
    size_t way_index_;
    // Stores nodes that are intersections of loop road loop and adjacent roads.
    std::unordered_set<uint64_t> node_ids_of_intersections_;
  };

  // Sets "culdesac" labels to loop roads and saves ways.
  void fix(sequence<OSMWay>& osm_way_seq) {
    size_t number_of_culdesac = 0;
    for (const auto& loop_way_id_to_meta : loops_meta_) {
      const auto& meta = loop_way_id_to_meta.second;
      if (meta.is_culdesac()) {
        auto way_it = osm_way_seq.at(meta.get_way_index());
        auto way = *way_it;
        way.set_use(Use::kCuldesac);
        way_it = way;
        ++number_of_culdesac;
      }
    }

    LOG_INFO("Added " + std::to_string(number_of_culdesac) + " culdesac roundabouts from " +
             std::to_string(loops_meta_.size()) + " candidates.");
  }

  std::unordered_multimap<uint64_t, uint64_t> node_to_loop_way_;
  std::unordered_map<uint64_t, loop_meta> loops_meta_;
};

// Construct PBFGraphParser based on properties file and input PBF extract
struct graph_callback : public OSMPBF::Callback {
public:
  graph_callback() = delete;
  graph_callback(const graph_callback&) = delete;
  virtual ~graph_callback() {
  }

  graph_callback(const boost::property_tree::ptree& pt, OSMData& osmdata)
      : lua_(get_lua(pt)), osmdata_(osmdata) {
    current_way_node_index_ = last_node_ = last_way_ = last_relation_ = 0;

    highway_cutoff_rc_ = RoadClass::kPrimary;
    for (auto& level : TileHierarchy::levels()) {
      if (level.name == "highway") {
        highway_cutoff_rc_ = level.importance;
      }
    }

    include_driveways_ = pt.get<bool>("include_driveways", true);
    infer_internal_intersections_ =
        pt.get<bool>("data_processing.infer_internal_intersections", true);
    infer_turn_channels_ = pt.get<bool>("data_processing.infer_turn_channels", true);
    use_direction_on_ways_ = pt.get<bool>("data_processing.use_direction_on_ways", false);
    allow_alt_name_ = pt.get<bool>("data_processing.allow_alt_name", false);
    use_urban_tag_ = pt.get<bool>("data_processing.use_urban_tag", false);
    use_rest_area_ = pt.get<bool>("data_processing.use_rest_area", false);
    use_admin_db_ = pt.get<bool>("data_processing.use_admin_db", true);

    empty_node_results_ = lua_.Transform(OSMType::kNode, 0, {});
    empty_way_results_ = lua_.Transform(OSMType::kWay, 0, {});
    empty_relation_results_ = lua_.Transform(OSMType::kRelation, 0, {});

    tag_handlers_["driving_side"] = [this]() {
      if (!use_admin_db_) {
        way_.set_drive_on_right(tag_.second == "right" ? true : false);
      }
    };
    tag_handlers_["internal_intersection"] = [this]() {
      if (!infer_internal_intersections_) {
        way_.set_internal(tag_.second == "true" ? true : false);
      }
    };
    tag_handlers_["turn_channel"] = [this]() {
      if (!infer_turn_channels_) {
        way_.set_turn_channel(tag_.second == "true" ? true : false);
      }
    };

    tag_handlers_["layer"] = [this]() {
      auto layer = static_cast<int8_t>(std::stoi(tag_.second));
      way_.set_layer(layer);
    };

    tag_handlers_["road_class"] = [this]() {
      RoadClass roadclass = (RoadClass)std::stoi(tag_.second);
      switch (roadclass) {

        case RoadClass::kMotorway:
          way_.set_road_class(RoadClass::kMotorway);
          break;
        case RoadClass::kTrunk:
          way_.set_road_class(RoadClass::kTrunk);
          break;
        case RoadClass::kPrimary:
          way_.set_road_class(RoadClass::kPrimary);
          break;
        case RoadClass::kSecondary:
          way_.set_road_class(RoadClass::kSecondary);
          break;
        case RoadClass::kTertiary:
          way_.set_road_class(RoadClass::kTertiary);
          break;
        case RoadClass::kUnclassified:
          way_.set_road_class(RoadClass::kUnclassified);
          break;
        case RoadClass::kResidential:
          way_.set_road_class(RoadClass::kResidential);
          break;
        default:
          way_.set_road_class(RoadClass::kServiceOther);
          break;
      }
    };
    // these flags indicate if a user set the access tags on this way.
    tag_handlers_["auto_tag"] = [this]() {
      osm_access_.set_auto_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["truck_tag"] = [this]() {
      osm_access_.set_truck_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["bus_tag"] = [this]() {
      osm_access_.set_bus_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["foot_tag"] = [this]() {
      osm_access_.set_foot_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["bike_tag"] = [this]() {
      osm_access_.set_bike_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["moped_tag"] = [this]() {
      osm_access_.set_moped_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["motorcycle_tag"] = [this]() {
      osm_access_.set_motorcycle_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["hov_tag"] = [this]() {
      osm_access_.set_hov_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["taxi_tag"] = [this]() {
      osm_access_.set_taxi_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["motorroad_tag"] = [this]() {
      osm_access_.set_motorroad_tag(true);
      has_user_tags_ = true;
    };
    tag_handlers_["wheelchair"] = [this]() {
      way_.set_wheelchair_tag(true);
      way_.set_wheelchair(tag_.second == "true" ? true : false);
    };
    tag_handlers_["sidewalk"] = [this]() {
      if (tag_.second == "both" || tag_.second == "yes" || tag_.second == "shared" ||
          tag_.second == "raised") {
        way_.set_sidewalk_left(true);
        way_.set_sidewalk_right(true);
      } else if (tag_.second == "left") {
        way_.set_sidewalk_left(true);
      } else if (tag_.second == "right") {
        way_.set_sidewalk_right(true);
      }
    };
    tag_handlers_["auto_forward"] = [this]() {
      way_.set_auto_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["truck_forward"] = [this]() {
      way_.set_truck_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["bus_forward"] = [this]() {
      way_.set_bus_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["bike_forward"] = [this]() {
      way_.set_bike_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["emergency_forward"] = [this]() {
      way_.set_emergency_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["hov_forward"] = [this]() {
      way_.set_hov_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["taxi_forward"] = [this]() {
      way_.set_taxi_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["moped_forward"] = [this]() {
      way_.set_moped_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["motorcycle_forward"] = [this]() {
      way_.set_motorcycle_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["pedestrian_forward"] = [this]() {
      way_.set_pedestrian_forward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["auto_backward"] = [this]() {
      way_.set_auto_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["truck_backward"] = [this]() {
      way_.set_truck_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["bus_backward"] = [this]() {
      way_.set_bus_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["bike_backward"] = [this]() {
      way_.set_bike_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["emergency_backward"] = [this]() {
      way_.set_emergency_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["hov_backward"] = [this]() {
      way_.set_hov_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["taxi_backward"] = [this]() {
      way_.set_taxi_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["moped_backward"] = [this]() {
      way_.set_moped_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["motorcycle_backward"] = [this]() {
      way_.set_motorcycle_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["pedestrian_backward"] = [this]() {
      way_.set_pedestrian_backward(tag_.second == "true" ? true : false);
    };
    tag_handlers_["private"] = [this]() {
      // Make sure we do not unset this flag if set previously
      if (tag_.second == "true")
        way_.set_destination_only(true);
    };
    tag_handlers_["service"] = [this]() {
      if (tag_.second == "rest_area") {
        service_ = tag_.second;
      }
    };
    tag_handlers_["amenity"] = [this]() {
      if (tag_.second == "yes") {
        amenity_ = tag_.second;
      }
    };

    tag_handlers_["use"] = [this]() {
      Use use = (Use)std::stoi(tag_.second);
      switch (use) {
        case Use::kCycleway:
          way_.set_use(Use::kCycleway);
          break;
        case Use::kFootway:
          way_.set_use(Use::kFootway);
          break;
        case Use::kSidewalk:
          way_.set_use(Use::kSidewalk);
          break;
        case Use::kPedestrian:
          way_.set_use(Use::kPedestrian);
          break;
        case Use::kPath:
          way_.set_use(Use::kPath);
          break;
        case Use::kSteps:
          way_.set_use(Use::kSteps);
          break;
        case Use::kBridleway:
          way_.set_use(Use::kBridleway);
          break;
        case Use::kPedestrianCrossing:
          way_.set_use(Use::kPedestrianCrossing);
          break;
        case Use::kLivingStreet:
          way_.set_use(Use::kLivingStreet);
          break;
        case Use::kParkingAisle:
          way_.set_destination_only(true);
          way_.set_use(Use::kParkingAisle);
          break;
        case Use::kDriveway:
          way_.set_destination_only(true);
          way_.set_use(Use::kDriveway);
          break;
        case Use::kAlley:
          way_.set_use(Use::kAlley);
          break;
        case Use::kEmergencyAccess:
          way_.set_use(Use::kEmergencyAccess);
          break;
        case Use::kDriveThru:
          way_.set_destination_only(true);
          way_.set_use(Use::kDriveThru);
          break;
        case Use::kServiceRoad:
          way_.set_use(Use::kServiceRoad);
          break;
        case Use::kTrack:
          way_.set_use(Use::kTrack);
          break;
        case Use::kOther:
          way_.set_use(Use::kOther);
          break;
        case Use::kRoad:
        default:
          way_.set_use(Use::kRoad);
          break;
      }
    };
    tag_handlers_["no_thru_traffic"] = [this]() {
      way_.set_no_thru_traffic(tag_.second == "true" ? true : false);
    };
    tag_handlers_["oneway"] = [this]() { way_.set_oneway(tag_.second == "true" ? true : false); };
    tag_handlers_["oneway_reverse"] = [this]() {
      way_.set_oneway_reverse(tag_.second == "true" ? true : false);
    };
    tag_handlers_["roundabout"] = [this]() {
      way_.set_roundabout(tag_.second == "true" ? true : false);
    };
    tag_handlers_["link"] = [this]() { way_.set_link(tag_.second == "true" ? true : false); };
    tag_handlers_["ferry"] = [this]() { way_.set_ferry(tag_.second == "true" ? true : false); };
    tag_handlers_["rail"] = [this]() { way_.set_rail(tag_.second == "true" ? true : false); };
    tag_handlers_["duration"] = [this]() {
      std::size_t found = tag_.second.find(":");
      if (found != std::string::npos) {
        std::vector<std::string> time = GetTagTokens(tag_.second, ':');
        uint32_t hour = 0, min = 0, sec = 0;
        if (time.size() == 1) { // minutes
          std::stringstream ss(time.at(0));
          ss >> min;
          min *= 60;
        } else if (time.size() == 2) { // hours and min
          std::stringstream ss(tag_.second);
          ss >> hour;
          ss.ignore();
          hour *= 3600;

          ss >> min;
          min *= 60;
        } else if (time.size() == 3) { // hours, min, and sec
          std::stringstream ss(tag_.second);
          ss >> hour;
          ss.ignore();
          hour *= 3600;

          ss >> min;
          ss.ignore();
          min *= 60;

          ss >> sec;
        }
        way_.set_duration(hour + min + sec);
      }
    };
    tag_handlers_["name"] = [this]() {
      if (!tag_.second.empty())
        name_ = tag_.second;
    };
    tag_handlers_["name:en"] = [this]() {
      if (!tag_.second.empty())
        way_.set_name_en_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["alt_name"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_)
        way_.set_alt_name_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["official_name"] = [this]() {
      if (!tag_.second.empty())
        way_.set_official_name_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["tunnel:name"] = [this]() {
      if (!tag_.second.empty())
        way_.set_tunnel_name_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["max_speed"] = [this]() {
      try {
        if (tag_.second == "unlimited") {
          // this way has an unlimited speed limit (german autobahn)
          max_speed_ = kUnlimitedSpeedLimit;
        } else {
          max_speed_ = std::stof(tag_.second);
        }
        way_.set_tagged_speed(true);
        has_max_speed_ = true;
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["average_speed"] = [this]() {
      try {
        average_speed_ = std::stof(tag_.second);
        has_average_speed_ = true;
        way_.set_tagged_speed(true);
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["advisory_speed"] = [this]() {
      try {
        advisory_speed_ = std::stof(tag_.second);
        has_advisory_speed_ = true;
        way_.set_tagged_speed(true);
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["forward_speed"] = [this]() {
      try {
        way_.set_forward_speed(std::stof(tag_.second));
        way_.set_forward_tagged_speed(true);
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["backward_speed"] = [this]() {
      try {
        way_.set_backward_speed(std::stof(tag_.second));
        way_.set_backward_tagged_speed(true);
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["maxspeed:hgv"] = [this]() {
      try {
        way_.set_truck_speed(std::stof(tag_.second));
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["truck_route"] = [this]() {
      way_.set_truck_route(tag_.second == "true" ? true : false);
    };
    tag_handlers_["hazmat"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kHazmat);
      restriction.set_value(tag_.second == "true" ? true : false);
      restriction.set_modes(kTruckAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxheight"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxHeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxwidth"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWidth);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxlength"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxLength);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxweight"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxaxleload"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxAxleLoad);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["hov_type"] = [this]() {
      // If this tag is set then the way is either HOV-2 or HOV-3.
      // There are no other real-world hov levels.
      std::string hov_type = tag_.second;
      if (hov_type == "HOV2") {
        way_.set_hov_type(valhalla::baldr::HOVEdgeType::kHOV2);
      } else if (hov_type == "HOV3") {
        way_.set_hov_type(valhalla::baldr::HOVEdgeType::kHOV3);
      } else {
        LOG_WARN("Unrecognized HOV type: " + hov_type);
        way_.set_hov_type(valhalla::baldr::HOVEdgeType::kHOV3);
      }
    };
    tag_handlers_["default_speed"] = [this]() {
      try {
        default_speed_ = std::stof(tag_.second);
        has_default_speed_ = true;
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["ref"] = [this]() {
      if (!tag_.second.empty()) {
        if (!use_direction_on_ways_)
          way_.set_ref_index(osmdata_.name_offset_map.index(tag_.second));
        else
          ref_ = tag_.second;
      }
    };
    tag_handlers_["int_ref"] = [this]() {
      if (!tag_.second.empty()) {
        if (!use_direction_on_ways_)
          way_.set_int_ref_index(osmdata_.name_offset_map.index(tag_.second));
        else
          int_ref_ = tag_.second;
      }
    };
    tag_handlers_["direction"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        direction_ = tag_.second;
    };
    tag_handlers_["int_direction"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        int_direction_ = tag_.second;
    };
    tag_handlers_["sac_scale"] = [this]() {
      std::string value = tag_.second;
      boost::algorithm::to_lower(value);

      if (value.find("difficult_alpine_hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kDifficultAlpineHiking);

      } else if (value.find("demanding_alpine_hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kDemandingAlpineHiking);

      } else if (value.find("alpine_hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kAlpineHiking);

      } else if (value.find("demanding_mountain_hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kDemandingMountainHiking);

      } else if (value.find("mountain_hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kMountainHiking);

      } else if (value.find("hiking") != std::string::npos) {
        way_.set_sac_scale(SacScale::kHiking);

      } else {
        way_.set_sac_scale(SacScale::kNone);
      }
    };
    tag_handlers_["surface"] = [this]() {
      std::string value = tag_.second;
      boost::algorithm::to_lower(value);

      // Find unpaved before paved since they have common string
      if (value.find("unpaved") != std::string::npos) {
        way_.set_surface(Surface::kGravel);

      } else if (value.find("paved") != std::string::npos ||
                 value.find("pavement") != std::string::npos ||
                 value.find("asphalt") != std::string::npos ||
                 // concrete, concrete:lanes, concrete:plates
                 value.find("concrete") != std::string::npos ||
                 value.find("cement") != std::string::npos ||
                 value.find("chipseal") != std::string::npos ||
                 value.find("metal") != std::string::npos) {
        way_.set_surface(Surface::kPavedSmooth);

      } else if (value.find("tartan") != std::string::npos ||
                 value.find("pavingstone") != std::string::npos ||
                 value.find("paving_stones") != std::string::npos ||
                 value.find("sett") != std::string::npos ||
                 value.find("grass_paver") != std::string::npos) {
        way_.set_surface(Surface::kPaved);

      } else if (value.find("cobblestone") != std::string::npos ||
                 value.find("brick") != std::string::npos) {
        way_.set_surface(Surface::kPavedRough);

      } else if (value.find("compacted") != std::string::npos ||
                 value.find("wood") != std::string::npos ||
                 value.find("boardwalk") != std::string::npos) {
        way_.set_surface(Surface::kCompacted);

      } else if (value.find("dirt") != std::string::npos ||
                 value.find("natural") != std::string::npos ||
                 value.find("earth") != std::string::npos ||
                 value.find("ground") != std::string::npos ||
                 value.find("mud") != std::string::npos) {
        way_.set_surface(Surface::kDirt);

      } else if (value.find("gravel") != std::string::npos || // gravel, fine_gravel
                 value.find("pebblestone") != std::string::npos ||
                 value.find("sand") != std::string::npos) {
        way_.set_surface(Surface::kGravel);
      } else if (value.find("grass") != std::string::npos ||
                 value.find("stepping_stones") != std::string::npos) {
        way_.set_surface(Surface::kPath);
        // We have to set a flag as surface may come before Road classes and Uses
      } else {
        has_surface_ = false;
      }
    };
    // surface tag should win over tracktype.
    tag_handlers_["tracktype"] = [this]() {
      if (!has_surface_tag_) {
        has_surface_ = true;
        if (tag_.second == "grade1") {
          way_.set_surface(Surface::kPavedRough);
        } else if (tag_.second == "grade2") {
          way_.set_surface(Surface::kCompacted);
        } else if (tag_.second == "grade3") {
          way_.set_surface(Surface::kDirt);
        } else if (tag_.second == "grade4") {
          way_.set_surface(Surface::kGravel);
        } else if (tag_.second == "grade5") {
          way_.set_surface(Surface::kPath);
        } else {
          has_surface_ = false;
        }
      }
    };
    tag_handlers_["bicycle"] = [this]() {
      if (tag_.second == "dismount") {
        way_.set_dismount(true);
      } else if (tag_.second == "use_sidepath") {
        way_.set_use_sidepath(true);
      }
    };
    tag_handlers_["shoulder_right"] = [this]() {
      way_.set_shoulder_right(tag_.second == "true" ? true : false);
    };
    tag_handlers_["shoulder_left"] = [this]() {
      way_.set_shoulder_left(tag_.second == "true" ? true : false);
    };
    tag_handlers_["cycle_lane_right"] = [this]() {
      CycleLane cyclelane_right = (CycleLane)std::stoi(tag_.second);
      switch (cyclelane_right) {
        case CycleLane::kDedicated:
          way_.set_cyclelane_right(CycleLane::kDedicated);
          break;
        case CycleLane::kSeparated:
          way_.set_cyclelane_right(CycleLane::kSeparated);
          break;
        case CycleLane::kShared:
          way_.set_cyclelane_right(CycleLane::kShared);
          break;
        case CycleLane::kNone:
        default:
          way_.set_cyclelane_right(CycleLane::kNone);
          break;
      }
    };
    tag_handlers_["cycle_lane_left"] = [this]() {
      CycleLane cyclelane_left = (CycleLane)std::stoi(tag_.second);
      switch (cyclelane_left) {
        case CycleLane::kDedicated:
          way_.set_cyclelane_left(CycleLane::kDedicated);
          break;
        case CycleLane::kSeparated:
          way_.set_cyclelane_left(CycleLane::kSeparated);
          break;
        case CycleLane::kShared:
          way_.set_cyclelane_left(CycleLane::kShared);
          break;
        case CycleLane::kNone:
        default:
          way_.set_cyclelane_left(CycleLane::kNone);
          break;
      }
    };
    tag_handlers_["cycle_lane_right_opposite"] = [this]() {
      way_.set_cyclelane_right_opposite(tag_.second == "true" ? true : false);
    };
    tag_handlers_["cycle_lane_left_opposite"] = [this]() {
      way_.set_cyclelane_left_opposite(tag_.second == "true" ? true : false);
    };
    tag_handlers_["lanes"] = [this]() {
      way_.set_lanes(std::stoi(tag_.second));
      way_.set_tagged_lanes(true);
    };
    tag_handlers_["forward_lanes"] = [this]() {
      way_.set_forward_lanes(std::stoi(tag_.second));
      way_.set_forward_tagged_lanes(true);
    };
    tag_handlers_["backward_lanes"] = [this]() {
      way_.set_backward_lanes(std::stoi(tag_.second));
      way_.set_backward_tagged_lanes(true);
    };
    tag_handlers_["tunnel"] = [this]() { way_.set_tunnel(tag_.second == "true" ? true : false); };
    tag_handlers_["toll"] = [this]() { way_.set_toll(tag_.second == "true" ? true : false); };
    tag_handlers_["bridge"] = [this]() { way_.set_bridge(tag_.second == "true" ? true : false); };
    tag_handlers_["seasonal"] = [this]() { way_.set_seasonal(tag_.second == "true" ? true : false); };
    tag_handlers_["bike_network_mask"] = [this]() { way_.set_bike_network(std::stoi(tag_.second)); };
    //    tag_handlers_["bike_national_ref"] = [this]() {
    //      if (!tag_.second.empty())
    //        way_.set_bike_national_ref_index(osmdata_.name_offset_map.index(tag_.second));
    //    };
    //    tag_handlers_["bike_regional_ref"] = [this]() {
    //      if (!tag_.second.empty())
    //        way_.set_bike_regional_ref_index(osmdata_.name_offset_map.index(tag_.second));
    //    };
    //    tag_handlers_["bike_local_ref"] = [this]() {
    //      if (!tag_.second.empty())
    //        way_.set_bike_local_ref_index(osmdata_.name_offset_map.index(tag_.second));
    //    };
    tag_handlers_["destination"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:forward"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_forward_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:backward"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_backward_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:ref"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_ref_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:ref:to"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_ref_to_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:street"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_street_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:street:to"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_street_to_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["junction:ref"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_junction_ref_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["turn:lanes"] = [this]() {
      // Turn lanes in the forward direction
      way_.set_fwd_turn_lanes_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["turn:lanes:forward"] = [this]() {
      // Turn lanes in the forward direction
      way_.set_fwd_turn_lanes_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["turn:lanes:backward"] = [this]() {
      // Turn lanes in the reverse direction
      way_.set_bwd_turn_lanes_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:base"] = [this]() {
      way_.set_fwd_jct_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:base:forward"] = [this]() {
      way_.set_fwd_jct_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:overlay"] = [this]() {
      way_.set_fwd_jct_overlay_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:overlay:forward"] = [this]() {
      way_.set_fwd_jct_overlay_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:base:backward"] = [this]() {
      way_.set_bwd_jct_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:jct:overlay:backward"] = [this]() {
      way_.set_bwd_jct_overlay_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:signboard:base"] = [this]() {
      way_.set_fwd_signboard_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:signboard:base:forward"] = [this]() {
      way_.set_fwd_signboard_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["guidance_view:signboard:base:backward"] = [this]() {
      way_.set_bwd_signboard_base_index(osmdata_.name_offset_map.index(tag_.second));
    };
  }

  static std::string get_lua(const boost::property_tree::ptree& pt) {
    auto graph_lua_name = pt.get_optional<std::string>("graph_lua_name");
    if (graph_lua_name) {
      LOG_INFO("Using LUA script: " + *graph_lua_name);
      std::ifstream lua(*graph_lua_name);
      if (!lua.is_open()) {
        throw std::runtime_error("Failed to open: " + *graph_lua_name);
      }
      return std::string((std::istreambuf_iterator<char>(lua)), std::istreambuf_iterator<char>());
    }
    return std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len);
  }

  virtual void node_callback(const uint64_t osmid,
                             const double lng,
                             const double lat,
                             const OSMPBF::Tags& tags) override {
    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid < last_node_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_node_ = osmid;

    // Handle bike share stations separately
    boost::optional<Tags> results = boost::none;
    if (bss_nodes_) {
      // Get tags - do't bother with Lua callout if the taglist is empty
      if (tags.size() > 0) {
        results = lua_.Transform(OSMType::kNode, osmid, tags);
      } else {
        results = empty_node_results_;
      }

      for (auto& key_value : *results) {
        if (key_value.first == "amenity" && key_value.second == "bicycle_rental") {
          // Create a new node and set its attributes
          OSMNode n{osmid};
          n.set_latlng(lng, lat);
          n.set_type(NodeType::kBikeShare);
          bss_nodes_->push_back(n);
          return; // we are done.
        }
      }
      return; // not found
    }

    // if we found all of the node ids we were looking for already we can bail
    if (current_way_node_index_ >= way_nodes_->size()) {
      return;
    }

    // if the current osmid of this node of this pbf file is greater than the waynode we are looking
    // for then it must be in another pbf file. so we need to move on to the next waynode that could
    // possibly actually be in this pbf file
    if (osmid > (*(*way_nodes_)[current_way_node_index_]).node.osmid_) {
      current_way_node_index_ =
          way_nodes_->find_first_of(OSMWayNode{{osmid}},
                                    [](const OSMWayNode& a, const OSMWayNode& b) {
                                      return a.node.osmid_ <= b.node.osmid_;
                                    },
                                    current_way_node_index_);
    }

    // if this nodes id is less than the waynode we are looking for then we know its a node we can
    // skip because it means there were no ways that we kept that referenced it. also we could run out
    // of waynodes to look for and in that case we are done as well
    if (current_way_node_index_ >= way_nodes_->size() ||
        osmid < (*(*way_nodes_)[current_way_node_index_]).node.osmid_) {
      return;
    }

    // Get tags if not already available.  Don't bother calling Lua if there
    // are no OSM tags to process.
    if (tags.size() > 0) {
      results = results ? results : lua_.Transform(OSMType::kNode, osmid, tags);
    } else {
      results = results ? results : empty_node_results_;
    }

    const auto highway = results->find("highway");
    bool is_highway_junction =
        ((highway != results->end()) && (highway->second == "motorway_junction"));

    const auto junction = results->find("junction");
    bool maybe_named_junction =
        junction != results->end() && (junction->second == "named" || junction->second == "yes");
    bool named_junction = false;

    // Create a new node and set its attributes
    OSMNode n;
    n.set_id(osmid);
    n.set_latlng(lng, lat);
    bool intersection = false;
    if (is_highway_junction) {
      n.set_type(NodeType::kMotorWayJunction);
    }

    for (const auto& tag : *results) {
      // TODO: instead of checking this, we should delete these tag/values completely in lua
      // and save our CPUs the wasted time of iterating over them again for nothing
      auto hasTag = !tag.second.empty();
      if (tag.first == "iso:3166_1" && !use_admin_db_ && hasTag) {
        // Add the country iso code to the unique node names list and store its index in the OSM
        // node
        n.set_country_iso_index(osmdata_.node_names.index(tag.second));
        ++osmdata_.node_name_count;
      } else if ((tag.first == "state_iso_code" && !use_admin_db_) && hasTag) {
        // Add the state iso code to the unique node names list and store its index in the OSM
        // node
        n.set_state_iso_index(osmdata_.node_names.index(tag.second));
        ++osmdata_.node_name_count;
      } else if (tag.first == "highway") {
        n.set_traffic_signal(tag.second == "traffic_signals");
        n.set_stop_sign(tag.second == "stop");
        n.set_yield_sign(tag.second == "give_way");
      } else if (tag.first == "forward_signal") {
        n.set_forward_signal(tag.second == "true");
      } else if (tag.first == "backward_signal") {
        n.set_backward_signal(tag.second == "true");
      } else if (tag.first == "forward_stop") {
        n.set_forward_stop(tag.second == "true");
        n.set_direction(true);
      } else if (tag.first == "backward_stop") {
        n.set_backward_stop(tag.second == "true");
        n.set_direction(true);
      } else if (tag.first == "forward_yield") {
        n.set_forward_yield(tag.second == "true");
        n.set_direction(true);
      } else if (tag.first == "backward_yield") {
        n.set_backward_yield(tag.second == "true");
        n.set_direction(true);
      } else if (tag.first == "stop" || tag.first == "give_way") {
        n.set_minor(tag.second == "minor");
      } else if (use_urban_tag_ && tag.first == "urban") {
        n.set_urban(tag.second == "true");
      } else if (tag.first == "exit_to" && is_highway_junction && hasTag) {
        // Add the name to the unique node names list and store its index in the OSM node
        n.set_exit_to_index(osmdata_.node_names.index(tag.second));
        ++osmdata_.node_exit_to_count;
      } else if (tag.first == "ref" && is_highway_junction && hasTag) {
        // Add the name to the unique node names list and store its index in the OSM node
        n.set_ref_index(osmdata_.node_names.index(tag.second));
        ++osmdata_.node_ref_count;
      } else if (tag.first == "name" && (is_highway_junction || maybe_named_junction) && hasTag) {
        // Add the name to the unique node names list and store its index in the OSM node
        n.set_name_index(osmdata_.node_names.index(tag.second));
        ++osmdata_.node_name_count;
        named_junction = maybe_named_junction;
      } else if (tag.first == "gate" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kGate);
      } else if (tag.first == "bollard" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kBollard);
      } else if (tag.first == "toll_booth" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kTollBooth);
      } else if (tag.first == "border_control" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kBorderControl);
      } else if (tag.first == "cash_only_toll" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kTollBooth);
        n.set_cash_only_toll(true);
      } else if (tag.first == "toll_gantry" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kTollGantry);
      } else if (tag.first == "sump_buster" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kSumpBuster);
      } else if (tag.first == "access_mask") {
        n.set_access(std::stoi(tag.second));
      } else if (tag.first == "tagged_access") {
        n.set_tagged_access(std::stoi(tag.second));
      } else if (tag.first == "private") {
        n.set_private_access(tag.second == "true");
      }
    }

    // If we ended up storing a name for a regular junction flag that
    n.set_named_intersection(named_junction);

    // If way parsing marked it as the beginning or end of a way (dead ends) we'll keep that too
    sequence<OSMWayNode>::iterator element = (*way_nodes_)[current_way_node_index_];
    auto way_node = *element;
    intersection = intersection || way_node.node.intersection_;

    // If multiple ways reference this its also an intersection
    if (!intersection && current_way_node_index_ < way_nodes_->size() - 1 &&
        osmid == (*(*way_nodes_)[current_way_node_index_ + 1]).node.osmid_) {
      intersection = true;
    }

    // Finally set the intersection flag for any reason that we outlined above
    if (intersection) {
      n.set_intersection(true);
      osmdata_.node_count++;
    }

    // Update all copies of this node that various ways referenced
    while (current_way_node_index_ < way_nodes_->size() &&
           (way_node = element = (*way_nodes_)[current_way_node_index_]).node.osmid_ == osmid) {
      // we need to keep the duplicate flag that way parsing set
      n.flat_loop_ = way_node.node.flat_loop_;
      way_node.node = n;
      element = way_node;
      ++current_way_node_index_;
      osmdata_.edge_count += intersection;
    }
    if (++osmdata_.osm_node_count % 5000000 == 0) {
      LOG_DEBUG("Processed " + std::to_string(osmdata_.osm_node_count) + " nodes on ways");
    }
    osmdata_.edge_count -= intersection; // more accurate but undercounts by skipping lone edges
  }

  virtual void way_callback(const uint64_t osmid,
                            const OSMPBF::Tags& tags,
                            const std::vector<uint64_t>& nodes) override {
    osmid_ = osmid;

    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid_ < last_way_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_way_ = osmid_;

    // Do not add ways with < 2 nodes. Log error or add to a problem list
    // TODO - find out if we do need these, why they exist...
    if (nodes.size() < 2) {
      return;
    }

    // Throw away closed features with following tags: building, landuse,
    // leisure, natural. See: http://wiki.openstreetmap.org/wiki/Key:area
    if (nodes[0] == nodes[nodes.size() - 1]) {
      for (const auto& tag : tags) {
        if (tag.first == "building" || tag.first == "landuse" || tag.first == "leisure" ||
            tag.first == "natural") {
          // LOG_INFO("Loop wayid " + std::to_string(osmid) + " Discard?");
          return;
        }
      }
    }

    // Transform tags. If no results that means the way does not have tags
    // suitable for use in routing.
    Tags results =
        tags.size() == 0 ? empty_way_results_ : lua_.Transform(OSMType::kWay, osmid_, tags);
    if (results.size() == 0) {
      return;
    }

    // Throw away driveways if include_driveways_ is false
    Tags::const_iterator driveways;
    try {
      if (!include_driveways_ && (driveways = results.find("use")) != results.end() &&
          static_cast<Use>(std::stoi(driveways->second)) == Use::kDriveway) {

        // only private driveways.
        Tags::const_iterator priv;
        if ((priv = results.find("private")) != results.end() && priv->second == "true") {
          return;
        }
      }
    } catch (const std::invalid_argument& arg) {
      LOG_INFO("invalid_argument thrown for way id: " + std::to_string(osmid_));
    } catch (const std::out_of_range& oor) {
      LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
    }

    // Add the refs to the reference list and mark the nodes that care about when processing nodes
    loop_nodes_.clear();
    auto way_node_index = way_nodes_->size();
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto& node = nodes[i];

      // Check whether the node is on a part of a way doubling back on itself
      OSMNode osm_node{node};
      auto inserted = loop_nodes_.insert(std::make_pair(node, i));
      bool flattening = inserted.first->second > 0 && i < nodes.size() - 1 &&
                        nodes[i + 1] == nodes[inserted.first->second - 1];
      bool unflattening = i > 0 && inserted.first->second < nodes.size() - 1 &&
                          nodes[i - 1] == nodes[inserted.first->second + 1];
      osm_node.flat_loop_ = flattening || unflattening;
      osm_node.intersection_ = i == 0 || i == nodes.size() - 1;

      // Keep the node
      way_nodes_->push_back(
          {osm_node, static_cast<uint32_t>(ways_->size()), static_cast<uint32_t>(i)});

      // If this way is a loop (node occurs twice) we can make our lives way easier if we simply
      // split it up into multiple edges in the graph. If a problem is hard, avoid the problem!
      if (!inserted.second) {
        // We'll make an intersection in the middle of the loop
        auto way_node_itr = (*way_nodes_)[way_node_index + (i + inserted.first->second) / 2];
        auto way_node = *way_node_itr;
        way_node.node.intersection_ = true;
        way_node_itr = way_node;
        // Update the index in case the node is used again (a future loop)
        inserted.first->second = i;
      }
    }
    ++osmdata_.osm_way_count;
    osmdata_.osm_way_node_count += nodes.size();

    default_speed_ = 0.0f, max_speed_ = 0.0f;
    average_speed_ = 0.0f, advisory_speed_ = 0.0f;
    has_default_speed_ = false, has_max_speed_ = false;
    has_average_speed_ = false, has_advisory_speed_ = false;
    has_surface_ = true;
    name_ = {}, service_ = {}, amenity_ = {};

    // Process tags
    way_ = OSMWay{osmid_};
    way_.set_node_count(nodes.size());

    osm_access_ = OSMAccess{osmid_};
    has_user_tags_ = false;
    ref_ = int_ref_ = direction_ = int_direction_ = {};

    const auto& surface_exists = results.find("surface");
    has_surface_tag_ = (surface_exists != results.end());
    if (!has_surface_tag_) {
      has_surface_ = false;
    }

    way_.set_drive_on_right(true); // default

    for (const auto& kv : results) {
      tag_ = kv;
      const auto it = tag_handlers_.find(tag_.first);
      if (it != tag_handlers_.end()) {
        try {
          it->second();
        } catch (const std::exception& ex) {
          std::stringstream ss;
          ss << "Error during parsing of `" << tag_.first << "` tag on the way " << osmid_ << ": "
             << std::string{ex.what()};
          LOG_WARN(ss.str());
        }

      }
      // motor_vehicle:conditional=no @ (16:30-07:00)
      else if (tag_.first.substr(0, 20) == "motorcar:conditional" ||
               tag_.first.substr(0, 25) == "motor_vehicle:conditional" ||
               tag_.first.substr(0, 19) == "bicycle:conditional" ||
               tag_.first.substr(0, 22) == "motorcycle:conditional" ||
               tag_.first.substr(0, 16) == "foot:conditional" ||
               tag_.first.substr(0, 22) == "pedestrian:conditional" ||
               tag_.first.substr(0, 15) == "hgv:conditional" ||
               tag_.first.substr(0, 17) == "moped:conditional" ||
               tag_.first.substr(0, 16) == "mofa:conditional" ||
               tag_.first.substr(0, 15) == "psv:conditional" ||
               tag_.first.substr(0, 16) == "taxi:conditional" ||
               tag_.first.substr(0, 15) == "bus:conditional" ||
               tag_.first.substr(0, 15) == "hov:conditional" ||
               tag_.first.substr(0, 21) == "emergency:conditional") {

        std::vector<std::string> tokens = GetTagTokens(tag_.second, '@');
        std::string tmp = tokens.at(0);
        boost::algorithm::trim(tmp);

        AccessType type = AccessType::kTimedDenied;
        if (tmp == "no") {
          type = AccessType::kTimedDenied;
        } else if (tmp == "yes" || tmp == "private" || tmp == "delivery" || tmp == "designated") {
          type = AccessType::kTimedAllowed;
        } else if (tmp == "destination") {
          type = AccessType::kDestinationAllowed;
        }

        if (tokens.size() == 2 && tmp.size()) {

          uint16_t mode = 0;
          if (tag_.first.substr(0, 20) == "motorcar:conditional" ||
              tag_.first.substr(0, 25) == "motor_vehicle:conditional") {
            mode = (kAutoAccess | kTruckAccess | kEmergencyAccess | kTaxiAccess | kBusAccess |
                    kHOVAccess | kMopedAccess | kMotorcycleAccess);
          } else if (tag_.first.substr(0, 19) == "bicycle:conditional") {
            mode = kBicycleAccess;
          } else if (tag_.first.substr(0, 16) == "foot:conditional" ||
                     tag_.first.substr(0, 22) == "pedestrian:conditional") {
            mode = (kPedestrianAccess | kWheelchairAccess);
          } else if (tag_.first.substr(0, 15) == "hgv:conditional") {
            mode = kTruckAccess;
          } else if (tag_.first.substr(0, 17) == "moped:conditional" ||
                     tag_.first.substr(0, 16) == "mofa:conditional") {
            mode = kMopedAccess;
          } else if (tag_.first.substr(0, 22) == "motorcycle:conditional") {
            mode = kMotorcycleAccess;
          } else if (tag_.first.substr(0, 15) == "psv:conditional") {
            mode = (kTaxiAccess | kBusAccess);
          } else if (tag_.first.substr(0, 16) == "taxi:conditional") {
            mode = kTaxiAccess;
          } else if (tag_.first.substr(0, 15) == "bus:conditional") {
            mode = kBusAccess;
          } else if (tag_.first.substr(0, 15) == "hov:conditional") {
            mode = kHOVAccess;
          } else if (tag_.first.substr(0, 21) == "emergency:conditional") {
            mode = kEmergencyAccess;
          }
          std::string tmp = tokens.at(1);
          boost::algorithm::trim(tmp);
          std::vector<std::string> conditions = GetTagTokens(tmp, ';');

          for (const auto& condition : conditions) {
            std::vector<uint64_t> values = get_time_range(condition);

            for (const auto& v : values) {
              OSMAccessRestriction restriction;
              restriction.set_type(static_cast<AccessType>(type));
              restriction.set_modes(mode);
              restriction.set_value(v);
              osmdata_.access_restrictions.insert({osmid_, restriction});
            }
          }
        }
      }
    }

    // We need to set a data processing flag so we need to
    // process in pbfgraphparser instead of lua because of config option use_rest_area
    if (use_rest_area_ && service_ == "rest_area") {
      if (amenity_ == "yes") {
        way_.set_use(Use::kServiceArea);
      } else {
        way_.set_use(Use::kRestArea);
      }
    }
    if (use_direction_on_ways_ && !ref_.empty()) {
      if (direction_.empty()) {
        way_.set_ref_index(osmdata_.name_offset_map.index(ref_));
      } else {
        std::vector<std::string> refs = GetTagTokens(ref_);
        std::vector<std::string> directions = GetTagTokens(direction_);

        std::string tmp_ref;
        if (refs.size() == directions.size()) {
          for (uint32_t i = 0; i < refs.size(); i++) {
            if (!tmp_ref.empty()) {
              tmp_ref += ";";
            }
            if (!directions.at(i).empty())
              tmp_ref += refs.at(i) + " " + directions.at(i);
            else
              tmp_ref += refs.at(i);
          }
          way_.set_ref_index(osmdata_.name_offset_map.index(tmp_ref));
        } else
          way_.set_ref_index(osmdata_.name_offset_map.index(ref_));
      }
    }

    if (use_direction_on_ways_ && !int_ref_.empty()) {
      if (int_direction_.empty()) {
        way_.set_int_ref_index(osmdata_.name_offset_map.index(int_ref_));
      } else {
        std::vector<std::string> int_refs = GetTagTokens(int_ref_);
        std::vector<std::string> int_directions = GetTagTokens(int_direction_);

        std::string tmp_ref;
        if (int_refs.size() == int_directions.size()) {
          for (uint32_t i = 0; i < int_refs.size(); i++) {
            if (!tmp_ref.empty()) {
              tmp_ref += ";";
            }
            if (!int_directions.at(i).empty())
              tmp_ref += int_refs.at(i) + " " + int_directions.at(i);
            else
              tmp_ref += int_refs.at(i);
          }
          way_.set_int_ref_index(osmdata_.name_offset_map.index(tmp_ref));
        } else
          way_.set_int_ref_index(osmdata_.name_offset_map.index(int_ref_));
      }
    }

    // add int_refs to the end of the refs for now.  makes sure that we don't add dups.
    if (use_direction_on_ways_ && way_.int_ref_index()) {
      std::string tmp = osmdata_.name_offset_map.name(way_.ref_index());

      std::vector<std::string> rs = GetTagTokens(tmp);
      std::vector<std::string> is = GetTagTokens(osmdata_.name_offset_map.name(way_.int_ref_index()));
      bool bFound = false;

      for (auto& i : is) {
        for (auto& r : rs) {
          if (i == r) {
            bFound = true;
            break;
          }
        }
        if (!bFound) {
          if (!tmp.empty()) {
            tmp += ";";
          }
          tmp += i;
        }
        bFound = false;
      }
      if (!tmp.empty()) {
        way_.set_ref_index(osmdata_.name_offset_map.index(tmp));
      }
      // no matter what, clear out the int_ref.
      way_.set_int_ref_index(0);
    }

    // Process mtb tags.
    auto mtb_scale = results.find("mtb:scale");
    bool has_mtb_scale = mtb_scale != results.end();
    if (has_mtb_scale) {
      int scale = get_number("mtb:scale", mtb_scale->second);
      if (scale >= 0) {
        // Set surface based on scale
        uint32_t scale = stoi(mtb_scale->second);
        if (scale == 0) {
          way_.set_surface(Surface::kDirt);
        } else if (scale == 1) {
          way_.set_surface(Surface::kGravel);
        } else {
          way_.set_surface(Surface::kPath);
        }
        has_surface_ = true;

        // Set bicycle access to true for all but the highest scale.
        bool access = scale < kMaxMtbScale;
        if (access && !way_.oneway_reverse()) {
          way_.set_bike_forward(true);
        }
        if (access && !way_.oneway()) {
          way_.set_bike_backward(true);
        }
      }
    }

    auto mtb_uphill_scale = results.find("mtb:scale:uphill");
    bool has_mtb_uphill_scale = mtb_uphill_scale != results.end();
    if (has_mtb_uphill_scale) {
      int scale = get_number("mtb:uphill:scale", mtb_uphill_scale->second);
      if (scale >= 0) {
        // Set surface based on scale (if no scale exists)
        uint32_t scale = stoi(mtb_uphill_scale->second);
        if (!has_mtb_scale) {
          if (scale < 2) {
            way_.set_surface(Surface::kGravel);
          } else {
            way_.set_surface(Surface::kPath);
          }
          has_surface_ = true;
        }

        // Set bicycle access to true for all but the highest scale.
        bool access = scale < kMaxMtbUphillScale;
        if (access && !way_.oneway_reverse()) {
          way_.set_bike_forward(true);
        }
        if (access && !way_.oneway()) {
          way_.set_bike_backward(true);
        }
      }
    }

    // IMBA scale
    auto mtb_imba_scale = results.find("mtb:scale:imba");
    bool has_mtb_imba = mtb_imba_scale != results.end();
    if (has_mtb_imba) {
      // Update bike access (only if neither mtb:scale nor mtb:scale:uphill is present)
      if (!has_mtb_scale && !has_mtb_uphill_scale) {
        if (!way_.oneway_reverse()) {
          way_.set_bike_forward(true);
        }
        if (!way_.oneway()) {
          way_.set_bike_backward(true);
        }
      }
    }

    // Only has MTB description - set bicycle access.
    bool has_mtb_desc = results.find("mtb:description") != results.end();
    if (has_mtb_desc && !has_mtb_scale && !has_mtb_uphill_scale && !has_mtb_imba) {
      if (!way_.oneway_reverse()) {
        way_.set_bike_forward(true);
      }
      if (!way_.oneway()) {
        way_.set_bike_backward(true);
      }
    }

    // if no surface and tracktype but we have a sac_scale, set surface to path.
    if (!has_surface_) {
      if (results.find("sac_scale") != results.end()) {
        way_.set_surface(Surface::kPath);
      } else {
        // If no surface has been set by a user, assign a surface based on Road Class and Use
        switch (way_.road_class()) {

          case RoadClass::kMotorway:
          case RoadClass::kTrunk:
          case RoadClass::kPrimary:
          case RoadClass::kSecondary:
          case RoadClass::kTertiary:
          case RoadClass::kUnclassified:
          case RoadClass::kResidential:
            way_.set_surface(Surface::kPavedSmooth);
            break;
          default:
            switch (way_.use()) {
              case Use::kFootway:
              case Use::kPedestrian:
              case Use::kSidewalk:
              case Use::kPath:
              case Use::kBridleway:
                way_.set_surface(Surface::kCompacted);
                break;
              case Use::kTrack:
                way_.set_surface(Surface::kDirt);
                break;
              case Use::kRoad:
              case Use::kParkingAisle:
              case Use::kDriveway:
              case Use::kAlley:
              case Use::kEmergencyAccess:
              case Use::kDriveThru:
              case Use::kLivingStreet:
              case Use::kServiceRoad:
                way_.set_surface(Surface::kPavedSmooth);
                break;
              case Use::kCycleway:
              case Use::kSteps:
                way_.set_surface(Surface::kPaved);
                break;
              default:
                // TODO:  see if we can add more logic when a user does not
                // specify a surface.
                way_.set_surface(Surface::kPaved);
                break;
            }
            break;
        }
      }
    }

    // set the speed
    if (has_average_speed_) {
      way_.set_speed(average_speed_);
    } else if (has_advisory_speed_) {
      way_.set_speed(advisory_speed_);
    } else if (has_max_speed_ && max_speed_ != kUnlimitedSpeedLimit) {
      // don't use unlimited speed limit for default edge speed
      way_.set_speed(max_speed_);
    } else if (has_default_speed_ && !way_.forward_tagged_speed() && !way_.backward_tagged_speed()) {
      way_.set_speed(default_speed_);
    }

    // set the speed limit
    if (has_max_speed_) {
      way_.set_speed_limit(max_speed_);
    }

    // I hope this does not happen, but it probably will (i.e., user sets forward speed
    // and not the backward speed and vice versa.)
    if (way_.forward_tagged_speed() && !way_.backward_tagged_speed()) {
      if (!way_.oneway()) {
        way_.set_backward_speed(way_.forward_speed());
        way_.set_backward_tagged_speed(true);
      } else // fallback to default speed.
        way_.set_speed(default_speed_);
    } else if (!way_.forward_tagged_speed() && way_.backward_tagged_speed()) {
      if (!way_.oneway()) {
        way_.set_forward_speed(way_.backward_speed());
        way_.set_forward_tagged_speed(true);
      } else // fallback to default speed.
        way_.set_speed(default_speed_);
    }

    // ferries / auto trains need to be set to highway cut off in config.
    if (way_.ferry() || way_.rail()) {
      way_.set_road_class(highway_cutoff_rc_);
    }

    // Delete the name from from name field if it exists in the ref.
    if (!name_.empty() && way_.ref_index()) {
      std::vector<std::string> names = GetTagTokens(name_);
      std::vector<std::string> refs = GetTagTokens(osmdata_.name_offset_map.name(way_.ref_index()));
      bool bFound = false;

      std::string tmp;

      for (auto& name : names) {
        for (auto& ref : refs) {
          if (name == ref) {
            bFound = true;
            break;
          }
        }
        if (!bFound) {
          if (!tmp.empty()) {
            tmp += ";";
          }
          tmp += name;
        }
        bFound = false;
      }
      if (!tmp.empty()) {
        way_.set_name_index(osmdata_.name_offset_map.index(tmp));
      }
    } else {
      way_.set_name_index(osmdata_.name_offset_map.index(name_));
    }

    // Infer cul-de-sac if a road edge is a loop and is low classification.
    if (!way_.roundabout() && loop_nodes_.size() != nodes.size() && way_.use() == Use::kRoad &&
        way_.road_class() > RoadClass::kTertiary) {
      // Adds a loop road as a candidate to be a "culdesac" road.
      culdesac_processor_.add_candidate(way_.way_id(), ways_->size(), nodes);
    }

    if (has_user_tags_) {
      way_.set_has_user_tags(true);
      access_->push_back(osm_access_);
    }
    // Add the way to the list
    ways_->push_back(way_);
  }

  virtual void relation_callback(const uint64_t osmid,
                                 const OSMPBF::Tags& tags,
                                 const std::vector<OSMPBF::Member>& members) override {
    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid < last_relation_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_relation_ = osmid;

    // Get tags
    Tags results =
        tags.empty() ? empty_relation_results_ : lua_.Transform(OSMType::kRelation, osmid, tags);
    if (results.size() == 0) {
      return;
    }

    OSMRestriction restriction{};
    OSMRestriction to_restriction{};

    uint64_t from_way_id = 0;
    bool isRestriction = false, isTypeRestriction = false, hasRestriction = false;
    bool isRoad = false, isRoute = false, isBicycle = false, isConnectivity = false;
    bool isConditional = false, has_multiple_times = false;
    uint32_t bike_network_mask = 0;

    std::string network, ref, name, except;
    std::string from_lanes, from, to_lanes, to;
    std::string condition, direction;
    std::string hour_start, hour_end, day_start, day_end;
    uint32_t modes = 0;

    for (const auto& tag : results) {
      if (tag.first == "type") {
        if (tag.second == "restriction") {
          isRestriction = true;
        } else if (tag.second == "route") {
          isRoute = true;
        } else if (tag.second == "connectivity") {
          isConnectivity = true;
        }
      } else if (tag.first == "route") {
        if (tag.second == "road") {
          isRoad = true;
        } else if (tag.second == "bicycle" || tag.second == "mtb") {
          isBicycle = true;
        }
      } else if (tag.first == "restriction:conditional") {
        isConditional = true;
        condition = tag.second;
      } else if (tag.first == "direction") {
        direction = tag.second;
      } else if (tag.first == "network") {
        network = tag.second; // US:US
      } else if (tag.first == "ref") {
        ref = tag.second;
      } else if (tag.first == "name") {
        name = tag.second;
      } else if (tag.first == "except") {
        except = tag.second;
      } else if ((tag.first == "restriction" || tag.first == "restriction:motorcar" ||
                  tag.first == "restriction:motorcycle" || tag.first == "restriction:taxi" ||
                  tag.first == "restriction:bus" || tag.first == "restriction:bicycle" ||
                  tag.first == "restriction:hgv" || tag.first == "restriction:hazmat" ||
                  tag.first == "restriction:emergency" || tag.first == "restriction:foot") &&
                 !tag.second.empty()) {
        isRestriction = true;
        if (tag.first != "restriction") {
          isTypeRestriction = true;
        }

        if (tag.first == "restriction:motorcar") {
          modes |= (kAutoAccess | kMopedAccess);
        } else if (tag.first == "restriction:motorcycle") {
          modes |= kMotorcycleAccess;
        } else if (tag.first == "restriction:taxi") {
          modes |= kTaxiAccess;
        } else if (tag.first == "restriction:bus") {
          modes |= kBusAccess;
        } else if (tag.first == "restriction:bicycle") {
          modes |= kBicycleAccess;
        } else if (tag.first == "restriction:hgv" || tag.first == "restriction:hazmat") {
          modes |= kTruckAccess;
        } else if (tag.first == "restriction:emergency") {
          modes |= kEmergencyAccess;
        } else if (tag.first == "restriction:psv") {
          modes |= (kTaxiAccess | kBusAccess);
        } else if (tag.first == "restriction:foot") {
          modes |= (kPedestrianAccess | kWheelchairAccess);
        }

        RestrictionType type = (RestrictionType)std::stoi(tag.second);

        switch (type) {

          case RestrictionType::kNoLeftTurn:
          case RestrictionType::kNoRightTurn:
          case RestrictionType::kNoStraightOn:
          case RestrictionType::kNoUTurn:
          case RestrictionType::kOnlyRightTurn:
          case RestrictionType::kOnlyLeftTurn:
          case RestrictionType::kOnlyStraightOn:
          case RestrictionType::kNoEntry:
          case RestrictionType::kNoExit:
          case RestrictionType::kNoTurn:
            hasRestriction = true;
            restriction.set_type(type);
            break;
          default:
            return;
        }
      }
      // sample with date time.  1168738
      else if (tag.first == "hour_on") {
        // invalid data
        if (tag.second.find(":") == std::string::npos) {
          return;
        }

        // hour_on = 06:00;16:00
        if (tag.second.find(";") != std::string::npos) {
          has_multiple_times = true;
        }

        isConditional = true;
        hour_start = tag.second;
      } else if (tag.first == "hour_off") {
        // invalid data
        if (tag.second.find(":") == std::string::npos) {
          return;
        }

        // hour_on = 06:00;16:00
        if (tag.second.find(";") != std::string::npos) {
          has_multiple_times = true;
        }

        isConditional = true;
        hour_end = tag.second;
      } else if (tag.first == "day_on") {
        isConditional = true;
        day_start = tag.second;
      } else if (tag.first == "day_off") {
        isConditional = true;
        day_end = tag.second;
      } else if (tag.first == "bike_network_mask") {
        bike_network_mask = std::stoi(tag.second);
      } else if (tag.first == "to:lanes") {
        to_lanes = tag.second;
      } else if (tag.first == "from:lanes") {
        from_lanes = tag.second;
      } else if (tag.first == "to") {
        to = tag.second;
      } else if (tag.first == "from") {
        from = tag.second;
      }
    } // for (const auto& tag : results)

    std::vector<std::string> net = GetTagTokens(network, ':');
    bool special_network = false;
    if (net.size() == 3) {
      std::string value = net.at(2);
      boost::algorithm::to_lower(value);

      if (value == "turnpike" || value == "tp" || value == "fm" || value == "rm" || value == "loop" ||
          value == "spur" || value == "truck" || value == "business" || value == "bypass" ||
          value == "belt" || value == "alternate" || value == "alt" || value == "toll" ||
          value == "cr" || value == "byway" || value == "scenic" || value == "connector" ||
          value == "county")
        special_network = true;
    }

    if (isBicycle && isRoute && !network.empty()) {
      OSMBike bike;
      const uint32_t name_index = osmdata_.name_offset_map.index(name);
      const uint32_t ref_index = osmdata_.name_offset_map.index(ref);

      // if the network is not of type lcn, rcn, ncn, or mtb don't save.
      if (!bike_network_mask) {
        return;
      }

      bike.bike_network = bike_network_mask;
      bike.name_index = name_index;
      bike.ref_index = ref_index;

      for (const auto& member : members) {
        osmdata_.bike_relations.insert(BikeMultiMap::value_type(member.member_id, bike));
      }

    } else if (isRoad && isRoute && !network.empty() &&
               ((net.size() == 2 && !ref.empty()) ||
                (net.size() == 3 && net.at(0) == "US" && special_network))) {

      if (net.size() == 3 && net.at(2) == "Turnpike")
        net[2] = "TP";

      std::string reference;
      if (net.size() == 2 && !ref.empty()) {
        if (ref.size() == 4 && net.at(1).size() == 2) { // NJTP
          if (net.at(1) + "TP" == ref)
            reference = ref;
          else
            return;
        } else
          reference = net.at(1) + " " + ref; // US 51 or I 95
      } else if (special_network && !ref.empty())
        reference = net.at(2) + " " + ref;
      else
        reference = net.at(1) + net.at(2); // PATP

      bool bfound = false;
      for (const auto& member : members) {
        if (member.role.empty() || member.role == "forward" || member.role == "backward") {
          continue;
        }
        direction = member.role;
        osmdata_.add_to_name_map(member.member_id, direction, reference);
        bfound = true;
      }

      // direction is already set via a direction tag and not at the member level.
      if (!direction.empty() && !bfound) {
        for (const auto& member : members) {
          if (member.role == "forward") {
            osmdata_.add_to_name_map(member.member_id, direction, reference);
          } else if (member.role == "backward") {
            osmdata_.add_to_name_map(member.member_id, direction, reference, false);
          }
        }
      }
    } else if (isConnectivity && (!to_lanes.empty() || !to.empty()) &&
               (!from_lanes.empty() || !from.empty())) {
      uint32_t from_way_id = 0;
      uint32_t to_way_id = 0;
      for (const auto& member : members) {
        // from and to must be of type 1(way).
        if (member.role == "from" &&
            member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
          from_way_id = member.member_id;
        } else if (member.role == "to" &&
                   member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
          to_way_id = member.member_id;
        }
      }

      if (from_way_id && to_way_id) {
        uint32_t to_idx = osmdata_.name_offset_map.index(std::max(to, to_lanes));
        uint32_t from_idx = osmdata_.name_offset_map.index(std::max(from, from_lanes));
        osmdata_.lane_connectivity_map.insert(
            OSMLaneConnectivityMultiMap::value_type(to_way_id,
                                                    OSMLaneConnectivity{to_way_id, from_way_id,
                                                                        to_idx, from_idx}));
      }
    } else if (isRestriction && hasRestriction) {
      std::vector<uint64_t> vias;

      for (const auto& member : members) {

        // from and to must be of type 1(way).  via must be of type 0(node)
        if (member.role == "from" &&
            member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
          from_way_id = member.member_id;
        } else if (member.role == "to" &&
                   member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
          if (!restriction.to())
            restriction.set_to(member.member_id);
        } else if (member.role == "via" &&
                   member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_NODE) {
          if (vias.size()) { // mix of nodes and ways.  Not supported yet.
            from_way_id = 0;
            break;
          }
          restriction.set_via(member.member_id);
        } else if (member.role == "via" &&
                   member.member_type == OSMPBF::Relation::MemberType::Relation_MemberType_WAY) {
          if (restriction.via()) { // mix of nodes and ways.  Not supported yet.
            from_way_id = 0;
            break;
          }
          vias.push_back(member.member_id);
          osmdata_.via_set.insert(member.member_id);
        }
      }

      if (vias.size() > kMaxViasPerRestriction) {
        LOG_INFO("skipping restriction with vias > the max allowed.  OSMID: " +
                 std::to_string(osmid));
        from_way_id = 0;
      }
      // Add the restriction to the list.
      if (from_way_id != 0 && (restriction.via() || vias.size()) && restriction.to()) {
        // check for exceptions
        // isTypeRestriction == true means has restriction:<vehicle> key; otherwise, just a
        // restriction key
        if (!isTypeRestriction) {

          modes = (kAutoAccess | kMopedAccess | kTaxiAccess | kBusAccess | kBicycleAccess |
                   kTruckAccess | kEmergencyAccess | kMotorcycleAccess);
          // remove access as the restriction does not apply to these modes.
          std::vector<std::string> tokens = GetTagTokens(except);
          for (const auto& t : tokens) {
            if (t == "motorcar") {
              modes = modes & ~(kAutoAccess | kMopedAccess);
            } else if (t == "motorcycle") {
              modes = modes & ~kMotorcycleAccess;
            } else if (t == "psv") {
              modes = modes & ~(kTaxiAccess | kBusAccess);
            } else if (t == "taxi") {
              modes = modes & ~kTaxiAccess;
            } else if (t == "bus") {
              modes = modes & ~kBusAccess;
            } else if (t == "bicycle") {
              modes = modes & ~kBicycleAccess;
            } else if (t == "hgv") {
              modes = modes & ~kTruckAccess;
            } else if (t == "emergency") {
              modes = modes & ~kEmergencyAccess;
            } else if (t == "foot") {
              modes = modes & ~(kPedestrianAccess | kWheelchairAccess);
            }
          }
        }
        // restriction:<vehicle> key exists but it is a simple restriction
        // change to a complex restriction with modes.
        // or
        // restriction = x with except tags; change to a complex
        // restriction with modes.
        if (vias.size() == 0 &&
            (isTypeRestriction || isConditional || (!isTypeRestriction && except.size()))) {

          restriction.set_via(0);
          vias.push_back(restriction.to());
          osmdata_.via_set.insert(restriction.to());

          if (isConditional) {
            restriction.set_modes(modes);
            // simple restriction, but is a timed restriction
            // change to complex and set date and time info
            if (condition.empty()) {
              if (!day_start.empty() && !day_end.empty()) {
                condition = day_start + '-' + day_end;
              }
              // do we have multiple times entered?
              if (!has_multiple_times) {
                // no we do not...add the hours to the condition
                if (!hour_start.empty() && !hour_end.empty()) {
                  condition += ' ' + hour_start + '-' + hour_end;
                }
              }
              // yes multiple times
              // 06:00;17:00
              // 11:00;20:00
              else {
                std::vector<std::string> hour_on = GetTagTokens(hour_start, ';');
                std::vector<std::string> hour_off = GetTagTokens(hour_end, ';');

                if (hour_on.size() > 1 && hour_on.size() == hour_off.size()) {
                  std::string hours;
                  // convert to the format of 07:30-09:30,17:30-19:30
                  for (uint32_t i = 0; i < hour_on.size(); i++) {
                    if (!hours.empty()) {
                      hours += ",";
                    }
                    hours += hour_on.at(i) + "-";
                    hours += hour_off.at(i);
                  }
                  condition += " " + hours;
                } else {
                  return; // should not make it here; has to be bad data.
                }
              } // else
            }   // if (condition.empty())

            std::vector<std::string> conditions = GetTagTokens(condition, ';');

            if (conditions.size()) {
              restriction.set_from(from_way_id);
              restriction.set_vias(vias);
              // for bi-directional we need to create the restriction in reverse.  flip the to and
              // from. also in order to avoid duplicate data in the from and to restrictions, we
              // only need to store the mode, from, and to for the to_restrictions.
              to_restriction.set_from(restriction.to());
              to_restriction.set_to(from_way_id);
              to_restriction.set_modes(restriction.modes());
              complex_restrictions_to_->push_back(to_restriction);
            } else {
              return; // bad data
            }

            for (const auto& c : conditions) {
              std::vector<uint64_t> values = get_time_range(c);
              for (const auto& v : values) { // could have multiple time domains
                restriction.set_time_domain(v);
                complex_restrictions_from_->push_back(restriction);
              }
            }
            return;
          } // if (isConditional)
        }   // end turning into complex restriction

        restriction.set_modes(modes);

        // complex restrictions -- add to end map.
        if (vias.size()) {
          osmdata_.via_set.insert(from_way_id);
          osmdata_.via_set.insert(restriction.to());
          restriction.set_from(from_way_id);
          restriction.set_vias(vias);
          // for bi-directional we need to create the restriction in reverse.  flip the to and from.
          // also in order to avoid duplicate data in the from and to restrictions, we only need
          // to store the mode, from, and to for the to_restrictions.
          to_restriction.set_from(restriction.to());
          to_restriction.set_to(from_way_id);
          to_restriction.set_modes(restriction.modes());
          complex_restrictions_to_->push_back(to_restriction);
          complex_restrictions_from_->push_back(restriction);
        } else { // simple restriction
          osmdata_.restrictions.insert(RestrictionsMultiMap::value_type(from_way_id, restriction));
        }
      }
    }
  }

  virtual void changeset_callback(const uint64_t changeset_id) override {
    osmdata_.max_changeset_id_ = std::max(osmdata_.max_changeset_id_, changeset_id);
  }

  // lets the sequences be set and reset
  void reset(sequence<OSMWay>* ways,
             sequence<OSMWayNode>* way_nodes,
             sequence<OSMAccess>* access,
             sequence<OSMRestriction>* complex_restrictions_from,
             sequence<OSMRestriction>* complex_restrictions_to,
             sequence<OSMNode>* bss_nodes) {
    // reset the pointers (either null them out or set them to something valid)
    ways_.reset(ways);
    way_nodes_.reset(way_nodes);
    access_.reset(access);
    complex_restrictions_from_.reset(complex_restrictions_from);
    complex_restrictions_to_.reset(complex_restrictions_to);
    bss_nodes_.reset(bss_nodes);
  }

  // WayCallback tag handlers
  using TagHandler = std::function<void()>;
  std::unordered_map<std::string, TagHandler> tag_handlers_;
  // Tag handlers capture these fields
  OSMWay way_;
  std::pair<std::string, std::string> tag_;
  uint64_t osmid_;
  float default_speed_ = 0.0f, max_speed_ = 0.0f;
  float average_speed_ = 0.0f, advisory_speed_ = 0.0f;
  bool has_default_speed_ = false, has_max_speed_ = false;
  bool has_average_speed_ = false, has_advisory_speed_ = false;
  bool has_surface_ = true;
  bool has_surface_tag_ = true;
  OSMAccess osm_access_;
  bool has_user_tags_ = false;
  std::string ref_, int_ref_, direction_, int_direction_;
  std::string name_, service_, amenity_;

  // Configuration option to include driveways
  bool include_driveways_;

  // Configuration option indicating whether or not to infer internal intersections during the graph
  // enhancer phase or use the internal_intersection key from the pbf
  bool infer_internal_intersections_;

  // Configuration option indicating whether or not to infer turn channels during the graph
  // enhancer phase or use the turn_channel key from the pbf
  bool infer_turn_channels_;

  // Configuration option indicating whether or not to process the direction key on the ways or
  // utilize the guidance relation tags during the parsing phase
  bool use_direction_on_ways_;

  // Configuration option indicating whether or not to process the alt_name key on the ways during the
  // parsing phase
  bool allow_alt_name_;

  // Configuration option indicating whether or not to process the urban key on the ways during the
  // parsing phase or to get the density during the enhancer phase
  bool use_urban_tag_;

  // Configuration option indicating whether or not to process the rest/service area keys on the ways
  // during the parsing phase
  bool use_rest_area_;

  // Configuration option indicating whether or not to process the admin iso code keys on the
  // nodes during the parsing phase or to get the admin info from the admin db
  bool use_admin_db_;

  // Road class assignment needs to be set to the highway cutoff for ferries and auto trains.
  RoadClass highway_cutoff_rc_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData& osmdata_;

  // Ways and nodes written to file, nodes are written in the order they appear in way (shape)
  std::unique_ptr<sequence<OSMWay>> ways_;
  std::unique_ptr<sequence<OSMWayNode>> way_nodes_;
  // When updating the references with the node information we keep the last index we looked at
  // this lets us only have to iterate over the whole set once
  size_t current_way_node_index_;
  uint64_t last_node_, last_way_, last_relation_;
  std::unordered_map<uint64_t, size_t> loop_nodes_;

  // user entered access
  std::unique_ptr<sequence<OSMAccess>> access_;
  // from complex restrictions
  std::unique_ptr<sequence<OSMRestriction>> complex_restrictions_from_;
  //  used to find out if a wayid is the to edge for a complex restriction
  std::unique_ptr<sequence<OSMRestriction>> complex_restrictions_to_;

  // bss nodes
  std::unique_ptr<sequence<OSMNode>> bss_nodes_;

  // used to set "culdesac" labels to loop roads correctly
  culdesac_processor culdesac_processor_;

  // empty objects initialized with defaults to use when no tags are present on objects
  Tags empty_node_results_;
  Tags empty_way_results_;
  Tags empty_relation_results_;
};

} // namespace

namespace valhalla {
namespace mjolnir {

OSMData PBFGraphParser::ParseWays(const boost::property_tree::ptree& pt,
                                  const std::vector<std::string>& input_files,
                                  const std::string& ways_file,
                                  const std::string& way_nodes_file,
                                  const std::string& access_file) {
  // TODO: option 1: each one threads makes an osmdata and we splice them together at the end
  // option 2: synchronize around adding things to a single osmdata. will have to test to see
  // which is the least expensive (memory and speed). leaning towards option 2
  //  unsigned int threads =
  //      std::max(static_cast<unsigned int>(1),
  //               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));

  // Create OSM data. Set the member pointer so that the parsing callback methods can use it.
  OSMData osmdata{};
  graph_callback callback(pt, osmdata);

  LOG_INFO("Parsing files for ways: " + boost::algorithm::join(input_files, ", "));

  // hold open all the files so that if something else (like diff application)
  // needs to mess with them we wont have troubles with inodes changing underneath us
  std::list<std::ifstream> file_handles;
  for (const auto& input_file : input_files) {
    file_handles.emplace_back(input_file, std::ios::binary);
    if (!file_handles.back().is_open()) {
      throw std::runtime_error("Unable to open: " + input_file);
    }
  }

  callback.reset(new sequence<OSMWay>(ways_file, true),
                 new sequence<OSMWayNode>(way_nodes_file, true),
                 new sequence<OSMAccess>(access_file, true), nullptr, nullptr, nullptr);
  // Parse the ways and find all node Ids needed (those that are part of a
  // way's node list. Iterate through each pbf input file.
  LOG_INFO("Parsing ways...");
  for (auto& file_handle : file_handles) {
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
        callback.last_relation_ = 0;
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::WAYS |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }

  // Clarifies types of loop roads and saves fixed ways.
  callback.culdesac_processor_.clarify_and_fix(*callback.way_nodes_, *callback.ways_);

  LOG_INFO("Finished with " + std::to_string(osmdata.osm_way_count) + " routable ways containing " +
           std::to_string(osmdata.osm_way_node_count) + " nodes");
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

  // we need to sort the access tags so that we can easily find them.
  LOG_INFO("Sorting osm access tags by way id...");
  {
    sequence<OSMAccess> access(access_file, false);
    access.sort([](const OSMAccess& a, const OSMAccess& b) { return a.way_id() < b.way_id(); });
  }

  LOG_INFO("Finished");

  // Return OSM data
  osmdata.initialized = true;
  return osmdata;
}

void PBFGraphParser::ParseRelations(const boost::property_tree::ptree& pt,
                                    const std::vector<std::string>& input_files,
                                    const std::string& complex_restriction_from_file,
                                    const std::string& complex_restriction_to_file,
                                    OSMData& osmdata) {
  // TODO: option 1: each one threads makes an osmdata and we splice them together at the end
  // option 2: synchronize around adding things to a single osmdata. will have to test to see
  // which is the least expensive (memory and speed). leaning towards option 2
  //  unsigned int threads =
  //      std::max(static_cast<unsigned int>(1),
  //               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));

  // Create OSM data. Set the member pointer so that the parsing callback methods can use it.
  graph_callback callback(pt, osmdata);

  // Read the OSMData to files if not initialized.
  if (!osmdata.initialized)
    callback.osmdata_.read_from_temp_files(pt.get<std::string>("tile_dir"));

  LOG_INFO("Parsing files for relations: " + boost::algorithm::join(input_files, ", "));

  // hold open all the files so that if something else (like diff application)
  // needs to mess with them we wont have troubles with inodes changing underneath us
  std::list<std::ifstream> file_handles;
  for (const auto& input_file : input_files) {
    file_handles.emplace_back(input_file, std::ios::binary);
    if (!file_handles.back().is_open()) {
      throw std::runtime_error("Unable to open: " + input_file);
    }
  }

  callback.reset(nullptr, nullptr, nullptr,
                 new sequence<OSMRestriction>(complex_restriction_from_file, true),
                 new sequence<OSMRestriction>(complex_restriction_to_file, true), nullptr);

  // Parse relations.
  LOG_INFO("Parsing relations...");
  for (auto& file_handle : file_handles) {
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
        callback.last_relation_ = 0;
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::RELATIONS |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  LOG_INFO("Finished with " + std::to_string(osmdata.restrictions.size()) + " simple restrictions");
  LOG_INFO("Finished with " + std::to_string(osmdata.lane_connectivity_map.size()) +
           " lane connections");

  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

  // Sort complex restrictions. Keep this scoped so the file handles are closed when done sorting.
  LOG_INFO("Sorting complex restrictions by from id...");
  {
    sequence<OSMRestriction> complex_restrictions_from(complex_restriction_from_file, false);
    complex_restrictions_from.sort(
        [](const OSMRestriction& a, const OSMRestriction& b) { return a < b; });
  }

  // Sort complex restrictions. Keep this scoped so the file handles are closed when done sorting.
  LOG_INFO("Sorting complex restrictions by to id...");
  {
    sequence<OSMRestriction> complex_restrictions_to(complex_restriction_to_file, false);
    complex_restrictions_to.sort(
        [](const OSMRestriction& a, const OSMRestriction& b) { return a < b; });
  }
  LOG_INFO("Finished");
}

void PBFGraphParser::ParseNodes(const boost::property_tree::ptree& pt,
                                const std::vector<std::string>& input_files,
                                const std::string& way_nodes_file,
                                const std::string& bss_nodes_file,
                                OSMData& osmdata) {
  // TODO: option 1: each one threads makes an osmdata and we splice them together at the end
  // option 2: synchronize around adding things to a single osmdata. will have to test to see
  // which is the least expensive (memory and speed). leaning towards option 2
  //  unsigned int threads =
  //      std::max(static_cast<unsigned int>(1),
  //               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));

  // Create OSM data. Set the member pointer so that the parsing callback methods can use it.
  graph_callback callback(pt, osmdata);

  // Read the OSMData to files if not initialized.
  if (!osmdata.initialized)
    callback.osmdata_.read_from_temp_files(pt.get<std::string>("tile_dir"));

  LOG_INFO("Parsing files for nodes: " + boost::algorithm::join(input_files, ", "));

  // hold open all the files so that if something else (like diff application)
  // needs to mess with them we wont have troubles with inodes changing underneath us
  std::list<std::ifstream> file_handles;
  for (const auto& input_file : input_files) {
    file_handles.emplace_back(input_file, std::ios::binary);
    if (!file_handles.back().is_open()) {
      throw std::runtime_error("Unable to open: " + input_file);
    }
  }

  if (pt.get<bool>("import_bike_share_stations", false)) {
    LOG_INFO("Parsing bss nodes...");
    for (auto& file_handle : file_handles) {
      callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
          callback.last_relation_ = 0;
      // we send a null way_nodes file so that only the bike share stations are parsed
      callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr,
                     new sequence<OSMNode>(bss_nodes_file, true));
      OSMPBF::Parser::parse(file_handle, static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES),
                            callback);
    }
  }
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

  // we need to sort the refs so that we can easily (sequentially) update them
  // during node processing, we use memory mapping here because otherwise we aren't
  // using much mem, the scoping makes sure to let it go when done sorting
  LOG_INFO("Sorting osm way node references by node id...");
  {
    sequence<OSMWayNode> way_nodes(way_nodes_file, false);
    way_nodes.sort(
        [](const OSMWayNode& a, const OSMWayNode& b) { return a.node.osmid_ < b.node.osmid_; });
  }

  // Parse node in all the input files. Skip any that are not marked from
  // being used in a way.
  // TODO: we know how many knows we expect, stop early once we have that many
  LOG_INFO("Parsing nodes...");
  for (auto& file_handle : file_handles) {
    // each time we parse nodes we have to run through the way nodes file from the beginning because
    // because osm node ids are only sorted at the single pbf file level
    callback.reset(nullptr, new sequence<OSMWayNode>(way_nodes_file, false), nullptr, nullptr,
                   nullptr, nullptr);
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
        callback.last_relation_ = 0;
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  uint64_t max_osm_id = callback.last_node_;
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
  LOG_INFO("Finished with " + std::to_string(osmdata.osm_node_count) +
           " nodes contained in routable ways");

  // we need to sort the refs so that we easily iterate over them for building edges
  // so we line them first by way index then by shape index of the node
  LOG_INFO("Sorting osm way node references by way index and node shape index...");
  {
    sequence<OSMWayNode> way_nodes(way_nodes_file, false);
    way_nodes.sort([](const OSMWayNode& a, const OSMWayNode& b) {
      if (a.way_index == b.way_index) {
        // TODO: if its equal we have screwed something up, should we check and throw here?
        return a.way_shape_node_index < b.way_shape_node_index;
      }
      return a.way_index < b.way_index;
    });
  }

  // Some OSM extracts do not have changeset Ids. For these set the max changeset Id
  // to the max OSM Id
  if (osmdata.max_changeset_id_ == 0) {
    osmdata.max_changeset_id_ = max_osm_id;
    LOG_INFO("Finished: max_osm_id " + std::to_string(osmdata.max_changeset_id_));
  } else {
    LOG_INFO("Finished: changeset id " + std::to_string(osmdata.max_changeset_id_));
  }

  // Log some information about extra node information and names
  LOG_INFO("Number of nodes with refs (exits) = " + std::to_string(osmdata.node_ref_count));
  LOG_INFO("Number of nodes with exit_to = " + std::to_string(osmdata.node_exit_to_count));
  LOG_INFO("Number of nodes with names = " + std::to_string(osmdata.node_name_count));
  LOG_INFO("Number of way refs = " + std::to_string(osmdata.way_ref.size()));
  LOG_INFO("Number of reverse way refs = " + std::to_string(osmdata.way_ref_rev.size()));
  LOG_INFO("Unique Node Strings (names, refs, etc.) = " + std::to_string(osmdata.node_names.Size()));
  LOG_INFO("Unique Strings (names, refs, etc.) = " + std::to_string(osmdata.name_offset_map.Size()));
}

} // namespace mjolnir
} // namespace valhalla
