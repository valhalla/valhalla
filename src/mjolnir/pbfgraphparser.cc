#include <optional>
#include <thread>
#include <utility>

#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include "baldr/complexrestriction.h"
#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/tilehierarchy.h"
#include "graph_lua_proc.h"
#include "midgard/aabb2.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/polyline2.h"
#include "midgard/sequence.h"
#include "midgard/tiles.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmaccess.h"
#include "mjolnir/osmlinguistic.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/timeparsing.h"
#include "mjolnir/util.h"
#include "proto/common.pb.h"

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

    include_platforms_ = pt.get<bool>("include_platforms", false);
    include_driveways_ = pt.get<bool>("include_driveways", true);
    include_construction_ = pt.get<bool>("include_construction", false);
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
    tag_handlers_["private_hgv"] = [this]() {
      if (tag_.second == "true")
        way_.set_destination_only_hgv(true);
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
        case Use::kElevator:
          way_.set_use(Use::kElevator);
          break;
        case Use::kSteps:
          way_.set_use(Use::kSteps);
          break;
        case Use::kEscalator:
          way_.set_use(Use::kEscalator);
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
        case Use::kConstruction:
          way_.set_use(Use::kConstruction);
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
    tag_handlers_["name:left"] = [this]() {
      if (!tag_.second.empty())
        name_left_ = tag_.second;
    };
    tag_handlers_["name:right"] = [this]() {
      if (!tag_.second.empty())
        name_right_ = tag_.second;
    };
    tag_handlers_["name:forward"] = [this]() {
      if (!tag_.second.empty())
        name_forward_ = tag_.second;
    };
    tag_handlers_["name:backward"] = [this]() {
      if (!tag_.second.empty())
        name_backward_ = tag_.second;
    };
    tag_handlers_["alt_name"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_)
        alt_name_ = tag_.second;
    };
    tag_handlers_["alt_name:left"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_)
        alt_name_left_ = tag_.second;
    };
    tag_handlers_["alt_name:right"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_)
        alt_name_right_ = tag_.second;
    };
    tag_handlers_["official_name"] = [this]() {
      if (!tag_.second.empty())
        official_name_ = tag_.second;
    };
    tag_handlers_["official_name:left"] = [this]() {
      if (!tag_.second.empty())
        official_name_left_ = tag_.second;
    };
    tag_handlers_["official_name:right"] = [this]() {
      if (!tag_.second.empty())
        official_name_right_ = tag_.second;
    };
    tag_handlers_["tunnel:name"] = [this]() {
      if (!tag_.second.empty())
        tunnel_name_ = tag_.second;
    };
    tag_handlers_["tunnel:name:left"] = [this]() {
      if (!tag_.second.empty())
        tunnel_name_left_ = tag_.second;
    };
    tag_handlers_["tunnel:name:right"] = [this]() {
      if (!tag_.second.empty())
        tunnel_name_right_ = tag_.second;
    };
    tag_handlers_["level"] = [this]() {
      if (!tag_.second.empty())
        way_.set_level_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["level:ref"] = [this]() {
      if (!tag_.second.empty())
        way_.set_level_ref_index(osmdata_.name_offset_map.index(tag_.second));
    };
    tag_handlers_["name:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_ipa_ = tag_.second;
      }
    };
    tag_handlers_["name:left:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_left_ipa_ = tag_.second;
      }
    };
    tag_handlers_["name:right:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["name:forward:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_forward_ipa_ = tag_.second;
      }
    };
    tag_handlers_["name:backward:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_backward_ipa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:pronunciation"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_ipa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:left:pronunciation"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_left_ipa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:right:pronunciation"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["official_name:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_ipa_ = tag_.second;
      }
    };
    tag_handlers_["official_name:left:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        official_name_left_ipa_ = tag_.second;
        has_pronunciation_tags_ = true;
      }
    };
    tag_handlers_["official_name:right:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_ipa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:left:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_left_ipa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:right:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["name:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["name:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["name:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["name:forward:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_forward_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["name:backward:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_backward_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["official_name:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["official_name:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["official_name:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["name:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_katakana_ = tag_.second;
      }
    };
    tag_handlers_["name:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["name:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["name:forward:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_forward_katakana_ = tag_.second;
      }
    };
    tag_handlers_["name:backward:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_backward_katakana_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_katakana_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["official_name:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_katakana_ = tag_.second;
      }
    };
    tag_handlers_["official_name:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["official_name:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_katakana_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["name:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_jeita_ = tag_.second;
      }
    };
    tag_handlers_["name:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["name:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_right_jeita_ = tag_.second;
      }
    };
    tag_handlers_["name:forward:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_forward_jeita_ = tag_.second;
      }
    };
    tag_handlers_["name:backward:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        name_backward_jeita_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_jeita_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["alt_name:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty() && allow_alt_name_) {
        has_pronunciation_tags_ = true;
        alt_name_right_jeita_ = tag_.second;
      }
    };
    tag_handlers_["official_name:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_jeita_ = tag_.second;
      }
    };
    tag_handlers_["official_name:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["official_name:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        official_name_right_jeita_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_jeita_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["tunnel:name:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        tunnel_name_right_jeita_ = tag_.second;
      }
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
    tag_handlers_["maxspeed:hgv:forward"] = [this]() {
      try {
        way_.set_truck_speed_forward(std::stof(tag_.second));
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["maxspeed:hgv:backward"] = [this]() {
      try {
        way_.set_truck_speed_backward(std::stof(tag_.second));
      } catch (const std::out_of_range& oor) {
        LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid_));
      }
    };
    tag_handlers_["maxspeed:conditional"] = [this]() {
      std::vector<std::string> tokens = GetTagTokens(tag_.second, '@');
      if (tokens.size() < 2) {
        return; // just ignore bad entries
      }

      uint8_t speed;
      if (tokens.at(0) == "no" || tokens.at(0) == "none") {
        // Handle autobahns that have unlimited speed during smaller part of the day
        speed = kUnlimitedSpeedLimit;
      } else {
        try {
          const float parsed = std::stof(tokens.at(0));
          if (parsed > kMaxAssumedSpeed) {
            // LOG_WARN("Ignoring maxspeed:conditional that exceedes max for way id: " +
            //          std::to_string(osmid_));
            return;
          }
          speed = static_cast<uint8_t>(parsed + 0.5f);
        } catch (const std::invalid_argument&) {
          return; // ignore strange things like 'walk @...'
        }
      }

      std::vector<std::string> conditions = GetTagTokens(tokens.at(1), ';');
      for (const auto& c : conditions) {
        std::vector<uint64_t> values = get_time_range(c);
        for (const auto& v : values) {
          ConditionalSpeedLimit limit = {};
          limit.td_ = TimeDomain(v);
          limit.speed_ = speed;
          osmdata_.conditional_speeds.emplace(osmid_, limit);
        }
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
    tag_handlers_["hazmat_forward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kHazmat);
      restriction.set_value(tag_.second == "true" ? true : false);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kForward);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["hazmat_backward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kHazmat);
      restriction.set_value(tag_.second == "true" ? true : false);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kBackward);
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
    tag_handlers_["maxheight_forward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxHeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      restriction.set_direction(AccessRestrictionDirection::kForward);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxheight_backward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxHeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      restriction.set_direction(AccessRestrictionDirection::kBackward);
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
    tag_handlers_["maxwidth_forward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWidth);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      restriction.set_direction(AccessRestrictionDirection::kForward);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxwidth_backward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWidth);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess | kAutoAccess | kHOVAccess | kTaxiAccess | kBusAccess);
      restriction.set_direction(AccessRestrictionDirection::kBackward);
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
    tag_handlers_["maxlength_forward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxLength);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kForward);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxlength_backward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxLength);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kBackward);
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
    tag_handlers_["maxweight_forward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kForward);
      osmdata_.access_restrictions.insert(
          AccessRestrictionsMultiMap::value_type(osmid_, restriction));
    };
    tag_handlers_["maxweight_backward"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxWeight);
      restriction.set_value(std::stof(tag_.second) * 100);
      restriction.set_modes(kTruckAccess);
      restriction.set_direction(AccessRestrictionDirection::kBackward);
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
    tag_handlers_["maxaxles"] = [this]() {
      OSMAccessRestriction restriction;
      restriction.set_type(AccessType::kMaxAxles);
      restriction.set_value(std::stoul(tag_.second));
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
        ref_ = tag_.second;
      }
    };
    tag_handlers_["ref:left"] = [this]() {
      if (!tag_.second.empty())
        ref_left_ = tag_.second;
    };
    tag_handlers_["ref:right"] = [this]() {
      if (!tag_.second.empty())
        ref_right_ = tag_.second;
    };
    tag_handlers_["int_ref"] = [this]() {
      if (!tag_.second.empty()) {
        int_ref_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:left"] = [this]() {
      if (!tag_.second.empty())
        int_ref_left_ = tag_.second;
    };
    tag_handlers_["int_ref:right"] = [this]() {
      if (!tag_.second.empty())
        int_ref_right_ = tag_.second;
    };
    tag_handlers_["ref:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          ref_ipa_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
          pronunciationMap[std::make_pair(t, ipa)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["ref:left:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_left_ipa_ = tag_.second;
      }
    };
    tag_handlers_["ref:right:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          int_ref_ipa_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
          pronunciationMap[std::make_pair(t, ipa)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["int_ref:left:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_left_ipa_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:right:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_right_ipa_ = tag_.second;
      }
    };
    tag_handlers_["ref:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          ref_nt_sampa_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
          pronunciationMap[std::make_pair(t, nt_sampa)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["ref:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["ref:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          int_ref_nt_sampa_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
          pronunciationMap[std::make_pair(t, nt_sampa)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["int_ref:left:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_left_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:right:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_right_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["ref:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          ref_katakana_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
          pronunciationMap[std::make_pair(t, katakana)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["ref:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["ref:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          int_ref_katakana_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
          pronunciationMap[std::make_pair(t, katakana)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["int_ref:left:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_left_katakana_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:right:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_right_katakana_ = tag_.second;
      }
    };
    tag_handlers_["ref:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          ref_jeita_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
          pronunciationMap[std::make_pair(t, jeita)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["ref:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["ref:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        ref_right_jeita_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        if (use_direction_on_ways_)
          int_ref_jeita_ = tag_.second;
        else {
          const uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
          pronunciationMap[std::make_pair(t, jeita)] = osmdata_.name_offset_map.index(tag_.second);
        }
      }
    };
    tag_handlers_["int_ref:left:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_left_jeita_ = tag_.second;
      }
    };
    tag_handlers_["int_ref:right:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        int_ref_right_jeita_ = tag_.second;
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
    tag_handlers_["direction:pronunciation"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        direction_pronunciation_ = tag_.second;
    };
    tag_handlers_["int_direction:pronunciation"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        int_direction_pronunciation_ = tag_.second;
    };
    tag_handlers_["direction:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        direction_pronunciation_nt_sampa_ = tag_.second;
    };
    tag_handlers_["int_direction:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        int_direction_pronunciation_nt_sampa_ = tag_.second;
    };
    tag_handlers_["direction:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        direction_pronunciation_katakana_ = tag_.second;
    };
    tag_handlers_["int_direction:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        int_direction_pronunciation_katakana_ = tag_.second;
    };
    tag_handlers_["direction:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        direction_pronunciation_jeita_ = tag_.second;
    };
    tag_handlers_["int_direction:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty() && use_direction_on_ways_)
        int_direction_pronunciation_jeita_ = tag_.second;
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

    // surface and tracktype tag should win over smoothness.
    tag_handlers_["smoothness"] = [this]() {
      if (!has_surface_tag_ && !has_tracktype_tag_) {
        has_surface_ = true;
        if (tag_.second == "excellent" || tag_.second == "good") {
          way_.set_surface(Surface::kPavedSmooth);
        } else if (tag_.second == "intermediate") {
          way_.set_surface(Surface::kPavedRough);
        } else if (tag_.second == "bad") {
          way_.set_surface(Surface::kCompacted);
        } else if (tag_.second == "very_bad") {
          way_.set_surface(Surface::kDirt);
        } else if (tag_.second == "horrible") {
          way_.set_surface(Surface::kGravel);
        } else if (tag_.second == "very_horrible") {
          way_.set_surface(Surface::kPath);
        } else {
          has_surface_ = false;
        }
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
    tag_handlers_["indoor"] = [this]() { way_.set_indoor(tag_.second == "yes" ? true : false); };
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
        destination_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:forward"] = [this]() {
      if (!tag_.second.empty()) {
        destination_forward_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:backward"] = [this]() {
      if (!tag_.second.empty()) {
        destination_backward_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:ref"] = [this]() {
      if (!tag_.second.empty()) {
        destination_ref_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:ref:to"] = [this]() {
      if (!tag_.second.empty()) {
        destination_ref_to_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:int_ref"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_int_ref_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:int_ref:to"] = [this]() {
      if (!tag_.second.empty()) {
        way_.set_destination_int_ref_to_index(osmdata_.name_offset_map.index(tag_.second));
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:street"] = [this]() {
      if (!tag_.second.empty()) {
        destination_street_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:street:to"] = [this]() {
      if (!tag_.second.empty()) {
        destination_street_to_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["junction:ref"] = [this]() {
      if (!tag_.second.empty()) {
        junction_ref_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["junction:name"] = [this]() {
      if (!tag_.second.empty()) {
        junction_name_ = tag_.second;
        way_.set_exit(true);
      }
    };
    tag_handlers_["destination:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:forward:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_forward_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:backward:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_backward_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:to:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_to_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:to:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_to_ipa_ = tag_.second;
      }
    };
    tag_handlers_["junction:ref:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_ref_ipa_ = tag_.second;
      }
    };
    tag_handlers_["junction:name:pronunciation"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_name_ipa_ = tag_.second;
      }
    };
    tag_handlers_["destination:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:forward:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_forward_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:backward:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_backward_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:to:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_to_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:to:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_to_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["junction:ref:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_ref_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["junction:name:pronunciation:nt-sampa"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_name_nt_sampa_ = tag_.second;
      }
    };
    tag_handlers_["destination:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:forward:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_forward_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:backward:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_backward_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:to:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_to_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:to:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_to_katakana_ = tag_.second;
      }
    };
    tag_handlers_["junction:ref:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_ref_katakana_ = tag_.second;
      }
    };
    tag_handlers_["junction:name:pronunciation:katakana"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_name_katakana_ = tag_.second;
      }
    };
    tag_handlers_["destination:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:forward:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_forward_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:backward:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_backward_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:ref:to:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_ref_to_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_jeita_ = tag_.second;
      }
    };
    tag_handlers_["destination:street:to:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        destination_street_to_jeita_ = tag_.second;
      }
    };
    tag_handlers_["junction:ref:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_ref_jeita_ = tag_.second;
      }
    };
    tag_handlers_["junction:name:pronunciation:jeita"] = [this]() {
      if (!tag_.second.empty()) {
        has_pronunciation_tags_ = true;
        junction_name_jeita_ = tag_.second;
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
    tag_handlers_["lit"] = [this]() { way_.set_lit(tag_.second == "true" ? true : false); };
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
    std::optional<Tags> results = std::nullopt;
    if (bss_nodes_) {
      // Get tags - do't bother with Lua callout if the taglist is empty
      if (tags.size() > 0) {
        results = lua_.Transform(OSMType::kNode, osmid, tags);
      } else {
        results = empty_node_results_;
      }

      // bail if there is nothing bike related
      Tags::const_iterator found;
      if (!results || (found = results->find("amenity")) == results->end() ||
          found->second != "bicycle_rental") {
        return;
      }

      // Create a new node and set its attributes
      OSMNode n{osmid};
      n.set_latlng(lng, lat);
      n.set_type(NodeType::kBikeShare);
      valhalla::BikeShareStationInfo bss_info;

      for (auto& key_value : *results) {
        if (key_value.first == "name") {
          bss_info.set_name(key_value.second);
        } else if (key_value.first == "network") {
          bss_info.set_network(key_value.second);
        } else if (key_value.first == "ref") {
          bss_info.set_ref(key_value.second);
        } else if (key_value.first == "capacity") {
          auto capacity = std::strtoul(key_value.second.c_str(), nullptr, 10);
          if (capacity > 0) {
            bss_info.set_capacity(capacity);
          }
        } else if (key_value.first == "operator") {
          bss_info.set_operator_(key_value.second);
        }
      }

      std::string buffer;
      bss_info.SerializeToString(&buffer);
      n.set_bss_info_index(osmdata_.node_names.index(buffer));
      ++osmdata_.node_name_count;

      bss_nodes_->push_back(n);
      return; // we are done.
    }

    // if we found all of the node ids we were looking for already we can bail
    if (current_way_node_index_ >= way_nodes_->size()) {
      return;
    }

    // if the current osmid of this node of this pbf file is greater than the waynode we are looking
    // for then it must be in another pbf file. so we need to move on to the next waynode that could
    // possibly actually be in this pbf file
    if (osmid > (*(*way_nodes_)[current_way_node_index_]).node.osmid_) {
      current_way_node_index_ = way_nodes_->find_first_of(
          OSMWayNode{{osmid}},
          [](const OSMWayNode& a, const OSMWayNode& b) { return a.node.osmid_ <= b.node.osmid_; },
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

    const auto barrier_toll_booth = results->find("barrier");
    bool is_barrier_toll_booth =
        (barrier_toll_booth != results->end()) && (barrier_toll_booth->second == "toll_booth");

    const auto highway_toll_gantry = results->find("highway");
    bool is_highway_toll_gantry =
        (highway_toll_gantry != results->end()) && (highway_toll_gantry->second == "toll_gantry");

    bool is_toll_node = is_barrier_toll_booth || is_highway_toll_gantry;
    bool named_toll_node = false;

    OSMNode n;
    OSMNodeLinguistic linguistics;
    n.set_id(osmid);
    n.set_latlng(lng, lat);
    bool intersection = false;
    if (is_highway_junction) {
      n.set_type(NodeType::kMotorWayJunction);
    }
    ref_ = ref_language_ = ref_w_lang_ = name_ = language_ = name_w_lang_ = {};
    name_ipa_ = ref_ipa_ = name_nt_sampa_ = ref_nt_sampa_ = name_katakana_ = ref_katakana_ =
        name_jeita_ = ref_jeita_ = {};

    for (const auto& tag : *results) {
      tag_ = tag;

      bool is_lang_pronunciation = false;
      std::size_t found = tag_.first.find(":pronunciation");
      if (found != std::string::npos)
        is_lang_pronunciation = true;

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
        // Add the name to the unique node names list and store its index in the OSM node.

        // TODO  Need to process ref:right and ref:left and add correctly in graphbuilder.
        // ref:left means the ref for the left exit and ref:right means the ref for the
        // right exit at a split
        ref_ = tag.second;
        ++osmdata_.node_ref_count;
      } else if (tag.first == "name" &&
                 (is_highway_junction || maybe_named_junction || is_toll_node) && hasTag) {
        // Add the name to the unique node names list and store its index in the OSM node
        name_ = tag.second;
        ++osmdata_.node_name_count;
        named_junction = maybe_named_junction;
        named_toll_node = is_toll_node;
      } else if (tag.first == "name:pronunciation") {
        name_ipa_ = tag.second;
      } else if (tag.first == "name:pronunciation:nt-sampa") {
        name_nt_sampa_ = tag.second;
      } else if (tag.first == "name:pronunciation:katakana") {
        name_katakana_ = tag.second;
      } else if (tag.first == "name:pronunciation:jeita") {
        name_jeita_ = tag.second;
      } else if (tag.first == "ref:pronunciation") {
        ref_ipa_ = tag.second;
      } else if (tag.first == "ref:pronunciation:nt-sampa") {
        ref_nt_sampa_ = tag.second;
      } else if (tag.first == "ref:pronunciation:katakana") {
        ref_katakana_ = tag.second;
      } else if (tag.first == "ref:pronunciation:jeita") {
        ref_jeita_ = tag.second;
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
      } else if (tag.first == "building_entrance" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kBuildingEntrance);
      } else if (tag.first == "elevator" && tag.second == "true") {
        osmdata_.edge_count += !intersection;
        intersection = true;
        n.set_type(NodeType::kElevator);
      } else if (tag.first == "access_mask") {
        n.set_access(std::stoi(tag.second));
      } else if (tag.first == "tagged_access") {
        n.set_tagged_access(std::stoi(tag.second));
      } else if (tag.first == "private") {
        n.set_private_access(tag.second == "true");
      } else if (!is_lang_pronunciation) {
        if (boost::algorithm::starts_with(tag.first, "name:") &&
            (is_highway_junction || maybe_named_junction || is_toll_node) && hasTag) {
          ProcessNameTag(tag_, name_w_lang_, language_);
          ++osmdata_.node_name_count;
          named_junction = maybe_named_junction;
        } else if (boost::algorithm::starts_with(tag_.first, "ref:")) {
          ProcessNameTag(tag_, ref_w_lang_, ref_language_);
          ++osmdata_.node_ref_count;
        }
      } else {
        std::string t = tag_.first;
        PronunciationAlphabet alphabet = PronunciationAlphabet::kIpa;
        std::size_t found = t.find(":nt-sampa");
        if (found != std::string::npos)
          alphabet = PronunciationAlphabet::kNtSampa;
        else {
          found = t.find(":katakana");
          if (found != std::string::npos)
            alphabet = PronunciationAlphabet::kKatakana;
          else {
            found = t.find(":jeita");
            if (found != std::string::npos)
              alphabet = PronunciationAlphabet::kJeita;
          }
        }
        if (boost::algorithm::starts_with(t, "name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kNodeName, alphabet, &linguistics);
        } else if (boost::algorithm::starts_with(t, "ref:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kNodeRef, alphabet, &linguistics);
        }
      }
    }

    // begin name logic
    std::string l = language_;
    ProcessName(name_w_lang_, name_, language_);
    n.set_name_index(osmdata_.node_names.index(name_));
    linguistics.set_name_lang_index(osmdata_.node_names.index(language_));

    // begin ref logic
    l = ref_language_;
    ProcessName(ref_w_lang_, ref_, ref_language_);
    n.set_ref_index(osmdata_.node_names.index(ref_));
    linguistics.set_ref_lang_index(osmdata_.node_names.index(ref_language_));

    ProcessPronunciationName(OSMLinguistic::Type::kNodeName, PronunciationAlphabet::kIpa, name_ipa_,
                             &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeName, PronunciationAlphabet::kKatakana,
                             name_katakana_, &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeName, PronunciationAlphabet::kJeita,
                             name_jeita_, &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeName, PronunciationAlphabet::kNtSampa,
                             name_nt_sampa_, &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeRef, PronunciationAlphabet::kIpa, ref_ipa_,
                             &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeRef, PronunciationAlphabet::kKatakana,
                             ref_katakana_, &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeRef, PronunciationAlphabet::kJeita, ref_jeita_,
                             &linguistics);

    ProcessPronunciationName(OSMLinguistic::Type::kNodeRef, PronunciationAlphabet::kNtSampa,
                             ref_nt_sampa_, &linguistics);

    if (!linguistics.isEmpty()) {
      n.set_linguistic_info_index(osmdata_.node_linguistic_count);
      node_linguistics_->push_back(linguistics);
      ++osmdata_.node_linguistic_count;
    }

    // Different types of named nodes are tagged as a named intersection
    n.set_named_intersection(named_junction || named_toll_node);

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

    try {
      // Throw away use if include_driveways_ is false
      Tags::const_iterator use;
      if (!include_driveways_ && (use = results.find("use")) != results.end() &&
          static_cast<Use>(std::stoi(use->second)) == Use::kDriveway) {

        // only private use.
        Tags::const_iterator priv;
        if ((priv = results.find("private")) != results.end() && priv->second == "true") {
          return;
        }
      }
      // Throw away constructions if include_construction_ is false
      if (!include_construction_ && (use = results.find("use")) != results.end() &&
          static_cast<Use>(std::stoi(use->second)) == Use::kConstruction) {
        return;
      }
      // Throw away platforms if include_platforms_ is false
      if (!include_platforms_ && (use = results.find("use")) != results.end() &&
          static_cast<Use>(std::stoi(use->second)) == Use::kPlatform) {
        return;
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
    has_average_speed_ = false, has_advisory_speed_ = false, has_surface_ = true,

    name_ = language_ = name_w_lang_ = service_ = amenity_ = name_left_ = name_right_ = lang_left_ =
        lang_right_ = name_left_w_lang_ = name_right_w_lang_ = {};

    name_forward_ = name_backward_ = lang_forward_ = lang_backward_ = name_forward_w_lang_ =
        name_backward_w_lang_ = {};

    ref_ipa_ = ref_left_ipa_ = ref_right_ipa_ = ref_katakana_ = ref_left_katakana_ =
        ref_right_katakana_ = ref_jeita_ = ref_left_jeita_ = ref_right_jeita_ = ref_nt_sampa_ =
            ref_left_nt_sampa_ = ref_right_nt_sampa_ = {};

    int_ref_ipa_ = int_ref_left_ipa_ = int_ref_right_ipa_ = int_ref_katakana_ =
        int_ref_left_katakana_ = int_ref_right_katakana_ = int_ref_jeita_ = int_ref_left_jeita_ =
            int_ref_right_jeita_ = int_ref_nt_sampa_ = int_ref_left_nt_sampa_ =
                int_ref_right_nt_sampa_ = {};

    name_ipa_ = name_left_ipa_ = name_right_ipa_ = name_forward_ipa_ = name_backward_ipa_ =
        name_katakana_ = name_left_katakana_ = name_right_katakana_ = name_forward_katakana_ =
            name_backward_katakana_ = name_jeita_ = name_left_jeita_ = name_right_jeita_ =
                name_forward_jeita_ = name_backward_jeita_ = name_nt_sampa_ = name_left_nt_sampa_ =
                    name_right_nt_sampa_ = name_forward_nt_sampa_ = name_backward_nt_sampa_ = {};

    alt_name_ipa_ = alt_name_left_ipa_ = alt_name_right_ipa_ = alt_name_katakana_ =
        alt_name_left_katakana_ = alt_name_right_katakana_ = alt_name_jeita_ = alt_name_left_jeita_ =
            alt_name_right_jeita_ = alt_name_nt_sampa_ = alt_name_left_nt_sampa_ =
                alt_name_right_nt_sampa_ = {};

    official_name_ipa_ = official_name_left_ipa_ = official_name_right_ipa_ =
        official_name_katakana_ = official_name_left_katakana_ = official_name_right_katakana_ =
            official_name_jeita_ = official_name_left_jeita_ = official_name_right_jeita_ =
                official_name_nt_sampa_ = official_name_left_nt_sampa_ =
                    official_name_right_nt_sampa_ = {};

    tunnel_name_ipa_ = tunnel_name_left_ipa_ = tunnel_name_right_ipa_ = tunnel_name_katakana_ =
        tunnel_name_left_katakana_ = tunnel_name_right_katakana_ = tunnel_name_jeita_ =
            tunnel_name_left_jeita_ = tunnel_name_right_jeita_ = tunnel_name_nt_sampa_ =
                tunnel_name_left_nt_sampa_ = tunnel_name_right_nt_sampa_ = {};

    official_name_ = official_language_ = official_name_w_lang_ = official_name_left_ =
        official_name_right_ = official_lang_left_ = official_lang_right_ =
            official_name_left_w_lang_ = official_name_right_w_lang_ = {};

    tunnel_name_ = tunnel_language_ = tunnel_name_w_lang_ = tunnel_name_left_ = tunnel_name_right_ =
        tunnel_lang_left_ = tunnel_lang_right_ = tunnel_name_left_w_lang_ =
            tunnel_name_right_w_lang_ = {};

    alt_name_ = alt_language_ = alt_name_w_lang_ = alt_name_left_ = alt_name_right_ = alt_lang_left_ =
        alt_lang_right_ = alt_name_left_w_lang_ = alt_name_right_w_lang_ = {};

    destination_ = destination_language_ = destination_w_lang_ = destination_forward_ =
        destination_forward_language_ = destination_forward_w_lang_ = destination_backward_ =
            destination_backward_language_ = destination_backward_w_lang_ = {};

    destination_ref_ = destination_ref_language_ = destination_ref_w_lang_ = destination_ref_to_ =
        destination_ref_to_language_ = destination_ref_to_w_lang_ = destination_street_ =
            destination_street_language_ = destination_street_w_lang_ = destination_street_to_ =
                destination_street_to_language_ = destination_street_to_w_lang_ = {};

    junction_ref_ = junction_ref_language_ = junction_ref_w_lang_ = junction_name_ =
        junction_name_language_ = junction_name_w_lang_ = {};

    destination_ipa_ = destination_forward_ipa_ = destination_backward_ipa_ = destination_katakana_ =
        destination_forward_katakana_ = destination_backward_katakana_ = destination_jeita_ =
            destination_forward_jeita_ = destination_backward_jeita_ = destination_nt_sampa_ =
                destination_forward_nt_sampa_ = destination_backward_nt_sampa_ = {};

    destination_ref_ipa_ = destination_ref_to_ipa_ = destination_street_ipa_ =
        destination_street_to_ipa_ = junction_ref_ipa_ = junction_name_ipa_ = {};
    destination_ref_nt_sampa_ = destination_ref_to_nt_sampa_ = destination_street_nt_sampa_ =
        destination_street_to_nt_sampa_ = junction_ref_nt_sampa_ = junction_name_nt_sampa_ = {};
    destination_ref_katakana_ = destination_ref_to_katakana_ = destination_street_katakana_ =
        destination_street_to_katakana_ = junction_ref_katakana_ = junction_name_katakana_ = {};
    destination_ref_jeita_ = destination_ref_to_jeita_ = destination_street_jeita_ =
        destination_street_to_jeita_ = junction_ref_jeita_ = junction_name_jeita_ = {};
    // Process tags
    way_ = OSMWay{osmid_};
    way_.set_node_count(nodes.size());

    osm_access_ = OSMAccess{osmid_};
    pronunciationMap.clear();
    langMap.clear();
    has_user_tags_ = false, has_pronunciation_tags_ = false;
    ref_ = ref_language_ = ref_w_lang_ = ref_left_ = ref_right_ = ref_lang_left_ = ref_lang_right_ =
        ref_left_w_lang_ = ref_right_w_lang_ = {};

    int_ref_ = int_ref_language_ = int_ref_w_lang_ = int_ref_left_ = int_ref_right_ =
        int_ref_lang_left_ = int_ref_lang_right_ = int_ref_left_w_lang_ = int_ref_right_w_lang_ = {};

    direction_ = int_direction_ = {};
    direction_pronunciation_ = int_direction_pronunciation_ = {};

    const auto& surface_exists = results.find("surface");
    has_surface_tag_ = (surface_exists != results.end());
    if (!has_surface_tag_) {
      has_surface_ = false;
    }

    const auto& tracktype_exists = results.find("tracktype");
    has_tracktype_tag_ = (tracktype_exists != results.end());

    way_.set_drive_on_right(true); // default

    for (const auto& kv : results) {
      tag_ = kv;

      bool is_lang_pronunciation = false;
      std::size_t found = tag_.first.find(":pronunciation");
      if (found != std::string::npos)
        is_lang_pronunciation = true;

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
      else if (boost::algorithm::starts_with(tag_.first, "motorcar:conditional") ||
               boost::algorithm::starts_with(tag_.first, "motor_vehicle:conditional") ||
               boost::algorithm::starts_with(tag_.first, "bicycle:conditional") ||
               boost::algorithm::starts_with(tag_.first, "motorcycle:conditional") ||
               boost::algorithm::starts_with(tag_.first, "foot:conditional") ||
               boost::algorithm::starts_with(tag_.first, "pedestrian:conditional") ||
               boost::algorithm::starts_with(tag_.first, "hgv:conditional") ||
               boost::algorithm::starts_with(tag_.first, "moped:conditional") ||
               boost::algorithm::starts_with(tag_.first, "mofa:conditional") ||
               boost::algorithm::starts_with(tag_.first, "psv:conditional") ||
               boost::algorithm::starts_with(tag_.first, "taxi:conditional") ||
               boost::algorithm::starts_with(tag_.first, "bus:conditional") ||
               boost::algorithm::starts_with(tag_.first, "hov:conditional") ||
               boost::algorithm::starts_with(tag_.first, "emergency:conditional")) {

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
          if (boost::algorithm::starts_with(tag_.first, "motor_vehicle:conditional")) {
            mode = (kAutoAccess | kTruckAccess | kEmergencyAccess | kTaxiAccess | kBusAccess |
                    kHOVAccess | kMopedAccess | kMotorcycleAccess);
          } else if (boost::algorithm::starts_with(tag_.first, "motorcar:conditional")) {
            if (type == AccessType::kTimedAllowed) {
              mode = kAutoAccess | kHOVAccess | kTaxiAccess;
            } else {
              mode = (kAutoAccess | kTruckAccess | kEmergencyAccess | kTaxiAccess | kBusAccess |
                      kHOVAccess);
            }
          } else if (boost::algorithm::starts_with(tag_.first, "bicycle:conditional")) {
            mode = kBicycleAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "foot:conditional") ||
                     boost::algorithm::starts_with(tag_.first, "pedestrian:conditional")) {
            mode = (kPedestrianAccess | kWheelchairAccess);
          } else if (boost::algorithm::starts_with(tag_.first, "hgv:conditional")) {
            mode = kTruckAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "moped:conditional") ||
                     boost::algorithm::starts_with(tag_.first, "mofa:conditional")) {
            mode = kMopedAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "motorcycle:conditional")) {
            mode = kMotorcycleAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "psv:conditional")) {
            mode = (kTaxiAccess | kBusAccess);
          } else if (boost::algorithm::starts_with(tag_.first, "taxi:conditional")) {
            mode = kTaxiAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "bus:conditional")) {
            mode = kBusAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "hov:conditional")) {
            mode = kHOVAccess;
          } else if (boost::algorithm::starts_with(tag_.first, "emergency:conditional")) {
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
      } else if (!is_lang_pronunciation) {
        if (boost::algorithm::starts_with(tag_.first, "name:left:")) {
          ProcessLeftRightNameTag(tag_, name_left_w_lang_, lang_left_);
        } else if (boost::algorithm::starts_with(tag_.first, "name:right:")) {
          ProcessLeftRightNameTag(tag_, name_right_w_lang_, lang_right_);
        } else if (boost::algorithm::starts_with(tag_.first, "name:forward:")) {
          ProcessLeftRightNameTag(tag_, name_forward_w_lang_, lang_forward_);
        } else if (boost::algorithm::starts_with(tag_.first, "name:backward:")) {
          ProcessLeftRightNameTag(tag_, name_backward_w_lang_, lang_backward_);
        } else if (boost::algorithm::starts_with(tag_.first, "name:")) {
          ProcessNameTag(tag_, name_w_lang_, language_);
        } else if (boost::algorithm::starts_with(tag_.first, "official_name:left:")) {
          ProcessLeftRightNameTag(tag_, official_name_left_w_lang_, official_lang_left_);
        } else if (boost::algorithm::starts_with(tag_.first, "official_name:right:")) {
          ProcessLeftRightNameTag(tag_, official_name_right_w_lang_, official_lang_right_);
        } else if (boost::algorithm::starts_with(tag_.first, "official_name:")) {
          ProcessNameTag(tag_, official_name_w_lang_, official_language_);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(tag_.first, "alt_name:left:")) {
          ProcessLeftRightNameTag(tag_, alt_name_left_w_lang_, alt_lang_left_);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(tag_.first, "alt_name:right:")) {
          ProcessLeftRightNameTag(tag_, alt_name_right_w_lang_, alt_lang_right_);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(tag_.first, "alt_name:")) {
          ProcessNameTag(tag_, alt_name_w_lang_, alt_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "ref:left:")) {
          ProcessLeftRightNameTag(tag_, ref_left_w_lang_, ref_lang_left_);
        } else if (boost::algorithm::starts_with(tag_.first, "ref:right:")) {
          ProcessLeftRightNameTag(tag_, ref_right_w_lang_, ref_lang_right_);
        } else if (boost::algorithm::starts_with(tag_.first, "ref:")) {
          ProcessNameTag(tag_, ref_w_lang_, ref_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "int_ref:left:")) {
          ProcessLeftRightNameTag(tag_, int_ref_left_w_lang_, int_ref_lang_left_);
        } else if (boost::algorithm::starts_with(tag_.first, "int_ref:right:")) {
          ProcessLeftRightNameTag(tag_, int_ref_right_w_lang_, int_ref_lang_right_);
        } else if (boost::algorithm::starts_with(tag_.first, "int_ref:")) {
          ProcessNameTag(tag_, int_ref_w_lang_, int_ref_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "tunnel:name:left:")) {
          ProcessLeftRightNameTag(tag_, tunnel_name_left_w_lang_, tunnel_lang_left_);
        } else if (boost::algorithm::starts_with(tag_.first, "tunnel:name:right:")) {
          ProcessLeftRightNameTag(tag_, tunnel_name_right_w_lang_, tunnel_lang_right_);
        } else if (boost::algorithm::starts_with(tag_.first, "tunnel:name:")) {
          ProcessNameTag(tag_, tunnel_name_w_lang_, tunnel_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:backward:")) {
          ProcessNameTag(tag_, destination_backward_w_lang_, destination_backward_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:forward:")) {
          ProcessNameTag(tag_, destination_forward_w_lang_, destination_forward_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:ref:to:")) {
          ProcessNameTag(tag_, destination_ref_to_w_lang_, destination_ref_to_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:ref:")) {
          ProcessNameTag(tag_, destination_ref_w_lang_, destination_ref_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:street:to:")) {
          ProcessNameTag(tag_, destination_street_to_w_lang_, destination_street_to_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:street:")) {
          ProcessNameTag(tag_, destination_street_w_lang_, destination_street_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:")) {
          ProcessNameTag(tag_, destination_w_lang_, destination_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "junction:ref:")) {
          ProcessNameTag(tag_, junction_ref_w_lang_, junction_ref_language_);
        } else if (boost::algorithm::starts_with(tag_.first, "junction:name:")) {
          ProcessNameTag(tag_, junction_name_w_lang_, junction_name_language_);
        }
      } else { // is_lang_pronunciation = true
        std::string t = tag_.first;
        PronunciationAlphabet alphabet = PronunciationAlphabet::kIpa;
        std::size_t found = t.find(":nt-sampa");
        if (found != std::string::npos)
          alphabet = PronunciationAlphabet::kNtSampa;
        else {
          found = t.find(":katakana");
          if (found != std::string::npos)
            alphabet = PronunciationAlphabet::kKatakana;
          else {
            found = t.find(":jeita");
            if (found != std::string::npos)
              alphabet = PronunciationAlphabet::kJeita;
          }
        }

        if (boost::algorithm::starts_with(t, "name:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kNameLeft, alphabet);
        } else if (boost::algorithm::starts_with(t, "name:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kNameRight, alphabet);
        } else if (boost::algorithm::starts_with(t, "name:forward:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kNameForward, alphabet);
        } else if (boost::algorithm::starts_with(t, "name:backward:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kNameBackward, alphabet);
        } else if (boost::algorithm::starts_with(t, "name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kName, alphabet);
        } else if (boost::algorithm::starts_with(t, "official_name:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kOfficialNameLeft, alphabet);
        } else if (boost::algorithm::starts_with(t, "official_name:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kOfficialNameRight, alphabet);
        } else if (boost::algorithm::starts_with(t, "official_name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kOfficialName, alphabet);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(t, "alt_name:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kAltNameLeft, alphabet);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(t, "alt_name:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kAltNameRight, alphabet);
        } else if (allow_alt_name_ && boost::algorithm::starts_with(t, "alt_name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kAltName, alphabet);
        } else if (boost::algorithm::starts_with(t, "ref:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kRefLeft, alphabet);
        } else if (boost::algorithm::starts_with(t, "ref:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kRefRight, alphabet);
        } else if (boost::algorithm::starts_with(t, "ref:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kRef, alphabet);
        } else if (boost::algorithm::starts_with(t, "int_ref:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kIntRefLeft, alphabet);
        } else if (boost::algorithm::starts_with(t, "int_ref:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kIntRefRight, alphabet);
        } else if (boost::algorithm::starts_with(t, "int_ref:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kIntRef, alphabet);
        } else if (boost::algorithm::starts_with(t, "tunnel:name:left:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kTunnelNameLeft, alphabet);
        } else if (boost::algorithm::starts_with(t, "tunnel:name:right:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kTunnelNameRight, alphabet);
        } else if (boost::algorithm::starts_with(t, "tunnel:name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kTunnelName, alphabet);
        } else if (boost::algorithm::starts_with(t, "destination:forward:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationForward, alphabet);
        } else if (boost::algorithm::starts_with(t, "destination:backward:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationBackward, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:ref:to:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationRefTo, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:ref:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationRef, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:street:to:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationStreetTo, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "destination:street:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kDestinationStreet, alphabet);
        } else if (boost::algorithm::starts_with(t, "destination:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kDestination, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "junction:ref:")) {
          ProcessLeftRightPronunciationTag(OSMLinguistic::Type::kJunctionRef, alphabet);
        } else if (boost::algorithm::starts_with(tag_.first, "junction:name:")) {
          ProcessPronunciationTag(OSMLinguistic::Type::kJunctionName, alphabet);
        }
      }
    }

    if (!use_direction_on_ways_) {

      uint8_t alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
      uint8_t right = static_cast<uint8_t>(OSMLinguistic::Type::kRefRight);
      uint8_t left = static_cast<uint8_t>(OSMLinguistic::Type::kRefLeft);

      if (get_pronunciation_index(right, alpha) && get_pronunciation_index(left, alpha)) {
        uint8_t type = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
        ref_ipa_ = osmdata_.name_offset_map.name(get_pronunciation_index(type, alpha));
        pronunciationMap[std::make_pair(type, alpha)] = 0;
      }

      alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
      if (get_pronunciation_index(right, alpha) && get_pronunciation_index(left, alpha)) {
        uint8_t type = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
        ref_katakana_ = osmdata_.name_offset_map.name(get_pronunciation_index(type, alpha));
        pronunciationMap[std::make_pair(type, alpha)] = 0;
      }

      alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
      if (get_pronunciation_index(right, alpha) && get_pronunciation_index(left, alpha)) {
        uint8_t type = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
        ref_jeita_ = osmdata_.name_offset_map.name(get_pronunciation_index(type, alpha));
        pronunciationMap[std::make_pair(type, alpha)] = 0;
      }

      alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
      if (get_pronunciation_index(right, alpha) && get_pronunciation_index(left, alpha)) {
        uint8_t type = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
        ref_nt_sampa_ = osmdata_.name_offset_map.name(get_pronunciation_index(type, alpha));
        pronunciationMap[std::make_pair(type, alpha)] = 0;
      }
    }

    // We need to set a data processing flag so we need to
    // process in pbfgraphparser instead of lua because of config option use_rest_area
    if (use_rest_area_ && service_ == "rest_area" && way_.use() != Use::kConstruction) {
      if (amenity_ == "yes") {
        way_.set_use(Use::kServiceArea);
      } else {
        way_.set_use(Use::kRestArea);
      }
    }

    if (use_direction_on_ways_) {
      if (!ref_.empty())
        ProcessDirection(false);

      if (!int_ref_.empty())
        ProcessDirection(true);

      if (!ref_ipa_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kIpa, false);

      if (!int_ref_ipa_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kIpa, true);

      if (!ref_katakana_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kKatakana, false);

      if (!int_ref_katakana_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kKatakana, true);

      if (!ref_nt_sampa_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kNtSampa, false);

      if (!int_ref_nt_sampa_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kNtSampa, true);

      if (!ref_jeita_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kJeita, false);

      if (!int_ref_jeita_.empty())
        ProcessDirectionPronunciation(PronunciationAlphabet::kJeita, true);
    }

    // add int_refs to the end of the refs for now.  makes sure that we don't add dups.
    if (!int_ref_.empty()) {
      std::string tmp = ref_;
      std::vector<std::string> rs = GetTagTokens(tmp);
      std::vector<std::string> is = GetTagTokens(int_ref_);
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
        ref_ = tmp;
      }
      // no matter what, clear out the int_ref.
      way_.set_int_ref_index(0);
    }

    // add int_ref pronunciations to the end of the pronunciation refs for now.  makes sure that we
    // don't add dups.
    MergeRefPronunciations();

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
        if (access && !way_.oneway_reverse() && way_.use() != Use::kConstruction) {
          way_.set_bike_forward(true);
        }
        if (access && !way_.oneway() && way_.use() != Use::kConstruction) {
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
        if (access && !way_.oneway_reverse() && way_.use() != Use::kConstruction) {
          way_.set_bike_forward(true);
        }
        if (access && !way_.oneway() && way_.use() != Use::kConstruction) {
          way_.set_bike_backward(true);
        }
      }
    }

    // IMBA scale
    auto mtb_imba_scale = results.find("mtb:scale:imba");
    bool has_mtb_imba = mtb_imba_scale != results.end();
    if (has_mtb_imba) {
      // Update bike access (only if neither mtb:scale nor mtb:scale:uphill is present)
      if (!has_mtb_scale && !has_mtb_uphill_scale && way_.use() != Use::kConstruction) {
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
    if (has_mtb_desc && !has_mtb_scale && !has_mtb_uphill_scale && !has_mtb_imba &&
        way_.use() != Use::kConstruction) {
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
    if (!name_.empty() && !ref_.empty()) {
      std::vector<std::string> names = GetTagTokens(name_);
      std::vector<std::string> refs = GetTagTokens(ref_);
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
        name_ = tmp;
      } else
        name_ = "";
    }

    // edge case.  name = name:<lg> and we contain a -  or /
    // see https://www.openstreetmap.org/way/816515178#map=16/42.7888/-1.6391
    if (name_ == name_w_lang_ && !name_.empty() && GetTagTokens(language_).size() == 1) {
      bool process = false;
      std::vector<std::string> names;

      if (name_w_lang_.find(" - ") != std::string::npos) {
        names = GetTagTokens(name_w_lang_, " - ");
        process = true;
      } else if (name_w_lang_.find(" / ") != std::string::npos) {
        names = GetTagTokens(name_w_lang_, " / ");
        process = true;
      }

      if (process) {
        std::string tmp;
        std::string l;

        for (auto& name : names) {
          if (!tmp.empty()) {
            tmp += ";";
            l += ";";
          }
          tmp += name;
          l += language_;
        }
        if (!tmp.empty()) {
          name_ = tmp;
          name_w_lang_ = tmp;
          language_ = l;
        }
      }
    }

    // begin name logic
    std::string l = language_;
    ProcessName(name_w_lang_, name_, language_);
    way_.set_name_index(osmdata_.name_offset_map.index(name_));
    way_.set_name_lang_index(osmdata_.name_offset_map.index(language_));

    ProcessLRFBName(name_left_w_lang_, name_w_lang_, l, name_left_, lang_left_);
    way_.set_name_left_index(osmdata_.name_offset_map.index(name_left_));
    way_.set_name_left_lang_index(osmdata_.name_offset_map.index(lang_left_));

    ProcessLRFBName(name_right_w_lang_, name_w_lang_, l, name_right_, lang_right_);
    way_.set_name_right_index(osmdata_.name_offset_map.index(name_right_));
    way_.set_name_right_lang_index(osmdata_.name_offset_map.index(lang_right_));

    ProcessLRFBName(name_forward_w_lang_, name_w_lang_, l, name_forward_, lang_forward_);
    way_.set_name_forward_index(osmdata_.name_offset_map.index(name_forward_));
    way_.set_name_forward_lang_index(osmdata_.name_offset_map.index(lang_forward_));

    ProcessLRFBName(name_backward_w_lang_, name_w_lang_, l, name_backward_, lang_backward_);
    way_.set_name_backward_index(osmdata_.name_offset_map.index(name_backward_));
    way_.set_name_backward_lang_index(osmdata_.name_offset_map.index(lang_backward_));

    // begin official name logic
    l = official_language_;
    ProcessName(official_name_w_lang_, official_name_, official_language_);
    way_.set_official_name_index(osmdata_.name_offset_map.index(official_name_));
    way_.set_official_name_lang_index(osmdata_.name_offset_map.index(official_language_));

    ProcessLRFBName(official_name_left_w_lang_, official_name_w_lang_, l, official_name_left_,
                    official_lang_left_);
    way_.set_official_name_left_index(osmdata_.name_offset_map.index(official_name_left_));
    way_.set_official_name_left_lang_index(osmdata_.name_offset_map.index(official_lang_left_));

    ProcessLRFBName(official_name_right_w_lang_, official_name_w_lang_, l, official_name_right_,
                    official_lang_right_);
    way_.set_official_name_right_index(osmdata_.name_offset_map.index(official_name_right_));
    way_.set_official_name_right_lang_index(osmdata_.name_offset_map.index(official_lang_right_));

    // begin alt name logic
    l = alt_language_;
    ProcessName(alt_name_w_lang_, alt_name_, alt_language_);
    way_.set_alt_name_index(osmdata_.name_offset_map.index(alt_name_));
    way_.set_alt_name_lang_index(osmdata_.name_offset_map.index(alt_language_));

    ProcessLRFBName(alt_name_left_w_lang_, alt_name_w_lang_, l, alt_name_left_, alt_lang_left_);
    way_.set_alt_name_left_index(osmdata_.name_offset_map.index(alt_name_left_));
    way_.set_alt_name_left_lang_index(osmdata_.name_offset_map.index(alt_lang_left_));

    ProcessLRFBName(alt_name_right_w_lang_, alt_name_w_lang_, l, alt_name_right_, alt_lang_right_);
    way_.set_alt_name_right_index(osmdata_.name_offset_map.index(alt_name_right_));
    way_.set_alt_name_right_lang_index(osmdata_.name_offset_map.index(alt_lang_right_));

    // begin ref logic
    l = ref_language_;
    ProcessName(ref_w_lang_, ref_, ref_language_);
    way_.set_ref_index(osmdata_.name_offset_map.index(ref_));
    way_.set_ref_lang_index(osmdata_.name_offset_map.index(ref_language_));

    ProcessLRFBName(ref_left_w_lang_, ref_w_lang_, l, ref_left_, ref_lang_left_);
    way_.set_ref_left_index(osmdata_.name_offset_map.index(ref_left_));
    way_.set_ref_left_lang_index(osmdata_.name_offset_map.index(ref_lang_left_));

    ProcessLRFBName(ref_right_w_lang_, ref_w_lang_, l, ref_right_, ref_lang_right_);
    way_.set_ref_right_index(osmdata_.name_offset_map.index(ref_right_));
    way_.set_ref_right_lang_index(osmdata_.name_offset_map.index(ref_lang_right_));

    // begin int_ref logic
    l = int_ref_language_;
    ProcessName(int_ref_w_lang_, int_ref_, int_ref_language_);
    way_.set_int_ref_index(osmdata_.name_offset_map.index(int_ref_));
    way_.set_int_ref_lang_index(osmdata_.name_offset_map.index(int_ref_language_));

    ProcessLRFBName(int_ref_left_w_lang_, int_ref_w_lang_, l, int_ref_left_, int_ref_lang_left_);
    way_.set_int_ref_left_index(osmdata_.name_offset_map.index(int_ref_left_));
    way_.set_int_ref_left_lang_index(osmdata_.name_offset_map.index(int_ref_lang_left_));

    ProcessLRFBName(int_ref_right_w_lang_, int_ref_w_lang_, l, int_ref_right_, int_ref_lang_right_);
    way_.set_int_ref_right_index(osmdata_.name_offset_map.index(int_ref_right_));
    way_.set_int_ref_right_lang_index(osmdata_.name_offset_map.index(int_ref_lang_right_));

    // begin tunnel name logic
    l = tunnel_language_;
    ProcessName(tunnel_name_w_lang_, tunnel_name_, tunnel_language_);
    way_.set_tunnel_name_index(osmdata_.name_offset_map.index(tunnel_name_));
    way_.set_tunnel_name_lang_index(osmdata_.name_offset_map.index(tunnel_language_));

    ProcessLRFBName(tunnel_name_left_w_lang_, tunnel_name_w_lang_, l, tunnel_name_left_,
                    tunnel_lang_left_);
    way_.set_tunnel_name_left_index(osmdata_.name_offset_map.index(tunnel_name_left_));
    way_.set_tunnel_name_left_lang_index(osmdata_.name_offset_map.index(tunnel_lang_left_));

    ProcessLRFBName(tunnel_name_right_w_lang_, tunnel_name_w_lang_, l, tunnel_name_right_,
                    tunnel_lang_right_);
    way_.set_tunnel_name_right_index(osmdata_.name_offset_map.index(tunnel_name_right_));
    way_.set_tunnel_name_right_lang_index(osmdata_.name_offset_map.index(tunnel_lang_right_));

    uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kName);
    uint8_t alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    ProcessPronunciationName(OSMLinguistic::Type::kName, PronunciationAlphabet::kIpa, name_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kName, PronunciationAlphabet::kKatakana,
                             name_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kName, PronunciationAlphabet::kJeita, name_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kName, PronunciationAlphabet::kNtSampa,
                             name_nt_sampa_);

    ProcessPronunciationLRFBName(ipa_name, ipa_lang, name_left_ipa_, OSMLinguistic::Type::kNameLeft,
                                 PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_name, katakana_lang, name_left_katakana_,
                                 OSMLinguistic::Type::kNameLeft, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_name, jeita_lang, name_left_jeita_,
                                 OSMLinguistic::Type::kNameLeft, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_name, nt_sampa_lang, name_left_nt_sampa_,
                                 OSMLinguistic::Type::kNameLeft, PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_name, ipa_lang, name_right_ipa_, OSMLinguistic::Type::kNameRight,
                                 PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_name, katakana_lang, name_right_katakana_,
                                 OSMLinguistic::Type::kNameRight, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_name, jeita_lang, name_right_jeita_,
                                 OSMLinguistic::Type::kNameRight, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_name, nt_sampa_lang, name_right_nt_sampa_,
                                 OSMLinguistic::Type::kNameRight, PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_name, ipa_lang, name_backward_ipa_,
                                 OSMLinguistic::Type::kNameBackward, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_name, katakana_lang, name_backward_katakana_,
                                 OSMLinguistic::Type::kNameBackward,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_name, jeita_lang, name_backward_jeita_,
                                 OSMLinguistic::Type::kNameBackward, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_name, nt_sampa_lang, name_backward_nt_sampa_,
                                 OSMLinguistic::Type::kNameBackward, PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_name, ipa_lang, name_forward_ipa_,
                                 OSMLinguistic::Type::kNameForward, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_name, katakana_lang, name_forward_katakana_,
                                 OSMLinguistic::Type::kNameForward, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_name, jeita_lang, name_forward_jeita_,
                                 OSMLinguistic::Type::kNameForward, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_name, nt_sampa_lang, name_forward_nt_sampa_,
                                 OSMLinguistic::Type::kNameForward, PronunciationAlphabet::kNtSampa);

    t = static_cast<uint8_t>(OSMLinguistic::Type::kAltName);
    alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_alt_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_alt_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_alt_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_alt_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_alt_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_alt_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_alt_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_alt_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    ProcessPronunciationName(OSMLinguistic::Type::kAltName, PronunciationAlphabet::kIpa,
                             alt_name_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kAltName, PronunciationAlphabet::kKatakana,
                             alt_name_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kAltName, PronunciationAlphabet::kJeita,
                             alt_name_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kAltName, PronunciationAlphabet::kNtSampa,
                             alt_name_nt_sampa_);

    ProcessPronunciationLRFBName(ipa_alt_name, ipa_alt_lang, alt_name_left_ipa_,
                                 OSMLinguistic::Type::kAltNameLeft, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_alt_name, katakana_alt_lang, alt_name_left_katakana_,
                                 OSMLinguistic::Type::kAltNameLeft, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_alt_name, jeita_alt_lang, alt_name_left_jeita_,
                                 OSMLinguistic::Type::kAltNameLeft, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_alt_name, nt_sampa_alt_lang, alt_name_left_nt_sampa_,
                                 OSMLinguistic::Type::kAltNameLeft, PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_alt_name, ipa_alt_lang, alt_name_right_ipa_,
                                 OSMLinguistic::Type::kAltNameRight, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_alt_name, katakana_alt_lang, alt_name_right_katakana_,
                                 OSMLinguistic::Type::kAltNameRight,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_alt_name, jeita_alt_lang, alt_name_right_jeita_,
                                 OSMLinguistic::Type::kAltNameRight, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_alt_name, nt_sampa_alt_lang, alt_name_right_nt_sampa_,
                                 OSMLinguistic::Type::kAltNameRight, PronunciationAlphabet::kNtSampa);

    t = static_cast<uint8_t>(OSMLinguistic::Type::kOfficialName);
    alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_official_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_official_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_official_name =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_official_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_official_name =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_official_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_official_name =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_official_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    ProcessPronunciationName(OSMLinguistic::Type::kOfficialName, PronunciationAlphabet::kIpa,
                             official_name_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kOfficialName, PronunciationAlphabet::kKatakana,
                             official_name_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kOfficialName, PronunciationAlphabet::kJeita,
                             official_name_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kOfficialName, PronunciationAlphabet::kNtSampa,
                             official_name_nt_sampa_);

    ProcessPronunciationLRFBName(ipa_official_name, ipa_official_lang, official_name_left_ipa_,
                                 OSMLinguistic::Type::kOfficialNameLeft, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_official_name, katakana_official_lang,
                                 official_name_left_katakana_, OSMLinguistic::Type::kOfficialNameLeft,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_official_name, jeita_official_lang, official_name_left_jeita_,
                                 OSMLinguistic::Type::kOfficialNameLeft,
                                 PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_official_name, nt_sampa_official_lang,
                                 official_name_left_nt_sampa_, OSMLinguistic::Type::kOfficialNameLeft,
                                 PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_official_name, ipa_official_lang, official_name_right_ipa_,
                                 OSMLinguistic::Type::kOfficialNameRight,
                                 PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_official_name, katakana_official_lang,
                                 official_name_right_katakana_,
                                 OSMLinguistic::Type::kOfficialNameRight,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_official_name, jeita_official_lang, official_name_right_jeita_,
                                 OSMLinguistic::Type::kOfficialNameRight,
                                 PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_official_name, nt_sampa_official_lang,
                                 official_name_right_nt_sampa_,
                                 OSMLinguistic::Type::kOfficialNameRight,
                                 PronunciationAlphabet::kNtSampa);

    t = static_cast<uint8_t>(OSMLinguistic::Type::kTunnelName);
    alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_tunnel_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_tunnel_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_tunnel_name =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_tunnel_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_tunnel_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_tunnel_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_tunnel_name =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_tunnel_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    ProcessPronunciationName(OSMLinguistic::Type::kTunnelName, PronunciationAlphabet::kIpa,
                             tunnel_name_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kTunnelName, PronunciationAlphabet::kKatakana,
                             tunnel_name_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kTunnelName, PronunciationAlphabet::kJeita,
                             tunnel_name_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kTunnelName, PronunciationAlphabet::kNtSampa,
                             tunnel_name_nt_sampa_);

    ProcessPronunciationLRFBName(ipa_tunnel_name, ipa_tunnel_lang, tunnel_name_left_ipa_,
                                 OSMLinguistic::Type::kTunnelNameLeft, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_tunnel_name, katakana_tunnel_lang,
                                 tunnel_name_left_katakana_, OSMLinguistic::Type::kTunnelNameLeft,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_tunnel_name, jeita_tunnel_lang, tunnel_name_left_jeita_,
                                 OSMLinguistic::Type::kTunnelNameLeft, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_tunnel_name, nt_sampa_tunnel_lang,
                                 tunnel_name_left_nt_sampa_, OSMLinguistic::Type::kTunnelNameLeft,
                                 PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_tunnel_name, ipa_tunnel_lang, tunnel_name_right_ipa_,
                                 OSMLinguistic::Type::kTunnelNameRight, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_tunnel_name, katakana_tunnel_lang,
                                 tunnel_name_right_katakana_, OSMLinguistic::Type::kTunnelNameRight,
                                 PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_tunnel_name, jeita_tunnel_lang, tunnel_name_right_jeita_,
                                 OSMLinguistic::Type::kTunnelNameRight,
                                 PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_tunnel_name, nt_sampa_tunnel_lang,
                                 tunnel_name_right_nt_sampa_, OSMLinguistic::Type::kTunnelNameRight,
                                 PronunciationAlphabet::kNtSampa);

    t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
    alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_ref_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_ref_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_ref_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_ref_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_ref_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_ref_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_ref_name = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_ref_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    if (!use_direction_on_ways_) {
      ProcessPronunciationName(OSMLinguistic::Type::kRef, PronunciationAlphabet::kIpa, ref_ipa_);

      ProcessPronunciationName(OSMLinguistic::Type::kRef, PronunciationAlphabet::kKatakana,
                               ref_katakana_);

      ProcessPronunciationName(OSMLinguistic::Type::kRef, PronunciationAlphabet::kJeita, ref_jeita_);

      ProcessPronunciationName(OSMLinguistic::Type::kRef, PronunciationAlphabet::kNtSampa,
                               ref_nt_sampa_);
    }

    ProcessPronunciationLRFBName(ipa_ref_name, ipa_ref_lang, ref_left_ipa_,
                                 OSMLinguistic::Type::kRefLeft, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_ref_name, katakana_ref_lang, ref_left_katakana_,
                                 OSMLinguistic::Type::kRefLeft, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_ref_name, jeita_ref_lang, ref_left_jeita_,
                                 OSMLinguistic::Type::kRefLeft, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_ref_name, nt_sampa_ref_lang, ref_left_nt_sampa_,
                                 OSMLinguistic::Type::kRefLeft, PronunciationAlphabet::kNtSampa);

    ProcessPronunciationLRFBName(ipa_ref_name, ipa_ref_lang, ref_right_ipa_,
                                 OSMLinguistic::Type::kRefRight, PronunciationAlphabet::kIpa);

    ProcessPronunciationLRFBName(katakana_ref_name, katakana_ref_lang, ref_right_katakana_,
                                 OSMLinguistic::Type::kRefRight, PronunciationAlphabet::kKatakana);

    ProcessPronunciationLRFBName(jeita_ref_name, jeita_ref_lang, ref_right_jeita_,
                                 OSMLinguistic::Type::kRefRight, PronunciationAlphabet::kJeita);

    ProcessPronunciationLRFBName(nt_sampa_ref_name, nt_sampa_ref_lang, ref_right_nt_sampa_,
                                 OSMLinguistic::Type::kRefRight, PronunciationAlphabet::kNtSampa);

    // begin destination logic
    ProcessName(destination_w_lang_, destination_, destination_language_);
    way_.set_destination_index(osmdata_.name_offset_map.index(destination_));
    way_.set_destination_lang_index(osmdata_.name_offset_map.index(destination_language_));

    ProcessName(destination_forward_w_lang_, destination_forward_, destination_forward_language_);
    way_.set_destination_forward_index(osmdata_.name_offset_map.index(destination_forward_));
    way_.set_destination_forward_lang_index(
        osmdata_.name_offset_map.index(destination_forward_language_));

    ProcessName(destination_backward_w_lang_, destination_backward_, destination_backward_language_);
    way_.set_destination_backward_index(osmdata_.name_offset_map.index(destination_backward_));
    way_.set_destination_backward_lang_index(
        osmdata_.name_offset_map.index(destination_backward_language_));

    ProcessName(junction_ref_w_lang_, junction_ref_, junction_ref_language_);
    way_.set_junction_ref_index(osmdata_.name_offset_map.index(junction_ref_));
    way_.set_junction_ref_lang_index(osmdata_.name_offset_map.index(junction_ref_language_));

    ProcessName(junction_name_w_lang_, junction_name_, junction_name_language_);
    way_.set_junction_name_index(osmdata_.name_offset_map.index(junction_name_));
    way_.set_junction_name_lang_index(osmdata_.name_offset_map.index(junction_name_language_));

    ProcessName(destination_ref_w_lang_, destination_ref_, destination_ref_language_);
    way_.set_destination_ref_index(osmdata_.name_offset_map.index(destination_ref_));
    way_.set_destination_ref_lang_index(osmdata_.name_offset_map.index(destination_ref_language_));

    ProcessName(destination_ref_to_w_lang_, destination_ref_to_, destination_ref_to_language_);
    way_.set_destination_ref_to_index(osmdata_.name_offset_map.index(destination_ref_to_));
    way_.set_destination_ref_to_lang_index(
        osmdata_.name_offset_map.index(destination_ref_to_language_));

    ProcessName(destination_street_to_w_lang_, destination_street_to_,
                destination_street_to_language_);
    way_.set_destination_street_to_index(osmdata_.name_offset_map.index(destination_street_to_));
    way_.set_destination_street_to_lang_index(
        osmdata_.name_offset_map.index(destination_street_to_language_));

    ProcessName(destination_street_w_lang_, destination_street_, destination_street_language_);
    way_.set_destination_street_index(osmdata_.name_offset_map.index(destination_street_));
    way_.set_destination_street_lang_index(
        osmdata_.name_offset_map.index(destination_street_language_));

    t = static_cast<uint8_t>(OSMLinguistic::Type::kDestination);
    alpha = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
    std::string ipa_destination = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string ipa_destination_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
    std::string katakana_destination =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string katakana_destination_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kJeita);
    std::string jeita_destination = osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string jeita_destination_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    alpha = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
    std::string nt_sampa_destination =
        osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha));
    std::string nt_sampa_destination_lang = osmdata_.name_offset_map.name(get_lang_index(t, alpha));

    ProcessPronunciationName(OSMLinguistic::Type::kDestination, PronunciationAlphabet::kIpa,
                             destination_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestination, PronunciationAlphabet::kKatakana,
                             destination_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestination, PronunciationAlphabet::kJeita,
                             destination_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestination, PronunciationAlphabet::kNtSampa,
                             destination_nt_sampa_);

    if (destination_forward_ipa_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationForward, PronunciationAlphabet::kIpa,
                               destination_forward_ipa_);
    else
      ProcessPronunciationLRFBName(ipa_destination, ipa_destination_lang, destination_forward_ipa_,
                                   OSMLinguistic::Type::kDestinationForward,
                                   PronunciationAlphabet::kIpa);

    if (destination_forward_katakana_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationForward,
                               PronunciationAlphabet::kKatakana, destination_forward_katakana_);
    else
      ProcessPronunciationLRFBName(katakana_destination, katakana_lang, destination_forward_katakana_,
                                   OSMLinguistic::Type::kDestinationForward,
                                   PronunciationAlphabet::kKatakana);

    if (destination_forward_jeita_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationForward,
                               PronunciationAlphabet::kJeita, destination_forward_jeita_);
    else
      ProcessPronunciationLRFBName(jeita_destination, jeita_destination_lang,
                                   destination_forward_jeita_,
                                   OSMLinguistic::Type::kDestinationForward,
                                   PronunciationAlphabet::kJeita);

    if (destination_forward_nt_sampa_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationForward,
                               PronunciationAlphabet::kNtSampa, destination_forward_nt_sampa_);
    else
      ProcessPronunciationLRFBName(nt_sampa_destination, nt_sampa_destination_lang,
                                   destination_forward_nt_sampa_,
                                   OSMLinguistic::Type::kDestinationForward,
                                   PronunciationAlphabet::kNtSampa);

    if (destination_backward_ipa_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationBackward, PronunciationAlphabet::kIpa,
                               destination_backward_ipa_);
    else
      ProcessPronunciationLRFBName(ipa_destination, ipa_destination_lang, destination_backward_ipa_,
                                   OSMLinguistic::Type::kDestinationBackward,
                                   PronunciationAlphabet::kIpa);

    if (destination_backward_katakana_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationBackward,
                               PronunciationAlphabet::kKatakana, destination_backward_katakana_);
    else
      ProcessPronunciationLRFBName(katakana_destination, katakana_destination_lang,
                                   destination_backward_katakana_,
                                   OSMLinguistic::Type::kDestinationBackward,
                                   PronunciationAlphabet::kKatakana);

    if (destination_backward_jeita_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationBackward,
                               PronunciationAlphabet::kJeita, destination_backward_jeita_);
    else
      ProcessPronunciationLRFBName(jeita_destination, jeita_destination_lang,
                                   destination_backward_jeita_,
                                   OSMLinguistic::Type::kDestinationBackward,
                                   PronunciationAlphabet::kJeita);

    if (destination_backward_nt_sampa_.empty())
      ProcessPronunciationName(OSMLinguistic::Type::kDestinationBackward,
                               PronunciationAlphabet::kNtSampa, destination_backward_nt_sampa_);
    else
      ProcessPronunciationLRFBName(nt_sampa_destination, nt_sampa_destination_lang,
                                   destination_backward_nt_sampa_,
                                   OSMLinguistic::Type::kDestinationBackward,
                                   PronunciationAlphabet::kNtSampa);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRef, PronunciationAlphabet::kIpa,
                             destination_ref_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRef, PronunciationAlphabet::kKatakana,
                             destination_ref_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRef, PronunciationAlphabet::kJeita,
                             destination_ref_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRef, PronunciationAlphabet::kNtSampa,
                             destination_ref_nt_sampa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRefTo, PronunciationAlphabet::kIpa,
                             destination_ref_to_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRefTo, PronunciationAlphabet::kKatakana,
                             destination_ref_to_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRefTo, PronunciationAlphabet::kJeita,
                             destination_ref_to_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationRefTo, PronunciationAlphabet::kNtSampa,
                             destination_ref_to_nt_sampa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreet, PronunciationAlphabet::kIpa,
                             destination_street_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreet,
                             PronunciationAlphabet::kKatakana, destination_street_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreet, PronunciationAlphabet::kJeita,
                             destination_street_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreet, PronunciationAlphabet::kNtSampa,
                             destination_street_nt_sampa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreetTo, PronunciationAlphabet::kIpa,
                             destination_street_to_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreetTo,
                             PronunciationAlphabet::kKatakana, destination_street_to_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreetTo, PronunciationAlphabet::kJeita,
                             destination_street_to_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kDestinationStreetTo,
                             PronunciationAlphabet::kNtSampa, destination_street_to_nt_sampa_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionRef, PronunciationAlphabet::kIpa,
                             junction_ref_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionRef, PronunciationAlphabet::kKatakana,
                             junction_ref_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionRef, PronunciationAlphabet::kJeita,
                             junction_ref_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionRef, PronunciationAlphabet::kNtSampa,
                             junction_ref_nt_sampa_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionName, PronunciationAlphabet::kIpa,
                             junction_name_ipa_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionName, PronunciationAlphabet::kKatakana,
                             junction_name_katakana_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionName, PronunciationAlphabet::kJeita,
                             junction_name_jeita_);

    ProcessPronunciationName(OSMLinguistic::Type::kJunctionName, PronunciationAlphabet::kNtSampa,
                             junction_name_nt_sampa_);

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

    if (has_pronunciation_tags_) {
      way_.set_has_pronunciation_tags(true);
      for (auto const& p : pronunciationMap) {
        if (p.second != 0) {
          OSMLinguistic ling;
          ling.key_.type_ = p.first.first;
          ling.key_.alpha_ = p.first.second;
          ling.name_offset_ = p.second;
          osmdata_.pronunciations.insert(LinguisticMultiMap::value_type(osmid_, ling));
        }
      }
      for (auto const& l : langMap) {
        if (l.second != 0) {
          OSMLinguistic ling;
          ling.key_.type_ = l.first.first;
          ling.key_.alpha_ = l.first.second;
          ling.name_offset_ = l.second;
          osmdata_.langs.insert(LinguisticMultiMap::value_type(osmid_, ling));
        }
      }
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
    bool isConditional = false, isProbable = false, has_multiple_times = false;
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
      } else if (tag.first == "restriction:probable") {
        // probability=73
        std::vector<std::string> prob_tok = GetTagTokens(tag.second, '=');
        if (prob_tok.size() == 2) {
          const auto& p = stoi(prob_tok.at(1));
          if (p > 0) {
            isProbable = true;
            restriction.set_probability(p);
          } else // A complex restriction can not have a 0 probability set.  range is 1 to 100
            return;
        }
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

    if (isProbable) {
      RestrictionType type = restriction.type();
      if (type == RestrictionType::kOnlyRightTurn || type == RestrictionType::kOnlyLeftTurn ||
          type == RestrictionType::kOnlyStraightOn)
        restriction.set_type(RestrictionType::kOnlyProbable);
      else
        restriction.set_type(RestrictionType::kNoProbable);
    }

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
        // if probable restriction, change to a complex restriction
        if (vias.size() == 0 && (isTypeRestriction || isConditional || isProbable ||
                                 (!isTypeRestriction && except.size()))) {

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
             sequence<OSMNode>* bss_nodes,
             sequence<OSMNodeLinguistic>* node_linguistics) {
    // reset the pointers (either null them out or set them to something valid)
    ways_.reset(ways);
    way_nodes_.reset(way_nodes);
    access_.reset(access);
    complex_restrictions_from_.reset(complex_restrictions_from);
    complex_restrictions_to_.reset(complex_restrictions_to);
    bss_nodes_.reset(bss_nodes);
    node_linguistics_.reset(node_linguistics);

    if (node_linguistics != nullptr) {
      // push empty struct at index 0
      OSMNodeLinguistic ling;
      node_linguistics_->push_back(ling);
      ++osmdata_.node_linguistic_count;
    }
  }

  void ProcessPronunciationTag(const OSMLinguistic::Type& type,
                               const PronunciationAlphabet& alphabet,
                               OSMNodeLinguistic* linguistics = nullptr) {

    uint32_t name = 0, lang = 0;
    uint8_t t = static_cast<uint8_t>(type);
    uint8_t alpha = static_cast<uint8_t>(alphabet);

    if (linguistics) {
      if (type == OSMLinguistic::Type::kNodeName) {
        switch (alphabet) {
          case PronunciationAlphabet::kIpa:
            name = linguistics->name_pronunciation_ipa_index();
            lang = linguistics->name_pronunciation_ipa_lang_index();
            break;
          case PronunciationAlphabet::kKatakana:
            name = linguistics->name_pronunciation_katakana_index();
            lang = linguistics->name_pronunciation_katakana_lang_index();
            break;
          case PronunciationAlphabet::kJeita:
            name = linguistics->name_pronunciation_jeita_index();
            lang = linguistics->name_pronunciation_jeita_lang_index();
            break;
          case PronunciationAlphabet::kNtSampa:
            name = linguistics->name_pronunciation_nt_sampa_index();
            lang = linguistics->name_pronunciation_nt_sampa_lang_index();
            break;
          case PronunciationAlphabet::kNone:
            break;
        }
      } else if (type == OSMLinguistic::Type::kNodeRef) {
        switch (alphabet) {
          case PronunciationAlphabet::kIpa:
            name = linguistics->ref_pronunciation_ipa_index();
            lang = linguistics->ref_pronunciation_ipa_lang_index();
            break;
          case PronunciationAlphabet::kKatakana:
            name = linguistics->ref_pronunciation_katakana_index();
            lang = linguistics->ref_pronunciation_katakana_lang_index();
            break;
          case PronunciationAlphabet::kJeita:
            name = linguistics->ref_pronunciation_jeita_index();
            lang = linguistics->ref_pronunciation_jeita_lang_index();
            break;
          case PronunciationAlphabet::kNtSampa:
            name = linguistics->ref_pronunciation_nt_sampa_index();
            lang = linguistics->ref_pronunciation_nt_sampa_lang_index();
            break;
          case PronunciationAlphabet::kNone:
            break;
        }
      }
    } else {
      name = get_pronunciation_index(t, alpha);
      lang = get_lang_index(t, alpha);
    }

    std::string name_w_lang, language;

    if (name != 0)
      name_w_lang =
          !linguistics ? osmdata_.name_offset_map.name(name) : osmdata_.node_names.name(name);

    if (lang != 0)
      language = !linguistics ? osmdata_.name_offset_map.name(lang) : osmdata_.node_names.name(lang);

    ProcessNameTag(tag_, name_w_lang, language, true);

    SavePronunciationData(t, alpha, name_w_lang, language, linguistics);
  }

  void ProcessLeftRightPronunciationTag(const OSMLinguistic::Type& type,
                                        const PronunciationAlphabet& alphabet) {
    uint32_t name = 0, lang = 0;
    std::string name_w_lang, language;
    uint8_t t = static_cast<uint8_t>(type);
    uint8_t alpha = static_cast<uint8_t>(alphabet);

    name = get_pronunciation_index(t, alpha);
    lang = get_lang_index(t, alpha);

    if (name != 0)
      name_w_lang = osmdata_.name_offset_map.name(name);

    if (lang != 0)
      language = osmdata_.name_offset_map.name(lang);

    ProcessLeftRightNameTag(tag_, name_w_lang, language, true);

    SavePronunciationData(t, alpha, name_w_lang, language);
  }

  void SavePronunciationData(const uint8_t type,
                             const uint8_t alphabet,
                             const std::string& pronunciation,
                             const std::string& language,
                             OSMNodeLinguistic* linguistics = nullptr) {
    if (!pronunciation.empty()) {
      has_pronunciation_tags_ = true;

      if (linguistics) {
        OSMLinguistic::Type t = static_cast<OSMLinguistic::Type>(type);
        PronunciationAlphabet alpha = static_cast<PronunciationAlphabet>(alphabet);
        if (t == OSMLinguistic::Type::kNodeName) {
          switch (alpha) {
            case PronunciationAlphabet::kIpa:
              linguistics->set_name_pronunciation_ipa_index(osmdata_.node_names.index(pronunciation));
              linguistics->set_name_pronunciation_ipa_lang_index(osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kKatakana:
              linguistics->set_name_pronunciation_katakana_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_name_pronunciation_katakana_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kJeita:
              linguistics->set_name_pronunciation_jeita_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_name_pronunciation_jeita_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kNtSampa:
              linguistics->set_name_pronunciation_nt_sampa_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_name_pronunciation_nt_sampa_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kNone:
              break;
          }
        } else if (t == OSMLinguistic::Type::kNodeRef) {
          switch (alpha) {
            case PronunciationAlphabet::kIpa:
              linguistics->set_ref_pronunciation_ipa_index(osmdata_.node_names.index(pronunciation));
              linguistics->set_ref_pronunciation_ipa_lang_index(osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kKatakana:
              linguistics->set_ref_pronunciation_katakana_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_ref_pronunciation_katakana_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kJeita:
              linguistics->set_ref_pronunciation_jeita_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_ref_pronunciation_jeita_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kNtSampa:
              linguistics->set_ref_pronunciation_nt_sampa_index(
                  osmdata_.node_names.index(pronunciation));
              linguistics->set_ref_pronunciation_nt_sampa_lang_index(
                  osmdata_.node_names.index(language));
              break;
            case PronunciationAlphabet::kNone:
              break;
          }
        }
      } else {

        pronunciationMap[std::make_pair(type, alphabet)] =
            osmdata_.name_offset_map.index(pronunciation);
        langMap[std::make_pair(type, alphabet)] = osmdata_.name_offset_map.index(language);
      }
    }
  }

  void ProcessNameTag(const std::pair<std::string, std::string>& tag,
                      std::string& name_w_lang,
                      std::string& language,
                      bool is_lang_pronunciation = false) {
    // If we start to use admin tags for HERE, then put use_admin_db_ check back
    // if (!use_admin_db_)
    //  return;

    std::string t = tag.first;
    std::size_t found = t.find(":pronunciation");
    if (found != std::string::npos && !is_lang_pronunciation)
      return;
    else {
      if (found != std::string::npos) {
        // remove :pronunciation or :pronunciation:<type>
        t = tag.first.substr(0, found);
        has_pronunciation_tags_ = true;
      }
    }

    if (boost::algorithm::starts_with(t, "tunnel:name:"))
      t = t.substr(7);
    else if (boost::algorithm::starts_with(t, "junction:name:") ||
             boost::algorithm::starts_with(t, "junction:ref:"))
      t = t.substr(9);
    else {
      found = t.find(":lang:");
      if (found != std::string::npos) {
        t = t.substr(found + 1);
      }
    }

    // if we have column at the and of token delete it
    // TODO It looks like road bug. We shouldn't have any column ant the end
    while (t.back() == ':') {
      t.pop_back();
    }

    std::vector<std::string> tokens = GetTagTokens(t, ':');
    if (tokens.size() == 2) {

      std::string lang = tokens.at(1);
      if (stringLanguage(lang) != Language::kNone &&
          !tag.second.empty()) // name:en, name:ar, name:fr, etc
      {
        t = tag.second;
        uint32_t count = std::count(t.begin(), t.end(), ';');
        std::string l = lang;
        for (uint32_t i = 0; i < count; i++) {
          l += ";" + lang;
        }
        if (name_w_lang.empty()) {
          name_w_lang = tag.second;
          language = l;
        } else {
          name_w_lang += ";" + tag.second;
          language += ";" + l;
        }
      }
    }
  }

  void ProcessLeftRightNameTag(const std::pair<std::string, std::string>& tag,
                               std::string& name_left_right_w_lang,
                               std::string& lang_left_right,
                               bool is_lang_pronunciation = false) {
    if (!use_admin_db_)
      return;

    std::string t = tag.first;
    std::size_t found = t.find(":pronunciation");
    if (found != std::string::npos && !is_lang_pronunciation)
      return;
    else {
      if (found != std::string::npos) {
        // remove :pronunciation or :pronunciation:<type>
        t = tag.first.substr(0, found);
        has_pronunciation_tags_ = true;
      }

      if (boost::algorithm::starts_with(t, "tunnel:name:"))
        t = t.substr(7);
    }

    std::vector<std::string> tokens;
    std::string lang;

    found = t.find(":lang:");
    // destination:forward:lang:nl
    if (found != std::string::npos) {
      tokens = GetTagTokens(t, ':');
      if (tokens.size() == 4)
        lang = tokens.at(3);
    } else {
      tokens = GetTagTokens(t, ':');
      if (tokens.size() == 3)
        lang = tokens.at(2);
    }

    found = t.find(":to:");
    if (found != std::string::npos && tokens.size() == 5) {
      lang = tokens.at(4);
    }

    if (!lang.empty()) {

      if (stringLanguage(lang) != Language::kNone && !tag_.second.empty()) // name:left:en
      {
        if (name_left_right_w_lang.empty()) {
          name_left_right_w_lang = tag_.second;
          lang_left_right = lang;
        } else {
          name_left_right_w_lang += ";" + tag_.second;
          lang_left_right += ";" + lang;
        }
      }
    }
  }

  void ProcessPronunciationName(const OSMLinguistic::Type& type,
                                const PronunciationAlphabet& alphabet,
                                std::string& name_w_pronunciation,
                                OSMNodeLinguistic* linguistics = nullptr) {
    uint32_t name_index = 0, lang_index = 0;
    std::string language, name_pronunciation_w_lang;
    uint8_t t = static_cast<uint8_t>(type);
    uint8_t alpha = static_cast<uint8_t>(alphabet);

    if (linguistics) {
      if (type == OSMLinguistic::Type::kNodeName) {
        switch (alphabet) {
          case PronunciationAlphabet::kIpa:
            name_index = linguistics->name_pronunciation_ipa_index();
            lang_index = linguistics->name_pronunciation_ipa_lang_index();
            break;
          case PronunciationAlphabet::kKatakana:
            name_index = linguistics->name_pronunciation_katakana_index();
            lang_index = linguistics->name_pronunciation_katakana_lang_index();
            break;
          case PronunciationAlphabet::kJeita:
            name_index = linguistics->name_pronunciation_jeita_index();
            lang_index = linguistics->name_pronunciation_jeita_lang_index();
            break;
          case PronunciationAlphabet::kNtSampa:
            name_index = linguistics->name_pronunciation_nt_sampa_index();
            lang_index = linguistics->name_pronunciation_nt_sampa_lang_index();
            break;
          case PronunciationAlphabet::kNone:
            break;
        }
      } else if (type == OSMLinguistic::Type::kNodeRef) {
        switch (alphabet) {
          case PronunciationAlphabet::kIpa:
            name_index = linguistics->ref_pronunciation_ipa_index();
            lang_index = linguistics->ref_pronunciation_ipa_lang_index();
            break;
          case PronunciationAlphabet::kKatakana:
            name_index = linguistics->ref_pronunciation_katakana_index();
            lang_index = linguistics->ref_pronunciation_katakana_lang_index();
            break;
          case PronunciationAlphabet::kJeita:
            name_index = linguistics->ref_pronunciation_jeita_index();
            lang_index = linguistics->ref_pronunciation_jeita_lang_index();
            break;
          case PronunciationAlphabet::kNtSampa:
            name_index = linguistics->ref_pronunciation_nt_sampa_index();
            lang_index = linguistics->ref_pronunciation_nt_sampa_lang_index();
            break;
          case PronunciationAlphabet::kNone:
            break;
        }
      }
    } else {
      name_index = get_pronunciation_index(t, alpha);
      lang_index = get_lang_index(t, alpha);
    }

    if (name_index != 0)
      name_pronunciation_w_lang = !linguistics ? osmdata_.name_offset_map.name(name_index)
                                               : osmdata_.node_names.name(name_index);

    if (lang_index != 0)
      language = !linguistics ? osmdata_.name_offset_map.name(lang_index)
                              : osmdata_.node_names.name(lang_index);

    ProcessName(name_pronunciation_w_lang, name_w_pronunciation, language);

    SavePronunciationData(t, alpha, name_w_pronunciation, language, linguistics);
  }

  void ProcessPronunciationLRFBName(const std::string& pronunciation_name,
                                    const std::string& pronunciation_lang,
                                    const std::string& name_lr_fb,
                                    const OSMLinguistic::Type& type,
                                    const PronunciationAlphabet& alphabet) {

    uint32_t name_lr_fb_w_lang_index = 0, lang_lr_fb_index = 0;
    uint8_t t = static_cast<uint8_t>(type);
    uint8_t alpha = static_cast<uint8_t>(alphabet);

    name_lr_fb_w_lang_index = get_pronunciation_index(t, alpha);
    lang_lr_fb_index = get_lang_index(t, alpha);

    const std::string name_lr_fb_w_lang = osmdata_.name_offset_map.name(name_lr_fb_w_lang_index);
    std::string name = name_lr_fb;
    std::string lang = osmdata_.name_offset_map.name(lang_lr_fb_index);

    ProcessLRFBName(name_lr_fb_w_lang, pronunciation_name, pronunciation_lang, name, lang);
    SavePronunciationData(t, alpha, name, lang);
  }

  void ProcessName(const std::string& name_w_lang, std::string& name, std::string& language) {
    if (!use_admin_db_)
      return;

    if (!name.empty()) {
      if (name_w_lang.empty())
        return;
      else {
        uint32_t count = std::count(name.begin(), name.end(), ';');
        language.insert(0, count + 1, ';');
        name += ";" + name_w_lang;
      }
    } else
      name = name_w_lang;
  }

  void ProcessLRFBName(const std::string& name_lr_fb_w_lang,
                       const std::string& name_w_lang,
                       const std::string& language,
                       std::string& name_lr_fb,
                       std::string& lang_lr_fb) {
    if (!use_admin_db_)
      return;

    if (!name_lr_fb.empty()) {

      if (name_lr_fb_w_lang.empty())

        if (!name_w_lang.empty()) { // other side of street name may not change
          name_lr_fb += ";" + name_w_lang;
          lang_lr_fb += ";" + language;
        } else
          return;
      else {
        std::string lang = lang_lr_fb;
        uint32_t count = std::count(name_lr_fb.begin(), name_lr_fb.end(), ';');
        for (uint32_t i = 0; i <= count; i++) {
          lang_lr_fb = ";" + lang;
        }

        if (!name_w_lang.empty()) { // other side of street name may not change
          name_lr_fb += ";" + name_lr_fb_w_lang + ";" + name_w_lang;
          lang_lr_fb += ";" + language;
        } else {
          name_lr_fb += ";" + name_lr_fb_w_lang;
        }
      }
    }
  }

  void ProcessDirection(bool int_ref) {

    std::string ref, direction;
    if (int_ref) {
      ref = int_ref_;
      direction = int_direction_;
    } else {
      ref = ref_;
      direction = direction_;
    }

    if (direction.empty()) {
      if (int_ref)
        int_ref_ = ref;
      else
        ref_ = ref;
    } else {
      std::vector<std::string> refs = GetTagTokens(ref);
      std::vector<std::string> directions = GetTagTokens(direction);

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
        if (int_ref)
          int_ref_ = tmp_ref;
        else
          ref_ = tmp_ref;
      } else {
        if (int_ref)
          int_ref_ = ref;
        else
          ref_ = ref;
      }
    }
  }

  void ProcessDirectionPronunciation(const PronunciationAlphabet type, bool int_ref) {

    std::string ref_pronunciation, direction_pronunciation;
    if (int_ref) {
      switch (type) {
        case PronunciationAlphabet::kIpa:
          ref_pronunciation = int_ref_ipa_;
          direction_pronunciation = int_direction_pronunciation_;
          break;
        case PronunciationAlphabet::kKatakana:
          ref_pronunciation = int_ref_katakana_;
          direction_pronunciation = int_direction_pronunciation_katakana_;
          break;
        case PronunciationAlphabet::kNtSampa:
          ref_pronunciation = int_ref_nt_sampa_;
          direction_pronunciation = int_direction_pronunciation_nt_sampa_;
          break;
        case PronunciationAlphabet::kJeita:
          ref_pronunciation = int_ref_jeita_;
          direction_pronunciation = int_direction_pronunciation_jeita_;
          break;
        case PronunciationAlphabet::kNone:
          break;
      }
    } else {
      switch (type) {
        case PronunciationAlphabet::kIpa:
          ref_pronunciation = ref_ipa_;
          direction_pronunciation = direction_pronunciation_;
          break;
        case PronunciationAlphabet::kKatakana:
          ref_pronunciation = ref_katakana_;
          direction_pronunciation = direction_pronunciation_katakana_;
          break;
        case PronunciationAlphabet::kNtSampa:
          ref_pronunciation = ref_nt_sampa_;
          direction_pronunciation = direction_pronunciation_nt_sampa_;
          break;
        case PronunciationAlphabet::kJeita:
          ref_pronunciation = ref_jeita_;
          direction_pronunciation = direction_pronunciation_jeita_;
          break;
        case PronunciationAlphabet::kNone:
          break;
      }
    }

    if (direction_pronunciation.empty()) {
      UpdateRefPronunciation(ref_pronunciation, type, int_ref);
    } else {
      std::vector<std::string> refs = GetTagTokens(ref_pronunciation);
      std::vector<std::string> directions = GetTagTokens(direction_pronunciation);

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
        UpdateRefPronunciation(tmp_ref, type, int_ref);
      } else
        UpdateRefPronunciation(ref_pronunciation, type, int_ref);
    }
  }

  void UpdateRefPronunciation(const std::string& ref_pronunciation,
                              const PronunciationAlphabet alphabet,
                              bool int_ref) {
    uint8_t alpha = static_cast<uint8_t>(alphabet);
    if (int_ref) {
      uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
      pronunciationMap[std::make_pair(t, alpha)] = osmdata_.name_offset_map.index(ref_pronunciation);

    } else {
      uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
      pronunciationMap[std::make_pair(t, alpha)] = osmdata_.name_offset_map.index(ref_pronunciation);
    }
  }

  void MergeRefPronunciations() {

    for (uint8_t alphabet = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
         alphabet != static_cast<uint8_t>(PronunciationAlphabet::kNone); ++alphabet) {
      // add int_ref pronunciations to the end of the pronunciation refs for now.  makes sure that we
      // don't add dups.

      uint32_t index = 0;
      std::string tmp;

      uint8_t t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
      uint8_t alpha = static_cast<uint8_t>(alphabet);

      index = get_pronunciation_index(t, alpha);
      t = static_cast<uint8_t>(OSMLinguistic::Type::kRef);
      tmp = (index ? osmdata_.name_offset_map.name(get_pronunciation_index(t, alpha)) : "");

      if (index) {
        std::vector<std::string> rs = GetTagTokens(tmp);
        std::vector<std::string> is = GetTagTokens(osmdata_.name_offset_map.name(index));
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

          pronunciationMap[std::make_pair(t, alpha)] = osmdata_.name_offset_map.index(tmp);
        }
        // no matter what, clear out the int_ref.
        t = static_cast<uint8_t>(OSMLinguistic::Type::kIntRef);
        pronunciationMap[std::make_pair(t, alpha)] = 0;
      }
    }
  }

  // WayCallback tag handlers
  using TagHandler = std::function<void()>;
  std::unordered_map<std::string, TagHandler> tag_handlers_;
  // Tag handlers capture these fields
  OSMWay way_;
  std::pair<std::string, std::string> tag_;
  uint64_t osmid_;

  const uint8_t ipa = static_cast<uint8_t>(PronunciationAlphabet::kIpa);
  const uint8_t nt_sampa = static_cast<uint8_t>(PronunciationAlphabet::kNtSampa);
  const uint8_t katakana = static_cast<uint8_t>(PronunciationAlphabet::kKatakana);
  const uint8_t jeita = static_cast<uint8_t>(PronunciationAlphabet::kJeita);

  float default_speed_ = 0.0f, max_speed_ = 0.0f;
  float average_speed_ = 0.0f, advisory_speed_ = 0.0f;
  bool has_default_speed_ = false, has_max_speed_ = false;
  bool has_average_speed_ = false, has_advisory_speed_ = false;
  bool has_surface_ = true;
  bool has_surface_tag_ = true, has_tracktype_tag_ = true;
  OSMAccess osm_access_;
  std::map<std::pair<uint8_t, uint8_t>, uint32_t> pronunciationMap;
  std::map<std::pair<uint8_t, uint8_t>, uint32_t> langMap;
  bool has_user_tags_ = false, has_pronunciation_tags_ = false;

  std::string ref_, ref_language_, ref_w_lang_, ref_left_, ref_right_, ref_lang_left_,
      ref_lang_right_, ref_left_w_lang_, ref_right_w_lang_, payment_, payment_forward_,
      payment_backward_;

  std::string int_ref_, int_ref_language_, int_ref_w_lang_, int_ref_left_, int_ref_right_,
      int_ref_lang_left_, int_ref_lang_right_, int_ref_left_w_lang_, int_ref_right_w_lang_;

  std::string direction_, int_direction_;

  std::string direction_pronunciation_, int_direction_pronunciation_,
      direction_pronunciation_nt_sampa_, int_direction_pronunciation_nt_sampa_,
      direction_pronunciation_katakana_, int_direction_pronunciation_katakana_,
      direction_pronunciation_jeita_, int_direction_pronunciation_jeita_;
  std::string name_, language_, name_w_lang_, service_, amenity_;

  std::string ref_ipa_, ref_left_ipa_, ref_right_ipa_;
  std::string ref_katakana_, ref_left_katakana_, ref_right_katakana_;
  std::string ref_jeita_, ref_left_jeita_, ref_right_jeita_;
  std::string ref_nt_sampa_, ref_left_nt_sampa_, ref_right_nt_sampa_;

  std::string int_ref_ipa_, int_ref_left_ipa_, int_ref_right_ipa_;
  std::string int_ref_katakana_, int_ref_left_katakana_, int_ref_right_katakana_;
  std::string int_ref_jeita_, int_ref_left_jeita_, int_ref_right_jeita_;
  std::string int_ref_nt_sampa_, int_ref_left_nt_sampa_, int_ref_right_nt_sampa_;

  std::string name_ipa_, name_left_ipa_, name_right_ipa_, name_forward_ipa_, name_backward_ipa_;
  std::string name_katakana_, name_left_katakana_, name_right_katakana_, name_forward_katakana_,
      name_backward_katakana_;
  std::string name_jeita_, name_left_jeita_, name_right_jeita_, name_forward_jeita_,
      name_backward_jeita_;
  std::string name_nt_sampa_, name_left_nt_sampa_, name_right_nt_sampa_, name_forward_nt_sampa_,
      name_backward_nt_sampa_;

  std::string alt_name_ipa_, alt_name_left_ipa_, alt_name_right_ipa_;
  std::string alt_name_katakana_, alt_name_left_katakana_, alt_name_right_katakana_;
  std::string alt_name_jeita_, alt_name_left_jeita_, alt_name_right_jeita_;
  std::string alt_name_nt_sampa_, alt_name_left_nt_sampa_, alt_name_right_nt_sampa_;

  std::string official_name_ipa_, official_name_left_ipa_, official_name_right_ipa_;
  std::string official_name_katakana_, official_name_left_katakana_, official_name_right_katakana_;
  std::string official_name_jeita_, official_name_left_jeita_, official_name_right_jeita_;
  std::string official_name_nt_sampa_, official_name_left_nt_sampa_, official_name_right_nt_sampa_;

  std::string tunnel_name_ipa_, tunnel_name_left_ipa_, tunnel_name_right_ipa_;
  std::string tunnel_name_katakana_, tunnel_name_left_katakana_, tunnel_name_right_katakana_;
  std::string tunnel_name_jeita_, tunnel_name_left_jeita_, tunnel_name_right_jeita_;
  std::string tunnel_name_nt_sampa_, tunnel_name_left_nt_sampa_, tunnel_name_right_nt_sampa_;

  std::string destination_ipa_, destination_forward_ipa_, destination_backward_ipa_;
  std::string destination_katakana_, destination_forward_katakana_, destination_backward_katakana_;
  std::string destination_jeita_, destination_forward_jeita_, destination_backward_jeita_;
  std::string destination_nt_sampa_, destination_forward_nt_sampa_, destination_backward_nt_sampa_;

  std::string destination_ref_ipa_, destination_ref_to_ipa_, destination_street_ipa_,
      destination_street_to_ipa_, junction_ref_ipa_, junction_name_ipa_;
  std::string destination_ref_nt_sampa_, destination_ref_to_nt_sampa_, destination_street_nt_sampa_,
      destination_street_to_nt_sampa_, junction_ref_nt_sampa_, junction_name_nt_sampa_;
  std::string destination_ref_katakana_, destination_ref_to_katakana_, destination_street_katakana_,
      destination_street_to_katakana_, junction_ref_katakana_, junction_name_katakana_;
  std::string destination_ref_jeita_, destination_ref_to_jeita_, destination_street_jeita_,
      destination_street_to_jeita_, junction_ref_jeita_, junction_name_jeita_;

  std::string name_left_, name_right_, lang_left_, lang_right_;
  std::string name_left_w_lang_, name_right_w_lang_;

  std::string name_forward_, name_backward_, lang_forward_, lang_backward_;
  std::string name_forward_w_lang_, name_backward_w_lang_;

  std::string official_name_, official_language_, official_name_w_lang_, official_name_left_,
      official_name_right_, official_lang_left_, official_lang_right_, official_name_left_w_lang_,
      official_name_right_w_lang_;

  std::string tunnel_name_, tunnel_language_, tunnel_name_w_lang_, tunnel_name_left_,
      tunnel_name_right_, tunnel_lang_left_, tunnel_lang_right_, tunnel_name_left_w_lang_,
      tunnel_name_right_w_lang_;

  std::string alt_name_, alt_language_, alt_name_w_lang_, alt_name_left_, alt_name_right_,
      alt_lang_left_, alt_lang_right_, alt_name_left_w_lang_, alt_name_right_w_lang_;

  std::string destination_, destination_language_, destination_w_lang_, destination_forward_,
      destination_forward_language_, destination_forward_w_lang_, destination_backward_,
      destination_backward_language_, destination_backward_w_lang_;

  std::string destination_ref_, destination_ref_language_, destination_ref_w_lang_,
      destination_ref_to_, destination_ref_to_language_, destination_ref_to_w_lang_,
      destination_street_, destination_street_language_, destination_street_w_lang_,
      destination_street_to_, destination_street_to_language_, destination_street_to_w_lang_;

  std::string junction_name_, junction_name_language_, junction_name_w_lang_, junction_ref_,
      junction_ref_language_, junction_ref_w_lang_;

  // Configuration option to include highway=platform for pedestrians
  bool include_platforms_;

  // Configuration option to include driveways
  bool include_driveways_;

  // Configuration option to include roads under construction
  bool include_construction_;

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

  // node linguistics
  std::unique_ptr<sequence<OSMNodeLinguistic>> node_linguistics_;

  // used to set "culdesac" labels to loop roads correctly
  culdesac_processor culdesac_processor_;

  // empty objects initialized with defaults to use when no tags are present on objects
  Tags empty_node_results_;
  Tags empty_way_results_;
  Tags empty_relation_results_;

  uint32_t get_pronunciation_index(const uint8_t type, const uint8_t alpha) {
    auto itr = pronunciationMap.find(std::make_pair(type, alpha));
    if (itr != pronunciationMap.end()) {
      return itr->second;
    }
    return 0;
  }

  uint32_t get_lang_index(const uint8_t type, const uint8_t alpha) {
    auto itr = langMap.find(std::make_pair(type, alpha));
    if (itr != langMap.end()) {
      return itr->second;
    }
    return 0;
  }
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
                 new sequence<OSMAccess>(access_file, true), nullptr, nullptr, nullptr, nullptr);
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
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

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
                 new sequence<OSMRestriction>(complex_restriction_to_file, true), nullptr, nullptr);

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
  LOG_INFO("Finished with " + std::to_string(osmdata.restrictions.size()) +
           " simple turn restrictions");
  LOG_INFO("Finished with " + std::to_string(osmdata.lane_connectivity_map.size()) +
           " lane connections");

  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

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
                                const std::string& linguistic_node_file,
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

    bool create = true;
    for (auto& file_handle : file_handles) {
      callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
          callback.last_relation_ = 0;
      // we send a null way_nodes file so that only the bike share stations are parsed
      callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr,
                     new sequence<OSMNode>(bss_nodes_file, create), nullptr);
      OSMPBF::Parser::parse(file_handle, static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES),
                            callback);
      create = false;
    }
    // Since the sequence must be flushed before reading it...
    callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    LOG_INFO("Found " + std::to_string(sequence<OSMNode>{bss_nodes_file, false}.size()) +
             " bss nodes...");
  }
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

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
                   nullptr, nullptr, new sequence<OSMNodeLinguistic>(linguistic_node_file, true));
    callback.current_way_node_index_ = callback.last_node_ = callback.last_way_ =
        callback.last_relation_ = 0;
    OSMPBF::Parser::parse(file_handle,
                          static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES |
                                                        OSMPBF::Interest::CHANGESETS),
                          callback);
  }
  uint64_t max_osm_id = callback.last_node_;
  callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
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
  LOG_INFO("Number of nodes with linguistics = " + std::to_string(osmdata.node_linguistic_count));
  LOG_INFO("Number of way refs = " + std::to_string(osmdata.way_ref.size()));
  LOG_INFO("Number of reverse way refs = " + std::to_string(osmdata.way_ref_rev.size()));
  LOG_INFO("Unique Node Strings (names, refs, etc.) = " + std::to_string(osmdata.node_names.Size()));
  LOG_INFO("Unique Strings (names, refs, etc.) = " + std::to_string(osmdata.name_offset_map.Size()));
}

} // namespace mjolnir
} // namespace valhalla
