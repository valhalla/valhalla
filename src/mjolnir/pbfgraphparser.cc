#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/osmpbfparser.h"
#include "mjolnir/util.h"

#include "graph_lua_proc.h"
#include "mjolnir/idtable.h"
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

// This value controls the initial size of the Id table. If this is exceeded
// the table will be resized and a warning is generated (indicating we should
// increase this value).
constexpr uint64_t kMaxOSMNodeId = 6800000000;

// Absurd classification.
constexpr uint32_t kAbsurdRoadClass = 777777;

// Construct PBFGraphParser based on properties file and input PBF extract
struct graph_callback : public OSMPBF::Callback {
public:
  graph_callback() = delete;
  graph_callback(const graph_callback&) = delete;
  virtual ~graph_callback() {
  }

  graph_callback(const boost::property_tree::ptree& pt, OSMData& osmdata)
      : shape_(kMaxOSMNodeId), intersection_(kMaxOSMNodeId), osmdata_(osmdata), lua_(get_lua(pt)) {

    current_way_node_index_ = last_node_ = last_way_ = last_relation_ = 0;

    highway_cutoff_rc_ = RoadClass::kPrimary;
    for (auto& level : TileHierarchy::levels()) {
      if (level.second.name == "highway") {
        highway_cutoff_rc_ = level.second.importance;
      }
    }

    include_driveways_ = pt.get<bool>("include_driveways", true);
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

  virtual void
  node_callback(uint64_t osmid, double lng, double lat, const OSMPBF::Tags& tags) override {
    boost::optional<Tags> results = boost::none;
    if (bss_nodes_) {
      // Get tags
      results = lua_.Transform(OSMType::kNode, tags);

      for (auto& key_value : *results) {
        if (key_value.first == "amenity" && key_value.second == "bicycle_rental") {
          // Create a new node and set its attributes
          OSMNode n{osmid};
          n.set_latlng(static_cast<float>(lng), static_cast<float>(lat));
          n.set_type(NodeType::kBikeShare);
          bss_nodes_->push_back(n);
          ++osmdata_.node_count;
        }
      }
    }

    // Check if it is in the list of nodes used by ways
    if (!shape_.get(osmid)) {
      return;
    }

    // Get tags if not already available
    results = results ? results : lua_.Transform(OSMType::kNode, tags);
    if (results->size() == 0) {
      return;
    }

    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid < last_node_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_node_ = osmid;

    const auto& highway_junction = results->find("highway");
    bool is_highway_junction =
        ((highway_junction != results->end()) && (highway_junction->second == "motorway_junction"));

    // Create a new node and set its attributes
    OSMNode n;
    n.set_id(osmid);
    n.set_latlng(static_cast<float>(lng), static_cast<float>(lat));
    if (is_highway_junction) {
      n.set_type(NodeType::kMotorWayJunction);
    }

    for (const auto& tag : *results) {

      if (tag.first == "highway") {
        n.set_traffic_signal(tag.second == "traffic_signals" ? true : false);
      } else if (tag.first == "forward_signal") {
        n.set_forward_signal(tag.second == "true" ? true : false);
      } else if (tag.first == "backward_signal") {
        n.set_backward_signal(tag.second == "true" ? true : false);
      } else if (is_highway_junction && (tag.first == "exit_to")) {
        bool hasTag = (tag.second.length() ? true : false);
        if (hasTag) {
          // Add the name to the unique node names list and store its index in the OSM node
          n.set_exit_to_index(osmdata_.node_names.index(tag.second));
          ++osmdata_.node_exit_to_count;
        }
      } else if (is_highway_junction && (tag.first == "ref")) {
        bool hasTag = (tag.second.length() ? true : false);
        if (hasTag) {
          // Add the name to the unique node names list and store its index in the OSM node
          n.set_ref_index(osmdata_.node_names.index(tag.second));
          ++osmdata_.node_ref_count;
        }
      } else if (is_highway_junction && (tag.first == "name")) {
        bool hasTag = (tag.second.length() ? true : false);
        if (hasTag) {
          // Add the name to the unique node names list and store its index in the OSM node
          n.set_name_index(osmdata_.node_names.index(tag.second));
          ++osmdata_.node_name_count;
        }
      } else if (tag.first == "gate") {
        if (tag.second == "true") {
          if (!intersection_.get(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kGate);
        }
      } else if (tag.first == "bollard") {
        if (tag.second == "true") {
          if (!intersection_.get(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kBollard);
        }
      } else if (tag.first == "toll_booth") {
        if (tag.second == "true") {
          if (!intersection_.get(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kTollBooth);
        }
      } else if (tag.first == "border_control") {
        if (tag.second == "true") {
          if (!intersection_.get(osmid)) {
            intersection_.set(osmid);
            ++osmdata_.edge_count;
          }
          n.set_type(NodeType::kBorderControl);
        }
      } else if (tag.first == "access_mask") {
        n.set_access(std::stoi(tag.second));
      }

      /* TODO: payment type.
      else if (tag.first == "payment_mask")
        n.set_payment_mask(std::stoi(tag.second));
      */
    }

    // Set the intersection flag (relies on ways being processed first to set
    // the intersection Id markers).
    if (intersection_.get(osmid)) {
      n.set_intersection(true);
      osmdata_.intersection_count++;
    }

    // find a node we need to update
    current_way_node_index_ = way_nodes_->find_first_of(OSMWayNode{{osmid}},
                                                        [](const OSMWayNode& a, const OSMWayNode& b) {
                                                          return a.node.osmid_ == b.node.osmid_;
                                                        },
                                                        current_way_node_index_);
    // we found the first one
    if (current_way_node_index_ < way_nodes_->size()) {
      // update all the nodes that match it
      OSMWayNode way_node;
      sequence<OSMWayNode>::iterator element = (*way_nodes_)[current_way_node_index_];
      while (current_way_node_index_ < way_nodes_->size() &&
             (way_node = element = (*way_nodes_)[current_way_node_index_]).node.osmid_ == osmid) {
        way_node.node = n;
        element = way_node;
        ++current_way_node_index_;
      }

      if (++osmdata_.osm_node_count % 5000000 == 0) {
        LOG_DEBUG("Processed " + std::to_string(osmdata_.osm_node_count) + " nodes on ways");
      }
    } // if we hit the end of the nodes and didnt find it that is a problem
    else {
      throw std::runtime_error("Didn't find OSMWayNode for node id: " + std::to_string(osmid));
    }
  }

  virtual void way_callback(uint64_t osmid,
                            const OSMPBF::Tags& tags,
                            const std::vector<uint64_t>& nodes) override {

    // Do not add ways with < 2 nodes. Log error or add to a problem list
    // TODO - find out if we do need these, why they exist...
    if (nodes.size() < 2) {
      return;
    }

    // Transform tags. If no results that means the way does not have tags
    // suitable for use in routing.
    Tags results = lua_.Transform(OSMType::kWay, tags);
    if (results.size() == 0) {
      return;
    }

    // Throw away closed features with following tags: building, landuse,
    // leisure, natural. See: http://wiki.openstreetmap.org/wiki/Key:area
    if (nodes[0] == nodes[nodes.size() - 1]) {
      for (const auto& tag : results) {
        if (tag.first == "building" || tag.first == "landuse" || tag.first == "leisure" ||
            tag.first == "natural") {
          // LOG_INFO("Loop wayid " + std::to_string(osmid) + " Discard?");
          return;
        }
      }
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
      LOG_INFO("invalid_argument thrown for way id: " + std::to_string(osmid));
    } catch (const std::out_of_range& oor) {
      LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
    }

    // Check for ways that loop back on themselves (simple check) and add
    // any wayids that have loops to a vector
    if (nodes.size() > 2) {
      for (size_t i = 2, j = 0; i < nodes.size(); i++, j++) {
        if (nodes[i] == nodes[j]) {
          loops_.push_back(osmid);
          break;
        }
      }
    }

    if (osmid > kMaxOSMWayId) {
      throw std::runtime_error("OSM way Id exceeds 32 bit maximum");
    }
    uint32_t wayid = static_cast<uint32_t>(osmid);

    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid < last_way_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_way_ = osmid;

    // Add the refs to the reference list and mark the nodes that care about when processing nodes
    loop_nodes_.clear();
    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto& node = nodes[i];
      if (shape_.get(node)) {
        intersection_.set(node);
        ++osmdata_.edge_count;
      } else {
        ++osmdata_.node_count;
      }
      way_nodes_->push_back({{node}, static_cast<uint32_t>(ways_->size()), static_cast<uint32_t>(i)});
      shape_.set(node);
      // If this way is a loop (node occurs twice) we can make our lives way easier if we simply
      // split it up into multiple edges in the graph. If a problem is hard, avoid the problem!
      auto inserted = loop_nodes_.insert(std::make_pair(node, i));
      if (inserted.second == false) {
        // Walk through nodes between the 2 nodes that form the loop and see if
        // there are already intersections
        bool intsct = false;
        for (size_t j = inserted.first->second + 1; j < i; ++j) {
          if (intersection_.get(nodes[j])) {
            intsct = true;
            break;
          }
        }
        if (!intsct) {
          intersection_.set(
              nodes[(i + inserted.first->second) / 2]); // TODO: update osmdata_.*_count?
        }

        // Update the index in case the node is used again (a future loop)
        inserted.first->second = i;
      }
    }
    intersection_.set(nodes.front());
    intersection_.set(nodes.back());
    osmdata_.edge_count += 2;
    ++osmdata_.osm_way_count;
    osmdata_.osm_way_node_count += nodes.size();

    float default_speed = 0.0f, max_speed = 0.0f;
    float average_speed = 0.0f, advisory_speed = 0.0f;
    bool has_default_speed = false, has_max_speed = false;
    bool has_average_speed = false, has_advisory_speed = false;
    bool has_surface = true;
    std::string name;

    // Process tags
    OSMWay w{wayid};
    w.set_node_count(nodes.size());

    OSMAccess access{wayid};
    bool has_user_tags = false;

    const auto& surface_exists = results.find("surface");
    bool has_surface_tag = (surface_exists != results.end());
    if (!has_surface_tag) {
      has_surface = false;
    }

    const auto& highway_junction = results.find("highway");
    bool is_highway_junction =
        ((highway_junction != results.end()) && (highway_junction->second == "motorway_junction"));

    for (const auto& tag : results) {

      if (tag.first == "road_class") {
        RoadClass roadclass = (RoadClass)std::stoi(tag.second);
        switch (roadclass) {

          case RoadClass::kMotorway:
            w.set_road_class(RoadClass::kMotorway);
            break;
          case RoadClass::kTrunk:
            w.set_road_class(RoadClass::kTrunk);
            break;
          case RoadClass::kPrimary:
            w.set_road_class(RoadClass::kPrimary);
            break;
          case RoadClass::kSecondary:
            w.set_road_class(RoadClass::kSecondary);
            break;
          case RoadClass::kTertiary:
            w.set_road_class(RoadClass::kTertiary);
            break;
          case RoadClass::kUnclassified:
            w.set_road_class(RoadClass::kUnclassified);
            break;
          case RoadClass::kResidential:
            w.set_road_class(RoadClass::kResidential);
            break;
          default:
            w.set_road_class(RoadClass::kServiceOther);
            break;
        }
      }
      // these flags indicate if a user set the access tags on this way.
      else if (tag.first == "auto_tag") {
        access.set_auto_tag(true);
        has_user_tags = true;
      } else if (tag.first == "truck_tag") {
        access.set_truck_tag(true);
        has_user_tags = true;
      } else if (tag.first == "bus_tag") {
        access.set_bus_tag(true);
        has_user_tags = true;
      } else if (tag.first == "foot_tag") {
        access.set_foot_tag(true);
        has_user_tags = true;
      } else if (tag.first == "bike_tag") {
        access.set_bike_tag(true);
        has_user_tags = true;
      } else if (tag.first == "moped_tag") {
        access.set_moped_tag(true);
        has_user_tags = true;
      } else if (tag.first == "motorcycle_tag") {
        access.set_motorcycle_tag(true);
        has_user_tags = true;
      } else if (tag.first == "hov_tag") {
        access.set_hov_tag(true);
        has_user_tags = true;
      } else if (tag.first == "taxi_tag") {
        access.set_taxi_tag(true);
        has_user_tags = true;
      } else if (tag.first == "motorroad_tag") {
        access.set_motorroad_tag(true);
        has_user_tags = true;
      }

      else if (tag.first == "wheelchair") {
        w.set_wheelchair_tag(true);
        w.set_wheelchair(tag.second == "true" ? true : false);
      }

      else if (tag.first == "sidewalk") {
        if (tag.second == "both" || tag.second == "yes" || tag.second == "shared" ||
            tag.second == "raised") {
          w.set_sidewalk_left(true);
          w.set_sidewalk_right(true);
        } else if (tag.second == "left") {
          w.set_sidewalk_left(true);
        } else if (tag.second == "right") {
          w.set_sidewalk_right(true);
        }
      }

      else if (tag.first == "auto_forward") {
        w.set_auto_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "truck_forward") {
        w.set_truck_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "bus_forward") {
        w.set_bus_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "bike_forward") {
        w.set_bike_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "emergency_forward") {
        w.set_emergency_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "hov_forward") {
        w.set_hov_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "taxi_forward") {
        w.set_taxi_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "moped_forward") {
        w.set_moped_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "motorcycle_forward") {
        w.set_motorcycle_forward(tag.second == "true" ? true : false);
      } else if (tag.first == "auto_backward") {
        w.set_auto_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "truck_backward") {
        w.set_truck_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "bus_backward") {
        w.set_bus_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "bike_backward") {
        w.set_bike_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "emergency_backward") {
        w.set_emergency_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "hov_backward") {
        w.set_hov_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "taxi_backward") {
        w.set_taxi_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "moped_backward") {
        w.set_moped_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "motorcycle_backward") {
        w.set_motorcycle_backward(tag.second == "true" ? true : false);
      } else if (tag.first == "pedestrian") {
        w.set_pedestrian(tag.second == "true" ? true : false);
      } else if (tag.first == "private" && tag.second == "true") {
        // Make sure we do not unset this flag if set previously
        w.set_destination_only(true);
      } else if (tag.first == "use") {
        Use use = (Use)std::stoi(tag.second);
        switch (use) {
          case Use::kCycleway:
            w.set_use(Use::kCycleway);
            break;
          case Use::kFootway:
            w.set_use(Use::kFootway);
            break;
          case Use::kSidewalk:
            w.set_use(Use::kSidewalk);
            break;
          case Use::kPedestrian:
            w.set_use(Use::kPedestrian);
            break;
          case Use::kPath:
            w.set_use(Use::kPath);
            break;
          case Use::kSteps:
            w.set_use(Use::kSteps);
            break;
          case Use::kBridleway:
            w.set_use(Use::kBridleway);
            break;
          case Use::kLivingStreet:
            w.set_use(Use::kLivingStreet);
            break;
          case Use::kParkingAisle:
            w.set_destination_only(true);
            w.set_use(Use::kParkingAisle);
            break;
          case Use::kDriveway:
            w.set_destination_only(true);
            w.set_use(Use::kDriveway);
            break;
          case Use::kAlley:
            w.set_use(Use::kAlley);
            break;
          case Use::kEmergencyAccess:
            w.set_use(Use::kEmergencyAccess);
            break;
          case Use::kDriveThru:
            w.set_destination_only(true);
            w.set_use(Use::kDriveThru);
            break;
          case Use::kTrack:
            w.set_use(Use::kTrack);
            break;
          case Use::kOther:
            w.set_use(Use::kOther);
            break;
          case Use::kRoad:
          default:
            w.set_use(Use::kRoad);
            break;
        }
      } else if (tag.first == "no_thru_traffic") {
        w.set_no_thru_traffic(tag.second == "true" ? true : false);
      } else if (tag.first == "oneway") {
        w.set_oneway(tag.second == "true" ? true : false);
      } else if (tag.first == "oneway_reverse") {
        w.set_oneway_reverse(tag.second == "true" ? true : false);
      } else if (tag.first == "roundabout") {
        w.set_roundabout(tag.second == "true" ? true : false);
      } else if (tag.first == "link") {
        w.set_link(tag.second == "true" ? true : false);
      } else if (tag.first == "link_type") {
        w.set_turn_channel(tag.second == "slip" ? true : false);
      } else if (tag.first == "ferry") {
        w.set_ferry(tag.second == "true" ? true : false);
      } else if (tag.first == "rail") {
        w.set_rail(tag.second == "true" ? true : false);

      } else if (tag.first == "duration") {
        std::size_t found = tag.second.find(":");
        if (found == std::string::npos) {
          continue;
        }
        std::vector<std::string> time = GetTagTokens(tag.second, ':');
        uint32_t hour = 0, min = 0, sec = 0;
        if (time.size() == 1) { // minutes
          std::stringstream ss(time.at(0));
          ss >> min;
          min *= 60;
        } else if (time.size() == 2) { // hours and min
          std::stringstream ss(tag.second);
          ss >> hour;
          ss.ignore();
          hour *= 3600;

          ss >> min;
          min *= 60;
        } else if (time.size() == 3) { // hours, min, and sec
          std::stringstream ss(tag.second);
          ss >> hour;
          ss.ignore();
          hour *= 3600;

          ss >> min;
          ss.ignore();
          min *= 60;

          ss >> sec;
        }
        w.set_duration(hour + min + sec);
      }

      else if (tag.first == "name" && !tag.second.empty()) {
        name = tag.second;
      } else if (tag.first == "name:en" && !tag.second.empty()) {
        w.set_name_en_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "alt_name" && !tag.second.empty()) {
        w.set_alt_name_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "official_name" && !tag.second.empty()) {
        w.set_official_name_index(osmdata_.name_offset_map.index(tag.second));

      } else if (tag.first == "max_speed") {
        try {
          max_speed = std::stof(tag.second);
          has_max_speed = true;
          w.set_tagged_speed(true);
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      } else if (tag.first == "average_speed") {
        try {
          average_speed = std::stof(tag.second);
          has_average_speed = true;
          w.set_tagged_speed(true);
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      } else if (tag.first == "advisory_speed") {
        try {
          advisory_speed = std::stof(tag.second);
          has_advisory_speed = true;
          w.set_tagged_speed(true);
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      } else if (tag.first == "forward_speed") {
        try {
          w.set_forward_speed(std::stof(tag.second));
          w.set_forward_tagged_speed(true);
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      } else if (tag.first == "backward_speed") {
        try {
          w.set_backward_speed(std::stof(tag.second));
          w.set_backward_tagged_speed(true);
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      }

      else if (tag.first == "maxspeed:hgv") {
        try {
          w.set_truck_speed(std::stof(tag.second));
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      } else if (tag.first == "truck_route") {
        w.set_truck_route(tag.second == "true" ? true : false);
      }

      // motor_vehicle:conditional=no @ (16:30-07:00)
      else if (tag.first == "motorcar:conditional" || tag.first == "motor_vehicle:conditional" ||
               tag.first == "bicycle:conditional" || tag.first == "motorcycle:conditional" ||
               tag.first == "foot:conditional" || tag.first == "pedestrian:conditional" ||
               tag.first == "hgv:conditional" || tag.first == "moped:conditional" ||
               tag.first == "mofa:conditional" || tag.first == "psv:conditional" ||
               tag.first == "taxi:conditional" || tag.first == "bus:conditional" ||
               tag.first == "hov:conditional" || tag.first == "emergency:conditional") {

        std::vector<std::string> tokens = GetTagTokens(tag.second, '@');
        std::string tmp = tokens.at(0);
        boost::algorithm::trim(tmp);

        AccessType type = AccessType::kTimedDenied;
        if (tmp == "no") {
          type = AccessType::kTimedDenied;
        } else if (tmp == "yes" || tmp == "private" || tmp == "delivery" || tmp == "designated") {
          type = AccessType::kTimedAllowed;
        }

        if (tokens.size() == 2 && tmp.size()) {

          uint16_t mode = 0;
          if (tag.first == "motorcar:conditional" || tag.first == "motor_vehicle:conditional") {
            mode = (kAutoAccess | kTruckAccess | kEmergencyAccess | kTaxiAccess | kBusAccess |
                    kHOVAccess | kMopedAccess | kMotorcycleAccess);
          } else if (tag.first == "bicycle:conditional") {
            mode = kBicycleAccess;
          } else if (tag.first == "foot:conditional" || tag.first == "pedestrian:conditional") {
            mode = (kPedestrianAccess | kWheelchairAccess);
          } else if (tag.first == "hgv:conditional") {
            mode = kTruckAccess;
          } else if (tag.first == "moped:conditional" || tag.first == "mofa:conditional") {
            mode = kMopedAccess;
          } else if (tag.first == "motorcycle:conditional") {
            mode = kMotorcycleAccess;
          } else if (tag.first == "psv:conditional") {
            mode = (kTaxiAccess | kBusAccess);
          } else if (tag.first == "taxi:conditional") {
            mode = kTaxiAccess;
          } else if (tag.first == "bus:conditional") {
            mode = kBusAccess;
          } else if (tag.first == "hov:conditional") {
            mode = kHOVAccess;
          } else if (tag.first == "emergency:conditional") {
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
              osmdata_.access_restrictions.insert(
                  AccessRestrictionsMultiMap::value_type(osmid, restriction));
            }
          }
        }
      }

      else if (tag.first == "hazmat") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kHazmat);
        restriction.set_value(tag.second == "true" ? true : false);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      } else if (tag.first == "maxheight") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kMaxHeight);
        restriction.set_value(std::stof(tag.second) * 100);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      } else if (tag.first == "maxwidth") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kMaxWidth);
        restriction.set_value(std::stof(tag.second) * 100);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      } else if (tag.first == "maxlength") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kMaxLength);
        restriction.set_value(std::stof(tag.second) * 100);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      } else if (tag.first == "maxweight") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kMaxWeight);
        restriction.set_value(std::stof(tag.second) * 100);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      } else if (tag.first == "maxaxleload") {
        OSMAccessRestriction restriction;
        restriction.set_type(AccessType::kMaxAxleLoad);
        restriction.set_value(std::stof(tag.second) * 100);
        restriction.set_modes(kTruckAccess);
        osmdata_.access_restrictions.insert(
            AccessRestrictionsMultiMap::value_type(osmid, restriction));
      }

      else if (tag.first == "default_speed") {
        try {
          default_speed = std::stof(tag.second);
          has_default_speed = true;
        } catch (const std::out_of_range& oor) {
          LOG_INFO("out_of_range thrown for way id: " + std::to_string(osmid));
        }
      }

      else if (tag.first == "ref" && !tag.second.empty()) {
        w.set_ref_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "int_ref" && !tag.second.empty()) {
        w.set_int_ref_index(osmdata_.name_offset_map.index(tag.second));

      } else if (tag.first == "sac_scale") {
        std::string value = tag.second;
        boost::algorithm::to_lower(value);

        if (value.find("difficult_alpine_hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kDifficultAlpineHiking);

        } else if (value.find("demanding_alpine_hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kDemandingAlpineHiking);

        } else if (value.find("alpine_hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kAlpineHiking);

        } else if (value.find("demanding_mountain_hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kDemandingMountainHiking);

        } else if (value.find("mountain_hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kMountainHiking);

        } else if (value.find("hiking") != std::string::npos) {
          w.set_sac_scale(SacScale::kHiking);

        } else {
          w.set_sac_scale(SacScale::kNone);
        }
      }

      else if (tag.first == "surface") {
        std::string value = tag.second;
        boost::algorithm::to_lower(value);

        // Find unpaved before paved since they have common string
        if (value.find("unpaved") != std::string::npos) {
          w.set_surface(Surface::kGravel);

        } else if (value.find("paved") != std::string::npos ||
                   value.find("pavement") != std::string::npos ||
                   value.find("asphalt") != std::string::npos ||
                   value.find("concrete") != std::string::npos ||
                   value.find("cement") != std::string::npos) {
          w.set_surface(Surface::kPavedSmooth);

        } else if (value.find("tartan") != std::string::npos ||
                   value.find("pavingstone") != std::string::npos ||
                   value.find("paving_stones") != std::string::npos ||
                   value.find("sett") != std::string::npos) {
          w.set_surface(Surface::kPaved);

        } else if (value.find("cobblestone") != std::string::npos ||
                   value.find("brick") != std::string::npos) {
          w.set_surface(Surface::kPavedRough);

        } else if (value.find("compacted") != std::string::npos ||
                   value.find("wood") != std::string::npos ||
                   value.find("boardwalk") != std::string::npos) {
          w.set_surface(Surface::kCompacted);

        } else if (value.find("dirt") != std::string::npos ||
                   value.find("natural") != std::string::npos ||
                   value.find("earth") != std::string::npos ||
                   value.find("ground") != std::string::npos ||
                   value.find("mud") != std::string::npos) {
          w.set_surface(Surface::kDirt);

        } else if (value.find("gravel") != std::string::npos ||
                   value.find("pebblestone") != std::string::npos ||
                   value.find("sand") != std::string::npos) {
          w.set_surface(Surface::kGravel);
        } else if (value.find("grass") != std::string::npos) {
          w.set_surface(Surface::kPath);
          // We have to set a flag as surface may come before Road classes and Uses
        } else {
          has_surface = false;
        }
      }

      // surface tag should win over tracktype.
      else if (tag.first == "tracktype" && !has_surface_tag) {

        has_surface = true;

        if (tag.second == "grade1") {
          w.set_surface(Surface::kPavedRough);
        } else if (tag.second == "grade2") {
          w.set_surface(Surface::kCompacted);
        } else if (tag.second == "grade3") {
          w.set_surface(Surface::kDirt);
        } else if (tag.second == "grade4") {
          w.set_surface(Surface::kGravel);
        } else if (tag.second == "grade5") {
          w.set_surface(Surface::kPath);
        } else {
          has_surface = false;
        }
      }

      else if (tag.first == "bicycle") {
        if (tag.second == "dismount") {
          w.set_dismount(true);
        } else if (tag.second == "use_sidepath") {
          w.set_use_sidepath(true);
        }
      }

      else if (tag.first == "shoulder_right") {
        w.set_shoulder_right(tag.second == "true" ? true : false);
      } else if (tag.first == "shoulder_left") {
        w.set_shoulder_left(tag.second == "true" ? true : false);
      }

      else if (tag.first == "cycle_lane_right") {
        CycleLane cyclelane_right = (CycleLane)std::stoi(tag.second);
        switch (cyclelane_right) {
          case CycleLane::kDedicated:
            w.set_cyclelane_right(CycleLane::kDedicated);
            break;
          case CycleLane::kSeparated:
            w.set_cyclelane_right(CycleLane::kSeparated);
            break;
          case CycleLane::kShared:
            w.set_cyclelane_right(CycleLane::kShared);
            break;
          case CycleLane::kNone:
          default:
            w.set_cyclelane_right(CycleLane::kNone);
            break;
        }
      } else if (tag.first == "cycle_lane_left") {
        CycleLane cyclelane_left = (CycleLane)std::stoi(tag.second);
        switch (cyclelane_left) {
          case CycleLane::kDedicated:
            w.set_cyclelane_left(CycleLane::kDedicated);
            break;
          case CycleLane::kSeparated:
            w.set_cyclelane_left(CycleLane::kSeparated);
            break;
          case CycleLane::kShared:
            w.set_cyclelane_left(CycleLane::kShared);
            break;
          case CycleLane::kNone:
          default:
            w.set_cyclelane_left(CycleLane::kNone);
            break;
        }
      }

      else if (tag.first == "cycle_lane_right_opposite") {
        w.set_cyclelane_right_opposite(tag.second == "true" ? true : false);
      } else if (tag.first == "cycle_lane_left_opposite") {
        w.set_cyclelane_left_opposite(tag.second == "true" ? true : false);
      }

      else if (tag.first == "lanes") {
        w.set_lanes(std::stoi(tag.second));
        w.set_tagged_lanes(true);
      } else if (tag.first == "forward_lanes") {
        w.set_forward_lanes(std::stoi(tag.second));
        w.set_forward_tagged_lanes(true);
      } else if (tag.first == "backward_lanes") {
        w.set_backward_lanes(std::stoi(tag.second));
        w.set_backward_tagged_lanes(true);
      }

      else if (tag.first == "tunnel") {
        w.set_tunnel(tag.second == "true" ? true : false);
      } else if (tag.first == "toll") {
        w.set_toll(tag.second == "true" ? true : false);
      } else if (tag.first == "bridge") {
        w.set_bridge(tag.second == "true" ? true : false);
      } else if (tag.first == "seasonal") {
        w.set_seasonal(tag.second == "true" ? true : false);

      } else if (tag.first == "bike_network_mask") {
        w.set_bike_network(std::stoi(tag.second));
      } else if (tag.first == "bike_national_ref" && !tag.second.empty()) {
        w.set_bike_national_ref_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "bike_regional_ref" && !tag.second.empty()) {
        w.set_bike_regional_ref_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "bike_local_ref" && !tag.second.empty()) {
        w.set_bike_local_ref_index(osmdata_.name_offset_map.index(tag.second));

      } else if (tag.first == "destination" && !tag.second.empty()) {
        w.set_destination_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:forward" && !tag.second.empty()) {
        w.set_destination_forward_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:backward" && !tag.second.empty()) {
        w.set_destination_backward_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:ref" && !tag.second.empty()) {
        w.set_destination_ref_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:ref:to" && !tag.second.empty()) {
        w.set_destination_ref_to_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:street" && !tag.second.empty()) {
        w.set_destination_street_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "destination:street:to" && !tag.second.empty()) {
        w.set_destination_street_to_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "junction:ref" && !tag.second.empty()) {
        w.set_junction_ref_index(osmdata_.name_offset_map.index(tag.second));
        w.set_exit(true);
      } else if (tag.first == "turn:lanes" || tag.first == "turn:lanes:forward") {
        // Turn lanes in the forward direction
        w.set_fwd_turn_lanes_index(osmdata_.name_offset_map.index(tag.second));
      } else if (tag.first == "turn:lanes:backward") {
        // Turn lanes in the reverse direction
        w.set_bwd_turn_lanes_index(osmdata_.name_offset_map.index(tag.second));
      }
    }

    // if no surface and tracktype but we have a sac_scale, set surface to path.
    if (!has_surface) {
      if (results.find("sac_scale") != results.end() || results.find("mtb:scale") != results.end() ||
          results.find("mtb:scale:imba") != results.end() ||
          results.find("mtb:scale:uphill") != results.end() ||
          results.find("mtb:description") != results.end()) {
        w.set_surface(Surface::kPath);
      } else {
        // If no surface has been set by a user, assign a surface based on Road Class and Use
        switch (w.road_class()) {

          case RoadClass::kMotorway:
          case RoadClass::kTrunk:
          case RoadClass::kPrimary:
          case RoadClass::kSecondary:
          case RoadClass::kTertiary:
          case RoadClass::kUnclassified:
          case RoadClass::kResidential:
            w.set_surface(Surface::kPavedSmooth);
            break;
          default:
            switch (w.use()) {
              case Use::kFootway:
              case Use::kPedestrian:
              case Use::kSidewalk:
              case Use::kPath:
              case Use::kBridleway:
                w.set_surface(Surface::kCompacted);
                break;
              case Use::kTrack:
                w.set_surface(Surface::kDirt);
                break;
              case Use::kRoad:
              case Use::kParkingAisle:
              case Use::kDriveway:
              case Use::kAlley:
              case Use::kEmergencyAccess:
              case Use::kDriveThru:
              case Use::kLivingStreet:
                w.set_surface(Surface::kPavedSmooth);
                break;
              case Use::kCycleway:
              case Use::kSteps:
                w.set_surface(Surface::kPaved);
                break;
              default:
                // TODO:  see if we can add more logic when a user does not
                // specify a surface.
                w.set_surface(Surface::kPaved);
                break;
            }
            break;
        }
      }
    }

    // set the speed
    if (has_average_speed) {
      w.set_speed(average_speed);
    } else if (has_advisory_speed) {
      w.set_speed(advisory_speed);
    } else if (has_max_speed) {
      w.set_speed(max_speed);
    } else if (has_default_speed && !w.forward_tagged_speed() && !w.backward_tagged_speed()) {
      w.set_speed(default_speed);
    }

    // set the speed limit
    if (has_max_speed) {
      w.set_speed_limit(max_speed);
    }

    // I hope this does not happen, but it probably will (i.e., user sets forward speed
    // and not the backward speed and vice versa.)
    if (w.forward_tagged_speed() && !w.backward_tagged_speed() && !w.oneway()) {
      w.set_backward_speed(w.forward_speed());
      w.set_backward_tagged_speed(true);
    } else if (!w.forward_tagged_speed() && w.backward_tagged_speed() && !w.oneway()) {
      w.set_forward_speed(w.backward_speed());
      w.set_forward_tagged_speed(true);
    }

    // default to drive on right.
    w.set_drive_on_right(true);

    // ferries / auto trains need to be set to highway cut off in config.
    if (w.ferry() || w.rail()) {
      w.set_road_class(highway_cutoff_rc_);
    }

    // Delete the name from from name field if it exists in the ref.
    if (!name.empty() && w.ref_index()) {
      std::vector<std::string> names = GetTagTokens(name);
      std::vector<std::string> refs = GetTagTokens(osmdata_.name_offset_map.name(w.ref_index()));
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
        w.set_name_index(osmdata_.name_offset_map.index(tmp));
      }
    } else {
      w.set_name_index(osmdata_.name_offset_map.index(name));
    }

    // Infer cul-de-sac if a road edge is a loop and is low classification.
    if (!w.roundabout() && loop_nodes_.size() != nodes.size() && w.use() == Use::kRoad &&
        w.road_class() > RoadClass::kTertiary) {
      w.set_use(Use::kCuldesac);
    }

    if (has_user_tags) {
      w.set_has_user_tags(true);
      access_->push_back(access);
    }
    // Add the way to the list
    ways_->push_back(w);
  }

  virtual void relation_callback(const uint64_t osmid,
                                 const OSMPBF::Tags& tags,
                                 const std::vector<OSMPBF::Member>& members) override {
    // Get tags
    Tags results = lua_.Transform(OSMType::kRelation, tags);
    if (results.size() == 0) {
      return;
    }

    // unsorted extracts are just plain nasty, so they can bugger off!
    if (osmid < last_relation_) {
      throw std::runtime_error("Detected unsorted input data");
    }
    last_relation_ = osmid;

    OSMRestriction restriction{};
    OSMRestriction to_restriction{};

    uint32_t from_way_id = 0;
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
                  tag.first == "restriction:emergency") &&
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
              condition = day_start + "-";
              condition += day_end;
              // do we have multiple times entered?
              if (!has_multiple_times) {
                // no we do not...add the hours to the condition
                condition += " " + hour_start + "-";
                condition += hour_end;
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
              // from. also in order to avoid duplicate data in the from and to restrictions, we only
              // need to store the mode, from, and to for the to_restrictions.
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

  // Output list of wayids that have loops
  void output_loops() {
    std::ofstream loop_file;
    loop_file.open("loop_ways.txt", std::ofstream::out | std::ofstream::trunc);
    for (auto& wayid : loops_) {
      loop_file << wayid << std::endl;
    }
    loop_file.close();
    loops_.clear();
    loops_.shrink_to_fit();
  }

  // Configuration option to include driveways
  bool include_driveways_;

  // Road class assignment needs to be set to the highway cutoff for ferries and auto trains.
  RoadClass highway_cutoff_rc_;

  // Lua Tag Transformation class
  LuaTagTransform lua_;

  // Pointer to all the OSM data (for use by callbacks)
  OSMData& osmdata_;

  // Mark the OSM Node Ids used by ways
  // TODO: remove interesection_ as you already know it if you
  // encounter more than one consecutive OSMWayNode with the same id
  IdTable shape_, intersection_;

  // Ways and nodes written to file, nodes are written in the order they appear in way (shape)
  std::unique_ptr<sequence<OSMWay>> ways_;
  std::unique_ptr<sequence<OSMWayNode>> way_nodes_;
  // When updating the references with the node information we keep the last index we looked at
  // this lets us only have to iterate over the whole set once
  size_t current_way_node_index_;
  uint64_t last_node_, last_way_, last_relation_;
  std::unordered_map<uint64_t, size_t> loop_nodes_;

  // List of wayids with loops
  std::vector<uint64_t> loops_;

  // user entered access
  std::unique_ptr<sequence<OSMAccess>> access_;
  // from complex restrictions
  std::unique_ptr<sequence<OSMRestriction>> complex_restrictions_from_;
  //  used to find out if a wayid is the to edge for a complex restriction
  std::unique_ptr<sequence<OSMRestriction>> complex_restrictions_to_;

  // bss nodes
  std::unique_ptr<sequence<OSMNode>> bss_nodes_;
};

} // namespace

namespace valhalla {
namespace mjolnir {

OSMData PBFGraphParser::Parse(const boost::property_tree::ptree& pt,
                              const std::vector<std::string>& input_files,
                              const std::string& ways_file,
                              const std::string& way_nodes_file,
                              const std::string& access_file,
                              const std::string& complex_restriction_from_file,
                              const std::string& complex_restriction_to_file,
                              const std::string& bss_nodes_file) {
  // TODO: option 1: each one threads makes an osmdata and we splice them together at the end
  // option 2: synchronize around adding things to a single osmdata. will have to test to see
  // which is the least expensive (memory and speed). leaning towards option 2
  unsigned int threads =
      std::max(static_cast<unsigned int>(1),
               pt.get<unsigned int>("concurrency", std::thread::hardware_concurrency()));

  // Create OSM data. Set the member pointer so that the parsing callback methods can use it.
  OSMData osmdata{};
  graph_callback callback(pt, osmdata);

  LOG_INFO("Parsing files: " + boost::algorithm::join(input_files, ", "));

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
      callback.reset(nullptr, nullptr, nullptr, nullptr, nullptr,
                     new sequence<OSMNode>(bss_nodes_file, true));
      OSMPBF::Parser::parse(file_handle, static_cast<OSMPBF::Interest>(OSMPBF::Interest::NODES),
                            callback);
    }
  }

  callback.reset(new sequence<OSMWay>(ways_file, true),
                 new sequence<OSMWayNode>(way_nodes_file, true),
                 new sequence<OSMAccess>(access_file, true),
                 new sequence<OSMRestriction>(complex_restriction_from_file, true),
                 new sequence<OSMRestriction>(complex_restriction_to_file, true), nullptr);
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
  callback.output_loops();
  LOG_INFO("Finished with " + std::to_string(osmdata.osm_way_count) + " routable ways containing " +
           std::to_string(osmdata.osm_way_node_count) + " nodes");

  // we need to sort the access tags so that we can easily find them.
  LOG_INFO("Sorting osm access tags by way id...");
  {
    sequence<OSMAccess> access(access_file, false);
    access.sort([](const OSMAccess& a, const OSMAccess& b) { return a.way_id() < b.way_id(); });
  }

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

  // we need to sort the refs so that we can easily (sequentially) update them
  // during node processing, we use memory mapping here because otherwise we aren't
  // using much mem, the scoping makes sure to let it go when done sorting
  LOG_INFO("Sorting osm way node references by node id...");
  {
    sequence<OSMWayNode> way_nodes(way_nodes_file, false);
    way_nodes.sort(
        [](const OSMWayNode& a, const OSMWayNode& b) { return a.node.osmid_ < b.node.osmid_; });
  }
  LOG_INFO("Finished");

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

  // Return OSM data
  return osmdata;
}

} // namespace mjolnir
} // namespace valhalla
