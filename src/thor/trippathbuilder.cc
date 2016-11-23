#include <string>
#include <cstdint>
#include <ostream>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/signinfo.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/encoded.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/proto/trippath.pb.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::odin;

namespace {

template<class iter>
void AddPartialShape(std::vector<PointLL>& shape, iter start, iter end,
                     float partial_length, bool back_insert,
                     const PointLL& last) {
  auto push = [&shape, &back_insert] (const PointLL& point) {
    if(back_insert)
    shape.push_back(point);
    else
    shape.insert(shape.begin(), point);
  };

  //yeah we dont add shape if we dont have any length to add
  if (partial_length > 0.f) {
    //if we are adding on to a shape that already has points we dont want to actually add the first one
    if (!back_insert)
      push(*start);
    //for each segment
    for (; start != end - 1; ++start) {
      //is this segment longer than what we have left, then we found the segment the point lies on
      const auto length = (start + 1)->Distance(*start);
      if (length > partial_length) {
        push(last);
        return;
      }
      //just take the point from this segment
      push(*(start + 1));
      partial_length -= length;
    }
  }
}

void TrimShape(std::vector<PointLL>& shape, const float start,
               const PointLL& start_vertex, const float end,
               const PointLL& end_vertex) {
  //clip up to the start point
  float along = 0.f;
  auto current = shape.begin();
  while (current != shape.end() - 1) {
    along += (current + 1)->Distance(*current);
    //just crossed it
    if (along > start) {
      along = start;
      *current = start_vertex;
      shape.erase(shape.begin(), current);
      break;
    }
    ++current;
  }

  //clip after the end point
  current = shape.begin();
  while (current != shape.end() - 1) {
    along += (current + 1)->Distance(*current);
    //just crossed it
    if (along > end) {
      *(++current) = end_vertex;
      shape.erase(++current, shape.end());
      break;
    }
    ++current;
  }
}

uint32_t GetAdminIndex(
    const AdminInfo& admin_info,
    std::unordered_map<AdminInfo, uint32_t, AdminInfo::AdminInfoHasher>& admin_info_map,
    std::vector<AdminInfo>& admin_info_list) {

  uint32_t admin_index = 0;
  auto existing_admin = admin_info_map.find(admin_info);

  // If admin was not processed yet
  if (existing_admin == admin_info_map.end()) {

    // Assign new admin index
    admin_index = admin_info_list.size();

    // Add admin info to list
    admin_info_list.emplace_back(admin_info);

    // Add admin info/index pair to map
    admin_info_map.emplace(admin_info, admin_index);
  }
  // Use known admin
  else {
    admin_index = existing_admin->second;
  }
  return admin_index;
}

void AssignAdmins(TripPath& trip_path,
                  const std::vector<AdminInfo>& admin_info_list) {
  // Assign the admins
  for (const auto& admin_info : admin_info_list) {
    TripPath_Admin* trip_admin = trip_path.add_admin();
    trip_admin->set_country_code(admin_info.country_iso());
    trip_admin->set_country_text(admin_info.country_text());
    trip_admin->set_state_code(admin_info.state_iso());
    trip_admin->set_state_text(admin_info.state_text());

    valhalla::midgard::logging::Log("admin_state_iso::" + admin_info.state_iso(), " [ANALYTICS] ");
    valhalla::midgard::logging::Log("admin_country_iso::" + admin_info.country_iso(), " [ANALYTICS] ");
  }
 }
}

namespace valhalla {
namespace thor {

namespace {
TripPath_RoadClass GetTripPathRoadClass(RoadClass road_class) {
  switch (road_class) {
    case RoadClass::kMotorway:
      return TripPath_RoadClass_kMotorway;
    case RoadClass::kTrunk:
      return TripPath_RoadClass_kTrunk;
    case RoadClass::kPrimary:
      return TripPath_RoadClass_kPrimary;
    case RoadClass::kSecondary:
      return TripPath_RoadClass_kSecondary;
    case RoadClass::kTertiary:
      return TripPath_RoadClass_kTertiary;
    case RoadClass::kUnclassified:
      return TripPath_RoadClass_kUnclassified;
    case RoadClass::kResidential:
      return TripPath_RoadClass_kResidential;
    case RoadClass::kServiceOther:
      return TripPath_RoadClass_kServiceOther;
  }
}

TripPath_Use GetTripPathUse(Use use) {
  switch (use) {
    case Use::kRoad:
      return TripPath_Use_kRoadUse;
    case Use::kRamp:
      return TripPath_Use_kRampUse;
    case Use::kTurnChannel:
      return TripPath_Use_kTurnChannelUse;
    case Use::kTrack:
      return TripPath_Use_kTrackUse;
    case Use::kDriveway:
      return TripPath_Use_kDrivewayUse;
    case Use::kAlley:
      return TripPath_Use_kAlleyUse;
    case Use::kParkingAisle:
      return TripPath_Use_kParkingAisleUse;
    case Use::kEmergencyAccess:
      return TripPath_Use_kEmergencyAccessUse;
    case Use::kDriveThru:
      return TripPath_Use_kDriveThruUse;
    case Use::kCuldesac:
      return TripPath_Use_kCuldesacUse;
    case Use::kCycleway:
      return TripPath_Use_kCyclewayUse;
    case Use::kMountainBike:
      return TripPath_Use_kMountainBikeUse;
    case Use::kSidewalk:
      //return TripPath_Use_kSidewalkUse;
      return TripPath_Use_kFootwayUse;   // TODO: update when odin has been updated
    case Use::kFootway:
      return TripPath_Use_kFootwayUse;
    case Use::kSteps:
      return TripPath_Use_kStepsUse;
    case Use::kPath:
      return TripPath_Use_kPathUse;
    case Use::kPedestrian:
      return TripPath_Use_kPedestrianUse;
    case Use::kBridleway:
      return TripPath_Use_kBridlewayUse;
    case Use::kOther:
      return TripPath_Use_kOtherUse;
    case Use::kFerry:
      return TripPath_Use_kFerryUse;
    case Use::kRailFerry:
      return TripPath_Use_kRailFerryUse;
    case Use::kRail:
      return TripPath_Use_kRailUse;
    case Use::kBus:
      return TripPath_Use_kBusUse;
    case Use::kRailConnection:
      return TripPath_Use_kRailConnectionUse;
    case Use::kBusConnection:
      return TripPath_Use_kBusConnectionUse;
    case Use::kTransitConnection:
      return TripPath_Use_kTransitConnectionUse;
  }
}

TripPath_VehicleType GetTripPathVehicleType(uint8_t type) {
  switch (type) {
    case static_cast<uint8_t>(VehicleType::kCar):
      return TripPath_VehicleType::TripPath_VehicleType_kCar;
    case static_cast<uint8_t>(VehicleType::kMotorcycle):
      return TripPath_VehicleType::TripPath_VehicleType_kMotorcycle;
    case static_cast<uint8_t>(VehicleType::kBus):
      return TripPath_VehicleType::TripPath_VehicleType_kAutoBus;
    case static_cast<uint8_t>(VehicleType::kTractorTrailer):
      return TripPath_VehicleType::TripPath_VehicleType_kTractorTrailer;
  }
}

TripPath_PedestrianType GetTripPathPedestrianType(uint8_t type) {
  switch (type) {
    case static_cast<uint8_t>(PedestrianType::kFoot):
      return TripPath_PedestrianType::TripPath_PedestrianType_kFoot;
    case static_cast<uint8_t>(PedestrianType::kWheelchair):
      return TripPath_PedestrianType::TripPath_PedestrianType_kWheelchair;
    case static_cast<uint8_t>(PedestrianType::kSegway):
      return TripPath_PedestrianType::TripPath_PedestrianType_kSegway;
  }
}

TripPath_BicycleType GetTripPathBicycleType(uint8_t type) {
  switch (type) {
    case static_cast<uint8_t>(BicycleType::kRoad):
      return TripPath_BicycleType::TripPath_BicycleType_kRoad;
    case static_cast<uint8_t>(BicycleType::kCross):
      return TripPath_BicycleType::TripPath_BicycleType_kCross;
    case static_cast<uint8_t>(BicycleType::kHybrid):
      return TripPath_BicycleType::TripPath_BicycleType_kHybrid;
    case static_cast<uint8_t>(BicycleType::kMountain):
      return TripPath_BicycleType::TripPath_BicycleType_kMountain;
  }
}

TripPath_TransitType GetTripPathTransitType(TransitType transit_type) {
  switch (transit_type) {
    case TransitType::kTram:        // Tram, streetcar, lightrail
      return TripPath_TransitType::TripPath_TransitType_kTram;
    case TransitType::kMetro:      // Subway, metro
      return TripPath_TransitType::TripPath_TransitType_kMetro;
    case TransitType::kRail:        // Rail
      return TripPath_TransitType::TripPath_TransitType_kRail;
    case TransitType::kBus:         // Bus
      return TripPath_TransitType::TripPath_TransitType_kBus;
    case TransitType::kFerry:       // Ferry
      return TripPath_TransitType::TripPath_TransitType_kFerry;
    case TransitType::kCableCar:    // Cable car
      return TripPath_TransitType::TripPath_TransitType_kCableCar;
    case TransitType::kGondola:     // Gondola (suspended cable car)
      return TripPath_TransitType::TripPath_TransitType_kGondola;
    case TransitType::kFunicular:   // Funicular (steep incline)
      return TripPath_TransitType::TripPath_TransitType_kFunicular;
  }
}

TripPath_Traversability GetTripPathTraversability(Traversability traversability) {
  switch (traversability) {
    case Traversability::kNone:
      return TripPath_Traversability_kNone;
    case Traversability::kForward:
      return TripPath_Traversability_kForward;
    case Traversability::kBackward:
      return TripPath_Traversability_kBackward;
    case Traversability::kBoth:
      return TripPath_Traversability_kBoth;
  }
}

TripPath_Location_SideOfStreet GetTripPathSideOfStreet(
    PathLocation::SideOfStreet sos) {
  switch (sos) {
    case PathLocation::SideOfStreet::NONE:
      return TripPath_Location_SideOfStreet_kNone;
    case PathLocation::SideOfStreet::LEFT:
      return TripPath_Location_SideOfStreet_kLeft;
    case PathLocation::SideOfStreet::RIGHT:
      return TripPath_Location_SideOfStreet_kRight;
  }
}

TripPath_Node_Type GetTripPathNodeType(NodeType node_type) {
  switch (node_type) {
    case NodeType::kStreetIntersection:
      return TripPath_Node_Type_kStreetIntersection;
    case NodeType::kGate:
      return TripPath_Node_Type_kGate;
    case NodeType::kBollard:
      return TripPath_Node_Type_kBollard;
    case NodeType::kTollBooth:
      return TripPath_Node_Type_kTollBooth;
    case NodeType::kMultiUseTransitStop:
      return TripPath_Node_Type_kMultiUseTransitStop;
    case NodeType::kBikeShare:
      return TripPath_Node_Type_kBikeShare;
    case NodeType::kParking:
      return TripPath_Node_Type_kParking;
    case NodeType::kMotorWayJunction:
      return TripPath_Node_Type_kMotorwayJunction;
    case NodeType::kBorderControl:
      return TripPath_Node_Type_kBorderControl;
  }
}

}

// Default constructor
TripPathBuilder::TripPathBuilder() {
}

// Destructor
TripPathBuilder::~TripPathBuilder() {
}

// For now just find the length of the path!
// TODO - probably need the location information passed in - to
// add to the TripPath
TripPath TripPathBuilder::Build(GraphReader& graphreader,
                                const std::shared_ptr<sif::DynamicCost>* mode_costing,
                                const std::vector<PathInfo>& path,
                                PathLocation& origin,
                                PathLocation& dest,
                                const std::vector<PathLocation>& through_loc) {
  // TripPath is a protocol buffer that contains information about the trip
  TripPath trip_path;

  // Get the local tile level
  uint32_t local_level = graphreader.GetTileHierarchy().levels().rbegin()->first;

  // Set origin (assumed to be a break)
  TripPath_Location* tp_orig = trip_path.add_location();
  TripPath_LatLng* orig_ll = tp_orig->mutable_ll();
  orig_ll->set_lat(origin.latlng_.lat());
  orig_ll->set_lng(origin.latlng_.lng());
  tp_orig->set_type(TripPath_Location_Type_kBreak);
  if (!origin.name_.empty())
    tp_orig->set_name(origin.name_);
  if (!origin.street_.empty())
    tp_orig->set_street(origin.street_);
  if (!origin.city_.empty())
    tp_orig->set_city(origin.city_);
  if (!origin.state_.empty())
    tp_orig->set_state(origin.state_);
  if (!origin.zip_.empty())
    tp_orig->set_postal_code(origin.zip_);
  if (!origin.country_.empty())
    tp_orig->set_country(origin.country_);
  if (origin.heading_)
    tp_orig->set_heading(*origin.heading_);

  // Add list of through locations
  for (auto through : through_loc) {
    TripPath_Location* tp_through = trip_path.add_location();
    TripPath_LatLng* through_ll = tp_through->mutable_ll();
    through_ll->set_lat(through.latlng_.lat());
    through_ll->set_lng(through.latlng_.lng());
    tp_through->set_type(TripPath_Location_Type_kThrough);
    if (!through.name_.empty())
      tp_through->set_name(through.name_);
    if (!through.street_.empty())
      tp_through->set_street(through.street_);
    if (!through.city_.empty())
      tp_through->set_city(through.city_);
    if (!through.state_.empty())
      tp_through->set_state(through.state_);
    if (!through.zip_.empty())
      tp_through->set_postal_code(through.zip_);
    if (!through.country_.empty())
      tp_through->set_country(through.country_);
    if (through.heading_)
      tp_through->set_heading(*through.heading_);
    if (through.date_time_)
      tp_through->set_date_time(*through.date_time_);
  }

  // Set destination (assumed to be a break)
  TripPath_Location* tp_dest = trip_path.add_location();
  TripPath_LatLng* dest_ll = tp_dest->mutable_ll();
  dest_ll->set_lat(dest.latlng_.lat());
  dest_ll->set_lng(dest.latlng_.lng());
  tp_dest->set_type(TripPath_Location_Type_kBreak);
  if (!dest.name_.empty())
    tp_dest->set_name(dest.name_);
  if (!dest.street_.empty())
    tp_dest->set_street(dest.street_);
  if (!dest.city_.empty())
    tp_dest->set_city(dest.city_);
  if (!dest.state_.empty())
    tp_dest->set_state(dest.state_);
  if (!dest.zip_.empty())
    tp_dest->set_postal_code(dest.zip_);
  if (!dest.country_.empty())
    tp_dest->set_country(dest.country_);
  if (dest.heading_)
    tp_dest->set_heading(*dest.heading_);

  uint32_t origin_sec_from_mid = 0;
  if (origin.date_time_)
    origin_sec_from_mid = DateTime::seconds_from_midnight(*origin.date_time_);

  // Create an array of travel types per mode
  uint8_t travel_types[4];
  for (uint32_t i = 0; i < 4; i++) {
    travel_types[i] = (mode_costing[i] != nullptr)  ?
          mode_costing[i]->travel_type() : 0;
  }

  // Get the first nodes graph id by using the end node of the first edge to get the tile with the opposing edge
  // then use the opposing index to get the opposing edge, and its end node is the begin node of the original edge
  auto* first_edge =
      graphreader.GetGraphTile(path.front().edgeid)->directededge(
          path.front().edgeid);
  auto* first_tile = graphreader.GetGraphTile(first_edge->endnode());
  auto* first_node = first_tile->node(first_edge->endnode());
  GraphId startnode = first_tile->directededge(
      first_node->edge_index() + first_edge->opp_index())->endnode();

  // Partial edge at the start and side of street (sos)
  float start_pct;
  PathLocation::SideOfStreet start_sos;
  PointLL start_vrt;
  for(const auto& e : origin.edges) {
    if (e.id == path.front().edgeid) {
      start_pct = e.dist;
      start_sos = e.sos;
      start_vrt = e.projected;
      break;
    }
  }
  // Set the origin side of street, if one exists
  if (start_sos != PathLocation::SideOfStreet::NONE)
    tp_orig->set_side_of_street(GetTripPathSideOfStreet(start_sos));

  // Partial edge at the end
  float end_pct;
  PathLocation::SideOfStreet end_sos;
  PointLL end_vrt;
  for(const auto&e : dest.edges) {
    if (e.id == path.back().edgeid) {
      end_pct = e.dist;
      end_sos = e.sos;
      end_vrt = e.projected;
      break;
    }
  }

  // Set the destination side of street, if one exists
  if (end_sos != PathLocation::SideOfStreet::NONE)
    tp_dest->set_side_of_street(GetTripPathSideOfStreet(end_sos));

  // Structures to process admins
  std::unordered_map<AdminInfo, uint32_t, AdminInfo::AdminInfoHasher> admin_info_map;
  std::vector<AdminInfo> admin_info_list;
  uint32_t last_node_admin_index;

  // If the path was only one edge we have a special case
  if (path.size() == 1) {
    const GraphTile* tile = graphreader.GetGraphTile(path.front().edgeid);
    const DirectedEdge* edge = tile->directededge(path.front().edgeid);

    // Get the shape. Reverse if the directed edge direction does
    // not match the traversal direction (based on start and end percent).
    auto shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
    if (edge->forward() != (start_pct < end_pct)) {
      std::reverse(shape.begin(), shape.end());
    }

    // If traversing the opposing direction: adjust start and end percent
    // and reverse the edge and side of street if traversing the opposite
    // direction
    if (start_pct > end_pct) {
      start_pct = 1.0f - start_pct;
      end_pct   = 1.0f - end_pct;
      edge = graphreader.GetOpposingEdge(path.front().edgeid, tile);
      if (end_sos == PathLocation::SideOfStreet::LEFT) {
        tp_dest->set_side_of_street(GetTripPathSideOfStreet(PathLocation::SideOfStreet::RIGHT));
      } else if (end_sos == PathLocation::SideOfStreet::RIGHT) {
        tp_dest->set_side_of_street(GetTripPathSideOfStreet(PathLocation::SideOfStreet::LEFT));
      }
    }

    float total = static_cast<float>(edge->length());
    TrimShape(shape, start_pct * total, start_vrt, end_pct * total, end_vrt);

    // Add trip edge
    auto trip_edge = AddTripEdge(path.front().edgeid, path.front().trip_id,
                       0, path.front().mode,
                       travel_types[static_cast<int>(path.front().mode)],
                       edge, trip_path.add_node(), tile,
                       std::abs(end_pct - start_pct));
    trip_edge->set_begin_shape_index(0);
    trip_edge->set_end_shape_index(shape.size()-1);
    auto* node = trip_path.add_node();
    node->set_elapsed_time(path.front().elapsed_time);

    const GraphTile* end_tile = graphreader.GetGraphTile(edge->endnode());
    if (end_tile == nullptr)
      node->set_admin_index(0);
    else {
      node->set_admin_index(
          GetAdminIndex(
              end_tile->admininfo(end_tile->node(edge->endnode())->admin_index()),
                admin_info_map, admin_info_list));
    }

    uint32_t elapsedtime = node->elapsed_time();

    auto* last_tile = graphreader.GetGraphTile(startnode);
    if (dest.date_time_) { // arrive by

      uint64_t sec = DateTime::seconds_since_epoch(*dest.date_time_,
                                                   DateTime::get_tz_db().
                                                   from_index(last_tile->node(startnode)->timezone()));

      std::string origin_date, dest_date;
      DateTime::seconds_to_date(false, sec - elapsedtime, sec, DateTime::get_tz_db().
                                from_index(first_node->timezone()),
                                DateTime::get_tz_db().
                                from_index(last_tile->node(startnode)->timezone()),
                                origin_date, dest_date);

      tp_orig->set_date_time(origin_date);
      origin.date_time_ = tp_orig->date_time();
      tp_dest->set_date_time(dest_date);

    } else if (origin.date_time_) { // leave at
      uint64_t sec = DateTime::seconds_since_epoch(*origin.date_time_,
                                                   DateTime::get_tz_db().
                                                   from_index(first_node->timezone()));

      std::string origin_date, dest_date;
      DateTime::seconds_to_date(true, sec, sec + elapsedtime, DateTime::get_tz_db().
                                from_index(first_node->timezone()),
                                DateTime::get_tz_db().
                                from_index(last_tile->node(startnode)->timezone()),
                                origin_date, dest_date);

      tp_dest->set_date_time(dest_date);
      dest.date_time_ = tp_dest->date_time();
      tp_orig->set_date_time(origin_date);
    }

    trip_path.set_shape(encode<std::vector<PointLL> >(shape));
    // Assign the trip path admins
    AssignAdmins(trip_path, admin_info_list);
    return trip_path;
  }

  // Iterate through path
  uint32_t elapsedtime = 0;
  uint32_t block_id = 0;
  uint32_t prior_opp_local_index = -1;
  std::vector<PointLL> trip_shape;
  std::string arrival_time;
  bool assumed_schedule = false;
  // TODO: this is temp until we use transit stop type from transitland
  TripPath_TransitStopInfo_Type prev_transit_node_type =
      TripPath_TransitStopInfo_Type_kStop;
  for (auto edge_itr = path.begin(); edge_itr != path.end(); ++edge_itr) {
    const GraphId& edge = edge_itr->edgeid;
    const uint32_t trip_id = edge_itr->trip_id;
    const GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);
    const sif::TravelMode mode = edge_itr->mode;
    const uint8_t travel_type = travel_types[static_cast<uint32_t>(mode)];

    // Skip transition edges - these are optional in the path. So we need
    // to make sure we get the node info from the correct tile
    if (directededge->trans_up() || directededge->trans_down()) {
      continue;
    }

    // Add a node to the trip path and set its attributes.
    TripPath_Node* trip_node = trip_path.add_node();

    // Set node attributes - only set if they are true since they are optional
    const GraphTile* start_tile = graphreader.GetGraphTile(startnode);
    const NodeInfo* node = start_tile->node(startnode);
    trip_node->set_type(GetTripPathNodeType(node->type()));

    if (node->intersection() == IntersectionType::kFork)
      trip_node->set_fork(true);

    // Assign the elapsed time from the start of the leg
    trip_node->set_elapsed_time(elapsedtime);

    // Assign the admin index
    trip_node->set_admin_index(GetAdminIndex(
            start_tile->admininfo(node->admin_index()),
            admin_info_map, admin_info_list));

    ///////////////////////////////////////////////////////////////////////////
    // Add transit information if this is a transit stop. TODO - can we move
    // this to another method?
    if (node->is_transit()) {
      // Get the transit stop information and add transit stop info
      const TransitStop* transit_stop = start_tile->GetTransitStop(
          start_tile->node(startnode)->stop_index());
      TripPath_TransitStopInfo* transit_stop_info = trip_node
          ->mutable_transit_stop_info();

      // TODO: for now we will set to station for rail and stop for others
      //       in future, we will set based on transitland value
      // Set type
      if (directededge->use() == Use::kRail) {
        transit_stop_info->set_type(TripPath_TransitStopInfo_Type_kStation);
        prev_transit_node_type = TripPath_TransitStopInfo_Type_kStation;
      } else if (directededge->use() == Use::kTransitConnection) {
        transit_stop_info->set_type(prev_transit_node_type);
      } else {
        transit_stop_info->set_type(TripPath_TransitStopInfo_Type_kStop);
        prev_transit_node_type = TripPath_TransitStopInfo_Type_kStop;
      }

      if (transit_stop) {
        // Set onstop_id
        if (transit_stop->one_stop_offset())
          transit_stop_info->set_onestop_id(graphtile->GetName(transit_stop->one_stop_offset()));

        // Set name
        if (transit_stop->name_offset())
          transit_stop_info->set_name(graphtile->GetName(transit_stop->name_offset()));

        // Set latitude and longitude
        TripPath_LatLng* stop_ll = transit_stop_info->mutable_ll();
        stop_ll->set_lat(node->latlng().lat());
        stop_ll->set_lng(node->latlng().lng());
      }

      // Set the arrival time at this node (based on schedule from last trip
      // departure)
      if (!arrival_time.empty()) {
        transit_stop_info->set_arrival_date_time(arrival_time);
      }

      // If this edge has a trip id then there is a transit departure
      if (trip_id) {
        const TransitDeparture* transit_departure = graphtile
            ->GetTransitDeparture(graphtile->directededge(edge.id())->lineid(),
                                  trip_id);

        assumed_schedule = false;
        uint32_t date, day = 0;
        if (origin.date_time_) {
          date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(*origin.date_time_));

          if (graphtile->header()->date_created() > date) {
            transit_stop_info->set_assumed_schedule(true);
            assumed_schedule = true;
          } else {
            day = date - graphtile->header()->date_created();
            if (day > graphtile->GetTransitSchedule(transit_departure->schedule_index())->end_day()) {
              transit_stop_info->set_assumed_schedule(true);
              assumed_schedule = true;
            }
          }
        }

        if (transit_departure) {

          std::string dt = DateTime::get_duration(*origin.date_time_,
                           (transit_departure->departure_time() - origin_sec_from_mid),
                           DateTime::get_tz_db().from_index(node->timezone()));

          std::size_t found = dt.find_last_of(" "); // remove tz abbrev.
          if (found != std::string::npos)
            dt = dt.substr(0,found);

          // Set departure time from this transit stop
          transit_stop_info->set_departure_date_time(dt);

          //TODO:  set removed tz abbrev on transit_stop_info for departure.

          // Copy the arrival time for use at the next transit stop
          arrival_time = DateTime::get_duration(*origin.date_time_,
                                     (transit_departure->departure_time() +
                                     transit_departure->elapsed_time()) -
                                     origin_sec_from_mid,
                                     DateTime::get_tz_db().from_index(node->timezone()));

          found = arrival_time.find_last_of(" "); //remove tz abbrev.
          if (found != std::string::npos)
            arrival_time = arrival_time.substr(0,found);

          //TODO:  set removed tz abbrev on transit_stop_info for arrival.

          // Get the block Id
          block_id = transit_departure->blockid();
        }
      } else {
        // No departing trip, set the arrival time (for next stop) to empty
        // and set block Id to 0
        arrival_time = "";
        block_id = 0;

        if (assumed_schedule)
          transit_stop_info->set_assumed_schedule(true);
        assumed_schedule = false;
      }

      // Set is_parent_stop. TODO - update with station hierarchy
      transit_stop_info->set_is_parent_stop(false);
    }

    // Add edge to the trip node and set its attributes
    auto is_first_edge = edge_itr == path.begin();
    auto is_last_edge = edge_itr == path.end() - 1;
    float length_pct = (
        is_first_edge ? 1.f - start_pct : (is_last_edge ? end_pct : 1.f));
    TripPath_Edge* trip_edge = AddTripEdge(edge, trip_id, block_id, mode,
                                           travel_type, directededge, trip_node,
                                           graphtile, length_pct);

    // Get the shape and set shape indexes (directed edge forward flag
    // determines whether shape is traversed forward or reverse).
    auto edgeinfo = graphtile->edgeinfo(directededge->edgeinfo_offset());
    if (is_first_edge) {
      trip_edge->set_begin_shape_index(0);
    } else {
      trip_edge->set_begin_shape_index(trip_shape.size() - 1);
    }

    // We need to clip the shape if its at the beginning or end and isnt a full length
    if (is_first_edge || is_last_edge) {
      float length = static_cast<float>(directededge->length()) * length_pct;
      if (directededge->forward() == is_last_edge) {
        AddPartialShape<std::vector<PointLL>::const_iterator>(
            trip_shape, edgeinfo.shape().begin(), edgeinfo.shape().end(),
            length, is_last_edge, is_last_edge ? end_vrt : start_vrt);
      } else {
        AddPartialShape<std::vector<PointLL>::const_reverse_iterator>(
            trip_shape, edgeinfo.shape().rbegin(), edgeinfo.shape().rend(),
            length, is_last_edge, is_last_edge ? end_vrt : start_vrt);
      }
    }    // Just get the shape in there in the right direction
    else {
      if (directededge->forward())
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().begin() + 1,
                          edgeinfo.shape().end());
      else
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().rbegin() + 1,
                          edgeinfo.shape().rend());
    }
    trip_edge->set_end_shape_index(trip_shape.size() - 1);

    // Add connected edges from the start node. Do this after the first trip
    // edge is added
    //
    //Our path is from 1 to 2 to 3 (nodes) to ... n nodes.
    //Each letter represents the edge info.
    //So at node 2, we will store the edge info for D and we will store the
    //intersecting edge info for B, C, E, F, and G.  We need to make sure
    //that we don't store the edge info from A and D again.  Also, do not store transition edges.
    //
    //     (X)    (3)   (X)
    //       \\   ||   //
    //      C \\ D|| E//
    //         \\ || //
    //      B   \\||//   F
    // (X)======= (2) ======(X)
    //            ||\\
    //          A || \\ G
    //            ||  \\
    //            (1)  (X)
    if (startnode.Is_Valid()) {
      // Get the graph tile and the first edge from the node
      const GraphTile* tile = graphreader.GetGraphTile(startnode);
      const NodeInfo* nodeinfo = tile->node(startnode);
      if (mode == TravelMode::kPedestrian || mode == TravelMode::kBicycle) {
        // Iterate through edges on this level to find any intersecting edges
        // Follow any upwards or downward transitions
        const DirectedEdge* de = tile->directededge(nodeinfo->edge_index());
        for (uint32_t idx1 = 0; idx1 < nodeinfo->edge_count();
                            ++idx1, de++) {
          // Skip shortcut edges and edges on the path
          if (de->is_shortcut() ||
              de->localedgeidx() == prior_opp_local_index ||
              de->localedgeidx() == directededge->localedgeidx()) {
            continue;
          }

          // If transition edge - get directed edges at the next level
          if (de->trans_up() || de->trans_down()) {
            // Get the end node tile and its directed edges
            GraphId endnode = de->endnode();
            const GraphTile* endtile = graphreader.GetGraphTile(endnode);
            const NodeInfo* nodeinfo2 = endtile->node(endnode);
            const DirectedEdge* de2 = endtile->directededge(nodeinfo2->edge_index());
            for (uint32_t idx2 = 0; idx2 < nodeinfo2->edge_count();
                           ++idx2, de2++) {
              // Skip shortcut edges, edges on the path and transition edges
              if (de2->is_shortcut() ||
                  de2->localedgeidx() == prior_opp_local_index ||
                  de2->localedgeidx() == directededge->localedgeidx() ||
                  de2->trans_up() || de2->trans_down()) {
                continue;
              }
              AddTripIntersectingEdge(de->localedgeidx(), prior_opp_local_index,
                      directededge->localedgeidx(), nodeinfo2,
                      trip_node, de2);
            }
          } else {
            // On the same hierarchy level and not on the path
            AddTripIntersectingEdge(de->localedgeidx(), prior_opp_local_index,
                                directededge->localedgeidx(), nodeinfo,
                                trip_node, de);
          }
        }
      } else {
        // Driving routes - do not need intersecting edges since the node info
        // contains driveabilty at all regular edges.
        for (uint32_t edge_idx = 0; edge_idx < nodeinfo->local_edge_count();
                    ++edge_idx) {
          // Add intersecting edge if this edge is not on the path
          if ((edge_idx != prior_opp_local_index) &&
              (edge_idx != directededge->localedgeidx())) {
            AddTripIntersectingEdge(edge_idx, prior_opp_local_index,
                                  directededge->localedgeidx(), nodeinfo,
                                  trip_node, nullptr);
          }
        }
      }
    }

    // Update elapsed time at the end of the edge, store this at the next node.
    elapsedtime = edge_itr->elapsed_time;

    // Set the endnode of this directed edge as the startnode of the next edge.
    startnode = directededge->endnode();

    // Save the index of the opposing local directed edge at the end node
    prior_opp_local_index = directededge->opp_local_idx();
  }

  auto* last_tile = graphreader.GetGraphTile(startnode);
  if (dest.date_time_) { // arrive by

    uint64_t sec = DateTime::seconds_since_epoch(*dest.date_time_,
                                                 DateTime::get_tz_db().
                                                 from_index(last_tile->node(startnode)->timezone()));

    std::string origin_date, dest_date;
    DateTime::seconds_to_date(false, sec - elapsedtime, sec, DateTime::get_tz_db().
                              from_index(first_node->timezone()),
                              DateTime::get_tz_db().
                              from_index(last_tile->node(startnode)->timezone()),
                              origin_date, dest_date);

    tp_orig->set_date_time(origin_date);
    origin.date_time_ = tp_orig->date_time();
    tp_dest->set_date_time(dest_date);

  } else if (origin.date_time_) { // leave at
    uint64_t sec = DateTime::seconds_since_epoch(*origin.date_time_,
                                                 DateTime::get_tz_db().
                                                 from_index(first_node->timezone()));

    std::string origin_date, dest_date;
    DateTime::seconds_to_date(true, sec, sec + elapsedtime, DateTime::get_tz_db().
                              from_index(first_node->timezone()),
                              DateTime::get_tz_db().
                              from_index(last_tile->node(startnode)->timezone()),
                              origin_date, dest_date);

    tp_dest->set_date_time(dest_date);
    dest.date_time_ = tp_dest->date_time();
    tp_orig->set_date_time(origin_date);
  }

  // Add the last node
  auto* node = trip_path.add_node();
  node->set_admin_index(GetAdminIndex(
          last_tile->admininfo(last_tile->node(startnode)->admin_index()),
          admin_info_map, admin_info_list));
  node->set_elapsed_time(elapsedtime);

  // Assign the admins
  AssignAdmins(trip_path, admin_info_list);

  // Set the bounding box of the shape
  AABB2<PointLL> bbox(trip_shape);
  TripPath_LatLng* min_ll = trip_path.mutable_bbox()->mutable_min_ll();
  min_ll->set_lat(bbox.miny());
  min_ll->set_lng(bbox.minx());
  TripPath_LatLng* max_ll = trip_path.mutable_bbox()->mutable_max_ll();
  max_ll->set_lat(bbox.maxy());
  max_ll->set_lng(bbox.maxx());

  // Encode shape and add to trip path.
  std::string encoded_shape_ = encode<std::vector<PointLL> >(trip_shape);
  trip_path.set_shape(encoded_shape_);

  //hand it back
  return trip_path;
}

// Add a trip edge to the trip node and set its attributes
TripPath_Edge* TripPathBuilder::AddTripEdge(const GraphId& edge,
                                            const uint32_t trip_id,
                                            const uint32_t block_id,
                                            const sif::TravelMode mode,
                                            const uint8_t travel_type,
                                            const DirectedEdge* directededge,
                                            TripPath_Node* trip_node,
                                            const GraphTile* graphtile,
                                            const float length_percentage) {

  // Index of the directed edge within the tile
  uint32_t idx = edge.id();

  TripPath_Edge* trip_edge = trip_node->mutable_edge();

  // Get the edgeinfo and list of names - add to the trip edge.
  auto edgeinfo = graphtile->edgeinfo(directededge->edgeinfo_offset());
  std::vector<std::string> names = edgeinfo.GetNames();
  for (const auto& name : names) {
    trip_edge->add_name(name);
  }

#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE(std::string("wayid=") + std::to_string(edgeinfo.wayid()));
#endif

  // Set the exits (if the directed edge has exit sign information)
  if (directededge->exitsign()) {
    std::vector<SignInfo> signs = graphtile->GetSigns(idx);
    if (!signs.empty()) {
      TripPath_Sign* trip_exit = trip_edge->mutable_sign();
      for (const auto& sign : signs) {
        switch (sign.type()) {
          case Sign::Type::kExitNumber: {
            trip_exit->add_exit_number(sign.text());
            break;
          }
          case Sign::Type::kExitBranch: {
            trip_exit->add_exit_branch(sign.text());
            break;
          }
          case Sign::Type::kExitToward: {
            trip_exit->add_exit_toward(sign.text());
            break;
          }
          case Sign::Type::kExitName: {
            trip_exit->add_exit_name(sign.text());
            break;
          }
        }
      }
    }
  }

  // Set road class
  trip_edge->set_road_class(
      GetTripPathRoadClass(directededge->classification()));

  // Set speed and length
  trip_edge->set_length(directededge->length() * 0.001f * length_percentage);  // Convert to km
  trip_edge->set_speed(directededge->speed());

  uint8_t kAccess = 0;
  if (mode == sif::TravelMode::kBicycle)
    kAccess = kBicycleAccess;
  else if (mode == sif::TravelMode::kDrive)
    kAccess = kAutoAccess;
  else if (mode == sif::TravelMode::kPedestrian || mode == sif::TravelMode::kPublicTransit)
    kAccess = kPedestrianAccess;

  // Test whether edge is traversed forward or reverse and set driveability and heading
  if (directededge->forward()) {
    if ((directededge->forwardaccess() & kAccess)
        && (directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kBoth);
    else if ((directededge->forwardaccess() & kAccess)
        && !(directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kForward);
    else if (!(directededge->forwardaccess() & kAccess)
        && (directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kBackward);
    else
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kNone);

    trip_edge->set_begin_heading(
        std::round(
            PointLL::HeadingAlongPolyline(
                edgeinfo.shape(),
                GetOffsetForHeading(directededge->classification(),
                                    directededge->use()))));
    trip_edge->set_end_heading(
        std::round(
            PointLL::HeadingAtEndOfPolyline(
                edgeinfo.shape(),
                GetOffsetForHeading(directededge->classification(),
                                    directededge->use()))));
  } else {
    // Reverse driveability and heading
    if ((directededge->forwardaccess() & kAccess)
        && (directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kBoth);
    else if (!(directededge->forwardaccess() & kAccess)
        && (directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kForward);
    else if ((directededge->forwardaccess() & kAccess)
        && !(directededge->reverseaccess() & kAccess))
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kBackward);
    else
      trip_edge->set_traversability(
          TripPath_Traversability::TripPath_Traversability_kNone);

    trip_edge->set_begin_heading(
        std::round(
            fmod(
                (PointLL::HeadingAtEndOfPolyline(
                    edgeinfo.shape(),
                    GetOffsetForHeading(directededge->classification(),
                                        directededge->use())) + 180.0f),
                360)));

    trip_edge->set_end_heading(
        std::round(
            fmod(
                (PointLL::HeadingAlongPolyline(
                    edgeinfo.shape(),
                    GetOffsetForHeading(directededge->classification(),
                                        directededge->use())) + 180.0f),
                360)));
  }

  // Set the trip path use based on directed edge use
  trip_edge->set_use(GetTripPathUse(directededge->use()));

  if (directededge->toll())
    trip_edge->set_toll(true);

  if (directededge->unpaved())
    trip_edge->set_unpaved(true);

  if (directededge->tunnel())
    trip_edge->set_tunnel(true);

  if (directededge->bridge())
    trip_edge->set_bridge(true);

  if (directededge->roundabout())
    trip_edge->set_roundabout(true);

  if (directededge->internal())
    trip_edge->set_internal_intersection(true);

  // Set drive_on_right the right way
  trip_edge->set_drive_on_right(directededge->drive_on_right());

  // Set the mode and travel type
  if (mode == sif::TravelMode::kBicycle) {
    trip_edge->set_travel_mode(TripPath_TravelMode::TripPath_TravelMode_kBicycle);
    trip_edge->set_bicycle_type(GetTripPathBicycleType(travel_type));
  } else if (mode == sif::TravelMode::kDrive) {
    trip_edge->set_travel_mode(TripPath_TravelMode::TripPath_TravelMode_kDrive);
    trip_edge->set_vehicle_type(GetTripPathVehicleType(travel_type));
  } else if (mode == sif::TravelMode::kPedestrian) {
    trip_edge->set_travel_mode(TripPath_TravelMode::TripPath_TravelMode_kPedestrian);
    trip_edge->set_pedestrian_type(GetTripPathPedestrianType(travel_type));
  } else if (mode == sif::TravelMode::kPublicTransit) {
    trip_edge->set_travel_mode(TripPath_TravelMode::TripPath_TravelMode_kTransit);
  }

  // Set edge id (graphid value)
  trip_edge->set_id(edge.value);

  // Set way id (base data id)
  trip_edge->set_way_id(edgeinfo.wayid());

  // Set weighted grade
  trip_edge->set_weighted_grade((directededge->weighted_grade() - 6.f) / 0.6f);

  // Set maximum upward grade
  trip_edge->set_max_upward_grade(directededge->max_up_slope());

  // Set maximum downward grade
  trip_edge->set_max_downward_grade(directededge->max_down_slope());

  /////////////////////////////////////////////////////////////////////////////
  // Process transit information
  if (trip_id
      && (directededge->use() == Use::kRail || directededge->use() == Use::kBus)) {

    TripPath_TransitRouteInfo* transit_route_info = trip_edge
        ->mutable_transit_route_info();

    // Set block_id and trip_id
    transit_route_info->set_block_id(block_id);
    transit_route_info->set_trip_id(trip_id);

    const TransitDeparture* transit_departure = graphtile->GetTransitDeparture(
        directededge->lineid(), trip_id);

    if (transit_departure) {

      // Set headsign
      if (transit_departure->headsign_offset())
        transit_route_info->set_headsign(
            graphtile->GetName(transit_departure->headsign_offset()));

      const TransitRoute* transit_route = graphtile->GetTransitRoute(
          transit_departure->routeid());

      if (transit_route) {
        trip_edge->set_transit_type(GetTripPathTransitType(
              transit_route->route_type()));

        // Set onestop_id
        if (transit_route->one_stop_offset())
          transit_route_info->set_onestop_id(
              graphtile->GetName(transit_route->one_stop_offset()));

        // Set short_name
        if (transit_route->short_name_offset())
          transit_route_info->set_short_name(
              graphtile->GetName(transit_route->short_name_offset()));

        // Set long_name
        if (transit_route->long_name_offset())
          transit_route_info->set_long_name(
              graphtile->GetName(transit_route->long_name_offset()));

        // Set color and text_color
        transit_route_info->set_color(transit_route->route_color());
        transit_route_info->set_text_color(transit_route->route_text_color());

        // Set description
        if (transit_route->desc_offset())
          transit_route_info->set_description(
              graphtile->GetName(transit_route->desc_offset()));

        // Set operator_onestop_id
        if (transit_route->op_by_onestop_id_offset())
          transit_route_info->set_operator_onestop_id(
              graphtile->GetName(transit_route->op_by_onestop_id_offset()));

        // Set operator_name
        if (transit_route->op_by_name_offset())
          transit_route_info->set_operator_name(
              graphtile->GetName(transit_route->op_by_name_offset()));

        // Set operator_url
        if (transit_route->op_by_website_offset())
          transit_route_info->set_operator_url(
              graphtile->GetName(transit_route->op_by_website_offset()));
      }
    }
  }

  return trip_edge;
}

void TripPathBuilder::AddTripIntersectingEdge(uint32_t local_edge_index,
                                              uint32_t prev_edge_index,
                                              uint32_t curr_edge_index,
                                              const baldr::NodeInfo* nodeinfo,
                                              odin::TripPath_Node* trip_node,
                                              const DirectedEdge* intersecting_de) {
  TripPath_IntersectingEdge* itersecting_edge =
      trip_node->add_intersecting_edge();

  // Set the heading for the intersecting edge
  itersecting_edge->set_begin_heading(nodeinfo->heading(local_edge_index));

  Traversability traversability = Traversability::kNone;
  if (intersecting_de != nullptr) {
    if (intersecting_de->forwardaccess() & kPedestrianAccess) {
      traversability = (intersecting_de->reverseaccess() & kPedestrianAccess) ?
          Traversability::kBoth : Traversability::kForward;
    } else {
      traversability = (intersecting_de->reverseaccess() & kPedestrianAccess) ?
          Traversability::kBackward : Traversability::kNone;
    }
  }
  // Set the walkability flag for the intersecting edge
  itersecting_edge->set_walkability(GetTripPathTraversability(traversability));

  traversability = Traversability::kNone;
  if (intersecting_de != nullptr) {
    if (intersecting_de->forwardaccess() & kBicycleAccess) {
      traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ?
          Traversability::kBoth : Traversability::kForward;
    } else {
      traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ?
          Traversability::kBackward : Traversability::kNone;
    }
  }
  // Set the cyclability flag for the intersecting edge
  itersecting_edge->set_cyclability(GetTripPathTraversability(traversability));

  // Set the driveability flag for the intersecting edge
  itersecting_edge->set_driveability(
      GetTripPathTraversability(nodeinfo->local_driveability(local_edge_index)));

  // Set the previous/intersecting edge name consistency
  itersecting_edge->set_prev_name_consistency(
      nodeinfo->name_consistency(prev_edge_index, local_edge_index));

  // Set the current/intersecting edge name consistency
  itersecting_edge->set_curr_name_consistency(
      nodeinfo->name_consistency(curr_edge_index, local_edge_index));
}

}
}
