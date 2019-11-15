#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>

#include "baldr/datetime.h"
#include "baldr/edgeinfo.h"
#include "baldr/graphconstants.h"
#include "baldr/signinfo.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "sif/costconstants.h"
#include "thor/attributes_controller.h"
#include "thor/match_result.h"
#include "thor/triplegbuilder.h"

#include <valhalla/proto/tripcommon.pb.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

void TrimShape(std::vector<PointLL>& shape,
               const float start,
               const PointLL& start_vertex,
               const float end,
               const PointLL& end_vertex) {
  // clip up to the start point if the start_vertex is valid
  float along = 0.f;
  auto current = shape.begin();
  if (start_vertex.IsValid()) {
    while (current != shape.end() - 1) {
      along += (current + 1)->Distance(*current);
      // just crossed it, replace the current vertex with the start position and erase
      // shape up to the current vertex
      if (along > start) {
        along = start;
        *current = start_vertex;
        shape.erase(shape.begin(), current);
        break;
      }
      ++current;
    }
  }

  // clip after the end point if the end vertex is valid
  current = shape.begin();
  if (end_vertex.IsValid()) {
    while (current != shape.end() - 1) {
      along += (current + 1)->Distance(*current);
      // just crossed it, replace the current vertex with the end vertex and erase
      // shape after the current vertex
      if (along > end) {
        *(++current) = end_vertex;
        shape.erase(++current, shape.end());
        break;
      }
      ++current;
    }
  }
}

uint32_t
GetAdminIndex(const AdminInfo& admin_info,
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

void AssignAdmins(const AttributesController& controller,
                  TripLeg& trip_path,
                  const std::vector<AdminInfo>& admin_info_list) {
  if (controller.category_attribute_enabled(kAdminCategory)) {
    // Assign the admins
    for (const auto& admin_info : admin_info_list) {
      TripLeg_Admin* trip_admin = trip_path.add_admin();

      // Set country code if requested
      if (controller.attributes.at(kAdminCountryCode)) {
        trip_admin->set_country_code(admin_info.country_iso());
      }

      // Set country text if requested
      if (controller.attributes.at(kAdminCountryText)) {
        trip_admin->set_country_text(admin_info.country_text());
      }

      // Set state code if requested
      if (controller.attributes.at(kAdminStateCode)) {
        trip_admin->set_state_code(admin_info.state_iso());
      }

      // Set state text if requested
      if (controller.attributes.at(kAdminStateText)) {
        trip_admin->set_state_text(admin_info.state_text());
      }
    }
  }
}

void SetShapeAttributes(const AttributesController& controller,
                        const GraphTile* tile,
                        const DirectedEdge* edge,
                        const std::shared_ptr<sif::DynamicCost>& costing,
                        std::vector<PointLL>::const_iterator shape_begin,
                        std::vector<PointLL>::const_iterator shape_end,
                        TripLeg& trip_path,
                        uint32_t second_of_week) {
  if (trip_path.has_shape_attributes()) {
    // calculates total edge time and total edge length
    double edge_time = costing->EdgeCost(edge, tile, second_of_week).secs; // seconds
    // TODO: get the measured length from shape (full shape) to increase precision
    double edge_length = edge->length(); // meters
    // Set the shape attributes
    for (++shape_begin; shape_begin < shape_end; ++shape_begin) {
      double distance = shape_begin->Distance(*(shape_begin - 1)); // meters
      double distance_pct = distance / edge_length;                // fraction of edge length
      double time = edge_time * distance_pct;                      // seconds

      // Set shape attributes time per shape point if requested
      if (controller.attributes.at(kShapeAttributesTime)) {
        // convert time to milliseconds and then round to an integer
        trip_path.mutable_shape_attributes()->add_time((time * kMillisecondPerSec) + 0.5);
      }

      // Set shape attributes length per shape point if requested
      if (controller.attributes.at(kShapeAttributesLength)) {
        // convert length to decimeters and then round to an integer
        trip_path.mutable_shape_attributes()->add_length((distance * kDecimeterPerMeter) + 0.5);
      }

      // Set shape attributes speed per shape point if requested
      if (controller.attributes.at(kShapeAttributesSpeed)) {
        // convert speed to decimeters per sec and then round to an integer
        trip_path.mutable_shape_attributes()->add_speed((distance * kDecimeterPerMeter / time) + 0.5);
      }
    }
  }
}

// Set the bounding box (min,max lat,lon) for the shape
void SetBoundingBox(TripLeg& trip_path, std::vector<PointLL>& shape) {
  AABB2<PointLL> bbox(shape);
  LatLng* min_ll = trip_path.mutable_bbox()->mutable_min_ll();
  min_ll->set_lat(bbox.miny());
  min_ll->set_lng(bbox.minx());
  LatLng* max_ll = trip_path.mutable_bbox()->mutable_max_ll();
  max_ll->set_lat(bbox.maxy());
  max_ll->set_lng(bbox.maxx());
}

// Associate RoadClass values to TripLeg proto
constexpr TripLeg_RoadClass kTripLegRoadClass[] =
    {TripLeg_RoadClass_kMotorway,    TripLeg_RoadClass_kTrunk,       TripLeg_RoadClass_kPrimary,
     TripLeg_RoadClass_kSecondary,   TripLeg_RoadClass_kTertiary,    TripLeg_RoadClass_kUnclassified,
     TripLeg_RoadClass_kResidential, TripLeg_RoadClass_kServiceOther};
TripLeg_RoadClass GetTripLegRoadClass(const RoadClass road_class) {
  return kTripLegRoadClass[static_cast<int>(road_class)];
}

// Associate Surface values to TripLeg proto
constexpr TripLeg_Surface kTripLegSurface[] =
    {TripLeg_Surface_kPavedSmooth, TripLeg_Surface_kPaved,     TripLeg_Surface_kPavedRough,
     TripLeg_Surface_kCompacted,   TripLeg_Surface_kDirt,      TripLeg_Surface_kGravel,
     TripLeg_Surface_kPath,        TripLeg_Surface_kImpassable};
TripLeg_Surface GetTripLegSurface(const Surface surface) {
  return kTripLegSurface[static_cast<int>(surface)];
}

// Associate vehicle types to TripLeg proto
// TODO - why doesn't these use an enum input?
constexpr TripLeg_VehicleType kTripLegVehicleType[] =
    {TripLeg_VehicleType::TripLeg_VehicleType_kCar,
     TripLeg_VehicleType::TripLeg_VehicleType_kMotorcycle,
     TripLeg_VehicleType::TripLeg_VehicleType_kAutoBus,
     TripLeg_VehicleType::TripLeg_VehicleType_kTractorTrailer,
     TripLeg_VehicleType::TripLeg_VehicleType_kMotorScooter};
TripLeg_VehicleType GetTripLegVehicleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(VehicleType::kMotorScooter)) ? kTripLegVehicleType[type]
                                                                    : kTripLegVehicleType[0];
}

// Associate pedestrian types to TripLeg proto
constexpr TripLeg_PedestrianType kTripLegPedestrianType[] =
    {TripLeg_PedestrianType::TripLeg_PedestrianType_kFoot,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kWheelchair,
     TripLeg_PedestrianType::TripLeg_PedestrianType_kSegway};
TripLeg_PedestrianType GetTripLegPedestrianType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(PedestrianType::kSegway)) ? kTripLegPedestrianType[type]
                                                                 : kTripLegPedestrianType[0];
}

// Associate bicycle types to TripLeg proto
constexpr TripLeg_BicycleType kTripLegBicycleType[] =
    {TripLeg_BicycleType::TripLeg_BicycleType_kRoad, TripLeg_BicycleType::TripLeg_BicycleType_kCross,
     TripLeg_BicycleType::TripLeg_BicycleType_kHybrid,
     TripLeg_BicycleType::TripLeg_BicycleType_kMountain};
TripLeg_BicycleType GetTripLegBicycleType(const uint8_t type) {
  return (type <= static_cast<uint8_t>(BicycleType::kMountain)) ? kTripLegBicycleType[type]
                                                                : kTripLegBicycleType[0];
}

// Associate transit types to TripLeg proto
constexpr TripLeg_TransitType kTripLegTransitType[] =
    {TripLeg_TransitType::TripLeg_TransitType_kTram,
     TripLeg_TransitType::TripLeg_TransitType_kMetro,
     TripLeg_TransitType::TripLeg_TransitType_kRail,
     TripLeg_TransitType::TripLeg_TransitType_kBus,
     TripLeg_TransitType::TripLeg_TransitType_kFerry,
     TripLeg_TransitType::TripLeg_TransitType_kCableCar,
     TripLeg_TransitType::TripLeg_TransitType_kGondola,
     TripLeg_TransitType::TripLeg_TransitType_kFunicular};
TripLeg_TransitType GetTripLegTransitType(const TransitType transit_type) {
  return kTripLegTransitType[static_cast<uint32_t>(transit_type)];
}

// Associate traversability values to TripLeg proto
constexpr TripLeg_Traversability kTripLegTraversability[] = {TripLeg_Traversability_kNone,
                                                             TripLeg_Traversability_kForward,
                                                             TripLeg_Traversability_kBackward,
                                                             TripLeg_Traversability_kBoth};
TripLeg_Traversability GetTripLegTraversability(const Traversability traversability) {
  return kTripLegTraversability[static_cast<uint32_t>(traversability)];
}

// Associate side of street to TripLeg proto
constexpr valhalla::Location::SideOfStreet kTripLegSideOfStreet[] = {valhalla::Location::kNone,
                                                                     valhalla::Location::kLeft,
                                                                     valhalla::Location::kRight};
valhalla::Location::SideOfStreet GetTripLegSideOfStreet(const valhalla::Location::SideOfStreet sos) {
  return kTripLegSideOfStreet[static_cast<uint32_t>(sos)];
}

TripLeg_Node_Type GetTripLegNodeType(const NodeType node_type) {
  switch (node_type) {
    case NodeType::kStreetIntersection:
      return TripLeg_Node_Type_kStreetIntersection;
    case NodeType::kGate:
      return TripLeg_Node_Type_kGate;
    case NodeType::kBollard:
      return TripLeg_Node_Type_kBollard;
    case NodeType::kTollBooth:
      return TripLeg_Node_Type_kTollBooth;
    case NodeType::kTransitEgress:
      return TripLeg_Node_Type_kTransitEgress;
    case NodeType::kTransitStation:
      return TripLeg_Node_Type_kTransitStation;
    case NodeType::kMultiUseTransitPlatform:
      return TripLeg_Node_Type_kTransitPlatform;
    case NodeType::kBikeShare:
      return TripLeg_Node_Type_kBikeShare;
    case NodeType::kParking:
      return TripLeg_Node_Type_kParking;
    case NodeType::kMotorWayJunction:
      return TripLeg_Node_Type_kMotorwayJunction;
    case NodeType::kBorderControl:
      return TripLeg_Node_Type_kBorderControl;
  }
  auto num = static_cast<uint8_t>(node_type);
  throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) +
                           " Unhandled NodeType: " + std::to_string(num));
}

// Associate cycle lane values to TripLeg proto
constexpr TripLeg_CycleLane kTripLegCycleLane[] = {TripLeg_CycleLane_kNoCycleLane,
                                                   TripLeg_CycleLane_kShared,
                                                   TripLeg_CycleLane_kDedicated,
                                                   TripLeg_CycleLane_kSeparated};
TripLeg_CycleLane GetTripLegCycleLane(const CycleLane cyclelane) {
  return kTripLegCycleLane[static_cast<uint32_t>(cyclelane)];
}

// Associate Use to TripLeg proto
TripLeg_Use GetTripLegUse(const Use use) {
  switch (use) {
    case Use::kRoad:
      return TripLeg_Use_kRoadUse;
    case Use::kRamp:
      return TripLeg_Use_kRampUse;
    case Use::kTurnChannel:
      return TripLeg_Use_kTurnChannelUse;
    case Use::kTrack:
      return TripLeg_Use_kTrackUse;
    case Use::kDriveway:
      return TripLeg_Use_kDrivewayUse;
    case Use::kAlley:
      return TripLeg_Use_kAlleyUse;
    case Use::kParkingAisle:
      return TripLeg_Use_kParkingAisleUse;
    case Use::kEmergencyAccess:
      return TripLeg_Use_kEmergencyAccessUse;
    case Use::kDriveThru:
      return TripLeg_Use_kDriveThruUse;
    case Use::kCuldesac:
      return TripLeg_Use_kCuldesacUse;
    case Use::kCycleway:
      return TripLeg_Use_kCyclewayUse;
    case Use::kMountainBike:
      return TripLeg_Use_kMountainBikeUse;
    case Use::kSidewalk:
      // return TripLeg_Use_kSidewalkUse;
      return TripLeg_Use_kFootwayUse; // TODO: update when odin has been updated
    case Use::kFootway:
      return TripLeg_Use_kFootwayUse;
    case Use::kSteps:
      return TripLeg_Use_kStepsUse;
    case Use::kPath:
      return TripLeg_Use_kPathUse;
    case Use::kPedestrian:
      return TripLeg_Use_kPedestrianUse;
    case Use::kBridleway:
      return TripLeg_Use_kBridlewayUse;
    case Use::kOther:
      return TripLeg_Use_kOtherUse;
    case Use::kFerry:
      return TripLeg_Use_kFerryUse;
    case Use::kRailFerry:
      return TripLeg_Use_kRailFerryUse;
    case Use::kRail:
      return TripLeg_Use_kRailUse;
    case Use::kBus:
      return TripLeg_Use_kBusUse;
    case Use::kEgressConnection:
      return TripLeg_Use_kEgressConnectionUse;
    case Use::kPlatformConnection:
      return TripLeg_Use_kPlatformConnectionUse;
    case Use::kTransitConnection:
      return TripLeg_Use_kTransitConnectionUse;
    // Should not see other values
    default:
      // TODO should we throw a runtime error?
      return TripLeg_Use_kRoadUse;
  }
}

/**
 * Removes all edges but the one with the id that we are passing
 * @param location  The location
 * @param edge_id   The edge id to keep
 */
void RemovePathEdges(valhalla::Location* location, const GraphId& edge_id) {
  auto pos = std::find_if(location->path_edges().begin(), location->path_edges().end(),
                          [&edge_id](const valhalla::Location::PathEdge& e) {
                            return e.graph_id() == edge_id;
                          });
  if (pos == location->path_edges().end()) {
    location->mutable_path_edges()->Clear();
  } else if (location->path_edges_size() > 1) {
    location->mutable_path_edges()->SwapElements(0, pos - location->path_edges().begin());
    location->mutable_path_edges()->DeleteSubrange(1, location->path_edges_size() - 1);
  }
}

/**
 *
 */
void CopyLocations(TripLeg& trip_path,
                   const valhalla::Location& origin,
                   const std::list<valhalla::Location>& throughs,
                   const valhalla::Location& dest,
                   const std::vector<PathInfo>::const_iterator path_begin,
                   const std::vector<PathInfo>::const_iterator path_end) {
  // origin
  trip_path.add_location()->CopyFrom(origin);
  auto pe = path_begin;
  RemovePathEdges(trip_path.mutable_location(trip_path.location_size() - 1), pe->edgeid);

  // throughs
  for (const auto& through : throughs) {
    // copy
    valhalla::Location* tp_through = trip_path.add_location();
    tp_through->CopyFrom(through);
    // id set
    std::unordered_set<uint64_t> ids;
    for (const auto& e : tp_through->path_edges()) {
      ids.insert(e.graph_id());
    }
    // find id
    auto found = std::find_if(pe, path_end, [&ids](const PathInfo& pi) {
      return ids.find(pi.edgeid) != ids.end();
    });
    pe = found;
    RemovePathEdges(trip_path.mutable_location(trip_path.location_size() - 1), pe->edgeid);
  }

  // destination
  trip_path.add_location()->CopyFrom(dest);
  RemovePathEdges(trip_path.mutable_location(trip_path.location_size() - 1), (path_end - 1)->edgeid);
}

/**
 * Set begin and end heading if requested.
 * @param  trip_edge  Trip path edge to add headings.
 * @param  controller Controller specifying attributes to add to trip edge.
 * @param  edge       Directed edge.
 * @param  shape      Trip shape.
 */
void SetHeadings(TripLeg_Edge* trip_edge,
                 const AttributesController& controller,
                 const DirectedEdge* edge,
                 const std::vector<PointLL>& shape,
                 const uint32_t begin_index) {
  if (controller.attributes.at(kEdgeBeginHeading) || controller.attributes.at(kEdgeEndHeading)) {
    float offset = GetOffsetForHeading(edge->classification(), edge->use());
    if (controller.attributes.at(kEdgeBeginHeading)) {
      trip_edge->set_begin_heading(
          std::round(PointLL::HeadingAlongPolyline(shape, offset, begin_index, shape.size() - 1)));
    }
    if (controller.attributes.at(kEdgeEndHeading)) {
      trip_edge->set_end_heading(
          std::round(PointLL::HeadingAtEndOfPolyline(shape, offset, begin_index, shape.size() - 1)));
    }
  }
}

/**
 * @param trip_node   Trip node to add transit nodes.
 * @param node        Start nodeinfo of the current edge.
 * @param startnode   Start node of the current edge.
 * @param start_tile  Tile of the start node.
 * @param graphtile   Graph tile of the current edge.
 * @param controller  Controller specifying attributes to add to trip edge.
 *
 */
void AddTransitNodes(TripLeg_Node* trip_node,
                     const NodeInfo* node,
                     const GraphId& startnode,
                     const GraphTile* start_tile,
                     const GraphTile* graphtile,
                     const AttributesController& controller) {

  if (node->type() == NodeType::kTransitStation) {
    const TransitStop* transit_station =
        start_tile->GetTransitStop(start_tile->node(startnode)->stop_index());
    TransitStationInfo* transit_station_info = trip_node->mutable_transit_station_info();

    if (transit_station) {
      // Set onstop_id if requested
      if (controller.attributes.at(kNodeTransitStationInfoOnestopId) &&
          transit_station->one_stop_offset()) {
        transit_station_info->set_onestop_id(graphtile->GetName(transit_station->one_stop_offset()));
      }

      // Set name if requested
      if (controller.attributes.at(kNodeTransitStationInfoName) && transit_station->name_offset()) {
        transit_station_info->set_name(graphtile->GetName(transit_station->name_offset()));
      }

      // Set latitude and longitude
      LatLng* stop_ll = transit_station_info->mutable_ll();
      // Set transit stop lat/lon if requested
      if (controller.attributes.at(kNodeTransitStationInfoLatLon)) {
        PointLL ll = node->latlng(start_tile->header()->base_ll());
        stop_ll->set_lat(ll.lat());
        stop_ll->set_lng(ll.lng());
      }
    }
  }

  if (node->type() == NodeType::kTransitEgress) {
    const TransitStop* transit_egress =
        start_tile->GetTransitStop(start_tile->node(startnode)->stop_index());
    TransitEgressInfo* transit_egress_info = trip_node->mutable_transit_egress_info();

    if (transit_egress) {
      // Set onstop_id if requested
      if (controller.attributes.at(kNodeTransitEgressInfoOnestopId) &&
          transit_egress->one_stop_offset()) {
        transit_egress_info->set_onestop_id(graphtile->GetName(transit_egress->one_stop_offset()));
      }

      // Set name if requested
      if (controller.attributes.at(kNodeTransitEgressInfoName) && transit_egress->name_offset()) {
        transit_egress_info->set_name(graphtile->GetName(transit_egress->name_offset()));
      }

      // Set latitude and longitude
      LatLng* stop_ll = transit_egress_info->mutable_ll();
      // Set transit stop lat/lon if requested
      if (controller.attributes.at(kNodeTransitEgressInfoLatLon)) {
        PointLL ll = node->latlng(start_tile->header()->base_ll());
        stop_ll->set_lat(ll.lat());
        stop_ll->set_lng(ll.lng());
      }
    }
  }
}

/**
 * Add trip edge. (TODO more comments)
 * @param  controller        Controller to determine which attributes to set.
 * @param  edge              Identifier of an edge within the tiled, hierarchical graph.
 * @param  trip_id           Trip Id (0 if not a transit edge).
 * @param  block_id          Transit block Id (0 if not a transit edge)
 * @param  mode              Travel mode for the edge: Biking, walking, etc.
 * @param  directededge      Directed edge information.
 * @param  drive_right       Right side driving for this edge.
 * @param  trip_node         Trip node to add the edge information to.
 * @param  graphtile         Graph tile for accessing data.
 * @param  second_of_week    The time, from the beginning of the week in seconds at which
 *                           the path entered this edge
 * @param  length_percentage Scale for the edge length for the partial distance
 *                       at begin and end edges
 */
TripLeg_Edge* AddTripEdge(const AttributesController& controller,
                          const GraphId& edge,
                          const uint32_t trip_id,
                          const uint32_t block_id,
                          const sif::TravelMode mode,
                          const uint8_t travel_type,
                          const std::shared_ptr<sif::DynamicCost>& costing,
                          const DirectedEdge* directededge,
                          const bool drive_on_right,
                          TripLeg_Node* trip_node,
                          const GraphTile* graphtile,
                          const uint32_t second_of_week,
                          const float length_percentage,
                          const bool has_time_restrictions) {

  // Index of the directed edge within the tile
  uint32_t idx = edge.id();

  TripLeg_Edge* trip_edge = trip_node->mutable_edge();

  // Get the edgeinfo
  auto edgeinfo = graphtile->edgeinfo(directededge->edgeinfo_offset());

  // Add names to edge if requested
  if (controller.attributes.at(kEdgeNames)) {
    auto names_and_types = edgeinfo.GetNamesAndTypes();
    for (const auto& name_and_type : names_and_types) {
      auto* trip_edge_name = trip_edge->mutable_name()->Add();
      trip_edge_name->set_value(name_and_type.first);
      trip_edge_name->set_is_route_number(name_and_type.second);
    }
  }

#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE(std::string("wayid=") + std::to_string(edgeinfo.wayid()));
#endif

  // Set the exits (if the directed edge has exit sign information) and if requested
  if (directededge->exitsign()) {
    std::vector<SignInfo> signs = graphtile->GetSigns(idx);
    if (!signs.empty()) {
      TripLeg_Sign* trip_exit = trip_edge->mutable_sign();
      for (const auto& sign : signs) {
        switch (sign.type()) {
          case Sign::Type::kExitNumber: {
            if (controller.attributes.at(kEdgeSignExitNumber)) {
              auto* trip_exit_number = trip_exit->mutable_exit_numbers()->Add();
              trip_exit_number->set_text(sign.text());
              trip_exit_number->set_is_route_number(sign.is_route_num());
            }
            break;
          }
          case Sign::Type::kExitBranch: {
            if (controller.attributes.at(kEdgeSignExitBranch)) {
              auto* trip_exit_onto_street = trip_exit->mutable_exit_onto_streets()->Add();
              trip_exit_onto_street->set_text(sign.text());
              trip_exit_onto_street->set_is_route_number(sign.is_route_num());
            }
            break;
          }
          case Sign::Type::kExitToward: {
            if (controller.attributes.at(kEdgeSignExitToward)) {
              auto* trip_exit_toward_location = trip_exit->mutable_exit_toward_locations()->Add();
              trip_exit_toward_location->set_text(sign.text());
              trip_exit_toward_location->set_is_route_number(sign.is_route_num());
            }
            break;
          }
          case Sign::Type::kExitName: {
            if (controller.attributes.at(kEdgeSignExitName)) {
              auto* trip_exit_name = trip_exit->mutable_exit_names()->Add();
              trip_exit_name->set_text(sign.text());
              trip_exit_name->set_is_route_number(sign.is_route_num());
            }
            break;
          }
        }
      }
    }
  }

  // If turn lanes exist
  if (directededge->turnlanes()) {
    auto turnlanes = graphtile->turnlanes(idx);
    for (auto tl : turnlanes) {
      TurnLane* turn_lane = trip_edge->add_turn_lanes();
      turn_lane->set_directions_mask(tl);
    }
  }

  // Set road class if requested
  if (controller.attributes.at(kEdgeRoadClass)) {
    trip_edge->set_road_class(GetTripLegRoadClass(directededge->classification()));
  }

  // Set length if requested. Convert to km
  if (controller.attributes.at(kEdgeLength)) {
    float km = std::max((directededge->length() * kKmPerMeter * length_percentage), 0.001f);
    trip_edge->set_length(km);
  }

  // Set speed if requested
  if (controller.attributes.at(kEdgeSpeed)) {
    // TODO: could get better precision speed here by calling GraphTile::GetSpeed but we'd need to
    // know whether or not the costing actually cares about the speed of the edge. Perhaps a refactor
    // of costing to have a GetSpeed function which EdgeCost calls internally but which we can also
    // call externally
    auto speed = directededge->length() /
                 costing->EdgeCost(directededge, graphtile, second_of_week).secs * 3.6;
    trip_edge->set_speed(speed);
  }

  uint8_t kAccess = 0;
  if (mode == sif::TravelMode::kBicycle) {
    kAccess = kBicycleAccess;
  } else if (mode == sif::TravelMode::kDrive) {
    kAccess = kAutoAccess;
  } else if (mode == sif::TravelMode::kPedestrian || mode == sif::TravelMode::kPublicTransit) {
    kAccess = kPedestrianAccess;
  }

  // Test whether edge is traversed forward or reverse
  if (directededge->forward()) {
    // Set traversability for forward directededge if requested
    if (controller.attributes.at(kEdgeTraversability)) {
      if ((directededge->forwardaccess() & kAccess) && (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBoth);
      } else if ((directededge->forwardaccess() & kAccess) &&
                 !(directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kForward);
      } else if (!(directededge->forwardaccess() & kAccess) &&
                 (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBackward);
      } else {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kNone);
      }
    }
  } else {
    // Set traversability for reverse directededge if requested
    if (controller.attributes.at(kEdgeTraversability)) {
      if ((directededge->forwardaccess() & kAccess) && (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBoth);
      } else if (!(directededge->forwardaccess() & kAccess) &&
                 (directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kForward);
      } else if ((directededge->forwardaccess() & kAccess) &&
                 !(directededge->reverseaccess() & kAccess)) {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kBackward);
      } else {
        trip_edge->set_traversability(TripLeg_Traversability::TripLeg_Traversability_kNone);
      }
    }
  }

  if (has_time_restrictions) {
    trip_edge->set_has_time_restrictions(has_time_restrictions);
  }

  // Set the trip path use based on directed edge use if requested
  if (controller.attributes.at(kEdgeUse)) {
    trip_edge->set_use(GetTripLegUse(directededge->use()));
  }

  // Set toll flag if requested
  if (directededge->toll() && controller.attributes.at(kEdgeToll)) {
    trip_edge->set_toll(true);
  }

  // Set unpaved flag if requested
  if (directededge->unpaved() && controller.attributes.at(kEdgeUnpaved)) {
    trip_edge->set_unpaved(true);
  }

  // Set tunnel flag if requested
  if (directededge->tunnel() && controller.attributes.at(kEdgeTunnel)) {
    trip_edge->set_tunnel(true);
  }

  // Set bridge flag if requested
  if (directededge->bridge() && controller.attributes.at(kEdgeBridge)) {
    trip_edge->set_bridge(true);
  }

  // Set roundabout flag if requested
  if (directededge->roundabout() && controller.attributes.at(kEdgeRoundabout)) {
    trip_edge->set_roundabout(true);
  }

  // Set internal intersection flag if requested
  if (directededge->internal() && controller.attributes.at(kEdgeInternalIntersection)) {
    trip_edge->set_internal_intersection(true);
  }

  // Set drive_on_right if requested
  if (controller.attributes.at(kEdgeDriveOnRight)) {
    trip_edge->set_drive_on_right(drive_on_right);
  }

  // Set surface if requested
  if (controller.attributes.at(kEdgeSurface)) {
    trip_edge->set_surface(GetTripLegSurface(directededge->surface()));
  }

  // Set the mode and travel type
  if (mode == sif::TravelMode::kBicycle) {
    // Override bicycle mode with pedestrian if dismount flag or steps
    if (directededge->dismount() || directededge->use() == Use::kSteps) {
      if (controller.attributes.at(kEdgeTravelMode)) {
        trip_edge->set_travel_mode(TripLeg_TravelMode::TripLeg_TravelMode_kPedestrian);
      }
      if (controller.attributes.at(kEdgePedestrianType)) {
        trip_edge->set_pedestrian_type(TripLeg_PedestrianType::TripLeg_PedestrianType_kFoot);
      }
    } else {
      if (controller.attributes.at(kEdgeTravelMode)) {
        trip_edge->set_travel_mode(TripLeg_TravelMode::TripLeg_TravelMode_kBicycle);
      }
      if (controller.attributes.at(kEdgeBicycleType)) {
        trip_edge->set_bicycle_type(GetTripLegBicycleType(travel_type));
      }
    }
  } else if (mode == sif::TravelMode::kDrive) {
    if (controller.attributes.at(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(TripLeg_TravelMode::TripLeg_TravelMode_kDrive);
    }
    if (controller.attributes.at(kEdgeVehicleType)) {
      trip_edge->set_vehicle_type(GetTripLegVehicleType(travel_type));
    }
  } else if (mode == sif::TravelMode::kPedestrian) {
    if (controller.attributes.at(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(TripLeg_TravelMode::TripLeg_TravelMode_kPedestrian);
    }
    if (controller.attributes.at(kEdgePedestrianType)) {
      trip_edge->set_pedestrian_type(GetTripLegPedestrianType(travel_type));
    }
  } else if (mode == sif::TravelMode::kPublicTransit) {
    if (controller.attributes.at(kEdgeTravelMode)) {
      trip_edge->set_travel_mode(TripLeg_TravelMode::TripLeg_TravelMode_kTransit);
    }
  }

  // Set edge id (graphid value) if requested
  if (controller.attributes.at(kEdgeId)) {
    trip_edge->set_id(edge.value);
  }

  // Set way id (base data id) if requested
  if (controller.attributes.at(kEdgeWayId)) {
    trip_edge->set_way_id(edgeinfo.wayid());
  }

  // Set weighted grade if requested
  if (controller.attributes.at(kEdgeWeightedGrade)) {
    trip_edge->set_weighted_grade((directededge->weighted_grade() - 6.f) / 0.6f);
  }

  // Set maximum upward and downward grade if requested (set to kNoElevationData if unavailable)
  if (controller.attributes.at(kEdgeMaxUpwardGrade)) {
    if (graphtile->header()->has_elevation()) {
      trip_edge->set_max_upward_grade(directededge->max_up_slope());
    } else {
      trip_edge->set_max_upward_grade(kNoElevationData);
    }
  }
  if (controller.attributes.at(kEdgeMaxDownwardGrade)) {
    if (graphtile->header()->has_elevation()) {
      trip_edge->set_max_downward_grade(directededge->max_down_slope());
    } else {
      trip_edge->set_max_downward_grade(kNoElevationData);
    }
  }

  // Set mean elevation if requested (set to kNoElevationData if unavailable)
  if (controller.attributes.at(kEdgeMeanElevation)) {
    if (graphtile->header()->has_elevation()) {
      trip_edge->set_mean_elevation(edgeinfo.mean_elevation());
    } else {
      trip_edge->set_mean_elevation(kNoElevationData);
    }
  }

  if (controller.attributes.at(kEdgeLaneCount)) {
    trip_edge->set_lane_count(directededge->lanecount());
  }

  if (directededge->laneconnectivity() && controller.attributes.at(kEdgeLaneConnectivity)) {
    for (const auto& l : graphtile->GetLaneConnectivity(idx)) {
      TripLeg_LaneConnectivity* path_lane = trip_edge->add_lane_connectivity();
      path_lane->set_from_way_id(l.from());
      path_lane->set_to_lanes(l.to_lanes());
      path_lane->set_from_lanes(l.from_lanes());
    }
  }

  if (directededge->cyclelane() != CycleLane::kNone && controller.attributes.at(kEdgeCycleLane)) {
    trip_edge->set_cycle_lane(GetTripLegCycleLane(directededge->cyclelane()));
  }

  if (controller.attributes.at(kEdgeBicycleNetwork)) {
    trip_edge->set_bicycle_network(directededge->bike_network());
  }

  if (controller.attributes.at(kEdgeSidewalk)) {
    if (directededge->sidewalk_left() && directededge->sidewalk_right()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kBothSides);
    } else if (directededge->sidewalk_left()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kLeft);
    } else if (directededge->sidewalk_right()) {
      trip_edge->set_sidewalk(TripLeg_Sidewalk::TripLeg_Sidewalk_kRight);
    }
  }

  if (controller.attributes.at(kEdgeDensity)) {
    trip_edge->set_density(directededge->density());
  }

  if (controller.attributes.at(kEdgeSpeedLimit)) {
    trip_edge->set_speed_limit(edgeinfo.speed_limit());
  }

  if (controller.attributes.at(kEdgeTruckSpeed)) {
    trip_edge->set_truck_speed(directededge->truck_speed());
  }

  if (directededge->truck_route() && controller.attributes.at(kEdgeTruckRoute)) {
    trip_edge->set_truck_route(true);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process transit information
  if (trip_id && (directededge->use() == Use::kRail || directededge->use() == Use::kBus)) {

    TripLeg_TransitRouteInfo* transit_route_info = trip_edge->mutable_transit_route_info();

    // Set block_id if requested
    if (controller.attributes.at(kEdgeTransitRouteInfoBlockId)) {
      transit_route_info->set_block_id(block_id);
    }

    // Set trip_id if requested
    if (controller.attributes.at(kEdgeTransitRouteInfoTripId)) {
      transit_route_info->set_trip_id(trip_id);
    }

    const TransitDeparture* transit_departure =
        graphtile->GetTransitDeparture(directededge->lineid(), trip_id,
                                       second_of_week % kSecondsPerDay);

    if (transit_departure) {

      // Set headsign if requested
      if (controller.attributes.at(kEdgeTransitRouteInfoHeadsign) &&
          transit_departure->headsign_offset()) {
        transit_route_info->set_headsign(graphtile->GetName(transit_departure->headsign_offset()));
      }

      const TransitRoute* transit_route = graphtile->GetTransitRoute(transit_departure->routeid());

      if (transit_route) {
        // Set transit type if requested
        if (controller.attributes.at(kEdgeTransitType)) {
          trip_edge->set_transit_type(GetTripLegTransitType(transit_route->route_type()));
        }

        // Set onestop_id if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoOnestopId) &&
            transit_route->one_stop_offset()) {
          transit_route_info->set_onestop_id(graphtile->GetName(transit_route->one_stop_offset()));
        }

        // Set short_name if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoShortName) &&
            transit_route->short_name_offset()) {
          transit_route_info->set_short_name(graphtile->GetName(transit_route->short_name_offset()));
        }

        // Set long_name if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoLongName) &&
            transit_route->long_name_offset()) {
          transit_route_info->set_long_name(graphtile->GetName(transit_route->long_name_offset()));
        }

        // Set color if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoColor)) {
          transit_route_info->set_color(transit_route->route_color());
        }

        // Set text_color if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoTextColor)) {
          transit_route_info->set_text_color(transit_route->route_text_color());
        }

        // Set description if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoDescription) &&
            transit_route->desc_offset()) {
          transit_route_info->set_description(graphtile->GetName(transit_route->desc_offset()));
        }

        // Set operator_onestop_id if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoOperatorOnestopId) &&
            transit_route->op_by_onestop_id_offset()) {
          transit_route_info->set_operator_onestop_id(
              graphtile->GetName(transit_route->op_by_onestop_id_offset()));
        }

        // Set operator_name if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoOperatorName) &&
            transit_route->op_by_name_offset()) {
          transit_route_info->set_operator_name(
              graphtile->GetName(transit_route->op_by_name_offset()));
        }

        // Set operator_url if requested
        if (controller.attributes.at(kEdgeTransitRouteInfoOperatorUrl) &&
            transit_route->op_by_website_offset()) {
          transit_route_info->set_operator_url(
              graphtile->GetName(transit_route->op_by_website_offset()));
        }
      }
    }
  }

  return trip_edge;
}

/**
 * Add trip intersecting edge.
 * @param  controller   Controller to determine which attributes to set.
 * @param  directededge Directed edge on the path.
 * @param  prev_de  Previous directed edge on the path.
 * @param  local_edge_index  Index of the local intersecting path edge at intersection.
 * @param  nodeinfo  Node information of the intersection.
 * @param  trip_node  Trip node that will store the intersecting edge information.
 * @param  intersecting_de Intersecting directed edge. Will be nullptr except when
 *                         on the local hierarchy.
 */
void AddTripIntersectingEdge(const AttributesController& controller,
                             const DirectedEdge* directededge,
                             const DirectedEdge* prev_de,
                             uint32_t local_edge_index,
                             const NodeInfo* nodeinfo,
                             TripLeg_Node* trip_node,
                             const DirectedEdge* intersecting_de) {
  TripLeg_IntersectingEdge* itersecting_edge = trip_node->add_intersecting_edge();

  // Set the heading for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeBeginHeading)) {
    itersecting_edge->set_begin_heading(nodeinfo->heading(local_edge_index));
  }

  Traversability traversability = Traversability::kNone;
  // Determine walkability
  if (intersecting_de->forwardaccess() & kPedestrianAccess) {
    traversability = (intersecting_de->reverseaccess() & kPedestrianAccess)
                         ? Traversability::kBoth
                         : Traversability::kForward;
  } else {
    traversability = (intersecting_de->reverseaccess() & kPedestrianAccess)
                         ? Traversability::kBackward
                         : Traversability::kNone;
  }
  // Set the walkability flag for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeWalkability)) {
    itersecting_edge->set_walkability(GetTripLegTraversability(traversability));
  }

  traversability = Traversability::kNone;
  // Determine cyclability
  if (intersecting_de->forwardaccess() & kBicycleAccess) {
    traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ? Traversability::kBoth
                                                                         : Traversability::kForward;
  } else {
    traversability = (intersecting_de->reverseaccess() & kBicycleAccess) ? Traversability::kBackward
                                                                         : Traversability::kNone;
  }
  // Set the cyclability flag for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeCyclability)) {
    itersecting_edge->set_cyclability(GetTripLegTraversability(traversability));
  }

  // Set the driveability flag for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeDriveability)) {
    itersecting_edge->set_driveability(
        GetTripLegTraversability(nodeinfo->local_driveability(local_edge_index)));
  }

  // Set the previous/intersecting edge name consistency if requested
  if (controller.attributes.at(kNodeIntersectingEdgeFromEdgeNameConsistency)) {
    bool name_consistency =
        (prev_de == nullptr) ? false : prev_de->name_consistency(local_edge_index);
    itersecting_edge->set_prev_name_consistency(name_consistency);
  }

  // Set the current/intersecting edge name consistency if requested
  if (controller.attributes.at(kNodeIntersectingEdgeToEdgeNameConsistency)) {
    itersecting_edge->set_curr_name_consistency(directededge->name_consistency(local_edge_index));
  }

  // Set the use for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeUse)) {
    itersecting_edge->set_use(GetTripLegUse(intersecting_de->use()));
  }

  // Set the road class for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeRoadClass)) {
    itersecting_edge->set_road_class(GetTripLegRoadClass(intersecting_de->classification()));
  }
}

} // namespace

namespace valhalla {
namespace thor {

TripLeg
TripLegBuilder::Build(const AttributesController& controller,
                      GraphReader& graphreader,
                      const std::shared_ptr<sif::DynamicCost>* mode_costing,
                      const std::vector<PathInfo>::const_iterator path_begin,
                      const std::vector<PathInfo>::const_iterator path_end,
                      valhalla::Location& origin,
                      valhalla::Location& dest,
                      const std::list<valhalla::Location>& through_loc,
                      TripLeg& trip_path,
                      const std::function<void()>* interrupt_callback,
                      std::unordered_map<size_t, std::pair<RouteDiscontinuity, RouteDiscontinuity>>*
                          route_discontinuities) {
  // Test interrupt prior to building trip path
  if (interrupt_callback) {
    (*interrupt_callback)();
  }

  // Set origin, any through locations, and destination. Origin and
  // destination are assumed to be breaks.
  CopyLocations(trip_path, origin, through_loc, dest, path_begin, path_end);
  auto* tp_orig = trip_path.mutable_location(0);
  auto* tp_dest = trip_path.mutable_location(trip_path.location_size() - 1);

  uint32_t origin_second_of_week = kInvalidSecondsOfWeek;
  if (origin.has_date_time()) {
    origin_second_of_week = DateTime::day_of_week(origin.date_time()) * kSecondsPerDay +
                            DateTime::seconds_from_midnight(origin.date_time());
  }

  // Create an array of travel types per mode
  uint8_t travel_types[4];
  for (uint32_t i = 0; i < 4; i++) {
    travel_types[i] = (mode_costing[i] != nullptr) ? mode_costing[i]->travel_type() : 0;
  }

  // Get the first nodes graph id by using the end node of the first edge to get the tile with the
  // opposing edge then use the opposing index to get the opposing edge, and its end node is the
  // begin node of the original edge
  auto* first_edge = graphreader.GetGraphTile(path_begin->edgeid)->directededge(path_begin->edgeid);
  auto* first_tile = graphreader.GetGraphTile(first_edge->endnode());
  auto* first_node = first_tile->node(first_edge->endnode());
  GraphId startnode =
      first_tile->directededge(first_node->edge_index() + first_edge->opp_index())->endnode();

  // Partial edge at the start and side of street (sos)
  float start_pct;
  valhalla::Location::SideOfStreet start_sos =
      valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone;
  PointLL start_vrt;
  for (const auto& e : origin.path_edges()) {
    if (e.graph_id() == path_begin->edgeid) {
      start_pct = e.percent_along();
      start_sos = e.side_of_street();
      start_vrt = PointLL(e.ll().lng(), e.ll().lat());
      break;
    }
  }

  // Set the origin projected location
  LatLng* proj_ll = tp_orig->mutable_projected_ll();
  proj_ll->set_lat(start_vrt.lat());
  proj_ll->set_lng(start_vrt.lng());

  // Set the origin side of street, if one exists
  if (start_sos != valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone) {
    tp_orig->set_side_of_street(GetTripLegSideOfStreet(start_sos));
  }

  // Partial edge at the end
  float end_pct;
  valhalla::Location::SideOfStreet end_sos =
      valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone;
  PointLL end_vrt;
  for (const auto& e : dest.path_edges()) {
    if (e.graph_id() == (path_end - 1)->edgeid) {
      end_pct = e.percent_along();
      end_sos = e.side_of_street();
      end_vrt = PointLL(e.ll().lng(), e.ll().lat());
      break;
    }
  }

  // Set the destination projected location
  proj_ll = tp_dest->mutable_projected_ll();
  proj_ll->set_lat(end_vrt.lat());
  proj_ll->set_lng(end_vrt.lng());

  // Set the destination side of street, if one exists
  if (end_sos != valhalla::Location::SideOfStreet::Location_SideOfStreet_kNone) {
    tp_dest->set_side_of_street(GetTripLegSideOfStreet(end_sos));
  }

  // Structures to process admins
  std::unordered_map<AdminInfo, uint32_t, AdminInfo::AdminInfoHasher> admin_info_map;
  std::vector<AdminInfo> admin_info_list;

  // initialize shape_attributes
  if (controller.category_attribute_enabled(kShapeAttributesCategory)) {
    trip_path.mutable_shape_attributes();
  }

  // If the path was only one edge we have a special case
  if ((path_end - path_begin) == 1) {
    const GraphTile* tile = graphreader.GetGraphTile(path_begin->edgeid);
    const DirectedEdge* edge = tile->directededge(path_begin->edgeid);

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
      end_pct = 1.0f - end_pct;
      edge = graphreader.GetOpposingEdge(path_begin->edgeid, tile);
      if (end_sos == valhalla::Location::SideOfStreet::Location_SideOfStreet_kLeft) {
        tp_dest->set_side_of_street(
            GetTripLegSideOfStreet(valhalla::Location::SideOfStreet::Location_SideOfStreet_kRight));
      } else if (end_sos == valhalla::Location::SideOfStreet::Location_SideOfStreet_kRight) {
        tp_dest->set_side_of_street(
            GetTripLegSideOfStreet(valhalla::Location::SideOfStreet::Location_SideOfStreet_kLeft));
      }
    }

    float total = static_cast<float>(edge->length());
    TrimShape(shape, start_pct * total, start_vrt, end_pct * total, end_vrt);

    // Driving on right from the start of the edge?
    const GraphId start_node = graphreader.GetOpposingEdge(path_begin->edgeid)->endnode();
    bool drive_on_right = graphreader.nodeinfo(start_node)->drive_on_right();

    // Add trip edge
    auto trip_edge = AddTripEdge(controller, path_begin->edgeid, path_begin->trip_id, 0,
                                 path_begin->mode, travel_types[static_cast<int>(path_begin->mode)],
                                 mode_costing[static_cast<uint32_t>(path_begin->mode)], edge,
                                 drive_on_right, trip_path.add_node(), tile, origin_second_of_week,
                                 std::abs(end_pct - start_pct), path_begin->has_time_restrictions);

    // Set begin shape index if requested
    if (controller.attributes.at(kEdgeBeginShapeIndex)) {
      trip_edge->set_begin_shape_index(0);
    }
    // Set end shape index if requested
    if (controller.attributes.at(kEdgeEndShapeIndex)) {
      trip_edge->set_end_shape_index(shape.size() - 1);
    }

    // Set shape attributes
    SetShapeAttributes(controller, tile, edge, mode_costing[static_cast<int>(path_begin->mode)],
                       shape.begin(), shape.end(), trip_path, origin_second_of_week);

    // Set begin and end heading if requested. Uses shape so
    // must be done after the edge's shape has been added.
    SetHeadings(trip_edge, controller, edge, shape, 0);

    auto* node = trip_path.add_node();
    if (controller.attributes.at(kNodeElapsedTime)) {
      node->set_elapsed_time(path_begin->elapsed_time);
    }

    const GraphTile* end_tile = graphreader.GetGraphTile(edge->endnode());
    if (end_tile == nullptr) {
      if (controller.attributes.at(kNodeaAdminIndex)) {
        node->set_admin_index(0);
      }
    } else {
      if (controller.attributes.at(kNodeaAdminIndex)) {
        node->set_admin_index(
            GetAdminIndex(end_tile->admininfo(end_tile->node(edge->endnode())->admin_index()),
                          admin_info_map, admin_info_list));
      }
    }

    // Set the bounding box of the shape
    SetBoundingBox(trip_path, shape);

    // Set shape if requested
    if (controller.attributes.at(kShape)) {
      trip_path.set_shape(encode<std::vector<PointLL>>(shape));
    }

    if (controller.attributes.at(kOsmChangeset)) {
      trip_path.set_osm_changeset(tile->header()->dataset_id());
    }

    // Assign the trip path admins
    AssignAdmins(controller, trip_path, admin_info_list);

    return trip_path;
  }

  // Iterate through path
  bool is_first_edge = true;
  double elapsedtime = 0;
  uint32_t block_id = 0;
  uint32_t prior_opp_local_index = -1;
  std::vector<PointLL> trip_shape;
  std::string arrival_time;
  bool assumed_schedule = false;
  uint64_t osmchangeset = 0;
  size_t edge_index = 0;
  const DirectedEdge* prev_de = nullptr;
  const GraphTile* graphtile = nullptr;
  uint32_t origin_epoch = 0;
  // TODO: this is temp until we use transit stop type from transitland
  TransitPlatformInfo_Type prev_transit_node_type = TransitPlatformInfo_Type_kStop;

  for (auto edge_itr = path_begin; edge_itr != path_end; ++edge_itr, ++edge_index) {
    const GraphId& edge = edge_itr->edgeid;
    const uint32_t trip_id = edge_itr->trip_id;
    graphtile = graphreader.GetGraphTile(edge, graphtile);
    const DirectedEdge* directededge = graphtile->directededge(edge);
    const sif::TravelMode mode = edge_itr->mode;
    const uint8_t travel_type = travel_types[static_cast<uint32_t>(mode)];
    const auto& costing = mode_costing[static_cast<uint32_t>(mode)];

    // Set node attributes - only set if they are true since they are optional
    const GraphTile* start_tile = graphtile;
    start_tile = graphreader.GetGraphTile(startnode, start_tile);
    const NodeInfo* node = start_tile->node(startnode);

    if (osmchangeset == 0 && controller.attributes.at(kOsmChangeset)) {
      osmchangeset = start_tile->header()->dataset_id();
    }

    // cache this the first time
    if (origin.has_date_time() && origin_epoch == 0) {
      origin_epoch =
          DateTime::seconds_since_epoch(origin.date_time(),
                                        DateTime::get_tz_db().from_index(node->timezone()));
    }

    // have to always compute the offset in case the timezone changes along the path
    // we could cache the timezone and just add seconds when the timezone doesnt change
    uint32_t second_of_week = kInvalidSecondsOfWeek;
    if (origin_epoch != 0) {
      second_of_week = DateTime::second_of_week(origin_epoch + static_cast<uint32_t>(elapsedtime),
                                                DateTime::get_tz_db().from_index(node->timezone()));
    }

    // Add a node to the trip path and set its attributes.
    TripLeg_Node* trip_node = trip_path.add_node();

    if (controller.attributes.at(kNodeType)) {
      trip_node->set_type(GetTripLegNodeType(node->type()));
    }

    if (node->intersection() == IntersectionType::kFork) {
      if (controller.attributes.at(kNodeFork)) {
        trip_node->set_fork(true);
      }
    }

    // Assign the elapsed time from the start of the leg
    if (controller.attributes.at(kNodeElapsedTime)) {
      trip_node->set_elapsed_time(elapsedtime);
    }

    // Assign the admin index
    if (controller.attributes.at(kNodeaAdminIndex)) {
      trip_node->set_admin_index(
          GetAdminIndex(start_tile->admininfo(node->admin_index()), admin_info_map, admin_info_list));
    }

    if (controller.attributes.at(kNodeTimeZone)) {
      auto tz = DateTime::get_tz_db().from_index(node->timezone());
      if (tz) {
        trip_node->set_time_zone(tz->name());
      }
    }

    AddTransitNodes(trip_node, node, startnode, start_tile, graphtile, controller);

    ///////////////////////////////////////////////////////////////////////////
    // Add transit information if this is a transit stop. TODO - can we move
    // this to another method?
    if (node->is_transit()) {
      // Get the transit stop information and add transit stop info
      const TransitStop* transit_platform = start_tile->GetTransitStop(node->stop_index());
      TransitPlatformInfo* transit_platform_info = trip_node->mutable_transit_platform_info();

      // TODO: for now we will set to station for rail and stop for others
      //       in future, we will set based on transitland value
      // Set type
      if (directededge->use() == Use::kRail) {
        // Set node transit info type if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(TransitPlatformInfo_Type_kStation);
        }
        prev_transit_node_type = TransitPlatformInfo_Type_kStation;
      } else if (directededge->use() == Use::kPlatformConnection) {
        // Set node transit info type if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(prev_transit_node_type);
        }
      } else { // bus logic
        // Set node transit info type if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(TransitPlatformInfo_Type_kStop);
        }
        prev_transit_node_type = TransitPlatformInfo_Type_kStop;
      }

      if (transit_platform) {
        // Set onstop_id if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoOnestopId) &&
            transit_platform->one_stop_offset()) {
          transit_platform_info->set_onestop_id(
              graphtile->GetName(transit_platform->one_stop_offset()));
        }

        // Set name if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoName) &&
            transit_platform->name_offset()) {
          transit_platform_info->set_name(graphtile->GetName(transit_platform->name_offset()));
        }

        // save station name and info for all platforms.
        const DirectedEdge* dir_edge = start_tile->directededge(node->edge_index());
        for (uint32_t index = 0; index < node->edge_count(); ++index, dir_edge++) {
          if (dir_edge->use() == Use::kPlatformConnection) {
            GraphId endnode = dir_edge->endnode();
            const GraphTile* endtile = graphreader.GetGraphTile(endnode);
            const NodeInfo* nodeinfo2 = endtile->node(endnode);
            const TransitStop* transit_station = endtile->GetTransitStop(nodeinfo2->stop_index());

            // Set station onstop_id if requested
            if (controller.attributes.at(kNodeTransitPlatformInfoStationOnestopId) &&
                transit_station->one_stop_offset()) {
              transit_platform_info->set_station_onestop_id(
                  endtile->GetName(transit_station->one_stop_offset()));
            }

            // Set station name if requested
            if (controller.attributes.at(kNodeTransitPlatformInfoStationName) &&
                transit_station->name_offset()) {
              transit_platform_info->set_station_name(
                  endtile->GetName(transit_station->name_offset()));
            }

            // only one de to station exists.  we are done.
            break;
          }
        }

        // Set latitude and longitude
        LatLng* stop_ll = transit_platform_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoLatLon)) {
          PointLL ll = node->latlng(start_tile->header()->base_ll());
          stop_ll->set_lat(ll.lat());
          stop_ll->set_lng(ll.lng());
        }
      }

      // Set the arrival time at this node (based on schedule from last trip
      // departure) if requested
      if (controller.attributes.at(kNodeTransitPlatformInfoArrivalDateTime) &&
          !arrival_time.empty()) {
        transit_platform_info->set_arrival_date_time(arrival_time);
      }

      // If this edge has a trip id then there is a transit departure
      if (trip_id) {

        const TransitDeparture* transit_departure =
            graphtile->GetTransitDeparture(graphtile->directededge(edge.id())->lineid(), trip_id,
                                           second_of_week % kSecondsPerDay);

        assumed_schedule = false;
        uint32_t date, day = 0;
        if (origin.has_date_time()) {
          date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(origin.date_time()));

          if (graphtile->header()->date_created() > date) {
            // Set assumed schedule if requested
            if (controller.attributes.at(kNodeTransitPlatformInfoAssumedSchedule)) {
              transit_platform_info->set_assumed_schedule(true);
            }
            assumed_schedule = true;
          } else {
            day = date - graphtile->header()->date_created();
            if (day > graphtile->GetTransitSchedule(transit_departure->schedule_index())->end_day()) {
              // Set assumed schedule if requested
              if (controller.attributes.at(kNodeTransitPlatformInfoAssumedSchedule)) {
                transit_platform_info->set_assumed_schedule(true);
              }
              assumed_schedule = true;
            }
          }
        }

        // TODO: all of the duration stuff below assumes the transit departure is on the same day as
        // the origin date time. if the trip took more than one day this will not be the case and
        // negative durations can occur
        if (transit_departure) {

          std::string dt = DateTime::get_duration(origin.date_time(),
                                                  (transit_departure->departure_time() -
                                                   (origin_second_of_week % kSecondsPerDay)),
                                                  DateTime::get_tz_db().from_index(node->timezone()));

          std::size_t found = dt.find_last_of(' '); // remove tz abbrev.
          if (found != std::string::npos) {
            dt = dt.substr(0, found);
          }

          // Set departure time from this transit stop if requested
          if (controller.attributes.at(kNodeTransitPlatformInfoDepartureDateTime)) {
            transit_platform_info->set_departure_date_time(dt);
          }

          // TODO:  set removed tz abbrev on transit_platform_info for departure.

          // Copy the arrival time for use at the next transit stop
          arrival_time = DateTime::get_duration(origin.date_time(),
                                                (transit_departure->departure_time() +
                                                 transit_departure->elapsed_time()) -
                                                    (origin_second_of_week % kSecondsPerDay),
                                                DateTime::get_tz_db().from_index(node->timezone()));

          found = arrival_time.find_last_of(' '); // remove tz abbrev.
          if (found != std::string::npos) {
            arrival_time = arrival_time.substr(0, found);
          }

          // TODO:  set removed tz abbrev on transit_platform_info for arrival.

          // Get the block Id
          block_id = transit_departure->blockid();
        }
      } else {
        // No departing trip, set the arrival time (for next stop) to empty
        // and set block Id to 0
        arrival_time = "";
        block_id = 0;

        // Set assumed schedule if requested
        if (controller.attributes.at(kNodeTransitPlatformInfoAssumedSchedule) && assumed_schedule) {
          transit_platform_info->set_assumed_schedule(true);
        }
        assumed_schedule = false;
      }
    }

    // Add edge to the trip node and set its attributes
    auto is_last_edge = edge_itr == (path_end - 1);
    float length_pct = (is_first_edge ? 1.f - start_pct : (is_last_edge ? end_pct : 1.f));
    TripLeg_Edge* trip_edge =
        AddTripEdge(controller, edge, trip_id, block_id, mode, travel_type, costing, directededge,
                    node->drive_on_right(), trip_node, graphtile, second_of_week, length_pct,
                    edge_itr->has_time_restrictions);

    // Get the shape and set shape indexes (directed edge forward flag
    // determines whether shape is traversed forward or reverse).
    auto edgeinfo = graphtile->edgeinfo(directededge->edgeinfo_offset());
    uint32_t begin_index = (is_first_edge) ? 0 : trip_shape.size() - 1;

    // Process the shape for edges where a route discontinuity occurs
    if (route_discontinuities && !route_discontinuities->empty() &&
        route_discontinuities->count(edge_index) > 0) {
      // Get edge shape and reverse it if directed edge is not forward.
      auto edge_shape = edgeinfo.shape();
      if (!directededge->forward()) {
        std::reverse(edge_shape.begin(), edge_shape.end());
      }

      // Grab the edge begin and end info
      auto& edge_begin_info = route_discontinuities->at(edge_index).first;
      auto& edge_end_info = route_discontinuities->at(edge_index).second;

      // Handle partial shape for first edge
      if (is_first_edge && !edge_begin_info.exists) {
        edge_begin_info.exists = true;
        edge_begin_info.distance_along = start_pct;
        edge_begin_info.vertex = start_vrt;
      }

      // Handle partial shape for last edge
      if (is_last_edge && !edge_end_info.exists) {
        edge_end_info.exists = true;
        edge_end_info.distance_along = end_pct;
        edge_end_info.vertex = end_vrt;
      }

      // Trim the shape
      auto edge_length = static_cast<float>(directededge->length());
      TrimShape(edge_shape, edge_begin_info.distance_along * edge_length, edge_begin_info.vertex,
                edge_end_info.distance_along * edge_length, edge_end_info.vertex);
      // Add edge shape to trip
      trip_shape.insert(trip_shape.end(),
                        (edge_shape.begin() + ((edge_begin_info.exists || is_first_edge) ? 0 : 1)),
                        edge_shape.end());

      // Adjust the length of the trimmed edge if this is a route (check if through_location exist).
      // Do not want to trim the edge length if this is part of map matching (TODO - verify).
      if (!through_loc.empty()) {
        trip_edge->set_length(edge_length * kKmPerMeter *
                              (edge_end_info.distance_along - edge_begin_info.distance_along));
      }

      // If edge_begin_info.exists and is not the first edge then increment begin_index since
      // the previous end shape index should not equal the current begin shape index because
      // of discontinuity
      if (edge_begin_info.exists && !is_first_edge) {
        ++begin_index;
      }

    } // We need to clip the shape if its at the beginning or end
    else if (is_first_edge || is_last_edge) {
      // Get edge shape and reverse it if directed edge is not forward.
      auto edge_shape = edgeinfo.shape();
      if (!directededge->forward()) {
        std::reverse(edge_shape.begin(), edge_shape.end());
      }
      float total = static_cast<float>(directededge->length());
      // Note: that this cannot be both the first and last edge, that special case is handled above
      // Trim the shape at the front for the first edge
      if (is_first_edge) {
        TrimShape(edge_shape, start_pct * total, start_vrt, total, edge_shape.back());
      }
      // And at the back if its the last edge
      else {
        TrimShape(edge_shape, 0, edge_shape.front(), end_pct * total, end_vrt);
      }
      // Keep the shape
      trip_shape.insert(trip_shape.end(), edge_shape.begin() + is_last_edge, edge_shape.end());
    } // Just get the shape in there in the right direction no clipping needed
    else {
      if (directededge->forward()) {
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().begin() + 1, edgeinfo.shape().end());
      } else {
        trip_shape.insert(trip_shape.end(), edgeinfo.shape().rbegin() + 1, edgeinfo.shape().rend());
      }
    }

    // Set begin shape index if requested
    if (controller.attributes.at(kEdgeBeginShapeIndex)) {
      trip_edge->set_begin_shape_index(begin_index);
    }

    // Set end shape index if requested
    if (controller.attributes.at(kEdgeEndShapeIndex)) {
      trip_edge->set_end_shape_index(trip_shape.size() - 1);
    }

    // Set shape attributes
    SetShapeAttributes(controller, graphtile, directededge, costing, trip_shape.begin() + begin_index,
                       trip_shape.end(), trip_path, second_of_week);

    // Set begin and end heading if requested. Uses trip_shape so
    // must be done after the edge's shape has been added.
    SetHeadings(trip_edge, controller, directededge, trip_shape, begin_index);

    // Add connected edges from the start node. Do this after the first trip
    // edge is added
    //
    // Our path is from 1 to 2 to 3 (nodes) to ... n nodes.
    // Each letter represents the edge info.
    // So at node 2, we will store the edge info for D and we will store the
    // intersecting edge info for B, C, E, F, and G.  We need to make sure
    // that we don't store the edge info from A and D again.
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
      // Iterate through edges on this level to find any intersecting edges
      // Follow any upwards or downward transitions
      const DirectedEdge* de = start_tile->directededge(node->edge_index());
      for (uint32_t idx1 = 0; idx1 < node->edge_count(); ++idx1, de++) {

        // Skip shortcut edges AND the opposing edge of the previous edge in the path AND
        // the current edge in the path AND the superceded edge of the current edge in the path
        // if the current edge in the path is a shortcut
        if (de->is_shortcut() || de->localedgeidx() == prior_opp_local_index ||
            de->localedgeidx() == directededge->localedgeidx() ||
            (directededge->is_shortcut() && directededge->shortcut() & de->superseded())) {
          continue;
        }

        // Add intersecting edges on the same hierarchy level and not on the path
        AddTripIntersectingEdge(controller, directededge, prev_de, de->localedgeidx(), node,
                                trip_node, de);
      }

      // Add intersecting edges on different levels (follow NodeTransitions)
      if (node->transition_count() > 0) {
        const NodeTransition* trans = start_tile->transition(node->transition_index());
        for (uint32_t i = 0; i < node->transition_count(); ++i, ++trans) {
          // Get the end node tile and its directed edges
          GraphId endnode = trans->endnode();
          const GraphTile* endtile = graphreader.GetGraphTile(endnode);
          if (endtile == nullptr) {
            continue;
          }
          const NodeInfo* nodeinfo2 = endtile->node(endnode);
          const DirectedEdge* de2 = endtile->directededge(nodeinfo2->edge_index());
          for (uint32_t idx2 = 0; idx2 < nodeinfo2->edge_count(); ++idx2, de2++) {
            // Skip shortcut edges and edges on the path
            if (de2->is_shortcut() || de2->localedgeidx() == prior_opp_local_index ||
                de2->localedgeidx() == directededge->localedgeidx()) {
              continue;
            }
            AddTripIntersectingEdge(controller, directededge, prev_de, de2->localedgeidx(), nodeinfo2,
                                    trip_node, de2);
          }
        }
      }
    }

    // Update elapsed time at the end of the edge, store this at the next node.
    elapsedtime = edge_itr->elapsed_time;

    // Set the endnode of this directed edge as the startnode of the next edge.
    startnode = directededge->endnode();

    if (!directededge->IsTransitLine()) {
      // Save the opposing edge as the previous DirectedEdge (for name consistency)
      const GraphTile* t2 =
          directededge->leaves_tile() ? graphreader.GetGraphTile(directededge->endnode()) : graphtile;
      if (t2 == nullptr) {
        continue;
      }
      GraphId oppedge = t2->GetOpposingEdgeId(directededge);
      prev_de = t2->directededge(oppedge);
    }

    // Save the index of the opposing local directed edge at the end node
    prior_opp_local_index = directededge->opp_local_idx();

    // set is_first edge to false
    is_first_edge = false;
  }

  // Add the last node
  auto* node = trip_path.add_node();
  if (controller.attributes.at(kNodeaAdminIndex)) {
    auto* last_tile = graphreader.GetGraphTile(startnode);
    node->set_admin_index(
        GetAdminIndex(last_tile->admininfo(last_tile->node(startnode)->admin_index()), admin_info_map,
                      admin_info_list));
  }
  if (controller.attributes.at(kNodeElapsedTime)) {
    node->set_elapsed_time(elapsedtime);
  }

  // Assign the admins
  AssignAdmins(controller, trip_path, admin_info_list);

  // Set the bounding box of the shape
  SetBoundingBox(trip_path, trip_shape);

  // Set shape if requested
  if (controller.attributes.at(kShape)) {
    trip_path.set_shape(encode<std::vector<PointLL>>(trip_shape));
  }

  if (osmchangeset != 0 && controller.attributes.at(kOsmChangeset)) {
    trip_path.set_osm_changeset(osmchangeset);
  }

  // hand it back
  return trip_path;
}

} // namespace thor
} // namespace valhalla
