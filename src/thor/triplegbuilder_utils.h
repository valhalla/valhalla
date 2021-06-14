#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/time_info.h"
#include "proto/api.pb.h"
#include "thor/attributes_controller.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace {

/**
 * A simple way to encapsulate the marshalling of native data types specific to multi modal into
 * the protobuf ones that we pass downstream to odin and other clients
 */
struct MultimodalBuilder {
  MultimodalBuilder(const valhalla::Location& origin, const TimeInfo& time_info)
      : origin(origin), time_info(time_info), arrival_time{}, block_id{0}, assumed_schedule{false},
        prev_transit_node_type{valhalla::TransitPlatformInfo::kStop} {
  }
  const valhalla::Location& origin;
  const valhalla::baldr::TimeInfo& time_info;
  std::string arrival_time;
  uint32_t block_id;
  bool assumed_schedule;
  // TODO: this is temp until we use transit stop type from transitland
  valhalla::TransitPlatformInfo::Type prev_transit_node_type;

  void Build(valhalla::TripLeg::Node* trip_node,
             uint32_t trip_id,
             const NodeInfo* node,
             const GraphId& startnode,
             const DirectedEdge* directededge,
             const GraphId& edge,
             graph_tile_ptr start_tile,
             graph_tile_ptr graphtile,
             const mode_costing_t& mode_costing,
             const AttributesController& controller,
             GraphReader& graphreader) {
    AddBssNode(trip_node, node, startnode, mode_costing, controller);
    AddTransitNodes(trip_node, node, startnode, start_tile, graphtile, controller);
    AddTransitInfo(trip_node, trip_id, node, startnode, directededge, edge, start_tile, graphtile,
                   mode_costing, controller, graphreader);
  }

private:
  /**
   *
   * @param trip_node
   * @param node
   * @param startnode
   * @param start_tile
   * @param graphtile
   * @param mode_costing
   * @param controller
   */
  void AddBssNode(TripLeg_Node* trip_node,
                  const NodeInfo* node,
                  const GraphId&,
                  const mode_costing_t& mode_costing,
                  const AttributesController&) {
    auto pedestrian_costing = mode_costing[static_cast<size_t>(TravelMode::kPedestrian)];
    auto bicycle_costing = mode_costing[static_cast<size_t>(TravelMode::kBicycle)];

    if (node->type() == NodeType::kBikeShare && pedestrian_costing && bicycle_costing) {
      auto* bss_station_info = trip_node->mutable_bss_info();
      // TODO: import more BSS data, can be used to display capacity in real time
      bss_station_info->set_name("BSS 42");
      bss_station_info->set_ref("BSS 42 ref");
      bss_station_info->set_capacity("42");
      bss_station_info->set_network("universe");
      bss_station_info->set_operator_("Douglas");
      bss_station_info->set_rent_cost(pedestrian_costing->BSSCost().secs);
      bss_station_info->set_return_cost(bicycle_costing->BSSCost().secs);
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
                       graph_tile_ptr start_tile,
                       graph_tile_ptr graphtile,
                       const AttributesController& controller) {

    if (node->type() == NodeType::kTransitStation) {
      const TransitStop* transit_station =
          start_tile->GetTransitStop(start_tile->node(startnode)->stop_index());
      TransitStationInfo* transit_station_info = trip_node->mutable_transit_station_info();

      if (transit_station) {
        // Set onstop_id if requested
        if (controller.attributes.at(kNodeTransitStationInfoOnestopId) &&
            transit_station->one_stop_offset()) {
          transit_station_info->set_onestop_id(
              graphtile->GetName(transit_station->one_stop_offset()));
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

  void AddTransitInfo(TripLeg_Node* trip_node,
                      uint32_t trip_id,
                      const NodeInfo* node,
                      const GraphId&,
                      const DirectedEdge* directededge,
                      const GraphId& edge,
                      graph_tile_ptr start_tile,
                      graph_tile_ptr graphtile,
                      const sif::mode_costing_t&,
                      const AttributesController& controller,
                      GraphReader& graphreader) {
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
            graph_tile_ptr endtile = graphreader.GetGraphTile(endnode);
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
                                           time_info.second_of_week % kSecondsPerDay);

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
                                                   (time_info.second_of_week % kSecondsPerDay)),
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
                                                    (time_info.second_of_week % kSecondsPerDay),
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
  }
};

static void AddSignInfo(const AttributesController& controller,
                        const std::vector<SignInfo>& edge_signs,
                        TripLeg_Sign* trip_sign) {

  if (!edge_signs.empty()) {
    for (const auto &sign : edge_signs) {
      switch (sign.type()) {
        case Sign::Type::kExitNumber: {
          if (controller.attributes.at(kEdgeSignExitNumber)) {
            auto *trip_sign_exit_number = trip_sign->mutable_exit_numbers()->Add();
            trip_sign_exit_number->set_text(sign.text());
            trip_sign_exit_number->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kExitBranch: {
          if (controller.attributes.at(kEdgeSignExitBranch)) {
            auto *trip_sign_exit_onto_street = trip_sign->mutable_exit_onto_streets()->Add();
            trip_sign_exit_onto_street->set_text(sign.text());
            trip_sign_exit_onto_street->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kExitToward: {
          if (controller.attributes.at(kEdgeSignExitToward)) {
            auto *trip_sign_exit_toward_location =
                trip_sign->mutable_exit_toward_locations()->Add();
            trip_sign_exit_toward_location->set_text(sign.text());
            trip_sign_exit_toward_location->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kExitName: {
          if (controller.attributes.at(kEdgeSignExitName)) {
            auto *trip_sign_exit_name = trip_sign->mutable_exit_names()->Add();
            trip_sign_exit_name->set_text(sign.text());
            trip_sign_exit_name->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kGuideBranch: {
          if (controller.attributes.at(kEdgeSignGuideBranch)) {
            auto *trip_sign_guide_onto_street = trip_sign->mutable_guide_onto_streets()->Add();
            trip_sign_guide_onto_street->set_text(sign.text());
            trip_sign_guide_onto_street->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kGuideToward: {
          if (controller.attributes.at(kEdgeSignGuideToward)) {
            auto *trip_sign_guide_toward_location =
                trip_sign->mutable_guide_toward_locations()->Add();
            trip_sign_guide_toward_location->set_text(sign.text());
            trip_sign_guide_toward_location->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kGuidanceViewJunction: {
          if (controller.attributes.at(kEdgeSignGuidanceViewJunction)) {
            auto *trip_sign_guidance_view_junction =
                trip_sign->mutable_guidance_view_junctions()->Add();
            trip_sign_guidance_view_junction->set_text(sign.text());
            trip_sign_guidance_view_junction->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        case Sign::Type::kGuidanceViewSignboard: {
          if (controller.attributes.at(kEdgeSignGuidanceViewSignboard)) {
            auto *trip_sign_guidance_view_signboard =
                trip_sign->mutable_guidance_view_signboards()->Add();
            trip_sign_guidance_view_signboard->set_text(sign.text());
            trip_sign_guidance_view_signboard->set_is_route_number(sign.is_route_num());
          }
          break;
        }
        default: {
          break;
        }
      }
    }
  }
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
                             valhalla::baldr::GraphReader& graphreader,
                             const graph_tile_ptr& graphtile,
                             const DirectedEdge* directededge,
                             const DirectedEdge* prev_de,
                             uint32_t local_edge_index,
                             const NodeInfo* nodeinfo,
                             TripLeg_Node* trip_node,
                             const DirectedEdge* intersecting_de) {
  TripLeg_IntersectingEdge* intersecting_edge = trip_node->add_intersecting_edge();

  // Set the heading for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeBeginHeading)) {
    intersecting_edge->set_begin_heading(nodeinfo->heading(local_edge_index));
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
    intersecting_edge->set_walkability(GetTripLegTraversability(traversability));
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
    intersecting_edge->set_cyclability(GetTripLegTraversability(traversability));
  }

  // Set the driveability flag for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeDriveability)) {
    intersecting_edge->set_driveability(
        GetTripLegTraversability(nodeinfo->local_driveability(local_edge_index)));
  }

  // Set the previous/intersecting edge name consistency if requested
  if (controller.attributes.at(kNodeIntersectingEdgeFromEdgeNameConsistency)) {
    bool name_consistency =
        (prev_de == nullptr) ? false : prev_de->name_consistency(local_edge_index);
    intersecting_edge->set_prev_name_consistency(name_consistency);
  }

  // Set the current/intersecting edge name consistency if requested
  if (controller.attributes.at(kNodeIntersectingEdgeToEdgeNameConsistency)) {
    intersecting_edge->set_curr_name_consistency(directededge->name_consistency(local_edge_index));
  }

  // Set the use for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeUse)) {
    intersecting_edge->set_use(GetTripLegUse(intersecting_de->use()));
  }

  // Set the road class for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeRoadClass)) {
    intersecting_edge->set_road_class(GetRoadClass(intersecting_de->classification()));
  }

  // Set the lane count for the intersecting edge if requested
  if (controller.attributes.at(kNodeIntersectingEdgeLaneCount)) {
    intersecting_edge->set_lane_count(intersecting_de->lanecount());
  }

  if (intersecting_de->use() == Use::kRestArea) {
    GraphId endnode = intersecting_de->endnode();
    valhalla::baldr::graph_tile_ptr t2 = graphreader.GetGraphTile(endnode);

//    PointLL pt = t2->get_node_ll(endnode);
//    printf("pt: %.6f, %.6f\n", pt.lat(), pt.lng());
//    printf("Rest area:\n");
//    printf("t2->id().level() = %d\n", t2->id().level());
//    printf("t2->id().tileid() = %d\n", t2->id().tileid());

    const DirectedEdge * de0 = t2->directededge(0);
    if (intersecting_de->sign()) {
      size_t idx = intersecting_de - de0;
      std::vector<SignInfo> edge_signs = t2->GetSigns(idx);
      if (!edge_signs.empty()) {
        TripLeg_Sign* trip_sign = intersecting_edge->mutable_sign();
        AddSignInfo(controller, edge_signs, trip_sign);
      }
    }
  }
}

/**
 * Adds the intersecting edges in the graph at the current node. Skips edges which are on the path
 * as well as those which are duplicates due to shortcut edges.
 * @param controller               tells us what info we should add about the intersecting edges
 * @param start_tile               the tile which contains the node
 * @param node                     the node at which we are copying intersecting edges
 * @param directededge             the current edge leaving the current node in the path
 * @param prev_de                  the previous edge in the path
 * @param prior_opp_local_index    opposing edge local index of previous edge in the path
 * @param graphreader              graph reader for graph access
 * @param trip_node                pbf node in the pbf structure we are building
 */
void AddIntersectingEdges(const AttributesController& controller,
                          const graph_tile_ptr& start_tile,
                          const NodeInfo* node,
                          const DirectedEdge* directededge,
                          const DirectedEdge* prev_de,
                          uint32_t prior_opp_local_index,
                          GraphReader& graphreader,
                          valhalla::TripLeg::Node* trip_node) {
  /* Add connected edges from the start node. Do this after the first trip
     edge is added

     Our path is from 1 to 2 to 3 (nodes) to ... n nodes.
     Each letter represents the edge info.
     So at node 2, we will store the edge info for D and we will store the
     intersecting edge info for B, C, E, F, and G.  We need to make sure
     that we don't store the edge info from A and D again.

         (X)    (3)   (X)
           \\   ||   //
          C \\ D|| E//
             \\ || //
          B   \\||//   F
     (X)======= (2) ======(X)
                ||\\
              A || \\ G
                ||  \\
                (1)  (X)
  */

  // prepare for some edges
  trip_node->mutable_intersecting_edge()->Reserve(node->local_edge_count());

  // Iterate through edges on this level to find any intersecting edges
  // Follow any upwards or downward transitions
  const DirectedEdge* intersecting_edge = start_tile->directededge(node->edge_index());
  for (uint32_t idx1 = 0; idx1 < node->edge_count(); ++idx1, intersecting_edge++) {

    // Skip shortcut edges AND the opposing edge of the previous edge in the path AND
    // the current edge in the path AND the superceded edge of the current edge in the path
    // if the current edge in the path is a shortcut
    if (intersecting_edge->is_shortcut() || intersecting_edge->localedgeidx() == prior_opp_local_index ||
        intersecting_edge->localedgeidx() == directededge->localedgeidx() ||
        (directededge->is_shortcut() && directededge->shortcut() & intersecting_edge->superseded())) {
      continue;
    }

    // Add intersecting edges on the same hierarchy level and not on the path
    AddTripIntersectingEdge(controller, graphreader, start_tile, directededge, prev_de,
                            intersecting_edge->localedgeidx(), node, trip_node, intersecting_edge);
  }

  // Add intersecting edges on different levels (follow NodeTransitions)
  if (node->transition_count() > 0) {
    const NodeTransition* trans = start_tile->transition(node->transition_index());
    for (uint32_t i = 0; i < node->transition_count(); ++i, ++trans) {
      // Get the end node tile and its directed edges
      GraphId endnode = trans->endnode();
      graph_tile_ptr endtile = graphreader.GetGraphTile(endnode);
      if (endtile == nullptr) {
        continue;
      }
      const NodeInfo* nodeinfo2 = endtile->node(endnode);
      const DirectedEdge* intersecting_edge2 = endtile->directededge(nodeinfo2->edge_index());
      for (uint32_t idx2 = 0; idx2 < nodeinfo2->edge_count(); ++idx2, intersecting_edge2++) {
        // Skip shortcut edges and edges on the path
        if (intersecting_edge2->is_shortcut() || intersecting_edge2->localedgeidx() == prior_opp_local_index ||
            intersecting_edge2->localedgeidx() == directededge->localedgeidx()) {
          continue;
        }

        AddTripIntersectingEdge(controller, graphreader, start_tile, directededge, prev_de,
                                intersecting_edge2->localedgeidx(), nodeinfo2, trip_node, intersecting_edge2);
      }
    }
  }
}

} // namespace
