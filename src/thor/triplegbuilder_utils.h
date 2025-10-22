#include "baldr/attributes_controller.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/time_info.h"
#include "proto/trip.pb.h"
#include "sif/dynamiccost.h"

#include <utility>

namespace {

/**
 * A simple way to encapsulate the marshalling of native data types specific to multi modal into
 * the protobuf ones that we pass downstream to odin and other clients
 */
struct MultimodalBuilder {
  MultimodalBuilder(const valhalla::Location& origin, const valhalla::baldr::TimeInfo& time_info)
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
             const valhalla::baldr::NodeInfo* node,
             const valhalla::baldr::GraphId& startnode,
             const valhalla::baldr::DirectedEdge* directededge,
             const valhalla::baldr::GraphId& edge,
             valhalla::baldr::graph_tile_ptr start_tile,
             valhalla::baldr::graph_tile_ptr graphtile,
             const valhalla::sif::mode_costing_t& mode_costing,
             const valhalla::baldr::AttributesController& controller,
             valhalla::baldr::GraphReader& graphreader) {

    AddBssNode(trip_node, node, directededge, start_tile, mode_costing, controller);
    AddTransitNodes(trip_node, node, startnode, std::move(start_tile), std::move(graphtile),
                    controller);
    AddTransitInfo(trip_node, trip_id, node, startnode, directededge, edge, start_tile, graphtile,
                   mode_costing, controller, graphreader);
  }

private:
  /**
   *
   * @param trip_node
   * @param node
   * @param directed_edge
   * @param start_tile
   * @param mode_costing
   * @param controller
   */
  void AddBssNode(valhalla::TripLeg_Node* trip_node,
                  const valhalla::baldr::NodeInfo* node,
                  const valhalla::baldr::DirectedEdge* directededge,
                  const valhalla::baldr::graph_tile_ptr& start_tile,
                  const valhalla::sif::mode_costing_t& mode_costing,
                  const valhalla::baldr::AttributesController&) {

    auto pedestrian_costing =
        mode_costing[static_cast<size_t>(valhalla::sif::travel_mode_t::kPedestrian)];
    auto bicycle_costing = mode_costing[static_cast<size_t>(valhalla::sif::travel_mode_t::kBicycle)];

    if (node->type() == valhalla::baldr::NodeType::kBikeShare && pedestrian_costing &&
        bicycle_costing) {
      valhalla::baldr::EdgeInfo edgeinfo = start_tile->edgeinfo(directededge);
      auto taggedValue = edgeinfo.GetTags();

      auto* bss_station_info = trip_node->mutable_bss_info();
      // TODO: import more BSS data, can be used to display capacity in real time
      auto tag_range = taggedValue.equal_range(valhalla::baldr::TaggedValue::kBssInfo);
      if (tag_range.first != tag_range.second) {
        bss_station_info->ParseFromString(tag_range.first->second);
      }
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
  void AddTransitNodes(valhalla::TripLeg_Node* trip_node,
                       const valhalla::baldr::NodeInfo* node,
                       const valhalla::baldr::GraphId& startnode,
                       const valhalla::baldr::graph_tile_ptr& start_tile,
                       const valhalla::baldr::graph_tile_ptr& graphtile,
                       const valhalla::baldr::AttributesController& controller) {

    if (node->type() == valhalla::baldr::NodeType::kTransitStation) {
      const valhalla::baldr::TransitStop* transit_station =
          start_tile->GetTransitStop(start_tile->node(startnode)->stop_index());
      valhalla::TransitStationInfo* transit_station_info = trip_node->mutable_transit_station_info();

      if (transit_station) {
        // Set onstop_id if requested
        if (controller(valhalla::baldr::kNodeTransitStationInfoOnestopId) &&
            transit_station->one_stop_offset()) {
          transit_station_info->set_onestop_id(
              graphtile->GetName(transit_station->one_stop_offset()));
        }

        // Set name if requested
        if (controller(valhalla::baldr::kNodeTransitStationInfoName) &&
            transit_station->name_offset()) {
          transit_station_info->set_name(graphtile->GetName(transit_station->name_offset()));
        }

        // Set latitude and longitude
        valhalla::LatLng* stop_ll = transit_station_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller(valhalla::baldr::kNodeTransitStationInfoLatLon)) {
          valhalla::midgard::PointLL ll = node->latlng(start_tile->header()->base_ll());
          stop_ll->set_lat(ll.lat());
          stop_ll->set_lng(ll.lng());
        }
      }
    }

    if (node->type() == valhalla::baldr::NodeType::kTransitEgress) {
      const valhalla::baldr::TransitStop* transit_egress =
          start_tile->GetTransitStop(start_tile->node(startnode)->stop_index());
      valhalla::TransitEgressInfo* transit_egress_info = trip_node->mutable_transit_egress_info();

      if (transit_egress) {
        // Set onstop_id if requested
        if (controller(valhalla::baldr::kNodeTransitEgressInfoOnestopId) &&
            transit_egress->one_stop_offset()) {
          transit_egress_info->set_onestop_id(graphtile->GetName(transit_egress->one_stop_offset()));
        }

        // Set name if requested
        if (controller(valhalla::baldr::kNodeTransitEgressInfoName) &&
            transit_egress->name_offset()) {
          transit_egress_info->set_name(graphtile->GetName(transit_egress->name_offset()));
        }

        // Set latitude and longitude
        valhalla::LatLng* stop_ll = transit_egress_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller(valhalla::baldr::kNodeTransitEgressInfoLatLon)) {
          valhalla::midgard::PointLL ll = node->latlng(start_tile->header()->base_ll());
          stop_ll->set_lat(ll.lat());
          stop_ll->set_lng(ll.lng());
        }
      }
    }
  }

  void AddTransitInfo(valhalla::TripLeg_Node* trip_node,
                      uint32_t trip_id,
                      const valhalla::baldr::NodeInfo* node,
                      const valhalla::baldr::GraphId&,
                      const valhalla::baldr::DirectedEdge* directededge,
                      const valhalla::baldr::GraphId& edge,
                      const valhalla::baldr::graph_tile_ptr& start_tile,
                      const valhalla::baldr::graph_tile_ptr& graphtile,
                      const valhalla::sif::mode_costing_t&,
                      const valhalla::baldr::AttributesController& controller,
                      valhalla::baldr::GraphReader& graphreader) {
    if (node->is_transit()) {
      // Get the transit stop information and add transit stop info
      const valhalla::baldr::TransitStop* transit_platform =
          start_tile->GetTransitStop(node->stop_index());
      valhalla::TransitPlatformInfo* transit_platform_info =
          trip_node->mutable_transit_platform_info();

      // TODO: for now we will set to station for rail and stop for others
      //   not sure how to deal with this in the future: maybe assume it'll be
      //   station for rail if there's a station and platform if not..
      // Set type
      if (directededge->use() == valhalla::baldr::Use::kRail) {
        // Set node transit info type if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(valhalla::TransitPlatformInfo_Type_kStation);
        }
        prev_transit_node_type = valhalla::TransitPlatformInfo_Type_kStation;
      } else if (directededge->use() == valhalla::baldr::Use::kPlatformConnection) {
        // Set node transit info type if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(prev_transit_node_type);
        }
      } else { // bus logic
        // Set node transit info type if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(valhalla::TransitPlatformInfo_Type_kStop);
        }
        prev_transit_node_type = valhalla::TransitPlatformInfo_Type_kStop;
      }

      if (transit_platform) {
        // Set onstop_id if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoOnestopId) &&
            transit_platform->one_stop_offset()) {
          transit_platform_info->set_onestop_id(
              graphtile->GetName(transit_platform->one_stop_offset()));
        }

        // Set name if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoName) &&
            transit_platform->name_offset()) {
          transit_platform_info->set_name(graphtile->GetName(transit_platform->name_offset()));
        }

        // save station name and info for all platforms.
        const valhalla::baldr::DirectedEdge* dir_edge = start_tile->directededge(node->edge_index());
        for (uint32_t index = 0; index < node->edge_count(); ++index, dir_edge++) {
          if (dir_edge->use() == valhalla::baldr::Use::kPlatformConnection) {
            valhalla::baldr::GraphId endnode = dir_edge->endnode();
            valhalla::baldr::graph_tile_ptr endtile = graphreader.GetGraphTile(endnode);
            const valhalla::baldr::NodeInfo* nodeinfo2 = endtile->node(endnode);
            const valhalla::baldr::TransitStop* transit_station =
                endtile->GetTransitStop(nodeinfo2->stop_index());

            // Set station onstop_id if requested
            if (controller(valhalla::baldr::kNodeTransitPlatformInfoStationOnestopId) &&
                transit_station->one_stop_offset()) {
              transit_platform_info->set_station_onestop_id(
                  endtile->GetName(transit_station->one_stop_offset()));
            }

            // Set station name if requested
            if (controller(valhalla::baldr::kNodeTransitPlatformInfoStationName) &&
                transit_station->name_offset()) {
              transit_platform_info->set_station_name(
                  endtile->GetName(transit_station->name_offset()));
            }

            // only one de to station exists.  we are done.
            break;
          }
        }

        // Set latitude and longitude
        valhalla::LatLng* stop_ll = transit_platform_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoLatLon)) {
          valhalla::midgard::PointLL ll = node->latlng(start_tile->header()->base_ll());
          stop_ll->set_lat(ll.lat());
          stop_ll->set_lng(ll.lng());
        }
      }

      // Set the arrival time at this node (based on schedule from last trip
      // departure) if requested
      if (controller(valhalla::baldr::kNodeTransitPlatformInfoArrivalDateTime) &&
          !arrival_time.empty()) {
        transit_platform_info->set_arrival_date_time(arrival_time);
      }

      // If this edge has a trip id then there is a transit departure
      if (trip_id) {

        const valhalla::baldr::TransitDeparture* transit_departure =
            graphtile->GetTransitDeparture(graphtile->directededge(edge.id())->lineid(), trip_id,
                                           time_info.second_of_week %
                                               valhalla::midgard::kSecondsPerDay);

        assumed_schedule = false;
        uint32_t origin_pivot_days, days_from_creation;
        if (!origin.date_time().empty()) {
          origin_pivot_days = valhalla::baldr::DateTime::days_from_pivot_date(
              valhalla::baldr::DateTime::get_formatted_date(origin.date_time()));
          days_from_creation = origin_pivot_days - graphtile->header()->date_created();

          // if the departure is in the past or too far in the future, we flag the schedule "assumed"
          if (graphtile->header()->date_created() > origin_pivot_days ||
              days_from_creation >
                  graphtile->GetTransitSchedule(transit_departure->schedule_index())->end_day()) {
            // Set assumed schedule if requested
            if (controller(valhalla::baldr::kNodeTransitPlatformInfoAssumedSchedule)) {
              transit_platform_info->set_assumed_schedule(true);
            }
            assumed_schedule = true;
          }
        }

        // TODO: all of the duration stuff below assumes the transit departure is on the same day as
        // the origin date time. if the trip took more than one day this will not be the case and
        // negative durations can occur
        if (transit_departure) {
          // round up the transit times to full minutes because date_time() will always round down
          // TODO: do (optional) seconds resolution for the input & output so that this becomes robust
          auto round_up_mins = [](uint32_t seconds) {
            auto remainder = seconds % valhalla::midgard::kSecondsPerMinute;
            return remainder ? seconds + (valhalla::midgard::kSecondsPerMinute - remainder) : seconds;
          };
          // round up the waiting time to full minutes, bcs time_info.date_time() floors minutes
          std::string dt =
              valhalla::baldr::DateTime::get_duration(time_info.date_time(),
                                                      round_up_mins(
                                                          transit_departure->departure_time() -
                                                          time_info.day_seconds()),
                                                      valhalla::baldr::DateTime::get_tz_db()
                                                          .from_index(node->timezone()));

          std::size_t found = dt.find_last_of(' '); // remove tz abbrev.
          if (found != std::string::npos) {
            dt = dt.substr(0, found);
          }

          // Set departure time from this transit stop if requested
          if (controller(valhalla::baldr::kNodeTransitPlatformInfoDepartureDateTime)) {
            transit_platform_info->set_departure_date_time(dt);
          }

          // TODO:  set removed tz abbrev on transit_platform_info for departure.

          // Copy the arrival time for use at the next transit stop
          arrival_time =
              valhalla::baldr::DateTime::get_duration(time_info.date_time(),
                                                      round_up_mins(
                                                          (transit_departure->departure_time() +
                                                           transit_departure->elapsed_time()) -
                                                          time_info.day_seconds()),
                                                      valhalla::baldr::DateTime::get_tz_db()
                                                          .from_index(node->timezone()));

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
        if (controller(valhalla::baldr::kNodeTransitPlatformInfoAssumedSchedule) &&
            assumed_schedule) {
          transit_platform_info->set_assumed_schedule(true);
        }
        assumed_schedule = false;
      }
    }
  }
};

} // namespace
