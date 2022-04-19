#include "baldr/attributes_controller.h"
#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/time_info.h"
#include "proto/api.pb.h"

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

    AddBssNode(trip_node, node, directededge, start_tile, mode_costing, controller);
    AddTransitNodes(trip_node, node, startnode, start_tile, graphtile, controller);
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
  void AddBssNode(TripLeg_Node* trip_node,
                  const NodeInfo* node,
                  const DirectedEdge* directededge,
                  graph_tile_ptr start_tile,
                  const mode_costing_t& mode_costing,
                  const AttributesController&) {

    auto pedestrian_costing = mode_costing[static_cast<size_t>(travel_mode_t::kPedestrian)];
    auto bicycle_costing = mode_costing[static_cast<size_t>(travel_mode_t::kBicycle)];

    if (node->type() == NodeType::kBikeShare && pedestrian_costing && bicycle_costing) {

      EdgeInfo edgeinfo = start_tile->edgeinfo(directededge);
      auto taggedValue = edgeinfo.GetTags();

      auto* bss_station_info = trip_node->mutable_bss_info();
      // TODO: import more BSS data, can be used to display capacity in real time
      auto tag_range = taggedValue.equal_range(baldr::TaggedValue::kBssInfo);
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
        if (controller(kNodeTransitStationInfoOnestopId) && transit_station->one_stop_offset()) {
          transit_station_info->set_onestop_id(
              graphtile->GetName(transit_station->one_stop_offset()));
        }

        // Set name if requested
        if (controller(kNodeTransitStationInfoName) && transit_station->name_offset()) {
          transit_station_info->set_name(graphtile->GetName(transit_station->name_offset()));
        }

        // Set latitude and longitude
        LatLng* stop_ll = transit_station_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller(kNodeTransitStationInfoLatLon)) {
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
        if (controller(kNodeTransitEgressInfoOnestopId) && transit_egress->one_stop_offset()) {
          transit_egress_info->set_onestop_id(graphtile->GetName(transit_egress->one_stop_offset()));
        }

        // Set name if requested
        if (controller(kNodeTransitEgressInfoName) && transit_egress->name_offset()) {
          transit_egress_info->set_name(graphtile->GetName(transit_egress->name_offset()));
        }

        // Set latitude and longitude
        LatLng* stop_ll = transit_egress_info->mutable_ll();
        // Set transit stop lat/lon if requested
        if (controller(kNodeTransitEgressInfoLatLon)) {
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
        if (controller(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(TransitPlatformInfo_Type_kStation);
        }
        prev_transit_node_type = TransitPlatformInfo_Type_kStation;
      } else if (directededge->use() == Use::kPlatformConnection) {
        // Set node transit info type if requested
        if (controller(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(prev_transit_node_type);
        }
      } else { // bus logic
        // Set node transit info type if requested
        if (controller(kNodeTransitPlatformInfoType)) {
          transit_platform_info->set_type(TransitPlatformInfo_Type_kStop);
        }
        prev_transit_node_type = TransitPlatformInfo_Type_kStop;
      }

      if (transit_platform) {
        // Set onstop_id if requested
        if (controller(kNodeTransitPlatformInfoOnestopId) && transit_platform->one_stop_offset()) {
          transit_platform_info->set_onestop_id(
              graphtile->GetName(transit_platform->one_stop_offset()));
        }

        // Set name if requested
        if (controller(kNodeTransitPlatformInfoName) && transit_platform->name_offset()) {
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
            if (controller(kNodeTransitPlatformInfoStationOnestopId) &&
                transit_station->one_stop_offset()) {
              transit_platform_info->set_station_onestop_id(
                  endtile->GetName(transit_station->one_stop_offset()));
            }

            // Set station name if requested
            if (controller(kNodeTransitPlatformInfoStationName) && transit_station->name_offset()) {
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
        if (controller(kNodeTransitPlatformInfoLatLon)) {
          PointLL ll = node->latlng(start_tile->header()->base_ll());
          stop_ll->set_lat(ll.lat());
          stop_ll->set_lng(ll.lng());
        }
      }

      // Set the arrival time at this node (based on schedule from last trip
      // departure) if requested
      if (controller(kNodeTransitPlatformInfoArrivalDateTime) && !arrival_time.empty()) {
        transit_platform_info->set_arrival_date_time(arrival_time);
      }

      // If this edge has a trip id then there is a transit departure
      if (trip_id) {

        const TransitDeparture* transit_departure =
            graphtile->GetTransitDeparture(graphtile->directededge(edge.id())->lineid(), trip_id,
                                           time_info.second_of_week % kSecondsPerDay);

        assumed_schedule = false;
        uint32_t date, day = 0;
        if (!origin.date_time().empty()) {
          date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(origin.date_time()));

          if (graphtile->header()->date_created() > date) {
            // Set assumed schedule if requested
            if (controller(kNodeTransitPlatformInfoAssumedSchedule)) {
              transit_platform_info->set_assumed_schedule(true);
            }
            assumed_schedule = true;
          } else {
            day = date - graphtile->header()->date_created();
            if (day > graphtile->GetTransitSchedule(transit_departure->schedule_index())->end_day()) {
              // Set assumed schedule if requested
              if (controller(kNodeTransitPlatformInfoAssumedSchedule)) {
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
          if (controller(kNodeTransitPlatformInfoDepartureDateTime)) {
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
        if (controller(kNodeTransitPlatformInfoAssumedSchedule) && assumed_schedule) {
          transit_platform_info->set_assumed_schedule(true);
        }
        assumed_schedule = false;
      }
    }
  }
};

} // namespace
