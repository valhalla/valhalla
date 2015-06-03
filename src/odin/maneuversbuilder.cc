#include <valhalla/midgard/util.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/turn.h>
#include <valhalla/baldr/streetnames.h>
#include <valhalla/baldr/streetnames_us.h>
#include "valhalla/baldr/streetnames_factory.h"
#include <proto/tripdirections.pb.h>
#include <proto/directions_options.pb.h>
#include <valhalla/odin/maneuversbuilder.h>
#include <valhalla/odin/signs.h>
#include <valhalla/odin/sign.h>

#include <iostream>
#include <algorithm>
#include <functional>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/format.hpp"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;

namespace {
void SortExitSignList(std::vector<Sign>* signs) {
  // Sort signs by descending consecutive count order
  std::sort(signs->begin(), signs->end(), [](Sign a, Sign b) {
    return b.consecutive_count() < a.consecutive_count();
  });
}

void CountAndSortExitSignList(std::vector<Sign>* prev_signs,
                              std::vector<Sign>* curr_signs) {
  // Increment count for consecutive exit signs
  for (Sign& curr_sign : *curr_signs) {
    for (Sign& prev_sign : *prev_signs) {
      if (curr_sign.text() == prev_sign.text()) {
        curr_sign.set_consecutive_count(curr_sign.consecutive_count() + 1);
        prev_sign.set_consecutive_count(curr_sign.consecutive_count());
      }
    }
  }

  // Sort the previous and current exit signs by descending consecutive count
  SortExitSignList(prev_signs);
  SortExitSignList(curr_signs);
}

}

namespace valhalla {
namespace odin {

ManeuversBuilder::ManeuversBuilder(const DirectionsOptions& directions_options,
                                   EnhancedTripPath* etp)
    : directions_options_(directions_options),
      trip_path_(etp) {
}

std::list<Maneuver> ManeuversBuilder::Build() {
  // Create the maneuvers
  std::list<Maneuver> maneuvers = Produce();

#ifdef LOGGING_LEVEL_TRACE
  int man_id = 1;
  LOG_TRACE("############################################");
  LOG_TRACE("MANEUVERS");
  for (const Maneuver& maneuver : maneuvers) {
    LOG_TRACE("---------------------------------------------");
    LOG_TRACE(std::to_string(man_id++) + ":  ");
    LOG_TRACE(std::string("  maneuver_PARAMETERS=") + maneuver.ToParameterString());
    LOG_TRACE(std::string("  maneuver=") + maneuver.ToString());
  }
#endif

  // Combine maneuvers
  Combine(maneuvers);

  // Calculate the consecutive exit sign count and then sort
  CountAndSortExitSigns(maneuvers);

#ifdef LOGGING_LEVEL_TRACE
  int combined_man_id = 1;
  LOG_TRACE("############################################");
  LOG_TRACE("COMBINED MANEUVERS");
  for (const Maneuver& maneuver : maneuvers) {
    LOG_TRACE("---------------------------------------------");
    LOG_TRACE(std::to_string(combined_man_id++) + ":  ");
    LOG_TRACE(std::string("  maneuver_PARAMETERS=") + maneuver.ToParameterString());
    LOG_TRACE(std::string("  maneuver=") + maneuver.ToString());
  }
#endif

#ifdef LOGGING_LEVEL_DEBUG
  std::vector<PointLL> shape = midgard::decode<std::vector<PointLL> >(
      trip_path_->shape());
//  int i = 0;
//  for (PointLL ll : shape) {
//    LOG_TRACE(std::string("shape lng/lat[") + std::to_string(i++) + "]=" + std::to_string(ll.lng()) + "," + std::to_string(ll.lat()));
//  }
  if (shape.empty() || (trip_path_->node_size() < 2))
  throw std::runtime_error("Error - No shape or invalid node count");
  PointLL first_point = shape.at(0);
  PointLL last_point = shape.at(shape.size() - 1);
  std::string first_name = (trip_path_->GetCurrEdge(0)->name_size() == 0) ? "" : trip_path_->GetCurrEdge(0)->name(0);
  auto last_node_index = (trip_path_->node_size() - 2);
  std::string last_name = (trip_path_->GetCurrEdge(last_node_index)->name_size() == 0) ? "" : trip_path_->GetCurrEdge(last_node_index)->name(0);
  std::string units = (directions_options_.units() == valhalla::odin::DirectionsOptions::kKilometers) ? "kilometers" : "miles";
  LOG_DEBUG(
      (boost::format(
              "ROUTE_REQUEST|-j '{\"locations\":[{\"lat\":%1$.6f,\"lon\":%2$.6f,\"street\":\"%3%\"},{\"lat\":%4$.6f,\"lon\":%5$.6f,\"street\":\"%6%\"}],\"costing\":\"auto\",\"directions_options\":{\"units\":\"%7%\"}}' --config ../conf/valhalla.json")
          % first_point.lat() % first_point.lng() % first_name
          % last_point.lat() % last_point.lng() % last_name
          % units).str());
#endif

  return maneuvers;
}

std::list<Maneuver> ManeuversBuilder::Produce() {
  std::list<Maneuver> maneuvers;

  // Validate trip path node list
  if (trip_path_->node_size() < 1) {
    throw std::runtime_error("Trip path does not have any nodes");
  }

  // Check for a single node
  if (trip_path_->node_size() == 1) {
    // TODO - handle origin and destination are the same
    throw std::runtime_error("Trip path has only one node");
  }

  LOG_INFO(
      std::string(
          "trip_path_->node_size()=" + std::to_string(trip_path_->node_size())));

  // Process the Destination maneuver
  maneuvers.emplace_front();
  CreateDestinationManeuver(maneuvers.front());

  // TODO - handle no edges

  // Initialize maneuver prior to loop
  maneuvers.emplace_front();
  InitializeManeuver(maneuvers.front(), trip_path_->GetLastNodeIndex());

  // Step through nodes in reverse order to produce maneuvers
  // excluding the last and first nodes
  for (int i = (trip_path_->GetLastNodeIndex() - 1); i > 0; --i) {

#ifdef LOGGING_LEVEL_TRACE
    auto* prev_edge = trip_path_->GetPrevEdge(i);
    auto* curr_edge = trip_path_->GetCurrEdge(i);
    auto* next_edge = trip_path_->GetNextEdge(i);
    LOG_TRACE("---------------------------------------------");
    LOG_TRACE(std::to_string(i) + ":  ");
    LOG_TRACE(std::string("  curr_edge_PARAMETERS=") + (curr_edge ? curr_edge->ToParameterString() : "NONE"));
    LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  prev2curr_turn_degree=") + std::to_string(
            GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading())));
    auto* node = trip_path_->GetEnhancedNode(i);
    for (size_t z = 0; z < node->intersecting_edge_size(); ++z) {
      auto* intersecting_edge = node->GetIntersectingEdge(z);
      LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
      LOG_TRACE(std::string("    prev2int_turn_degree=") + std::to_string(
              GetTurnDegree(prev_edge->end_heading(), intersecting_edge->begin_heading())));
    }
    LOG_TRACE(std::string("  node=") + node->ToString());
    IntersectingEdgeCounts xedge_counts;
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
        xedge_counts);
    LOG_TRACE(std::string("    right=") + std::to_string(xedge_counts.right)
        + std::string(" | right_similar=") + std::to_string(xedge_counts.right_similar)
        + std::string(" | right_driveable_outbound=") + std::to_string(xedge_counts.right_driveable_outbound)
        + std::string(" | right_similar_driveable_outbound=") + std::to_string(xedge_counts.right_similar_driveable_outbound));
    LOG_TRACE(std::string("    left =") + std::to_string(xedge_counts.left)
        + std::string(" | left_similar =") + std::to_string(xedge_counts.left_similar)
        + std::string(" | left_driveable_outbound =") + std::to_string(xedge_counts.left_driveable_outbound)
        + std::string(" | left_similar_driveable_outbound =") + std::to_string(xedge_counts.left_similar_driveable_outbound));
#endif

    if (CanManeuverIncludePrevEdge(maneuvers.front(), i)) {
      UpdateManeuver(maneuvers.front(), i);
    } else {
      // Finalize current maneuver
      FinalizeManeuver(maneuvers.front(), i);

      // Initialize new maneuver
      maneuvers.emplace_front();
      InitializeManeuver(maneuvers.front(), i);
    }
  }

#ifdef LOGGING_LEVEL_TRACE
  auto* curr_edge = trip_path_->GetCurrEdge(0);
  LOG_TRACE("---------------------------------------------");
  LOG_TRACE(std::string("0") + ":  ");
  LOG_TRACE(std::string("  curr_edge_PARAMETERS=") + (curr_edge ? curr_edge->ToParameterString() : "NONE"));
  LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
  auto* node = trip_path_->GetEnhancedNode(0);
  for (size_t z = 0; z < node->intersecting_edge_size(); ++z) {
    auto* intersecting_edge = node->GetIntersectingEdge(z);
    LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
  }
  LOG_TRACE("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  for (size_t z = 0; z < trip_path_->admin_size(); ++z) {
    auto* admin = trip_path_->GetAdmin(z);
    LOG_TRACE("ADMIN " + std::to_string(z) + ": " + admin->ToString());
  }

#endif

  // Process the Start maneuver
  CreateStartManeuver(maneuvers.front());

  return maneuvers;
}

void ManeuversBuilder::Combine(std::list<Maneuver>& maneuvers) {
  bool maneuvers_have_been_combined = true;

  // Continue trying to combine maneuvers until no maneuvers have been combined
  while (maneuvers_have_been_combined) {
    maneuvers_have_been_combined = false;

    auto prev_man = maneuvers.begin();
    auto curr_man = maneuvers.begin();
    auto next_man = maneuvers.begin();

    if (next_man != maneuvers.end())
      ++next_man;

    while (next_man != maneuvers.end()) {
      // Process common base names
      std::unique_ptr<StreetNames> common_base_names = curr_man->street_names()
          .FindCommonBaseNames(next_man->street_names());

      // Get the begin edge of the next maneuver
      auto* next_man_begin_edge = trip_path_->GetCurrEdge(
          next_man->begin_node_index());

      // Do not combine
      // if travel mode is different
      // OR next maneuver is destination
      if ((curr_man->travel_mode() != next_man->travel_mode())
          || (next_man->type() == TripDirections_Maneuver_Type_kDestination)) {
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if current or next maneuver is a ferry
      else if (curr_man->ferry() || next_man->ferry()) {
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Combine current internal maneuver with next maneuver
      else if (curr_man->internal_intersection() && (curr_man != next_man)) {
        curr_man = CombineInternalManeuver(maneuvers, prev_man, curr_man,
                                           next_man,
                                           (curr_man == maneuvers.begin()));
        maneuvers_have_been_combined = true;
        ++next_man;
      }
      // Combine current turn channel maneuver with next maneuver
      else if (curr_man->turn_channel() && (curr_man != next_man)) {
        curr_man = CombineTurnChannelManeuver(maneuvers, prev_man, curr_man,
                                              next_man,
                                              (curr_man == maneuvers.begin()));
        maneuvers_have_been_combined = true;
        ++next_man;
      }
      // NOTE: Logic may have to be adjusted depending on testing
      // Maybe not intersecting forward link
      // Maybe first edge in next is internal
      // Maybe no signs
      // Combine the 'same name straight' next maneuver with the current maneuver
      // if begin edge of next maneuver is not a turn channel
      // and the next maneuver is not an internal intersection maneuver
      // and the current maneuver is not a ramp
      // and the next maneuver is not a ramp
      // and current and next maneuvers have a common base name
      else if ((next_man->begin_relative_direction()
          == Maneuver::RelativeDirection::kKeepStraight)
          && (next_man_begin_edge && !next_man_begin_edge->turn_channel())
          && !next_man->internal_intersection() && !curr_man->ramp()
          && !next_man->ramp() && !curr_man->roundabout()
          && !next_man->roundabout() && !common_base_names->empty()) {
        // Update current maneuver street names
        curr_man->set_street_names(std::move(common_base_names));
        next_man = CombineSameNameStraightManeuver(maneuvers, curr_man,
                                                   next_man);
      } else {
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
    }
  }
}

std::list<Maneuver>::iterator ManeuversBuilder::CombineInternalManeuver(
    std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator prev_man,
    std::list<Maneuver>::iterator curr_man,
    std::list<Maneuver>::iterator next_man, bool start_man) {

  if (start_man) {
    // Determine turn degree current maneuver and next maneuver
    next_man->set_turn_degree(
        GetTurnDegree(curr_man->end_heading(), next_man->begin_heading()));
  } else {
    // Determine turn degree based on previous maneuver and next maneuver
    next_man->set_turn_degree(
        GetTurnDegree(prev_man->end_heading(), next_man->begin_heading()));
  }

  // Set the cross street names
  if (curr_man->HasUsableInternalIntersectionName()) {
    next_man->set_cross_street_names(curr_man->street_names().clone());
  }

  // Set the right and left internal turn counts
  next_man->set_internal_right_turn_count(
      curr_man->internal_right_turn_count());
  next_man->set_internal_left_turn_count(curr_man->internal_left_turn_count());

  // Set relative direction
  next_man->set_begin_relative_direction(
      ManeuversBuilder::DetermineRelativeDirection(next_man->turn_degree()));

  // Add distance
  next_man->set_distance(next_man->distance() + curr_man->distance());

  // Add time
  next_man->set_time(next_man->time() + curr_man->time());

  // TODO - heading?

  // Set begin node index
  next_man->set_begin_node_index(curr_man->begin_node_index());

  // Set begin shape index
  next_man->set_begin_shape_index(curr_man->begin_shape_index());

  if (start_man) {
    next_man->set_type(TripDirections_Maneuver_Type_kStart);
  } else {
    // Set maneuver type to 'none' so the type will be processed again
    next_man->set_type(TripDirections_Maneuver_Type_kNone);
    SetManeuverType(*(next_man));
  }

  return maneuvers.erase(curr_man);
}

std::list<Maneuver>::iterator ManeuversBuilder::CombineTurnChannelManeuver(
    std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator prev_man,
    std::list<Maneuver>::iterator curr_man,
    std::list<Maneuver>::iterator next_man, bool start_man) {

  if (start_man) {
    // Determine turn degree current maneuver and next maneuver
    next_man->set_turn_degree(
        GetTurnDegree(curr_man->end_heading(), next_man->begin_heading()));
  } else {
    // Determine turn degree based on previous maneuver and next maneuver
    next_man->set_turn_degree(
        GetTurnDegree(prev_man->end_heading(), next_man->begin_heading()));
  }

  // Set relative direction
  next_man->set_begin_relative_direction(curr_man->begin_relative_direction());

  // Add distance
  next_man->set_distance(next_man->distance() + curr_man->distance());

  // Add time
  next_man->set_time(next_man->time() + curr_man->time());

  // TODO - heading?

  // Set begin node index
  next_man->set_begin_node_index(curr_man->begin_node_index());

  // Set begin shape index
  next_man->set_begin_shape_index(curr_man->begin_shape_index());

  if (start_man) {
    next_man->set_type(TripDirections_Maneuver_Type_kStart);
  } else {
    // Set maneuver type to 'none' so the type will be processed again
    next_man->set_type(TripDirections_Maneuver_Type_kNone);
    SetManeuverType(*(next_man));
  }

  return maneuvers.erase(curr_man);
}

std::list<Maneuver>::iterator ManeuversBuilder::CombineSameNameStraightManeuver(
    std::list<Maneuver>& maneuvers, std::list<Maneuver>::iterator curr_man,
    std::list<Maneuver>::iterator next_man) {

  // Add distance
  curr_man->set_distance(curr_man->distance() + next_man->distance());

  // Add time
  curr_man->set_time(curr_man->time() + next_man->time());

  // Update end heading
  curr_man->set_end_heading(next_man->end_node_index());

  // Update end node index
  curr_man->set_end_node_index(next_man->end_node_index());

  // Update end shape index
  curr_man->set_end_shape_index(next_man->end_shape_index());

  // If needed, set ramp
  if (next_man->ramp())
    curr_man->set_ramp(true);

  // If needed, set ferry
  if (next_man->ferry())
    curr_man->set_ferry(true);

  // If needed, set rail_ferry
  if (next_man->rail_ferry())
    curr_man->set_rail_ferry(true);

  // If needed, set roundabout
  if (next_man->roundabout())
    curr_man->set_roundabout(true);

  // If needed, set portions_toll
  if (next_man->portions_toll())
    curr_man->set_portions_toll(true);

  // If needed, set portions_unpaved
  if (next_man->portions_unpaved())
    curr_man->set_portions_unpaved(true);

  // If needed, set portions_highway
  if (next_man->portions_highway())
    curr_man->set_portions_highway(true);

  return maneuvers.erase(next_man);
}

void ManeuversBuilder::CountAndSortExitSigns(std::list<Maneuver>& maneuvers) {

  auto prev_man = maneuvers.rbegin();
  auto curr_man = maneuvers.rbegin();

  if (prev_man != maneuvers.rend())
    ++prev_man;

  // Rank the exit signs
  while (prev_man != maneuvers.rend()) {

    // Increase the branch exit sign consecutive count
    // if it matches the succeeding named maneuver
    if (prev_man->HasExitBranchSign() && !curr_man->HasExitSign()
        && curr_man->HasStreetNames()) {
      for (Sign& sign : *(prev_man->mutable_signs()->mutable_exit_branch_list())) {
        for (const auto& street_name : curr_man->street_names()) {
          if (sign.text() == street_name->value()) {
            sign.set_consecutive_count(sign.consecutive_count() + 1);
          }
        }
      }
      SortExitSignList(prev_man->mutable_signs()->mutable_exit_number_list());
    }
    // Increase the consecutive count of signs that match their neighbor
    else if (prev_man->HasExitSign() && curr_man->HasExitSign()) {

      // Process the exit number signs
      CountAndSortExitSignList(
          prev_man->mutable_signs()->mutable_exit_number_list(),
          curr_man->mutable_signs()->mutable_exit_number_list());

      // Process the exit branch signs
      CountAndSortExitSignList(
          prev_man->mutable_signs()->mutable_exit_branch_list(),
          curr_man->mutable_signs()->mutable_exit_branch_list());

      // Process the exit toward signs
      CountAndSortExitSignList(
          prev_man->mutable_signs()->mutable_exit_toward_list(),
          curr_man->mutable_signs()->mutable_exit_toward_list());

      // Process the exit name signs
      CountAndSortExitSignList(
          prev_man->mutable_signs()->mutable_exit_name_list(),
          curr_man->mutable_signs()->mutable_exit_name_list());
    }

    // Update iterators
    curr_man = prev_man;
    ++prev_man;
  }

}

void ManeuversBuilder::CreateDestinationManeuver(Maneuver& maneuver) {
  int node_index = trip_path_->GetLastNodeIndex();

  // TODO - side of street
  // TripDirections_Maneuver_Type_kDestinationRight
  // TripDirections_Maneuver_Type_kDestinationLeft

  // Set the destination maneuver type
  maneuver.set_type(TripDirections_Maneuver_Type_kDestination);

  // Set the begin and end node index
  maneuver.set_begin_node_index(node_index);
  maneuver.set_end_node_index(node_index);

  // Set the begin and end shape index
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  maneuver.set_begin_shape_index(prev_edge->end_shape_index());
  maneuver.set_end_shape_index(prev_edge->end_shape_index());

}

void ManeuversBuilder::CreateStartManeuver(Maneuver& maneuver) {
  int node_index = 0;

  // TODO - side of street
  // TripDirections_Maneuver_Type_kStartRight
  // TripDirections_Maneuver_Type_kStartLeft

  // Set the start maneuver type
  maneuver.set_type(TripDirections_Maneuver_Type_kStart);

  FinalizeManeuver(maneuver, node_index);
}

void ManeuversBuilder::InitializeManeuver(Maneuver& maneuver, int node_index) {

  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  auto* curr_edge = trip_path_->GetCurrEdge(node_index);

  // Set the end heading
  maneuver.set_end_heading(prev_edge->end_heading());

  // Set the end node index
  maneuver.set_end_node_index(node_index);

  // Set the end shape index
  maneuver.set_end_shape_index(prev_edge->end_shape_index());

  // Ramp
  if (prev_edge->ramp()) {
    maneuver.set_ramp(true);
  }

  // Turn Channel
  if (prev_edge->turn_channel()) {
    maneuver.set_turn_channel(true);
  }

  // Ferry
  if (prev_edge->ferry()) {
    maneuver.set_ferry(true);
  }

  // Rail Ferry
  if (prev_edge->rail_ferry()) {
    maneuver.set_rail_ferry(true);
  }

  // Roundabout
  if (prev_edge->roundabout()) {
    maneuver.set_roundabout(true);
    maneuver.set_roundabout_exit_count(1);
  }

  // Internal Intersection
  if (prev_edge->internal_intersection()) {
    maneuver.set_internal_intersection(true);
  }

  // Travel mode
  maneuver.set_travel_mode(prev_edge->travel_mode());

  // Transit info
  if (prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit) {
    maneuver.set_rail(prev_edge->rail());
    maneuver.set_bus(prev_edge->bus());
    maneuver.set_transit_block_id(prev_edge->transit_block_id());
    maneuver.set_transit_trip_id(prev_edge->transit_trip_id());
    maneuver.set_transit_short_name(prev_edge->transit_info().short_name());
    maneuver.set_transit_long_name(prev_edge->transit_info().long_name());
    maneuver.set_transit_headsign(prev_edge->transit_info().headsign());
  }

  // Transit connection
  if (prev_edge->transit_connection()) {
    maneuver.set_transit_connection(true);
    // If current edge is transit then mark maneuver as transit connection start
    if (curr_edge
        && (curr_edge->travel_mode() == TripPath_TravelMode_kPublicTransit)) {
      maneuver.set_type(TripDirections_Maneuver_Type_kTransitConnectionStart);
      auto* node = trip_path_->GetEnhancedNode(node_index);
      maneuver.set_transit_connection_stop(
          TransitStop(node->transit_stop_info().name(),
                      node->transit_stop_info().arrival_date_time(),
                      node->transit_stop_info().departure_date_time()));
    }
    // else mark it as transit connection destination
    else {
      maneuver.set_type(
          TripDirections_Maneuver_Type_kTransitConnectionDestination);
      LOG_TRACE("ManeuverType=TRANSIT_CONNECTION_DESTINATION");
    }
  }

  // TODO - what about street names; maybe check name flag
  UpdateManeuver(maneuver, node_index);
}

void ManeuversBuilder::UpdateManeuver(Maneuver& maneuver, int node_index) {

  auto* prev_edge = trip_path_->GetPrevEdge(node_index);

  // Street names
  // Set if street names are empty and maneuver is not internal intersection
  // or usable internal intersection name exists
  if ((maneuver.street_names().empty() && !maneuver.internal_intersection())
      || UsableInternalIntersectionName(maneuver, node_index)) {
    maneuver.set_street_names(
        std::move(
            StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index),
                                       prev_edge->GetNameList())));
  }

  // Update the internal turn count
  UpdateInternalTurnCount(maneuver, node_index);

  // Distance
  maneuver.set_distance(
      maneuver.distance() + prev_edge->GetLength(directions_options_.units()));

  // Portions Toll
  if (prev_edge->toll()) {
    maneuver.set_portions_toll(true);
  }

  // Portions unpaved
  if (prev_edge->unpaved()) {
    maneuver.set_portions_unpaved(true);
  }

  // Portions highway
  if (prev_edge->IsHighway()) {
    maneuver.set_portions_highway(true);
  }

  // Roundabouts
  if (prev_edge->roundabout()) {
    IntersectingEdgeCounts xedge_counts;
    trip_path_->GetEnhancedNode(node_index)
        ->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                   xedge_counts);
    if (prev_edge->drive_on_right()) {
      maneuver.set_roundabout_exit_count(
          maneuver.roundabout_exit_count()
              + xedge_counts.right_driveable_outbound);
    } else {
      maneuver.set_roundabout_exit_count(
          maneuver.roundabout_exit_count()
              + xedge_counts.left_driveable_outbound);
    }
  }

  // Signs
  if (prev_edge->has_sign()) {
    // Exit number
    for (auto& text : prev_edge->sign().exit_number()) {
      maneuver.mutable_signs()->mutable_exit_number_list()->emplace_back(text);
    }

    // Exit branch
    for (auto& text : prev_edge->sign().exit_branch()) {
      maneuver.mutable_signs()->mutable_exit_branch_list()->emplace_back(text);
    }

    // Exit toward
    for (auto& text : prev_edge->sign().exit_toward()) {
      maneuver.mutable_signs()->mutable_exit_toward_list()->emplace_back(text);
    }

    // Exit name
    for (auto& text : prev_edge->sign().exit_name()) {
      maneuver.mutable_signs()->mutable_exit_name_list()->emplace_back(text);
    }
  }

  // Insert transit stop into the transit maneuver
  if (prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit) {
    auto* node = trip_path_->GetEnhancedNode(node_index);
    maneuver.InsertTransitStop(node->transit_stop_info().name(),
                               node->transit_stop_info().arrival_date_time(),
                               node->transit_stop_info().departure_date_time());
  }

}

void ManeuversBuilder::FinalizeManeuver(Maneuver& maneuver, int node_index) {

  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  auto* curr_edge = trip_path_->GetCurrEdge(node_index);

  // Set begin cardinal direction
  maneuver.set_begin_cardinal_direction(
      DetermineCardinalDirection(curr_edge->begin_heading()));

  // Set the begin heading
  maneuver.set_begin_heading(curr_edge->begin_heading());

  // Set the begin node index
  maneuver.set_begin_node_index(node_index);

  // Set the begin shape index
  maneuver.set_begin_shape_index(curr_edge->begin_shape_index());

  // if possible, set the turn degree and relative direction
  if (prev_edge) {
    maneuver.set_turn_degree(
        GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()));

    // Calculate and set the relative direction for the specified maneuver
    DetermineRelativeDirection(maneuver);

    // Set the time based on the delta of the elapsed time between the begin
    // and end nodes
    maneuver.set_time(
        trip_path_->node(maneuver.end_node_index()).elapsed_time()
            - trip_path_->node(maneuver.begin_node_index()).elapsed_time());

    // TODO - determine if we want to count right driveable at entrance node
    // Roundabouts
//    if (curr_edge->roundabout()) {
//      IntersectingEdgeCounts xedge_counts;
//      trip_path_->GetEnhancedNode(node_index)
//            ->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
//                                                       xedge_counts);
//      if (curr_edge->drive_on_right()) {
//        maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count()
//                                           + xedge_counts.right_driveable_outbound);
//      } else {
//        maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count()
//                                           + xedge_counts.left_driveable_outbound);
//      }
//    }

  }

  // Mark transit connection transfer
  if ((maneuver.type() == TripDirections_Maneuver_Type_kTransitConnectionStart)
      && prev_edge
      && (prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kTransitConnectionTransfer);
    LOG_TRACE("ManeuverType=TRANSIT_CONNECTION_TRANSFER");
  }


  // Add transit connection stop to a transit connection destination
  if ((maneuver.type() == TripDirections_Maneuver_Type_kTransitConnectionDestination)
      && prev_edge
      && (prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit)) {
    auto* node = trip_path_->GetEnhancedNode(node_index);
    maneuver.set_transit_connection_stop(
        TransitStop(node->transit_stop_info().name(),
                    node->transit_stop_info().arrival_date_time(),
                    node->transit_stop_info().departure_date_time()));
  }

  // Insert first transit stop
  if (maneuver.travel_mode() == TripPath_TravelMode_kPublicTransit) {
    auto* node = trip_path_->GetEnhancedNode(node_index);
    maneuver.InsertTransitStop(node->transit_stop_info().name(),
                               node->transit_stop_info().arrival_date_time(),
                               node->transit_stop_info().departure_date_time());
  }

  // Set the maneuver type
  SetManeuverType(maneuver);

  // Begin street names
  // TODO

}

void ManeuversBuilder::SetManeuverType(Maneuver& maneuver) {
  // If the type is already set then just return
  if (maneuver.type() != TripDirections_Maneuver_Type_kNone) {
    return;
  }

  auto* prev_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());
  auto* curr_edge = trip_path_->GetCurrEdge(maneuver.begin_node_index());

  // TODO - iterate and expand
  // TripDirections_Maneuver_Type_kBecomes
  // TripDirections_Maneuver_Type_kStayStraight
  // TripDirections_Maneuver_Type_kStayRight
  // TripDirections_Maneuver_Type_kStayLeft

  // Process the different transit types
  if (maneuver.travel_mode() == TripPath_TravelMode_kPublicTransit) {
    if (prev_edge
        && prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit) {
      // Process transit remain on
      if ((maneuver.transit_block_id() != 0)
          && (maneuver.transit_block_id() == prev_edge->transit_block_id())
          && (maneuver.transit_trip_id() != prev_edge->transit_trip_id())) {
        maneuver.set_type(TripDirections_Maneuver_Type_kTransitRemainOn);
      }
      // Process transit transfer at same platform
      else {
        maneuver.set_type(TripDirections_Maneuver_Type_kTransitTransfer);
      }
    }
    // Process simple transit
    else {
        maneuver.set_type(TripDirections_Maneuver_Type_kTransit);
    }
  }
  // Process post transit connection destination
  else if (prev_edge && prev_edge->transit_connection()
      && (maneuver.travel_mode() != TripPath_TravelMode_kPublicTransit)) {
    maneuver.set_type(
        TripDirections_Maneuver_Type_kPostTransitConnectionDestination);
    LOG_TRACE("ManeuverType=POST_TRANSIT_CONNECTION_DESTINATION");
  }
  // Process Internal Intersection
  else if (maneuver.internal_intersection()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kNone);
    LOG_TRACE("ManeuverType=INTERNAL_INTERSECTION");
  }
  // Process Turn Channel
  else if (maneuver.turn_channel()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kNone);
    LOG_TRACE("ManeuverType=TURN_CHANNNEL");
  }
  // Process exit
  else if (maneuver.ramp() && prev_edge
      && (prev_edge->IsHighway() || maneuver.HasExitNumberSign())) {
    switch (maneuver.begin_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight:
      case Maneuver::RelativeDirection::kRight: {
        maneuver.set_type(TripDirections_Maneuver_Type_kExitRight);
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft:
      case Maneuver::RelativeDirection::kLeft: {
        maneuver.set_type(TripDirections_Maneuver_Type_kExitLeft);
        break;
      }
      default: {
        LOG_ERROR(
            std::string("EXIT RelativeDirection=")
                + std::to_string(
                    static_cast<int>(maneuver.begin_relative_direction())));
        // TODO: determine how to handle, for now set to right
        maneuver.set_type(TripDirections_Maneuver_Type_kExitRight);
      }
    }LOG_TRACE("ManeuverType=EXIT");
  }
  // Process on ramp
  else if (maneuver.ramp() && prev_edge && !prev_edge->IsHighway()) {
    switch (maneuver.begin_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight:
      case Maneuver::RelativeDirection::kRight: {
        maneuver.set_type(TripDirections_Maneuver_Type_kRampRight);
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft:
      case Maneuver::RelativeDirection::kLeft: {
        maneuver.set_type(TripDirections_Maneuver_Type_kRampLeft);
        break;
      }
      case Maneuver::RelativeDirection::kKeepStraight: {
        maneuver.set_type(TripDirections_Maneuver_Type_kRampStraight);
        break;
      }
      default: {
        LOG_ERROR(
            std::string("RAMP RelativeDirection=")
                + std::to_string(
                    static_cast<int>(maneuver.begin_relative_direction())));
        // TODO: determine how to handle, for now set to right
        maneuver.set_type(TripDirections_Maneuver_Type_kRampRight);
      }
    }LOG_TRACE("ManeuverType=RAMP");
  }
  // Process merge
  else if (curr_edge->IsHighway() && prev_edge && prev_edge->ramp()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kMerge);
    LOG_TRACE("ManeuverType=MERGE");
  }
  // Process enter roundabout
  else if (maneuver.roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutEnter);
    LOG_TRACE("ManeuverType=ROUNDABOUT_ENTER");
  }
  // Process exit roundabout
  else if (prev_edge && prev_edge->roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutExit);
    LOG_TRACE("ManeuverType=ROUNDABOUT_EXIT");
  }
  // Process enter ferry
  else if (maneuver.ferry() || maneuver.rail_ferry()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kFerryEnter);
    LOG_TRACE("ManeuverType=FERRY_ENTER");
  }
  // Process exit ferry
  else if (prev_edge && (prev_edge->ferry() || prev_edge->rail_ferry())) {
    maneuver.set_type(TripDirections_Maneuver_Type_kFerryExit);
    LOG_TRACE("ManeuverType=FERRY_EXIT");
  }
  // Process simple direction
  else {
    SetSimpleDirectionalManeuverType(maneuver);
    LOG_TRACE("ManeuverType=SIMPLE");
  }

}

void ManeuversBuilder::SetSimpleDirectionalManeuverType(Maneuver& maneuver) {
  switch (Turn::GetType(maneuver.turn_degree())) {
    case Turn::Type::kStraight: {
      maneuver.set_type(TripDirections_Maneuver_Type_kContinue);
      if (trip_path_) {
        auto* man_begin_edge = trip_path_->GetCurrEdge(
            maneuver.begin_node_index());

        ////////////////////////////////////////////////////////////////////
        // If the maneuver begin edge is a turn channel
        // and the relative direction is not a keep straight
        // then set as slight right based on a relative keep right direction
        //  OR  set as slight left based on a relative keep left direction
        if (man_begin_edge && man_begin_edge->turn_channel()
            && (maneuver.begin_relative_direction()
                != Maneuver::RelativeDirection::kKeepStraight)) {
          if (maneuver.begin_relative_direction()
              == Maneuver::RelativeDirection::kKeepRight) {
            maneuver.set_type(TripDirections_Maneuver_Type_kSlightRight);
          } else if (maneuver.begin_relative_direction()
              == Maneuver::RelativeDirection::kKeepLeft) {
            maneuver.set_type(TripDirections_Maneuver_Type_kSlightLeft);
          }
        }
      }
      break;
    }
    case Turn::Type::kSlightRight: {
      maneuver.set_type(TripDirections_Maneuver_Type_kSlightRight);
      break;
    }
    case Turn::Type::kRight: {
      maneuver.set_type(TripDirections_Maneuver_Type_kRight);
      break;
    }
    case Turn::Type::kSharpRight: {
      maneuver.set_type(TripDirections_Maneuver_Type_kSharpRight);
      break;
    }
    case Turn::Type::kReverse: {
      if (maneuver.internal_left_turn_count()
          > maneuver.internal_right_turn_count()) {
        maneuver.set_type(TripDirections_Maneuver_Type_kUturnLeft);
      } else if (maneuver.internal_right_turn_count()
          > maneuver.internal_left_turn_count()) {
        maneuver.set_type(TripDirections_Maneuver_Type_kUturnRight);
      } else if (trip_path_->GetCurrEdge(maneuver.begin_node_index())
          ->drive_on_right()) {
        if (maneuver.turn_degree() < 180) {
          maneuver.set_type(TripDirections_Maneuver_Type_kUturnRight);
        } else {
          maneuver.set_type(TripDirections_Maneuver_Type_kUturnLeft);
        }
      } else {
        if (maneuver.turn_degree() > 180) {
          maneuver.set_type(TripDirections_Maneuver_Type_kUturnLeft);
        } else {
          maneuver.set_type(TripDirections_Maneuver_Type_kUturnRight);
        }
      }
      break;
    }
    case Turn::Type::kSharpLeft: {
      maneuver.set_type(TripDirections_Maneuver_Type_kSharpLeft);
      break;
    }
    case Turn::Type::kLeft: {
      maneuver.set_type(TripDirections_Maneuver_Type_kLeft);
      break;
    }
    case Turn::Type::kSlightLeft: {
      maneuver.set_type(TripDirections_Maneuver_Type_kSlightLeft);
      break;
    }
  }
}

TripDirections_Maneuver_CardinalDirection ManeuversBuilder::DetermineCardinalDirection(
    uint32_t heading) {
  if ((heading > 336) || (heading < 24)) {
    return TripDirections_Maneuver_CardinalDirection_kNorth;
  } else if ((heading > 23) && (heading < 67)) {
    return TripDirections_Maneuver_CardinalDirection_kNorthEast;
  } else if ((heading > 66) && (heading < 114)) {
    return TripDirections_Maneuver_CardinalDirection_kEast;
  } else if ((heading > 113) && (heading < 157)) {
    return TripDirections_Maneuver_CardinalDirection_kSouthEast;
  } else if ((heading > 156) && (heading < 204)) {
    return TripDirections_Maneuver_CardinalDirection_kSouth;
  } else if ((heading > 203) && (heading < 247)) {
    return TripDirections_Maneuver_CardinalDirection_kSouthWest;
  } else if ((heading > 246) && (heading < 294)) {
    return TripDirections_Maneuver_CardinalDirection_kWest;
  } else if ((heading > 293) && (heading < 337)) {
    return TripDirections_Maneuver_CardinalDirection_kNorthWest;
  }
  throw std::runtime_error("Turn degree out of range for cardinal direction.");
}

bool ManeuversBuilder::CanManeuverIncludePrevEdge(Maneuver& maneuver,
                                                  int node_index) {
  // TODO - fix it
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  auto* curr_edge = trip_path_->GetCurrEdge(node_index);

  /////////////////////////////////////////////////////////////////////////////
  // Process transit
  if ((maneuver.travel_mode() == TripPath_TravelMode_kPublicTransit)
      && (prev_edge->travel_mode() != TripPath_TravelMode_kPublicTransit)) {
    return false;
  }
  if ((prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit)
      && (maneuver.travel_mode() != TripPath_TravelMode_kPublicTransit)) {
    return false;
  }
  if ((maneuver.travel_mode() == TripPath_TravelMode_kPublicTransit)
      && (prev_edge->travel_mode() == TripPath_TravelMode_kPublicTransit)) {

    // Both block id and trip id must be the same so we can combine...
    if ((maneuver.transit_block_id() == prev_edge->transit_block_id())
        && (maneuver.transit_trip_id() == prev_edge->transit_trip_id())) {
      return true;
    }
    // ...otherwise, it is a transfer or remain on
    // therefore, we can not combine
    else {
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process transit connection
  if (maneuver.transit_connection() && !prev_edge->transit_connection()) {
    return false;
  }
  if (prev_edge->transit_connection() && !maneuver.transit_connection()) {
    return false;
  }
  if (maneuver.transit_connection() && prev_edge->transit_connection()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process travel mode
  if (maneuver.travel_mode() != prev_edge->travel_mode()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process internal intersection
  if (prev_edge->internal_intersection() && !maneuver.internal_intersection()) {
    return false;
  } else if (!prev_edge->internal_intersection()
      && maneuver.internal_intersection()) {
    return false;
  } else if (prev_edge->internal_intersection()
      && maneuver.internal_intersection()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process simple turn channel
  if (prev_edge->turn_channel() && !maneuver.turn_channel()) {
    return false;
  } else if (!prev_edge->turn_channel() && maneuver.turn_channel()) {
    return false;
  } else if (prev_edge->turn_channel() && maneuver.turn_channel()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process signs
  if (maneuver.HasExitSign()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process ramps
  if (maneuver.ramp() && !prev_edge->ramp()) {
    return false;
  }
  if (prev_edge->ramp() && !maneuver.ramp()) {
    return false;
  }
  if (maneuver.ramp() && prev_edge->ramp()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process ferries
  if (maneuver.ferry() && !prev_edge->ferry()) {
    return false;
  }
  if (prev_edge->ferry() && !maneuver.ferry()) {
    return false;
  }
  if (maneuver.ferry() && prev_edge->ferry()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process rail ferries
  if (maneuver.rail_ferry() && !prev_edge->rail_ferry()) {
    return false;
  }
  if (prev_edge->rail_ferry() && !maneuver.rail_ferry()) {
    return false;
  }
  if (maneuver.rail_ferry() && prev_edge->rail_ferry()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process roundabouts
  if (maneuver.roundabout() && !prev_edge->roundabout()) {
    return false;
  }
  if (prev_edge->roundabout() && !maneuver.roundabout()) {
    return false;
  }
  if (maneuver.roundabout() && prev_edge->roundabout()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process pencil point u-turns
  if (IsLeftPencilPointUturn(node_index, prev_edge, curr_edge)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kUturnLeft);
    return false;
  }
  if (IsRightPencilPointUturn(node_index, prev_edge, curr_edge)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kUturnRight);
    return false;
  }

  // TODO: add logic for 'T'

  std::unique_ptr<StreetNames> prev_edge_names = StreetNamesFactory::Create(
      trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

  // Process same names
  // TODO - do we need this anymore?
//  if (maneuver.street_names() == prev_edge_names) {
//    return true;
//  }

  // Process common base names
  std::unique_ptr<StreetNames> common_base_names = prev_edge_names
      ->FindCommonBaseNames(maneuver.street_names());
  if (!common_base_names->empty()) {
    maneuver.set_street_names(std::move(common_base_names));
    return true;
  }

  return false;

}

bool ManeuversBuilder::IsLeftPencilPointUturn(
    int node_index, EnhancedTripPath_Edge* prev_edge,
    EnhancedTripPath_Edge* curr_edge) const {

  uint32_t turn_degree = GetTurnDegree(prev_edge->end_heading(),
                                       curr_edge->begin_heading());

  // If drive on right
  // and the the turn is a sharp left (179 < turn < 211)
  //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
  // and oneway edges
  if (curr_edge->drive_on_right()
      && (((turn_degree > 179) && (turn_degree < 211))
          || (((prev_edge->length() < 50) || (curr_edge->length() < 50))
              && (turn_degree > 179) && (turn_degree < 226)))
      && prev_edge->IsOneway() && curr_edge->IsOneway()) {
    // If the above criteria is met then check the following criteria...

    IntersectingEdgeCounts xedge_counts;
    auto* node = trip_path_->GetEnhancedNode(node_index);
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                   xedge_counts);

    std::unique_ptr<StreetNames> prev_edge_names = StreetNamesFactory::Create(
        trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

    std::unique_ptr<StreetNames> curr_edge_names = StreetNamesFactory::Create(
        trip_path_->GetCountryCode(node_index), curr_edge->GetNameList());

    // Process common base names
    std::unique_ptr<StreetNames> common_base_names = prev_edge_names
        ->FindCommonBaseNames(*curr_edge_names);

    // If no intersecting driveable left road exists
    // and the from and to edges have a common base name
    // then it is a left pencil point u-turn
    if ((xedge_counts.left_driveable_outbound == 0)
        && !common_base_names->empty()) {
      return true;
    }
  }

  return false;
}

bool ManeuversBuilder::IsRightPencilPointUturn(
    int node_index, EnhancedTripPath_Edge* prev_edge,
    EnhancedTripPath_Edge* curr_edge) const {

  uint32_t turn_degree = GetTurnDegree(prev_edge->end_heading(),
                                       curr_edge->begin_heading());

  // If drive on left
  // and the turn is a sharp right (149 < turn < 181)
  //    or short distance (< 50m) and wider sharp right (134 < turn < 181)
  // and oneway edges
  if (curr_edge->drive_on_right()
      && (((turn_degree > 149) && (turn_degree < 181))
          || (((prev_edge->length() < 50) || (curr_edge->length() < 50))
              && (turn_degree > 134) && (turn_degree < 181)))
      && prev_edge->IsOneway() && curr_edge->IsOneway()) {
    // If the above criteria is met then check the following criteria...

    IntersectingEdgeCounts xedge_counts;
    auto* node = trip_path_->GetEnhancedNode(node_index);
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                   xedge_counts);

    std::unique_ptr<StreetNames> prev_edge_names = StreetNamesFactory::Create(
        trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

    std::unique_ptr<StreetNames> curr_edge_names = StreetNamesFactory::Create(
        trip_path_->GetCountryCode(node_index), curr_edge->GetNameList());

    // Process common base names
    std::unique_ptr<StreetNames> common_base_names = prev_edge_names
        ->FindCommonBaseNames(*curr_edge_names);

    // If no intersecting driveable right road exists
    // and the from and to edges have a common base name
    // then it is a right pencil point u-turn
    if ((xedge_counts.right_driveable_outbound == 0)
        && !common_base_names->empty()) {
      return true;
    }
  }

  return false;
}

void ManeuversBuilder::DetermineRelativeDirection(Maneuver& maneuver) {
  auto* prev_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());

  IntersectingEdgeCounts xedge_counts;
  auto* node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
  node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                 xedge_counts);

  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(maneuver.turn_degree());
  maneuver.set_begin_relative_direction(relative_direction);

  // Process driving mode
  if ((maneuver.travel_mode() == TripPath_TravelMode_kDrive)
      || (maneuver.travel_mode() == TripPath_TravelMode_kBicycle)) {
    if ((xedge_counts.right_similar_driveable_outbound == 0)
        && (xedge_counts.left_similar_driveable_outbound > 0)
        && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
      maneuver.set_begin_relative_direction(
          Maneuver::RelativeDirection::kKeepRight);
    } else if ((xedge_counts.right_similar_driveable_outbound > 0)
        && (xedge_counts.left_similar_driveable_outbound == 0)
        && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
      maneuver.set_begin_relative_direction(
          Maneuver::RelativeDirection::kKeepLeft);
    }
  }
  // Process non-driving mode
  else {
    if ((xedge_counts.right_similar == 0) && (xedge_counts.left_similar > 0)
        && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
      maneuver.set_begin_relative_direction(
          Maneuver::RelativeDirection::kKeepRight);
    } else if ((xedge_counts.right_similar > 0)
        && (xedge_counts.left_similar == 0)
        && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
      maneuver.set_begin_relative_direction(
          Maneuver::RelativeDirection::kKeepLeft);
    }
  }
}

Maneuver::RelativeDirection ManeuversBuilder::DetermineRelativeDirection(
    uint32_t turn_degree) {
  if ((turn_degree > 329) || (turn_degree < 31))
    return Maneuver::RelativeDirection::kKeepStraight;
  else if ((turn_degree > 30) && (turn_degree < 160))
    return Maneuver::RelativeDirection::kRight;
  else if ((turn_degree > 159) && (turn_degree < 201))
    return Maneuver::RelativeDirection::KReverse;
  else if ((turn_degree > 200) && (turn_degree < 330))
    return Maneuver::RelativeDirection::kLeft;
  else
    return Maneuver::RelativeDirection::kNone;
}

bool ManeuversBuilder::UsableInternalIntersectionName(Maneuver& maneuver,
                                                      int node_index) const {
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  auto* prev_prev_edge = trip_path_->GetPrevEdge(node_index, 2);
  uint32_t prev_prev_2prev_turn_degree = 0;
  if (prev_prev_edge) {
    prev_prev_2prev_turn_degree = GetTurnDegree(prev_prev_edge->end_heading(),
                                                prev_edge->begin_heading());
  }
  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(prev_prev_2prev_turn_degree);

  // Criteria for usable internal intersection name:
  // The maneuver is an internal intersection
  // Left turn for right side of the street driving
  // Right turn for left side of the street driving
  if (maneuver.internal_intersection()
      && ((prev_edge->drive_on_right()
          && (relative_direction == Maneuver::RelativeDirection::kLeft))
          || (!prev_edge->drive_on_right()
              && (relative_direction == Maneuver::RelativeDirection::kRight)))) {
    return true;
  }
  return false;
}

void ManeuversBuilder::UpdateInternalTurnCount(Maneuver& maneuver,
                                               int node_index) const {
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  auto* prev_prev_edge = trip_path_->GetPrevEdge(node_index, 2);
  uint32_t prev_prev_2prev_turn_degree = 0;
  if (prev_prev_edge) {
    prev_prev_2prev_turn_degree = GetTurnDegree(prev_prev_edge->end_heading(),
                                                prev_edge->begin_heading());
  }
  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(prev_prev_2prev_turn_degree);

  if (relative_direction == Maneuver::RelativeDirection::kRight) {
    maneuver.set_internal_right_turn_count(
        maneuver.internal_right_turn_count() + 1);
  }
  if (relative_direction == Maneuver::RelativeDirection::kLeft) {
    maneuver.set_internal_left_turn_count(
        maneuver.internal_left_turn_count() + 1);
  }
}

}
}

