#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include "baldr/streetnames.h"
#include "baldr/streetnames_factory.h"
#include "baldr/streetnames_us.h"
#include "baldr/turn.h"
#include "baldr/turnlanes.h"
#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_factory.h"
#include "baldr/verbal_text_formatter_us.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "worker.h"

#include "odin/maneuversbuilder.h"
#include "odin/sign.h"
#include "odin/signs.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;

namespace {

constexpr float kShortForkThreshold = 0.05f; // Kilometers

void SortExitSignList(std::vector<Sign>* signs) {
  // Sort signs by descending consecutive count order
  std::sort(signs->begin(), signs->end(),
            [](Sign a, Sign b) { return b.consecutive_count() < a.consecutive_count(); });
}

void CountAndSortExitSignList(std::vector<Sign>* prev_signs, std::vector<Sign>* curr_signs) {
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

} // namespace

namespace valhalla {
namespace odin {

ManeuversBuilder::ManeuversBuilder(const Options& options, EnhancedTripLeg* etp)
    : options_(options), trip_path_(etp) {
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

  // Calculate the consecutive exit sign count and then sort
  CountAndSortExitSigns(maneuvers);

  // Confirm maneuver type assignment
  ConfirmManeuverTypeAssignment(maneuvers);

  // Process the roundabout names
  ProcessRoundaboutNames(maneuvers);

  // Process the 'to stay on' attribute
  SetToStayOnAttribute(maneuvers);

  // Enhance signless interchanges
  EnhanceSignlessInterchnages(maneuvers);

  // Process the turn lanes
  ProcessTurnLanes(maneuvers);

#ifdef LOGGING_LEVEL_DEBUG
  std::vector<PointLL> shape = midgard::decode<std::vector<PointLL>>(trip_path_->shape());
  // Shape by index
  //  int i = 0;
  //  for (PointLL ll : shape) {
  //    LOG_TRACE(std::string("shape lng/lat[") + std::to_string(i++) + "]=" +
  //    std::to_string(ll.lng()) + "," + std::to_string(ll.lat()));
  //  }
  // Shape by lat/lon pairs
  //  std::string shape_json("\"shape\":[");
  //  for (PointLL ll : shape) {
  //    shape_json += "{\"lat\":" + std::to_string(ll.lat()) + ",\"lon\":" +
  //    std::to_string(ll.lng()) + "},";
  //  }
  //  shape_json.pop_back();
  //  shape_json += "]";
  //  LOG_TRACE(shape_json);

  if (shape.empty() || (trip_path_->node_size() < 2))
    throw valhalla_exception_t{213};
  const auto& orig = trip_path_->GetOrigin();
  const auto& dest = trip_path_->GetDestination();
  std::string first_name = (trip_path_->GetCurrEdge(0)->name_size() == 0)
                               ? ""
                               : trip_path_->GetCurrEdge(0)->name(0).value();
  auto last_node_index = (trip_path_->node_size() - 2);
  std::string last_name = (trip_path_->GetCurrEdge(last_node_index)->name_size() == 0)
                              ? ""
                              : trip_path_->GetCurrEdge(last_node_index)->name(0).value();
  std::string units =
      (options_.has_units())
          ? ((options_.units() == valhalla::Options::kilometers) ? "kilometers" : "miles")
          : "miles";
  LOG_DEBUG((boost::format("ROUTE_REQUEST|-j "
                           "'{\"locations\":[{\"lat\":%1$.6f,\"lon\":%2$.6f,\"street\":\"%3%\"},{"
                           "\"lat\":%4$.6f,\"lon\":%5$.6f,\"street\":\"%6%\"}],\"costing\":"
                           "\"auto\",\"units\":\"%7%\"}'") %
             orig.ll().lat() % orig.ll().lng() % first_name % dest.ll().lat() % dest.ll().lng() %
             last_name % units)
                .str());
#endif

  return maneuvers;
}

std::list<Maneuver> ManeuversBuilder::Produce() {
  std::list<Maneuver> maneuvers;

  // Validate trip path node list
  if (trip_path_->node_size() < 1) {
    throw valhalla_exception_t{210};
  }

  // Check for a single node
  if (trip_path_->node_size() == 1) {
    // TODO - handle origin and destination are the same
    throw valhalla_exception_t{211};
  }

  // Validate location count
  if (trip_path_->location_size() < 2) {
    throw valhalla_exception_t{212};
  }

  LOG_INFO(std::string("trip_path_->node_size()=" + std::to_string(trip_path_->node_size())));

  // Process the Destination maneuver
  maneuvers.emplace_front();
  CreateDestinationManeuver(maneuvers.front());

  // TODO - handle no edges

  // Initialize maneuver prior to loop
  maneuvers.emplace_front();
  InitializeManeuver(maneuvers.front(), trip_path_->GetLastNodeIndex());

#ifdef LOGGING_LEVEL_TRACE
  LOG_TRACE("=============================================");
  LOG_TRACE(std::string("osm_changeset=") + std::to_string(trip_path_->osm_changeset()));
#endif

  // Step through nodes in reverse order to produce maneuvers
  // excluding the last and first nodes
  for (int i = (trip_path_->GetLastNodeIndex() - 1); i > 0; --i) {

#ifdef LOGGING_LEVEL_TRACE
    auto prev_edge = trip_path_->GetPrevEdge(i);
    auto curr_edge = trip_path_->GetCurrEdge(i);
    auto next_edge = trip_path_->GetNextEdge(i);
    auto prev2curr_turn_degree = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());
    LOG_TRACE("---------------------------------------------");
    LOG_TRACE(std::to_string(i) + ":  ");
    LOG_TRACE(std::string("  curr_edge_PARAMETERS=") +
              (curr_edge ? curr_edge->ToParameterString() : "NONE"));
    LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  prev2curr_turn_degree=") + std::to_string(prev2curr_turn_degree) +
              " is a " + Turn::GetTypeString(Turn::GetType(prev2curr_turn_degree)));
    auto node = trip_path_->GetEnhancedNode(i);
    for (size_t z = 0; z < node->intersecting_edge_size(); ++z) {
      auto intersecting_edge = node->GetIntersectingEdge(z);
      auto xturn_degree = GetTurnDegree(prev_edge->end_heading(), intersecting_edge->begin_heading());
      LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
      LOG_TRACE(std::string("    prev2int_turn_degree=") + std::to_string(xturn_degree) + " is a " +
                Turn::GetTypeString(Turn::GetType(xturn_degree)));
    }
    LOG_TRACE(std::string("  node=") + node->ToString());
    IntersectingEdgeCounts xedge_counts;
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                   xedge_counts);
    LOG_TRACE(std::string("    right=") + std::to_string(xedge_counts.right) +
              std::string(" | right_similar=") + std::to_string(xedge_counts.right_similar) +
              std::string(" | right_traversable_outbound=") +
              std::to_string(xedge_counts.right_traversable_outbound) +
              std::string(" | right_similar_traversable_outbound=") +
              std::to_string(xedge_counts.right_similar_traversable_outbound));
    LOG_TRACE(std::string("    left =") + std::to_string(xedge_counts.left) +
              std::string(" | left_similar =") + std::to_string(xedge_counts.left_similar) +
              std::string(" | left_traversable_outbound =") +
              std::to_string(xedge_counts.left_traversable_outbound) +
              std::string(" | left_similar_traversable_outbound =") +
              std::to_string(xedge_counts.left_similar_traversable_outbound));
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
  auto curr_edge = trip_path_->GetCurrEdge(0);
  LOG_TRACE("---------------------------------------------");
  LOG_TRACE(std::string("0") + ":  ");
  LOG_TRACE(std::string("  curr_edge_PARAMETERS=") +
            (curr_edge ? curr_edge->ToParameterString() : "NONE"));
  LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
  auto node = trip_path_->GetEnhancedNode(0);
  for (size_t z = 0; z < node->intersecting_edge_size(); ++z) {
    auto intersecting_edge = node->GetIntersectingEdge(z);
    LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
  }
  LOG_TRACE("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
  for (size_t z = 0; z < trip_path_->admin_size(); ++z) {
    auto admin = trip_path_->GetAdmin(z);
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

    if (next_man != maneuvers.end()) {
      ++next_man;
    }

    while (next_man != maneuvers.end()) {
      // Process common base names
      std::unique_ptr<StreetNames> common_base_names =
          curr_man->street_names().FindCommonBaseNames(next_man->street_names());

      // Get the begin edge of the next maneuver
      auto next_man_begin_edge = trip_path_->GetCurrEdge(next_man->begin_node_index());

      bool is_first_man = (curr_man == maneuvers.begin());

      LOG_TRACE("+++ Combine TOP ++++++++++++++++++++++++++++++++++++++++++++");
      // Collapse the TransitConnectionStart Maneuver
      // if the transit connection stop is a simple stop (not a station)
      if ((curr_man->type() == DirectionsLeg_Maneuver_Type_kTransitConnectionStart) &&
          next_man->IsTransit() &&
          curr_man->transit_connection_platform_info().type() == TransitPlatformInfo_Type_kStop) {
        LOG_TRACE("+++ Combine: Collapse the TransitConnectionStart Maneuver +++");
        curr_man = CollapseTransitConnectionStartManeuver(maneuvers, curr_man, next_man);
        maneuvers_have_been_combined = true;
        ++next_man;
      }
      // Collapse the TransitConnectionDestination Maneuver
      // if the transit connection stop is a simple stop (not a station)
      else if ((next_man->type() == DirectionsLeg_Maneuver_Type_kTransitConnectionDestination) &&
               curr_man->IsTransit() &&
               next_man->transit_connection_platform_info().type() ==
                   TransitPlatformInfo_Type_kStop) {
        LOG_TRACE("+++ Combine: Collapse the TransitConnectionDestination Maneuver +++");
        next_man = CollapseTransitConnectionDestinationManeuver(maneuvers, curr_man, next_man);
        maneuvers_have_been_combined = true;
      }
      // Do not combine
      // if any transit connection maneuvers
      else if (curr_man->transit_connection() || next_man->transit_connection()) {
        LOG_TRACE("+++ Do Not Combine: if any transit connection maneuvers +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if driving side is different
      else if (curr_man->drive_on_right() != next_man->drive_on_right()) {
        LOG_TRACE("+++ Do Not Combine: if driving side is different +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if travel mode is different
      // OR next maneuver is destination
      else if ((curr_man->travel_mode() != next_man->travel_mode()) ||
               (next_man->type() == DirectionsLeg_Maneuver_Type_kDestination)) {
        LOG_TRACE(
            "+++ Do Not Combine: if travel mode is different or next maneuver is destination +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if next maneuver is a fork or a tee
      else if (next_man->fork() || next_man->tee()) {
        LOG_TRACE("+++ Do Not Combine: if next maneuver is a fork or a tee +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if current or next maneuver is a ferry
      else if (curr_man->ferry() || next_man->ferry()) {
        LOG_TRACE("+++ Do Not Combine: if current or next maneuver is a ferry +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Combine current internal maneuver with next maneuver
      else if (curr_man->internal_intersection() && (curr_man != next_man) &&
               !next_man->IsDestinationType()) {
        LOG_TRACE("+++ Combine: current internal maneuver with next maneuver +++");
        curr_man = CombineInternalManeuver(maneuvers, prev_man, curr_man, next_man, is_first_man);
        if (is_first_man) {
          prev_man = curr_man;
        }
        maneuvers_have_been_combined = true;
        ++next_man;
      }
      // Combine current turn channel maneuver with next maneuver
      else if (IsTurnChannelManeuverCombinable(prev_man, curr_man, next_man, is_first_man)) {
        LOG_TRACE("+++ Combine: current turn channel maneuver with next maneuver +++");
        curr_man = CombineTurnChannelManeuver(maneuvers, prev_man, curr_man, next_man, is_first_man);
        if (is_first_man) {
          prev_man = curr_man;
        }
        maneuvers_have_been_combined = true;
        ++next_man;
      }
      // Do not combine
      // if next maneuver has an intersecting forward link
      else if (next_man->intersecting_forward_edge()) {
        LOG_TRACE("+++ Do Not Combine: if next maneuver has an intersecting forward link +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }
      // Do not combine
      // if travel type is different (unnamed pedestrian/bike)
      else if ((curr_man->unnamed_walkway() != next_man->unnamed_walkway()) ||
               (curr_man->unnamed_cycleway() != next_man->unnamed_cycleway()) ||
               (curr_man->unnamed_mountain_bike_trail() != next_man->unnamed_mountain_bike_trail())) {
        LOG_TRACE("+++ Do Not Combine: if travel type is different +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
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
      else if ((next_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
               (next_man_begin_edge && !next_man_begin_edge->IsTurnChannelUse()) &&
               !next_man->internal_intersection() && !curr_man->ramp() && !next_man->ramp() &&
               !curr_man->roundabout() && !next_man->roundabout() && !common_base_names->empty()) {

        LOG_TRACE("+++ Combine: Several factors +++");
        // If needed, set the begin street names
        if (!curr_man->HasBeginStreetNames() && !curr_man->portions_highway() &&
            (curr_man->street_names().size() > common_base_names->size())) {
          curr_man->set_begin_street_names(curr_man->street_names().clone());
        }

        // Update current maneuver street names
        curr_man->set_street_names(std::move(common_base_names));

        next_man = CombineManeuvers(maneuvers, curr_man, next_man);
        maneuvers_have_been_combined = true;
      }
      // Combine unnamed straight maneuvers
      else if ((next_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
               !curr_man->HasStreetNames() && !next_man->HasStreetNames() && !curr_man->IsTransit() &&
               !next_man->IsTransit() &&
               (next_man_begin_edge && !next_man_begin_edge->IsTurnChannelUse()) &&
               !next_man->internal_intersection() && !curr_man->ramp() && !next_man->ramp() &&
               !curr_man->roundabout() && !next_man->roundabout()) {

        LOG_TRACE("+++ Combine: unnamed straight maneuvers +++");
        next_man = CombineManeuvers(maneuvers, curr_man, next_man);
        maneuvers_have_been_combined = true;
      }
      // Combine ramp maneuvers
      else if (AreRampManeuversCombinable(curr_man, next_man)) {
        LOG_TRACE("+++ Combine: ramp maneuvers +++");
        next_man = CombineManeuvers(maneuvers, curr_man, next_man);
        maneuvers_have_been_combined = true;
      } else {
        LOG_TRACE("+++ Do Not Combine +++");
        // Update with no combine
        prev_man = curr_man;
        curr_man = next_man;
        ++next_man;
      }

      LOG_TRACE("+++ Combine BOTTOM +++++++++++++++++++++++++++++++++++++++++");
    }
  }
}

std::list<Maneuver>::iterator
ManeuversBuilder::CollapseTransitConnectionStartManeuver(std::list<Maneuver>& maneuvers,
                                                         std::list<Maneuver>::iterator curr_man,
                                                         std::list<Maneuver>::iterator next_man) {

  // Set begin node index
  next_man->set_begin_node_index(curr_man->begin_node_index());

  // Set begin shape index
  next_man->set_begin_shape_index(curr_man->begin_shape_index());

  return maneuvers.erase(curr_man);
}

std::list<Maneuver>::iterator ManeuversBuilder::CollapseTransitConnectionDestinationManeuver(
    std::list<Maneuver>& maneuvers,
    std::list<Maneuver>::iterator curr_man,
    std::list<Maneuver>::iterator next_man) {

  // Set end node index
  curr_man->set_end_node_index(next_man->end_node_index());

  // Set end shape index
  curr_man->set_end_shape_index(next_man->end_shape_index());

  return maneuvers.erase(next_man);
}

std::list<Maneuver>::iterator
ManeuversBuilder::CombineInternalManeuver(std::list<Maneuver>& maneuvers,
                                          std::list<Maneuver>::iterator prev_man,
                                          std::list<Maneuver>::iterator curr_man,
                                          std::list<Maneuver>::iterator next_man,
                                          bool start_man) {

  if (start_man) {
    // Determine turn degree current maneuver and next maneuver
    next_man->set_turn_degree(GetTurnDegree(curr_man->end_heading(), next_man->begin_heading()));
  } else {
    // Determine turn degree based on previous maneuver and next maneuver
    next_man->set_turn_degree(GetTurnDegree(prev_man->end_heading(), next_man->begin_heading()));
  }

  // Set the cross street names
  if (curr_man->HasUsableInternalIntersectionName()) {
    next_man->set_cross_street_names(curr_man->street_names().clone());
  }

  // Set the right and left internal turn counts
  next_man->set_internal_right_turn_count(curr_man->internal_right_turn_count());
  next_man->set_internal_left_turn_count(curr_man->internal_left_turn_count());

  // Set relative direction
  next_man->set_begin_relative_direction(
      ManeuversBuilder::DetermineRelativeDirection(next_man->turn_degree()));

  // If the relative direction is straight
  // and both internal left and right turns exist
  // then update the relative direction
  if ((next_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
      (curr_man->internal_left_turn_count() > 0) && (curr_man->internal_right_turn_count() > 0)) {
    LOG_TRACE("both left and right internal turn counts are > 0");
    next_man->set_begin_relative_direction(ManeuversBuilder::DetermineRelativeDirection(
        GetTurnDegree(prev_man->end_heading(), curr_man->end_heading())));
  }

  // Add distance
  next_man->set_length(next_man->length() + curr_man->length());

  // Add time
  next_man->set_time(next_man->time() + curr_man->time());

  // Add basic time
  next_man->set_basic_time(next_man->basic_time() + curr_man->basic_time());

  // TODO - heading?

  // Set begin node index
  next_man->set_begin_node_index(curr_man->begin_node_index());

  // Set begin shape index
  next_man->set_begin_shape_index(curr_man->begin_shape_index());

  if (start_man) {
    next_man->set_type(DirectionsLeg_Maneuver_Type_kStart);
  } else {
    // Set maneuver type to 'none' so the type will be processed again
    next_man->set_type(DirectionsLeg_Maneuver_Type_kNone);
    SetManeuverType(*(next_man));
  }

  return maneuvers.erase(curr_man);
}

std::list<Maneuver>::iterator
ManeuversBuilder::CombineTurnChannelManeuver(std::list<Maneuver>& maneuvers,
                                             std::list<Maneuver>::iterator prev_man,
                                             std::list<Maneuver>::iterator curr_man,
                                             std::list<Maneuver>::iterator next_man,
                                             bool start_man) {

  if (start_man) {
    // Determine turn degree current maneuver and next maneuver
    next_man->set_turn_degree(GetTurnDegree(curr_man->end_heading(), next_man->begin_heading()));
  } else {
    // Determine turn degree based on previous maneuver and next maneuver
    next_man->set_turn_degree(GetTurnDegree(prev_man->end_heading(), next_man->begin_heading()));
  }

  // Set relative direction
  next_man->set_begin_relative_direction(curr_man->begin_relative_direction());

  // Add distance
  next_man->set_length(next_man->length() + curr_man->length());

  // Add time
  next_man->set_time(next_man->time() + curr_man->time());

  // Add basic time
  next_man->set_basic_time(next_man->basic_time() + curr_man->basic_time());

  // TODO - heading?

  // Set begin node index
  next_man->set_begin_node_index(curr_man->begin_node_index());

  // Set begin shape index
  next_man->set_begin_shape_index(curr_man->begin_shape_index());

  if (start_man) {
    next_man->set_type(DirectionsLeg_Maneuver_Type_kStart);
  } else {
    // Set maneuver type to 'none' so the type will be processed again
    next_man->set_type(DirectionsLeg_Maneuver_Type_kNone);
    SetManeuverType(*(next_man));
  }

  return maneuvers.erase(curr_man);
}

std::list<Maneuver>::iterator
ManeuversBuilder::CombineManeuvers(std::list<Maneuver>& maneuvers,
                                   std::list<Maneuver>::iterator curr_man,
                                   std::list<Maneuver>::iterator next_man) {

  // Add distance
  curr_man->set_length(curr_man->length() + next_man->length());

  // Add time
  curr_man->set_time(curr_man->time() + next_man->time());

  // Add basic time
  curr_man->set_basic_time(curr_man->basic_time() + next_man->basic_time());

  // Update end heading
  curr_man->set_end_heading(next_man->end_heading());

  // Update end node index
  curr_man->set_end_node_index(next_man->end_node_index());

  // Update end shape index
  curr_man->set_end_shape_index(next_man->end_shape_index());

  // If needed, set ramp
  if (next_man->ramp()) {
    curr_man->set_ramp(true);
  }

  // If needed, set ferry
  if (next_man->ferry()) {
    curr_man->set_ferry(true);
  }

  // If needed, set rail_ferry
  if (next_man->rail_ferry()) {
    curr_man->set_rail_ferry(true);
  }

  // If needed, set roundabout
  if (next_man->roundabout()) {
    curr_man->set_roundabout(true);
  }

  // If needed, set portions_toll
  if (next_man->portions_toll()) {
    curr_man->set_portions_toll(true);
  }

  if (next_man->has_time_restrictions()) {
    curr_man->set_has_time_restrictions(true);
  }

  // If needed, set portions_unpaved
  if (next_man->portions_unpaved()) {
    curr_man->set_portions_unpaved(true);
  }

  // If needed, set portions_highway
  if (next_man->portions_highway()) {
    curr_man->set_portions_highway(true);
  }

  return maneuvers.erase(next_man);
}

void ManeuversBuilder::CountAndSortExitSigns(std::list<Maneuver>& maneuvers) {

  auto prev_man = maneuvers.rbegin();
  auto curr_man = maneuvers.rbegin();

  if (prev_man != maneuvers.rend()) {
    ++prev_man;
  }

  // Rank the exit signs
  while (prev_man != maneuvers.rend()) {

    // Increase the branch exit sign consecutive count
    // if it matches the succeeding named maneuver
    if (prev_man->HasExitBranchSign() && !curr_man->HasExitSign() && curr_man->HasStreetNames()) {
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
      CountAndSortExitSignList(prev_man->mutable_signs()->mutable_exit_number_list(),
                               curr_man->mutable_signs()->mutable_exit_number_list());

      // Process the exit branch signs
      CountAndSortExitSignList(prev_man->mutable_signs()->mutable_exit_branch_list(),
                               curr_man->mutable_signs()->mutable_exit_branch_list());

      // Process the exit toward signs
      CountAndSortExitSignList(prev_man->mutable_signs()->mutable_exit_toward_list(),
                               curr_man->mutable_signs()->mutable_exit_toward_list());

      // Process the exit name signs
      CountAndSortExitSignList(prev_man->mutable_signs()->mutable_exit_name_list(),
                               curr_man->mutable_signs()->mutable_exit_name_list());
    }

    // Update iterators
    curr_man = prev_man;
    ++prev_man;
  }
}

void ManeuversBuilder::ConfirmManeuverTypeAssignment(std::list<Maneuver>& maneuvers) {

  for (auto& maneuver : maneuvers) {
    SetManeuverType(maneuver, false);
  }
}

void ManeuversBuilder::CreateDestinationManeuver(Maneuver& maneuver) {
  int node_index = trip_path_->GetLastNodeIndex();

  // Determine if the destination has a side of street
  // and set the appropriate destination maneuver type
  switch (trip_path_->GetDestination().side_of_street()) {
    case Location::kLeft: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kDestinationLeft);
      LOG_TRACE("ManeuverType=DESTINATION_LEFT");
      break;
    }
    case Location::kRight: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kDestinationRight);
      LOG_TRACE("ManeuverType=DESTINATION_RIGHT");
      break;
    }
    default: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kDestination);
      LOG_TRACE("ManeuverType=DESTINATION");
    }
  }

  // Set the begin and end node index
  maneuver.set_begin_node_index(node_index);
  maneuver.set_end_node_index(node_index);

  // Set the begin and end shape index
  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  maneuver.set_begin_shape_index(prev_edge->end_shape_index());
  maneuver.set_end_shape_index(prev_edge->end_shape_index());

  // Travel mode
  maneuver.set_travel_mode(prev_edge->travel_mode());

  // Vehicle type
  if (prev_edge->has_vehicle_type()) {
    maneuver.set_vehicle_type(prev_edge->vehicle_type());
  }

  // Pedestrian type
  if (prev_edge->has_pedestrian_type()) {
    maneuver.set_pedestrian_type(prev_edge->pedestrian_type());
  }

  // Bicycle type
  if (prev_edge->has_bicycle_type()) {
    maneuver.set_bicycle_type(prev_edge->bicycle_type());
  }

  // Transit type
  if (prev_edge->has_transit_type()) {
    maneuver.set_transit_type(prev_edge->transit_type());
  }

  // Set the verbal text formatter
  maneuver.set_verbal_formatter(
      VerbalTextFormatterFactory::Create(trip_path_->GetCountryCode(node_index),
                                         trip_path_->GetStateCode(node_index)));
}

void ManeuversBuilder::CreateStartManeuver(Maneuver& maneuver) {
  int node_index = 0;

  // Determine if the origin has a side of street
  // and set the appropriate start maneuver type
  switch (trip_path_->GetOrigin().side_of_street()) {
    case Location::kLeft: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kStartLeft);
      LOG_TRACE("ManeuverType=START_LEFT");
      break;
    }
    case Location::kRight: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kStartRight);
      LOG_TRACE("ManeuverType=START_RIGHT");
      break;
    }
    default: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kStart);
      LOG_TRACE("ManeuverType=START");
    }
  }

  FinalizeManeuver(maneuver, node_index);
}

void ManeuversBuilder::InitializeManeuver(Maneuver& maneuver, int node_index) {

  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  auto curr_edge = trip_path_->GetCurrEdge(node_index);

  // Set the end heading
  maneuver.set_end_heading(prev_edge->end_heading());

  // Set the end node index
  maneuver.set_end_node_index(node_index);

  // Set the end shape index
  maneuver.set_end_shape_index(prev_edge->end_shape_index());

  // Ramp
  if (prev_edge->IsRampUse()) {
    maneuver.set_ramp(true);
  }

  // Turn Channel
  if (prev_edge->IsTurnChannelUse()) {
    maneuver.set_turn_channel(true);
  }

  // Ferry
  if (prev_edge->IsFerryUse()) {
    maneuver.set_ferry(true);
  }

  // Rail Ferry
  if (prev_edge->IsRailFerryUse()) {
    maneuver.set_rail_ferry(true);
  }

  // Roundabout
  if (AreRoundaboutsProcessable(prev_edge->travel_mode()) && prev_edge->roundabout()) {
    maneuver.set_roundabout(true);
    maneuver.set_roundabout_exit_count(1);
  }

  // Internal Intersection - excluding the first and last edges
  if (prev_edge->internal_intersection() && !trip_path_->IsLastNodeIndex(node_index) &&
      !trip_path_->IsFirstNodeIndex(node_index - 1)) {
    maneuver.set_internal_intersection(true);
  }

  // Travel mode
  maneuver.set_travel_mode(prev_edge->travel_mode());

  // Driving side
  maneuver.set_drive_on_right(prev_edge->drive_on_right());

  // Vehicle type
  if (prev_edge->has_vehicle_type()) {
    maneuver.set_vehicle_type(prev_edge->vehicle_type());
  }

  // Pedestrian type
  if (prev_edge->has_pedestrian_type()) {
    maneuver.set_pedestrian_type(prev_edge->pedestrian_type());
  }

  // Bicycle type
  if (prev_edge->has_bicycle_type()) {
    maneuver.set_bicycle_type(prev_edge->bicycle_type());
  }

  // Transit type
  if (prev_edge->has_transit_type()) {
    maneuver.set_transit_type(prev_edge->transit_type());
  }

  // Unnamed walkway
  maneuver.set_unnamed_walkway(prev_edge->IsUnnamedWalkway());

  // Unnamed cycleway
  maneuver.set_unnamed_cycleway(prev_edge->IsUnnamedCycleway());

  // Unnamed mountain bike trail
  maneuver.set_unnamed_mountain_bike_trail(prev_edge->IsUnnamedMountainBikeTrail());

  // Transit info
  if (prev_edge->travel_mode() == TripLeg_TravelMode_kTransit) {
    maneuver.set_rail(prev_edge->IsRailUse());
    maneuver.set_bus(prev_edge->IsBusUse());
    auto* transit_info = maneuver.mutable_transit_info();
    const auto& pe_transit_info = prev_edge->transit_route_info();
    transit_info->onestop_id = pe_transit_info.onestop_id();
    transit_info->block_id = pe_transit_info.block_id();
    transit_info->trip_id = pe_transit_info.trip_id();
    transit_info->short_name = pe_transit_info.short_name();
    transit_info->long_name = pe_transit_info.long_name();
    transit_info->headsign = pe_transit_info.headsign();
    transit_info->color = pe_transit_info.color();
    transit_info->text_color = pe_transit_info.text_color();
    transit_info->description = pe_transit_info.description();
    transit_info->operator_onestop_id = pe_transit_info.operator_onestop_id();
    transit_info->operator_name = pe_transit_info.operator_name();
    transit_info->operator_url = pe_transit_info.operator_url();
    LOG_TRACE("TransitInfo=" + transit_info->ToParameterString());
  }

  // Transit connection
  if (prev_edge->IsTransitConnection()) {
    maneuver.set_transit_connection(true);

    // If previous edge is transit connection platform
    // and current edge is transit then mark maneuver as transit connection start
    if (prev_edge->IsPlatformConnectionUse() && curr_edge &&
        (curr_edge->travel_mode() == TripLeg_TravelMode_kTransit)) {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransitConnectionStart);
      LOG_TRACE("ManeuverType=TRANSIT_CONNECTION_START");
      auto node = trip_path_->GetEnhancedNode(node_index);
      maneuver.set_transit_connection_platform_info(node->transit_platform_info());
    }
    // else mark it as transit connection destination
    else {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransitConnectionDestination);
      LOG_TRACE("ManeuverType=TRANSIT_CONNECTION_DESTINATION");
    }
  }

  // TODO - what about street names; maybe check name flag
  UpdateManeuver(maneuver, node_index);
}

void ManeuversBuilder::UpdateManeuver(Maneuver& maneuver, int node_index) {

  auto prev_edge = trip_path_->GetPrevEdge(node_index);

  // Street names
  // Set if street names are empty and maneuver is not internal intersection
  // or usable internal intersection name exists
  if ((maneuver.street_names().empty() && !maneuver.internal_intersection()) ||
      UsableInternalIntersectionName(maneuver, node_index)) {
    maneuver.set_street_names(
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), prev_edge->GetNameList()));
  }

  // Update the internal turn count
  UpdateInternalTurnCount(maneuver, node_index);

  // Distance in kilometers
  maneuver.set_length(maneuver.length() + prev_edge->length());

  // Basic time (len/speed on each edge with no stop impact) in seconds
  maneuver.set_basic_time(
      maneuver.basic_time() +
      GetTime(prev_edge->length(), GetSpeed(maneuver.travel_mode(), prev_edge->speed())));

  // Portions Toll
  if (prev_edge->toll()) {
    maneuver.set_portions_toll(true);
  }
  if (prev_edge->has_time_restrictions()) {
    maneuver.set_has_time_restrictions(true);
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
  if (AreRoundaboutsProcessable(prev_edge->travel_mode()) && prev_edge->roundabout()) {
    TripLeg_TravelMode mode = prev_edge->travel_mode();

    // Adjust bicycle travel mode if roundabout is a road
    if ((mode == TripLeg_TravelMode_kBicycle) && (prev_edge->IsRoadUse())) {
      mode = TripLeg_TravelMode_kDrive;
    }
    // TODO might have to adjust for pedestrian too

    IntersectingEdgeCounts xedge_counts;
    trip_path_->GetEnhancedNode(node_index)
        ->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), mode, xedge_counts);
    if (prev_edge->drive_on_right()) {
      maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count() +
                                         xedge_counts.right_traversable_outbound);
    } else {
      maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count() +
                                         xedge_counts.left_traversable_outbound);
    }
  }

  // Signs
  if (prev_edge->has_sign()) {
    // Exit number
    for (const auto& exit_number : prev_edge->sign().exit_numbers()) {
      maneuver.mutable_signs()
          ->mutable_exit_number_list()
          ->emplace_back(exit_number.text(), exit_number.is_route_number());
    }

    // Exit branch
    for (const auto& exit_onto_street : prev_edge->sign().exit_onto_streets()) {
      maneuver.mutable_signs()
          ->mutable_exit_branch_list()
          ->emplace_back(exit_onto_street.text(), exit_onto_street.is_route_number());
    }

    // Exit toward
    for (const auto& exit_toward_location : prev_edge->sign().exit_toward_locations()) {
      maneuver.mutable_signs()
          ->mutable_exit_toward_list()
          ->emplace_back(exit_toward_location.text(), exit_toward_location.is_route_number());
    }

    // Exit name
    for (const auto& exit_name : prev_edge->sign().exit_names()) {
      maneuver.mutable_signs()->mutable_exit_name_list()->emplace_back(exit_name.text(),
                                                                       exit_name.is_route_number());
    }
  }

  // Insert transit stop into the transit maneuver
  if (prev_edge->travel_mode() == TripLeg_TravelMode_kTransit) {
    auto node = trip_path_->GetEnhancedNode(node_index);
    maneuver.InsertTransitStop(node->transit_platform_info());
  }
}

void ManeuversBuilder::FinalizeManeuver(Maneuver& maneuver, int node_index) {

  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  auto curr_edge = trip_path_->GetCurrEdge(node_index);
  auto node = trip_path_->GetEnhancedNode(node_index);

  // Set begin cardinal direction
  maneuver.set_begin_cardinal_direction(DetermineCardinalDirection(curr_edge->begin_heading()));

  // Set the begin heading
  maneuver.set_begin_heading(curr_edge->begin_heading());

  // Set the begin node index
  maneuver.set_begin_node_index(node_index);

  // Set the begin shape index
  maneuver.set_begin_shape_index(curr_edge->begin_shape_index());

  // Set the time based on the delta of the elapsed time between the begin
  // and end nodes
  maneuver.set_time(trip_path_->node(maneuver.end_node_index()).elapsed_time() -
                    trip_path_->node(maneuver.begin_node_index()).elapsed_time());

  // if possible, set the turn degree and relative direction
  if (prev_edge) {
    maneuver.set_turn_degree(GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()));

    // Calculate and set the relative direction for the specified maneuver
    DetermineRelativeDirection(maneuver);

    // TODO - determine if we want to count right traversable at entrance node
    // Roundabouts
    //    if (AreRoundaboutsProcessable(prev_edge->travel_mode()) && curr_edge->roundabout()) {
    //      IntersectingEdgeCounts xedge_counts;
    //      trip_path_->GetEnhancedNode(node_index)
    //            ->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
    //                                                       prev_edge->travel_mode(),
    //                                                       xedge_counts);
    //      if (curr_edge->drive_on_right()) {
    //        maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count()
    //                                           + xedge_counts.right_traversable_outbound);
    //      } else {
    //        maneuver.set_roundabout_exit_count(maneuver.roundabout_exit_count()
    //                                           + xedge_counts.left_traversable_outbound);
    //      }
    //    }
  }

  // Mark transit connection transfer
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kTransitConnectionStart) && prev_edge &&
      (prev_edge->travel_mode() == TripLeg_TravelMode_kTransit)) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransitConnectionTransfer);
    LOG_TRACE("ManeuverType=TRANSIT_CONNECTION_TRANSFER");
  }

  // Add transit connection stop to a transit connection destination
  if ((maneuver.type() == DirectionsLeg_Maneuver_Type_kTransitConnectionDestination) && prev_edge &&
      (prev_edge->travel_mode() == TripLeg_TravelMode_kTransit)) {
    auto node = trip_path_->GetEnhancedNode(node_index);
    maneuver.set_transit_connection_platform_info(node->transit_platform_info());
    LOG_TRACE("DirectionsLeg_Maneuver_Type_kTransitConnectionDestination "
              "set_transit_connection_platform_info");
  }

  // Insert first transit stop
  if (maneuver.travel_mode() == TripLeg_TravelMode_kTransit) {
    auto node = trip_path_->GetEnhancedNode(node_index);
    maneuver.InsertTransitStop(node->transit_platform_info());
  }

  // Set the begin intersecting edge name consistency
  maneuver.set_begin_intersecting_edge_name_consistency(node->HasIntersectingEdgeNameConsistency());

  // Set begin street names
  if (!curr_edge->IsHighway() && !curr_edge->internal_intersection() &&
      (curr_edge->name_size() > 1)) {
    std::unique_ptr<StreetNames> curr_edge_names =
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), curr_edge->GetNameList());
    std::unique_ptr<StreetNames> common_base_names =
        curr_edge_names->FindCommonBaseNames(maneuver.street_names());
    if (curr_edge_names->size() > common_base_names->size()) {
      maneuver.set_begin_street_names(std::move(curr_edge_names));
    }
  }

  // Set the verbal text formatter
  maneuver.set_verbal_formatter(
      VerbalTextFormatterFactory::Create(trip_path_->GetCountryCode(node_index),
                                         trip_path_->GetStateCode(node_index)));

  // Set the maneuver type
  SetManeuverType(maneuver);
}

void ManeuversBuilder::SetManeuverType(Maneuver& maneuver, bool none_type_allowed) {
  // If the type is already set then just return
  if (maneuver.type() != DirectionsLeg_Maneuver_Type_kNone) {
    return;
  }

  auto prev_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());
  auto curr_edge = trip_path_->GetCurrEdge(maneuver.begin_node_index());

  // Process the different transit types
  if (maneuver.travel_mode() == TripLeg_TravelMode_kTransit) {
    if (prev_edge && prev_edge->travel_mode() == TripLeg_TravelMode_kTransit) {
      // Process transit remain on
      if ((maneuver.transit_info().block_id != 0) &&
          (maneuver.transit_info().block_id == prev_edge->transit_route_info().block_id()) &&
          (maneuver.transit_info().trip_id != prev_edge->transit_route_info().trip_id())) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransitRemainOn);
        LOG_TRACE("ManeuverType=TRANSIT_REMAIN_ON");
      }
      // Process transit transfer at same platform
      else {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransitTransfer);
        LOG_TRACE("ManeuverType=TRANSIT_TRANSFER");
      }
    }
    // Process simple transit
    else {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kTransit);
      LOG_TRACE("ManeuverType=TRANSIT");
    }
  }
  // Process post transit connection destination
  else if (prev_edge && prev_edge->IsTransitConnectionUse() &&
           (maneuver.travel_mode() != TripLeg_TravelMode_kTransit)) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kPostTransitConnectionDestination);
    LOG_TRACE("ManeuverType=POST_TRANSIT_CONNECTION_DESTINATION");
  }
  // Process enter roundabout
  else if (maneuver.roundabout()) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kRoundaboutEnter);
    LOG_TRACE("ManeuverType=ROUNDABOUT_ENTER");
  }
  // Process exit roundabout
  else if (prev_edge && AreRoundaboutsProcessable(prev_edge->travel_mode()) &&
           prev_edge->roundabout()) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kRoundaboutExit);
    LOG_TRACE("ManeuverType=ROUNDABOUT_EXIT");
  }
  // Process fork
  else if (maneuver.fork()) {
    switch (maneuver.begin_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight:
      case Maneuver::RelativeDirection::kRight: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kStayRight);
        LOG_TRACE("ManeuverType=FORK_STAY_RIGHT");
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft:
      case Maneuver::RelativeDirection::kLeft: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kStayLeft);
        LOG_TRACE("ManeuverType=FORK_STAY_LEFT");
        break;
      }
      default: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kStayStraight);
        LOG_TRACE("ManeuverType=FORK_STAY_STRAIGHT");
      }
    }
  }
  // Process Internal Intersection
  else if (none_type_allowed && maneuver.internal_intersection()) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kNone);
    LOG_TRACE("ManeuverType=INTERNAL_INTERSECTION");
  }
  // Process Turn Channel
  else if (none_type_allowed && maneuver.turn_channel()) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kNone);
    LOG_TRACE("ManeuverType=TURN_CHANNNEL");
  }
  // Process exit
  else if (maneuver.ramp() && prev_edge && (prev_edge->IsHighway() || maneuver.HasExitNumberSign())) {
    switch (maneuver.begin_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight:
      case Maneuver::RelativeDirection::kRight: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kExitRight);
        LOG_TRACE("ManeuverType=EXIT_RIGHT");
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft:
      case Maneuver::RelativeDirection::kLeft: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kExitLeft);
        LOG_TRACE("ManeuverType=EXIT_LEFT");
        break;
      }
      default: {
        LOG_TRACE(std::string("EXIT RelativeDirection=") +
                  std::to_string(static_cast<int>(maneuver.begin_relative_direction())));
        // TODO: determine how to handle, for now set to right
        if (maneuver.drive_on_right()) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kExitRight);
          LOG_TRACE("ManeuverType=EXIT_RIGHT");
        } else {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kExitLeft);
          LOG_TRACE("ManeuverType=EXIT_LEFT");
        }
      }
    }
  }
  // Process on ramp
  else if (maneuver.ramp() && prev_edge && !prev_edge->IsHighway()) {
    switch (maneuver.begin_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight:
      case Maneuver::RelativeDirection::kRight: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampRight);
        LOG_TRACE("ManeuverType=RAMP_RIGHT");
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft:
      case Maneuver::RelativeDirection::kLeft: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampLeft);
        LOG_TRACE("ManeuverType=RAMP_LEFT");
        break;
      }
      case Maneuver::RelativeDirection::kKeepStraight: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampStraight);
        LOG_TRACE("ManeuverType=RAMP_STRAIGHT");
        break;
      }
      case Maneuver::RelativeDirection::KReverse: {
        switch (Turn::GetType(maneuver.turn_degree())) {
          case Turn::Type::kSharpLeft: {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampLeft);
            LOG_TRACE("ManeuverType=RAMP_LEFT");
            break;
          }
          // For now default to right
          default: {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampRight);
            LOG_TRACE("ManeuverType=RAMP_RIGHT");
          }
        }
        break;
      }
      default: {
        LOG_TRACE(std::string("RAMP RelativeDirection=") +
                  std::to_string(static_cast<int>(maneuver.begin_relative_direction())));
        // TODO: determine how to handle, for now set to right
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kRampRight);
        LOG_TRACE("ManeuverType=RAMP_RIGHT");
      }
    }
  }
  // Process merge
  else if (IsMergeManeuverType(maneuver, prev_edge.get(), curr_edge.get())) {
    switch (maneuver.merge_to_relative_direction()) {
      case Maneuver::RelativeDirection::kKeepRight: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kMergeRight);
        LOG_TRACE("ManeuverType=MERGE_RIGHT");
        break;
      }
      case Maneuver::RelativeDirection::kKeepLeft: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kMergeLeft);
        LOG_TRACE("ManeuverType=MERGE_LEFT");
        break;
      }
      default: {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kMerge);
        LOG_TRACE("ManeuverType=MERGE");
      }
    }
  }
  // Process enter ferry
  else if (maneuver.ferry() || maneuver.rail_ferry()) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kFerryEnter);
    LOG_TRACE("ManeuverType=FERRY_ENTER");
  }
  // Process exit ferry
  else if (prev_edge && (prev_edge->IsFerryUse() || prev_edge->IsRailFerryUse())) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kFerryExit);
    LOG_TRACE("ManeuverType=FERRY_EXIT");
  }
  // Process simple direction
  else {
    LOG_TRACE("ManeuverType=SIMPLE");
    SetSimpleDirectionalManeuverType(maneuver, prev_edge.get(), curr_edge.get());
  }
}

void ManeuversBuilder::SetSimpleDirectionalManeuverType(Maneuver& maneuver,
                                                        EnhancedTripLeg_Edge* prev_edge,
                                                        EnhancedTripLeg_Edge* curr_edge) {
  switch (Turn::GetType(maneuver.turn_degree())) {
    case Turn::Type::kStraight: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kContinue);
      LOG_TRACE("ManeuverType=CONTINUE");
      if (trip_path_) {
        auto man_begin_edge = trip_path_->GetCurrEdge(maneuver.begin_node_index());
        auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
        bool prev_edge_has_names = (prev_edge ? !prev_edge->IsUnnamed() : false);

        ////////////////////////////////////////////////////////////////////
        // If the maneuver begin edge is a turn channel
        // and the relative direction is not a keep straight
        // then set as slight right based on a relative keep right direction
        //  OR  set as slight left based on a relative keep left direction
        if (man_begin_edge && man_begin_edge->IsTurnChannelUse() &&
            (maneuver.begin_relative_direction() != Maneuver::RelativeDirection::kKeepStraight)) {
          if (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightRight);
            LOG_TRACE("ManeuverType=SLIGHT_RIGHT");
          } else if (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightLeft);
            LOG_TRACE("ManeuverType=SLIGHT_LEFT");
          }
        }
        ////////////////////////////////////////////////////////////////////
        // If internal intersection at beginning of maneuver
        else if (curr_edge && curr_edge->internal_intersection()) {
          ////////////////////////////////////////////////////////////////////
          // Straight turn type but left relative direction
          if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft) ||
              (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft)) {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightLeft);
            LOG_TRACE("ManeuverType=SLIGHT_LEFT");
          }
          ////////////////////////////////////////////////////////////////////
          // Straight turn type but right relative direction
          else if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight) ||
                   (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight)) {
            maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightRight);
            LOG_TRACE("ManeuverType=SLIGHT_RIGHT");
          }
        }
        ////////////////////////////////////////////////////////////////////
        // turn type is straight and the following...
        // If maneuver is named
        // and previous edge is named
        // and no intersecting edge name consistency
        // and node is not a motorway_junction
        // and maneuver is not a highway with intersecting edges
        // then it is a "becomes" maneuver
        //        else if (maneuver.HasStreetNames() && prev_edge_has_names && node
        //            && !node->HasIntersectingEdgeNameConsistency()
        //            && !node->motorway_junction()
        //            && !(man_begin_edge && man_begin_edge->IsHighway()
        //                && node->HasIntersectingEdges())) {
        //          // Verify that there are no common names with previous edge and
        //          // current maneuver
        //          std::unique_ptr<StreetNames> prev_edge_names =
        //              StreetNamesFactory::Create(
        //                  trip_path_->GetCountryCode(maneuver.begin_node_index()),
        //                  prev_edge->GetNameList());
        //          std::unique_ptr<StreetNames> common_base_names = prev_edge_names
        //              ->FindCommonBaseNames(maneuver.street_names());
        //          LOG_INFO("prev_edge_names->size()=" + std::to_string(prev_edge_names->size()));
        //          LOG_INFO("common_base_names->size()=" +
        //          std::to_string(common_base_names->size())); if (common_base_names->empty()) {
        //            maneuver.set_type(DirectionsLeg_Maneuver_Type_kBecomes);
        //            LOG_TRACE("ManeuverType=BECOMES");
        //          }
        //        }
      }
      break;
    }
    case Turn::Type::kSlightRight: {
      // TODO refactor with enhanced trip path clean up
      IntersectingEdgeCounts xedge_counts;
      auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
      if (node && prev_edge) {
        node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                       prev_edge->travel_mode(), xedge_counts);
      }
      if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
          !maneuver.intersecting_forward_edge() &&
          ((xedge_counts.right > 0) || ((xedge_counts.right == 0) && (xedge_counts.left == 0)))) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kContinue);
        LOG_TRACE("ManeuverType=CONTINUE (Turn::Type::kSlightRight)");
      } else {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightRight);
        LOG_TRACE("ManeuverType=SLIGHT_RIGHT");
      }
      break;
    }
    case Turn::Type::kRight: {
      auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
      if (node && node->HasTraversableOutboundIntersectingEdge(maneuver.travel_mode())) {
        auto right_most_turn_degree =
            node->GetRightMostTurnDegree(maneuver.turn_degree(), prev_edge->end_heading(),
                                         maneuver.travel_mode());
        if (maneuver.turn_degree() == right_most_turn_degree) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kRight);
          LOG_TRACE("ManeuverType=RIGHT");
          break;
        } else if ((maneuver.turn_degree() < right_most_turn_degree) &&
                   !node->HasSpecifiedTurnXEdge(Turn::Type::kSlightRight, prev_edge->end_heading(),
                                                maneuver.travel_mode())) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightRight);
          LOG_TRACE("ManeuverType=SLIGHT_RIGHT");
          break;
        } else if ((maneuver.turn_degree() > right_most_turn_degree) &&
                   !node->HasSpecifiedTurnXEdge(Turn::Type::kSharpRight, prev_edge->end_heading(),
                                                maneuver.travel_mode())) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kSharpRight);
          LOG_TRACE("ManeuverType=SHARP_RIGHT");
          break;
        }
      }
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kRight);
      LOG_TRACE("ManeuverType=RIGHT");
      break;
    }
    case Turn::Type::kSharpRight: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kSharpRight);
      LOG_TRACE("ManeuverType=SHARP_RIGHT");
      break;
    }
    case Turn::Type::kReverse: {
      if (maneuver.internal_left_turn_count() > maneuver.internal_right_turn_count()) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
        LOG_TRACE("kReverse: 1 ManeuverType=UTURN_LEFT");
      } else if (maneuver.internal_right_turn_count() > maneuver.internal_left_turn_count()) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
        LOG_TRACE("kReverse: 1 ManeuverType=UTURN_RIGHT");
      } else if (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
        LOG_TRACE("kReverse: 2 ManeuverType=UTURN_LEFT");
      } else if (maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
        LOG_TRACE("kReverse: 2 ManeuverType=UTURN_RIGHT");
      } else if (trip_path_->GetCurrEdge(maneuver.begin_node_index())->drive_on_right()) {
        if (maneuver.turn_degree() < 180) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
          LOG_TRACE("kReverse: 3 ManeuverType=UTURN_RIGHT");
        } else {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
          LOG_TRACE("kReverse: 3 ManeuverType=UTURN_LEFT");
        }
      } else {
        if (maneuver.turn_degree() > 180) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
          LOG_TRACE("kReverse: 4 ManeuverType=UTURN_LEFT");
        } else {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
          LOG_TRACE("kReverse: 4 ManeuverType=UTURN_RIGHT");
        }
      }
      break;
    }
    case Turn::Type::kSharpLeft: {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kSharpLeft);
      LOG_TRACE("ManeuverType=SHARP_LEFT");
      break;
    }
    case Turn::Type::kLeft: {
      auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
      if (node && node->HasTraversableOutboundIntersectingEdge(maneuver.travel_mode())) {
        auto left_most_turn_degree =
            node->GetLeftMostTurnDegree(maneuver.turn_degree(), prev_edge->end_heading(),
                                        maneuver.travel_mode());
        if (maneuver.turn_degree() == left_most_turn_degree) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kLeft);
          LOG_TRACE("ManeuverType=LEFT");
          break;
        } else if ((maneuver.turn_degree() > left_most_turn_degree) &&
                   !node->HasSpecifiedTurnXEdge(Turn::Type::kSlightLeft, prev_edge->end_heading(),
                                                maneuver.travel_mode())) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightLeft);
          LOG_TRACE("ManeuverType=SLIGHT_LEFT");
          break;
        } else if ((maneuver.turn_degree() < left_most_turn_degree) &&
                   !node->HasSpecifiedTurnXEdge(Turn::Type::kSharpLeft, prev_edge->end_heading(),
                                                maneuver.travel_mode())) {
          maneuver.set_type(DirectionsLeg_Maneuver_Type_kSharpLeft);
          LOG_TRACE("ManeuverType=SHARP_LEFT");
          break;
        }
      }
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kLeft);
      LOG_TRACE("ManeuverType=LEFT");
      break;
    }
    case Turn::Type::kSlightLeft: {
      // TODO refactor with enhanced trip path clean up
      IntersectingEdgeCounts xedge_counts;
      auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
      if (node && prev_edge) {
        node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                       prev_edge->travel_mode(), xedge_counts);
      }
      if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
          !maneuver.intersecting_forward_edge() &&
          ((xedge_counts.left > 0) || ((xedge_counts.right == 0) && (xedge_counts.left == 0)))) {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kContinue);
        LOG_TRACE("ManeuverType=CONTINUE (Turn::Type::kSlightLeft)");
      } else {
        maneuver.set_type(DirectionsLeg_Maneuver_Type_kSlightLeft);
        LOG_TRACE("ManeuverType=SLIGHT_LEFT");
      }
      break;
    }
  }
}

DirectionsLeg_Maneuver_CardinalDirection
ManeuversBuilder::DetermineCardinalDirection(uint32_t heading) {
  if ((heading > 336) || (heading < 24)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kNorth;
  } else if ((heading > 23) && (heading < 67)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kNorthEast;
  } else if ((heading > 66) && (heading < 114)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kEast;
  } else if ((heading > 113) && (heading < 157)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kSouthEast;
  } else if ((heading > 156) && (heading < 204)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kSouth;
  } else if ((heading > 203) && (heading < 247)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kSouthWest;
  } else if ((heading > 246) && (heading < 294)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kWest;
  } else if ((heading > 293) && (heading < 337)) {
    return DirectionsLeg_Maneuver_CardinalDirection_kNorthWest;
  }
  throw valhalla_exception_t{220};
}

bool ManeuversBuilder::CanManeuverIncludePrevEdge(Maneuver& maneuver, int node_index) {
  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  auto curr_edge = trip_path_->GetCurrEdge(node_index);

  /////////////////////////////////////////////////////////////////////////////
  // Process transit
  if ((maneuver.travel_mode() == TripLeg_TravelMode_kTransit) &&
      (prev_edge->travel_mode() != TripLeg_TravelMode_kTransit)) {
    return false;
  }
  if ((prev_edge->travel_mode() == TripLeg_TravelMode_kTransit) &&
      (maneuver.travel_mode() != TripLeg_TravelMode_kTransit)) {
    return false;
  }
  if ((maneuver.travel_mode() == TripLeg_TravelMode_kTransit) &&
      (prev_edge->travel_mode() == TripLeg_TravelMode_kTransit)) {

    // Both block id and trip id must be the same so we can combine...
    if ((maneuver.transit_info().block_id == prev_edge->transit_route_info().block_id()) &&
        (maneuver.transit_info().trip_id == prev_edge->transit_route_info().trip_id())) {
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
  // If maneuver and prev edge are transit connections
  if (maneuver.transit_connection() && prev_edge->IsTransitConnection()) {

    // Logic for a transit entrance in reverse
    if (prev_edge->IsEgressConnectionUse() && curr_edge->IsPlatformConnectionUse()) {
      return true;
    } else if (prev_edge->IsTransitConnectionUse() && curr_edge->IsEgressConnectionUse()) {
      return true;
    }

    // Logic for a transit exit in reverse
    if (prev_edge->IsEgressConnectionUse() && curr_edge->IsTransitConnectionUse()) {
      return true;
    } else if (prev_edge->IsPlatformConnectionUse() && curr_edge->IsEgressConnectionUse()) {
      return true;
    }

    // Combine for station transfer
    if (prev_edge->IsPlatformConnectionUse() && curr_edge->IsPlatformConnectionUse()) {
      return true;
    }

    // If the expected order of transit connection types was not found
    // then do not combine
    return false;
  } else if (maneuver.transit_connection() || prev_edge->IsTransitConnection()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process driving side
  if (maneuver.drive_on_right() != prev_edge->drive_on_right()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process travel mode and travel types (unnamed pedestrian and bike)
  if (maneuver.travel_mode() != prev_edge->travel_mode()) {
    return false;
  }
  if (maneuver.unnamed_walkway() != prev_edge->IsUnnamedWalkway()) {
    return false;
  }
  if (maneuver.unnamed_cycleway() != prev_edge->IsUnnamedCycleway()) {
    return false;
  }
  if (maneuver.unnamed_mountain_bike_trail() != prev_edge->IsUnnamedMountainBikeTrail()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process roundabouts
  if (AreRoundaboutsProcessable(prev_edge->travel_mode())) {
    if (maneuver.roundabout() && !prev_edge->roundabout()) {
      return false;
    }
    if (prev_edge->roundabout() && !maneuver.roundabout()) {
      return false;
    }
    if (maneuver.roundabout() && prev_edge->roundabout()) {
      return true;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process fork
  if (IsFork(node_index, prev_edge.get(), curr_edge.get()) ||
      IsPedestrianFork(node_index, prev_edge.get(), curr_edge.get())) {
    maneuver.set_fork(true);
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process internal intersection
  // Cannot be the first edge in the trip
  if (prev_edge->internal_intersection() && !maneuver.internal_intersection()) {
    return false;
  } else if (!prev_edge->internal_intersection() && maneuver.internal_intersection()) {
    return false;
  } else if (prev_edge->internal_intersection() && !trip_path_->IsFirstNodeIndex(node_index - 1) &&
             maneuver.internal_intersection()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process simple turn channel
  if (prev_edge->IsTurnChannelUse() && !maneuver.turn_channel()) {
    return false;
  } else if (!prev_edge->IsTurnChannelUse() && maneuver.turn_channel()) {
    return false;
  } else if (prev_edge->IsTurnChannelUse() && maneuver.turn_channel()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process signs
  if (maneuver.HasExitSign()) {
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process ramps
  if (maneuver.ramp() && !prev_edge->IsRampUse()) {
    return false;
  }
  if (prev_edge->IsRampUse() && !maneuver.ramp()) {
    return false;
  }
  if (maneuver.ramp() && prev_edge->IsRampUse()) {
    // Do not combine if ramp to ramp is not forward
    if (!curr_edge->IsForward(GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()))) {
      return false;
    }
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process ferries
  if (maneuver.ferry() && !prev_edge->IsFerryUse()) {
    return false;
  }
  if (prev_edge->IsFerryUse() && !maneuver.ferry()) {
    return false;
  }
  if (maneuver.ferry() && prev_edge->IsFerryUse()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process rail ferries
  if (maneuver.rail_ferry() && !prev_edge->IsRailFerryUse()) {
    return false;
  }
  if (prev_edge->IsRailFerryUse() && !maneuver.rail_ferry()) {
    return false;
  }
  if (maneuver.rail_ferry() && prev_edge->IsRailFerryUse()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process simple u-turns
  if (GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()) == 180) {
    // If drive on right then left u-turn
    if (prev_edge->drive_on_right()) {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
      LOG_TRACE("ManeuverType=SIMPLE_UTURN_LEFT");
    } else {
      maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
      LOG_TRACE("ManeuverType=SIMPLE_UTURN_RIGHT");
    }
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process pencil point u-turns
  if (IsLeftPencilPointUturn(node_index, prev_edge.get(), curr_edge.get())) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnLeft);
    LOG_TRACE("ManeuverType=PENCIL_POINT_UTURN_LEFT");
    return false;
  }
  if (IsRightPencilPointUturn(node_index, prev_edge.get(), curr_edge.get())) {
    maneuver.set_type(DirectionsLeg_Maneuver_Type_kUturnRight);
    LOG_TRACE("ManeuverType=PENCIL_POINT_UTURN_RIGHT");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Intersecting forward edge
  if (IsIntersectingForwardEdge(node_index, prev_edge.get(), curr_edge.get())) {
    maneuver.set_intersecting_forward_edge(true);
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process 'T' intersection
  if (IsTee(node_index, prev_edge.get(), curr_edge.get())) {
    maneuver.set_tee(true);
    return false;
  }

  std::unique_ptr<StreetNames> prev_edge_names =
      StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

  /////////////////////////////////////////////////////////////////////////////
  // Process common base names
  std::unique_ptr<StreetNames> common_base_names =
      prev_edge_names->FindCommonBaseNames(maneuver.street_names());
  if (!common_base_names->empty()) {
    maneuver.set_street_names(std::move(common_base_names));
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process unnamed edge
  if (!maneuver.HasStreetNames() && prev_edge->IsUnnamed() &&
      IncludeUnnamedPrevEdge(node_index, prev_edge.get(), curr_edge.get())) {
    return true;
  }

  return false;
}

bool ManeuversBuilder::IncludeUnnamedPrevEdge(int node_index,
                                              EnhancedTripLeg_Edge* prev_edge,
                                              EnhancedTripLeg_Edge* curr_edge) const {

  auto node = trip_path_->GetEnhancedNode(node_index);

  if (!node->HasIntersectingEdges()) {
    return true;
  } else if (curr_edge->IsStraightest(GetTurnDegree(prev_edge->end_heading(),
                                                    curr_edge->begin_heading()),
                                      node->GetStraightestIntersectingEdgeTurnDegree(
                                          prev_edge->end_heading()))) {
    return true;
  }

  return false;
}

Maneuver::RelativeDirection
ManeuversBuilder::DetermineMergeToRelativeDirection(EnhancedTripLeg_Node* node,
                                                    EnhancedTripLeg_Edge* prev_edge) const {

  IntersectingEdgeCounts xedge_counts;
  node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                 xedge_counts);
  if ((xedge_counts.left > 0) && (xedge_counts.left_similar == 0) && (xedge_counts.right == 0)) {
    // If intersecting edge to the left and not the right then merge to the left
    return Maneuver::RelativeDirection::kKeepLeft;
  } else if ((xedge_counts.right > 0) && (xedge_counts.right_similar == 0) &&
             (xedge_counts.left == 0)) {
    // If intersecting edge to the right and not the left then merge to the right
    return Maneuver::RelativeDirection::kKeepRight;
  }
  // default to none
  return Maneuver::RelativeDirection::kNone;
}

bool ManeuversBuilder::IsMergeManeuverType(Maneuver& maneuver,
                                           EnhancedTripLeg_Edge* prev_edge,
                                           EnhancedTripLeg_Edge* curr_edge) const {
  auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
  // Previous edge is ramp and current edge is not a ramp
  // Current edge is a highway OR
  // Current edge is a trunk or primary, oneway, forward turn degree, and
  // consistent name with intersecting edge
  if (prev_edge && prev_edge->IsRampUse() && !curr_edge->IsRampUse() &&
      (curr_edge->IsHighway() ||
       (((curr_edge->road_class() == TripLeg_RoadClass_kTrunk) ||
         (curr_edge->road_class() == TripLeg_RoadClass_kPrimary)) &&
        curr_edge->IsOneway() && curr_edge->IsForward(maneuver.turn_degree()) &&
        node->HasIntersectingEdgeCurrNameConsistency()))) {
    maneuver.set_merge_to_relative_direction(
        DetermineMergeToRelativeDirection(node.get(), prev_edge));
    return true;
  }

  return false;
}

bool ManeuversBuilder::IsFork(int node_index,
                              EnhancedTripLeg_Edge* prev_edge,
                              EnhancedTripLeg_Edge* curr_edge) const {

  auto node = trip_path_->GetEnhancedNode(node_index);

  // If node is fork
  // and prev to curr edge is relative straight
  // and the intersecting edge count is less than 3
  // and there is a relative straight intersecting edge
  if (node->fork() &&
      curr_edge->IsWiderForward(
          GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading())) &&
      (node->intersecting_edge_size() < 3) &&
      node->HasWiderForwardTraversableIntersectingEdge(prev_edge->end_heading(),
                                                       curr_edge->travel_mode())) {
    // If the above criteria is met then check the following criteria...

    // If node is a motorway junction
    // and current edge is not a service road class
    // and an intersecting edge is a service road class
    // then not a fork
    if (node->IsMotorwayJunction() && (curr_edge->road_class() != TripLeg_RoadClass_kServiceOther) &&
        node->HasSpecifiedRoadClassXEdge(TripLeg_RoadClass_kServiceOther)) {
      return false;
    }

    IntersectingEdgeCounts xedge_counts;
    // TODO: update to pass similar turn threshold
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                   xedge_counts);

    // if there is a similar traversable intersecting edge
    //   or there is a traversable intersecting edge and curr edge is link(ramp)
    //   and the straightest intersecting edge is not in the reversed direction
    if (((xedge_counts.left_similar_traversable_outbound > 0) ||
         (xedge_counts.right_similar_traversable_outbound > 0)) ||
        (((xedge_counts.left_traversable_outbound > 0) ||
          (xedge_counts.right_traversable_outbound > 0)) &&
         curr_edge->IsRampUse() &&
         !node->IsStraightestTraversableIntersectingEdgeReversed(prev_edge->end_heading(),
                                                                 prev_edge->travel_mode()))) {
      return true;
    }
  }
  // Possibly move some logic to data processing in the future
  // Verify that both previous and current edges are highways
  // and the path is in the forward direction
  // and there are at most 2 intersecting edges
  // and there is an intersecting highway edge in the forward direction
  else if (prev_edge->IsHighway() && curr_edge->IsHighway() &&
           curr_edge->IsWiderForward(
               GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading())) &&
           (node->intersecting_edge_size() < 3) &&
           node->HasWiderForwardTraversableHighwayXEdge(prev_edge->end_heading(),
                                                        curr_edge->travel_mode())) {
    return true;
  }

  return false;
}

bool ManeuversBuilder::IsPedestrianFork(int node_index,
                                        EnhancedTripLeg_Edge* prev_edge,
                                        EnhancedTripLeg_Edge* curr_edge) const {
  auto is_relative_straight = [](uint32_t turn_degree) -> bool {
    return ((turn_degree > 315) || (turn_degree < 45));
  };
  auto node = trip_path_->GetEnhancedNode(node_index);
  uint32_t path_turn_degree = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());
  bool is_pedestrian_travel_mode = ((prev_edge->travel_mode() == TripLeg_TravelMode_kPedestrian) &&
                                    (curr_edge->travel_mode() == TripLeg_TravelMode_kPedestrian));

  // Must be pedestrian travel mode
  // and the path turn degree is relative straight
  // and less than 3 intersecting edges
  if (is_pedestrian_travel_mode && is_relative_straight(path_turn_degree) &&
      (node->intersecting_edge_size() < 3)) {
    // If the above criteria is met then check the following criteria...
    IntersectingEdgeCounts xedge_counts;
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                   xedge_counts);

    TripLeg_Use xedge_use;
    uint32_t straightest_traversable_xedge_turn_degree =
        node->GetStraightestTraversableIntersectingEdgeTurnDegree(prev_edge->end_heading(),
                                                                  prev_edge->travel_mode(),
                                                                  &xedge_use);
    // if there is a similar traversable intersecting edge
    //    or there is a relative straight traversable intersecting edge
    //    and the current edge use has to be the same as the intersecting edge use
    // or the previous edge is a roundabout and the current edge is not a roundabout
    // then we have a pedestrian fork
    if (((((xedge_counts.left_similar_traversable_outbound > 0) ||
           (xedge_counts.right_similar_traversable_outbound > 0)) ||
          is_relative_straight(straightest_traversable_xedge_turn_degree)) &&
         (curr_edge->use() == xedge_use)) ||
        (prev_edge->roundabout() && !curr_edge->roundabout())) {
      return true;
    }
  }

  return false;
}

bool ManeuversBuilder::IsTee(int node_index,
                             EnhancedTripLeg_Edge* prev_edge,
                             EnhancedTripLeg_Edge* curr_edge) const {

  auto node = trip_path_->GetEnhancedNode(node_index);
  // Verify only one intersecting edge
  if (node->intersecting_edge_size() == 1) {
    // Assign turn type
    Turn::Type turn_type =
        Turn::GetType(GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()));
    // Assign intersecting turn type
    Turn::Type xturn_type = Turn::GetType(
        GetTurnDegree(prev_edge->end_heading(), node->intersecting_edge(0).begin_heading()));

    // Intersecting edge must be traversable
    if (!(node->GetIntersectingEdge(0)->IsTraversable(prev_edge->travel_mode()))) {
      return false;
    }

    // Determine if 'T' intersection
    if ((turn_type == Turn::Type::kRight) && (xturn_type == Turn::Type::kLeft)) {
      return true;
    } else if ((turn_type == Turn::Type::kLeft) && (xturn_type == Turn::Type::kRight)) {
      return true;
    }
  }
  return false;
}

bool ManeuversBuilder::IsLeftPencilPointUturn(int node_index,
                                              EnhancedTripLeg_Edge* prev_edge,
                                              EnhancedTripLeg_Edge* curr_edge) const {

  uint32_t turn_degree = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());

  // If drive on right
  // and the the turn is a sharp left (179 < turn < 211)
  //    or short distance (< 50m) and wider sharp left (179 < turn < 226)
  // and oneway edges
  if (curr_edge->drive_on_right() &&
      (((turn_degree > 179) && (turn_degree < 211)) ||
       (((prev_edge->length() < 50) || (curr_edge->length() < 50)) && (turn_degree > 179) &&
        (turn_degree < 226))) &&
      prev_edge->IsOneway() && curr_edge->IsOneway()) {
    // If the above criteria is met then check the following criteria...

    IntersectingEdgeCounts xedge_counts;
    auto node = trip_path_->GetEnhancedNode(node_index);
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                   xedge_counts);

    std::unique_ptr<StreetNames> prev_edge_names =
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

    std::unique_ptr<StreetNames> curr_edge_names =
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), curr_edge->GetNameList());

    // Process common base names
    std::unique_ptr<StreetNames> common_base_names =
        prev_edge_names->FindCommonBaseNames(*curr_edge_names);

    // If no intersecting traversable left road exists
    // and the from and to edges have a common base name
    // then it is a left pencil point u-turn
    if ((xedge_counts.left_traversable_outbound == 0) && !common_base_names->empty()) {
      return true;
    }
  }

  return false;
}

bool ManeuversBuilder::IsRightPencilPointUturn(int node_index,
                                               EnhancedTripLeg_Edge* prev_edge,
                                               EnhancedTripLeg_Edge* curr_edge) const {

  uint32_t turn_degree = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());

  // If drive on left
  // and the turn is a sharp right (149 < turn < 181)
  //    or short distance (< 50m) and wider sharp right (134 < turn < 181)
  // and oneway edges
  if (curr_edge->drive_on_right() &&
      (((turn_degree > 149) && (turn_degree < 181)) ||
       (((prev_edge->length() < 50) || (curr_edge->length() < 50)) && (turn_degree > 134) &&
        (turn_degree < 181))) &&
      prev_edge->IsOneway() && curr_edge->IsOneway()) {
    // If the above criteria is met then check the following criteria...

    IntersectingEdgeCounts xedge_counts;
    auto node = trip_path_->GetEnhancedNode(node_index);
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                   xedge_counts);

    std::unique_ptr<StreetNames> prev_edge_names =
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), prev_edge->GetNameList());

    std::unique_ptr<StreetNames> curr_edge_names =
        StreetNamesFactory::Create(trip_path_->GetCountryCode(node_index), curr_edge->GetNameList());

    // Process common base names
    std::unique_ptr<StreetNames> common_base_names =
        prev_edge_names->FindCommonBaseNames(*curr_edge_names);

    // If no intersecting traversable right road exists
    // and the from and to edges have a common base name
    // then it is a right pencil point u-turn
    if ((xedge_counts.right_traversable_outbound == 0) && !common_base_names->empty()) {
      return true;
    }
  }

  return false;
}

bool ManeuversBuilder::IsIntersectingForwardEdge(int node_index,
                                                 EnhancedTripLeg_Edge* prev_edge,
                                                 EnhancedTripLeg_Edge* curr_edge) const {

  auto node = trip_path_->GetEnhancedNode(node_index);
  uint32_t turn_degree = GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading());

  if (node->HasIntersectingEdges() && !node->IsMotorwayJunction() && !node->fork() &&
      !(curr_edge->IsHighway() && prev_edge->IsHighway())) {
    // if path edge is not forward
    // and forward intersecting edge exists
    // then return true
    if (!curr_edge->IsForward(turn_degree) &&
        node->HasFowardIntersectingEdge(prev_edge->end_heading())) {
      return true;
    }
    // if path edge is forward
    // and forward traversable significant road class intersecting edge exists
    // and path edge is not the straightest
    // then return true
    else if (curr_edge->IsForward(turn_degree) &&
             node->HasForwardTraversableSignificantRoadClassXEdge(prev_edge->end_heading(),
                                                                  prev_edge->travel_mode(),
                                                                  prev_edge->road_class()) &&
             !curr_edge->IsStraightest(turn_degree,
                                       node->GetStraightestTraversableIntersectingEdgeTurnDegree(
                                           prev_edge->end_heading(), prev_edge->travel_mode()))) {
      return true;
    }
  }

  return false;
}

void ManeuversBuilder::DetermineRelativeDirection(Maneuver& maneuver) {
  auto prev_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());
  auto curr_edge = trip_path_->GetCurrEdge(maneuver.begin_node_index());

  IntersectingEdgeCounts xedge_counts;
  auto node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
  node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(), prev_edge->travel_mode(),
                                                 xedge_counts);

  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(maneuver.turn_degree());
  maneuver.set_begin_relative_direction(relative_direction);

  // Adjust keep straight, if needed
  if (relative_direction == Maneuver::RelativeDirection::kKeepStraight) {
    if ((xedge_counts.right_similar_traversable_outbound == 0) &&
        (xedge_counts.left_similar_traversable_outbound > 0)) {
      maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepRight);
    } else if ((xedge_counts.right_similar_traversable_outbound > 0) &&
               (xedge_counts.left_similar_traversable_outbound == 0)) {
      maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepLeft);
    } else if ((xedge_counts.left_similar_traversable_outbound == 0) &&
               (xedge_counts.left_traversable_outbound > 0) &&
               (xedge_counts.right_traversable_outbound == 0)) {
      if (!curr_edge->IsStraightest(maneuver.turn_degree(),
                                    node->GetStraightestTraversableIntersectingEdgeTurnDegree(
                                        prev_edge->end_heading(), prev_edge->travel_mode()))) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepRight);
      } else if (maneuver.turn_channel() &&
                 (Turn::GetType(maneuver.turn_degree()) != Turn::Type::kStraight)) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepRight);
      } else if (maneuver.fork()) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepRight);
      }
    } else if ((xedge_counts.right_similar_traversable_outbound == 0) &&
               (xedge_counts.right_traversable_outbound > 0) &&
               (xedge_counts.left_traversable_outbound == 0)) {
      if (!curr_edge->IsStraightest(maneuver.turn_degree(),
                                    node->GetStraightestTraversableIntersectingEdgeTurnDegree(
                                        prev_edge->end_heading(), prev_edge->travel_mode()))) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepLeft);
      } else if (maneuver.turn_channel() &&
                 (Turn::GetType(maneuver.turn_degree()) != Turn::Type::kStraight)) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepLeft);
      } else if (maneuver.fork()) {
        maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepLeft);
      }
    }
  } else if ((relative_direction == Maneuver::RelativeDirection::kLeft) &&
             (Turn::GetType(maneuver.turn_degree()) == Turn::Type::kSlightLeft) &&
             node->HasSpecifiedTurnXEdge(Turn::Type::kLeft, prev_edge->end_heading(),
                                         maneuver.travel_mode())) {
    maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepLeft);
  } else if ((relative_direction == Maneuver::RelativeDirection::kRight) &&
             (Turn::GetType(maneuver.turn_degree()) == Turn::Type::kSlightRight) &&
             node->HasSpecifiedTurnXEdge(Turn::Type::kRight, prev_edge->end_heading(),
                                         maneuver.travel_mode())) {
    maneuver.set_begin_relative_direction(Maneuver::RelativeDirection::kKeepRight);
  }
}

Maneuver::RelativeDirection ManeuversBuilder::DetermineRelativeDirection(uint32_t turn_degree) {
  if ((turn_degree > 329) || (turn_degree < 31)) {
    return Maneuver::RelativeDirection::kKeepStraight;
  } else if ((turn_degree > 30) && (turn_degree < 160)) {
    return Maneuver::RelativeDirection::kRight;
  } else if ((turn_degree > 159) && (turn_degree < 201)) {
    return Maneuver::RelativeDirection::KReverse;
  } else if ((turn_degree > 200) && (turn_degree < 330)) {
    return Maneuver::RelativeDirection::kLeft;
  } else {
    return Maneuver::RelativeDirection::kNone;
  }
}

bool ManeuversBuilder::UsableInternalIntersectionName(Maneuver& maneuver, int node_index) const {
  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  auto prev_prev_edge = trip_path_->GetPrevEdge(node_index, 2);
  uint32_t prev_prev_2prev_turn_degree = 0;
  if (prev_prev_edge) {
    prev_prev_2prev_turn_degree =
        GetTurnDegree(prev_prev_edge->end_heading(), prev_edge->begin_heading());
  }
  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(prev_prev_2prev_turn_degree);

  // Criteria for usable internal intersection name:
  // The maneuver is an internal intersection
  // Left turn for right side of the street driving
  // Right turn for left side of the street driving
  if (maneuver.internal_intersection() &&
      ((prev_edge->drive_on_right() && (relative_direction == Maneuver::RelativeDirection::kLeft)) ||
       (!prev_edge->drive_on_right() &&
        (relative_direction == Maneuver::RelativeDirection::kRight)))) {
    return true;
  }
  return false;
}

void ManeuversBuilder::UpdateInternalTurnCount(Maneuver& maneuver, int node_index) const {
  auto prev_edge = trip_path_->GetPrevEdge(node_index);
  auto prev_prev_edge = trip_path_->GetPrevEdge(node_index, 2);
  uint32_t prev_prev_2prev_turn_degree = 0;
  if (prev_prev_edge) {
    prev_prev_2prev_turn_degree =
        GetTurnDegree(prev_prev_edge->end_heading(), prev_edge->begin_heading());
  }
  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(prev_prev_2prev_turn_degree);

  if (relative_direction == Maneuver::RelativeDirection::kRight) {
    maneuver.set_internal_right_turn_count(maneuver.internal_right_turn_count() + 1);
  }
  if (relative_direction == Maneuver::RelativeDirection::kLeft) {
    maneuver.set_internal_left_turn_count(maneuver.internal_left_turn_count() + 1);
  }
}

float ManeuversBuilder::GetSpeed(TripLeg_TravelMode travel_mode, float edge_speed) const {
  // TODO use pedestrian and bicycle speeds from costing options?
  if (travel_mode == TripLeg_TravelMode_kPedestrian) {
    return 5.1f;
  } else if (travel_mode == TripLeg_TravelMode_kBicycle) {
    return 20.0f;
  } else {
    return edge_speed;
  }
}

bool ManeuversBuilder::IsTurnChannelManeuverCombinable(std::list<Maneuver>::iterator prev_man,
                                                       std::list<Maneuver>::iterator curr_man,
                                                       std::list<Maneuver>::iterator next_man,
                                                       bool start_man) const {

  // Current maneuver must be a turn channel and not equal to the next maneuver
  if (curr_man->turn_channel() && (curr_man != next_man)) {

    uint32_t new_turn_degree;
    if (start_man) {
      // Determine turn degree current maneuver and next maneuver
      new_turn_degree = GetTurnDegree(curr_man->end_heading(), next_man->begin_heading());
    } else {
      // Determine turn degree based on previous maneuver and next maneuver
      new_turn_degree = GetTurnDegree(prev_man->end_heading(), next_man->begin_heading());
    }

    Turn::Type new_turn_type = Turn::GetType(new_turn_degree);

    // Process simple right turn channel
    // Combineable if begin of turn channel is relative right
    // and next maneuver is not relative left direction
    // and final turn type is right or straight (not left)
    if (((curr_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepRight) ||
         (curr_man->begin_relative_direction() == Maneuver::RelativeDirection::kRight)) &&
        (next_man->begin_relative_direction() != Maneuver::RelativeDirection::kLeft) &&
        ((new_turn_type == Turn::Type::kSlightRight) || (new_turn_type == Turn::Type::kRight) ||
         (new_turn_type == Turn::Type::kSharpRight) || (new_turn_type == Turn::Type::kStraight))) {
      return true;
    }

    // Process simple left turn channel
    // Combineable if begin of turn channel is relative left
    // and next maneuver is not relative right direction
    // and final turn type is left or straight (not right)
    if (((curr_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepLeft) ||
         (curr_man->begin_relative_direction() == Maneuver::RelativeDirection::kLeft)) &&
        (next_man->begin_relative_direction() != Maneuver::RelativeDirection::kRight) &&
        ((new_turn_type == Turn::Type::kSlightLeft) || (new_turn_type == Turn::Type::kLeft) ||
         (new_turn_type == Turn::Type::kSharpLeft) || (new_turn_type == Turn::Type::kStraight))) {
      return true;
    }

    // Process simple straight "turn channel"
    if ((curr_man->begin_relative_direction() == Maneuver::RelativeDirection::kKeepStraight) &&
        (new_turn_type == Turn::Type::kStraight)) {
      return true;
    }
  }
  return false;
}

bool ManeuversBuilder::AreRampManeuversCombinable(std::list<Maneuver>::iterator curr_man,
                                                  std::list<Maneuver>::iterator next_man) const {
  if (curr_man->ramp() && next_man->ramp() && !next_man->fork() &&
      !curr_man->internal_intersection() && !next_man->internal_intersection()) {
    auto node = trip_path_->GetEnhancedNode(next_man->begin_node_index());
    if (!node->HasTraversableOutboundIntersectingEdge(next_man->travel_mode()) ||
        node->IsStraightestTraversableIntersectingEdgeReversed(curr_man->end_heading(),
                                                               next_man->travel_mode()) ||
        (next_man->type() == DirectionsLeg_Maneuver_Type_kRampStraight)) {
      return true;
    }
  }
  return false;
}

bool ManeuversBuilder::AreRoundaboutsProcessable(const TripLeg_TravelMode travel_mode) const {
  if ((travel_mode == TripLeg_TravelMode_kDrive) || (travel_mode == TripLeg_TravelMode_kBicycle)) {
    return true;
  }
  return false;
}

void ManeuversBuilder::ProcessRoundaboutNames(std::list<Maneuver>& maneuvers) {
  // Set previous maneuver
  auto prev_man = maneuvers.begin();

  // Set current maneuver
  auto curr_man = maneuvers.begin();
  auto next_man = maneuvers.begin();
  if (next_man != maneuvers.end()) {
    ++next_man;
    curr_man = next_man;
  }

  // Set next maneuver
  if (next_man != maneuvers.end()) {
    ++next_man;
  }

  // Walk the maneuvers to find roundabout maneuvers
  while (next_man != maneuvers.end()) {

    // Process roundabout maneuvers
    if (curr_man->roundabout()) {
      // Get the non route numbers for the roundabout
      std::unique_ptr<StreetNames> non_route_numbers = curr_man->street_names().GetNonRouteNumbers();

      // Clear out the current street name values
      curr_man->ClearStreetNames();
      curr_man->ClearBeginStreetNames();

      if (!non_route_numbers->empty()) {
        // Determine if there are street name matches between incoming and outgoing names
        std::unique_ptr<StreetNames> prev_common_base_names =
            non_route_numbers->FindCommonBaseNames(prev_man->street_names());
        std::unique_ptr<StreetNames> next_common_base_names =
            non_route_numbers->FindCommonBaseNames(next_man->street_names());
        // Use roundabout name if did not match incoming and outgoing names
        if (prev_common_base_names->empty() && next_common_base_names->empty()) {
          // Set roundabout name
          curr_man->set_street_names(std::move(non_route_numbers));
        }
      }

      // Process roundabout exit names
      if (next_man->type() == DirectionsLeg_Maneuver_Type_kRoundaboutExit) {
        if (next_man->HasBeginStreetNames()) {
          curr_man->set_roundabout_exit_street_names(next_man->begin_street_names().clone());
        } else {
          curr_man->set_roundabout_exit_street_names(next_man->street_names().clone());
        }
      }
    }

    // on to the next maneuver...
    prev_man = curr_man;
    curr_man = next_man;
    ++next_man;
  }
}

void ManeuversBuilder::SetToStayOnAttribute(std::list<Maneuver>& maneuvers) {
  // Set previous maneuver
  auto prev_man = maneuvers.begin();

  // Set current maneuver
  auto curr_man = maneuvers.begin();
  auto next_man = maneuvers.begin();
  if (next_man != maneuvers.end()) {
    ++next_man;
    curr_man = next_man;
  }

  // Set next maneuver
  if (next_man != maneuvers.end()) {
    ++next_man;
  }

  // Walk the maneuvers to find 'to stay on' maneuvers
  while (next_man != maneuvers.end()) {
    switch (curr_man->type()) {
      case DirectionsLeg_Maneuver_Type_kSlightRight:
      case DirectionsLeg_Maneuver_Type_kSlightLeft:
      case DirectionsLeg_Maneuver_Type_kRight:
      case DirectionsLeg_Maneuver_Type_kSharpRight:
      case DirectionsLeg_Maneuver_Type_kSharpLeft:
      case DirectionsLeg_Maneuver_Type_kLeft: {
        if (!curr_man->HasBeginStreetNames() && curr_man->HasSimilarNames(&(*prev_man), true)) {
          curr_man->set_to_stay_on(true);
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kStayStraight:
      case DirectionsLeg_Maneuver_Type_kStayRight:
      case DirectionsLeg_Maneuver_Type_kStayLeft: {
        if (curr_man->HasSimilarNames(&(*prev_man), true)) {
          if (!curr_man->ramp()) {
            curr_man->set_to_stay_on(true);
          } else if (curr_man->HasSimilarNames(&(*next_man), true)) {
            curr_man->set_to_stay_on(true);
          }
        }
        break;
      }
      case DirectionsLeg_Maneuver_Type_kUturnRight:
      case DirectionsLeg_Maneuver_Type_kUturnLeft: {
        if (curr_man->HasSameNames(&(*prev_man), true)) {
          curr_man->set_to_stay_on(true);
        }
        break;
      }
    }
    // on to the next maneuver...
    prev_man = curr_man;
    curr_man = next_man;
    ++next_man;
  }
}

void ManeuversBuilder::EnhanceSignlessInterchnages(std::list<Maneuver>& maneuvers) {
  auto prev_man = maneuvers.begin();
  auto curr_man = maneuvers.begin();
  auto next_man = maneuvers.begin();

  if (next_man != maneuvers.end()) {
    ++next_man;
  }

  // Walk the maneuvers to find signless interchange maneuvers to enhance
  while (next_man != maneuvers.end()) {

    // If the current maneuver is a ramp OR nameless fork and does not have any signage
    // and the previous maneuver is not a ramp or fork
    // and the next maneuver is a 'Merge maneuver'
    // then add the first street name from the next maneuver
    // to the current maneuver branch sign list
    if ((curr_man->ramp() || (curr_man->fork() && !curr_man->HasStreetNames())) &&
        !curr_man->HasExitSign() && !(prev_man->ramp() || prev_man->fork()) &&
        next_man->IsMergeType() && next_man->HasStreetNames()) {
      curr_man->mutable_signs()
          ->mutable_exit_branch_list()
          ->emplace_back(next_man->street_names().front()->value(),
                         next_man->street_names().front()->is_route_number());
    }

    // on to the next maneuver...
    prev_man = curr_man;
    curr_man = next_man;
    ++next_man;
  }
}

uint16_t ManeuversBuilder::GetExpectedTurnLaneDirection(Maneuver& maneuver) const {
  auto turn_lane_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());
  if (turn_lane_edge) {
    switch (maneuver.type()) {
      case valhalla::DirectionsLeg_Maneuver_Type_kUturnLeft:
        if (turn_lane_edge->HasTurnLane(kTurnLaneReverse)) {
          return kTurnLaneReverse;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kSharpLeft:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSharpLeft)) {
          return kTurnLaneSharpLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kLeft:
        if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSlightLeft) &&
                   (maneuver.turn_degree() > 270)) {
          return kTurnLaneSlightLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSharpLeft) &&
                   (maneuver.turn_degree() < 270)) {
          return kTurnLaneSharpLeft;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kSlightLeft:
      case valhalla::DirectionsLeg_Maneuver_Type_kExitLeft:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSlightLeft)) {
          return kTurnLaneSlightLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kRampLeft:
        if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kLeft) &&
            turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSlightLeft)) {
          return kTurnLaneSlightLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kStayLeft:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSlightLeft)) {
          return kTurnLaneSlightLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneLeft)) {
          return kTurnLaneLeft;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneThrough) &&
                   (turn_lane_edge->HasTurnLane(kTurnLaneRight) ||
                    turn_lane_edge->HasTurnLane(kTurnLaneSlightRight))) {
          return kTurnLaneThrough;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kBecomes:
      case valhalla::DirectionsLeg_Maneuver_Type_kContinue:
      case valhalla::DirectionsLeg_Maneuver_Type_kRampStraight:
      case valhalla::DirectionsLeg_Maneuver_Type_kStayStraight:
        return kTurnLaneThrough;
      case valhalla::DirectionsLeg_Maneuver_Type_kStayRight:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSlightRight)) {
          return kTurnLaneSlightRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneThrough) &&
                   (turn_lane_edge->HasTurnLane(kTurnLaneLeft) ||
                    turn_lane_edge->HasTurnLane(kTurnLaneSlightLeft))) {
          return kTurnLaneThrough;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kSlightRight:
      case valhalla::DirectionsLeg_Maneuver_Type_kExitRight:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSlightRight)) {
          return kTurnLaneSlightRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kRampRight:
        if ((maneuver.begin_relative_direction() == Maneuver::RelativeDirection::kRight) &&
            turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSlightRight)) {
          return kTurnLaneSlightRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kRight:
        if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSlightRight) &&
                   (maneuver.turn_degree() < 90)) {
          return kTurnLaneSlightRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneSharpRight) &&
                   (maneuver.turn_degree() > 90)) {
          return kTurnLaneSharpRight;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kSharpRight:
        if (turn_lane_edge->HasTurnLane(kTurnLaneSharpRight)) {
          return kTurnLaneSharpRight;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        }
        break;
      case valhalla::DirectionsLeg_Maneuver_Type_kUturnRight:
        if (turn_lane_edge->HasTurnLane(kTurnLaneReverse)) {
          return kTurnLaneReverse;
        } else if (turn_lane_edge->HasTurnLane(kTurnLaneRight)) {
          return kTurnLaneRight;
        }
        break;
      default:
        return kTurnLaneNone;
    }
  }
  return kTurnLaneNone;
}

void ManeuversBuilder::ProcessTurnLanes(std::list<Maneuver>& maneuvers) {
  auto curr_man = maneuvers.begin();
  auto next_man = maneuvers.begin();

  if (next_man != maneuvers.end()) {
    ++next_man;
  }

  // Walk the maneuvers to activate turn lanes
  while (next_man != maneuvers.end()) {

    // Only process driving maneuvers
    if (curr_man->travel_mode() == TripLeg_TravelMode::TripLeg_TravelMode_kDrive) {

      // Keep track of the remaining step distance in kilometers
      float remaining_step_distance = curr_man->length();

      // Walk maneuvers by node (prev_edge of node has the turn lane info)
      // Assign turn lane at transition point
      auto prev_edge = trip_path_->GetPrevEdge(curr_man->begin_node_index());
      if (prev_edge && (prev_edge->turn_lanes_size() > 0)) {
        // If not a short fork then process for turn lanes
        if (!((remaining_step_distance < kShortForkThreshold) &&
              ((curr_man->type() == DirectionsLeg_Maneuver_Type_kStayLeft) ||
               (curr_man->type() == DirectionsLeg_Maneuver_Type_kStayRight) ||
               (curr_man->type() == DirectionsLeg_Maneuver_Type_kStayStraight)))) {
          prev_edge->ActivateTurnLanes(GetExpectedTurnLaneDirection(*(curr_man)),
                                       remaining_step_distance, curr_man->type(), next_man->type());
        }
      }

      // Assign turn lanes within step
      for (auto index = (curr_man->begin_node_index() + 1); index < curr_man->end_node_index();
           ++index) {
        auto prev_edge = trip_path_->GetPrevEdge(index);
        if (prev_edge) {
          // Update the remaining step distance
          remaining_step_distance -= prev_edge->length();

          if (prev_edge->turn_lanes_size() > 0) {
            // For now just assume 'through' - we can enhance if needed
            prev_edge->ActivateTurnLanes(kTurnLaneThrough, remaining_step_distance, curr_man->type(),
                                         next_man->type());
          }
        }
      }

      // Do we mark maneuver?
    }
    // on to the next maneuver...
    curr_man = next_man;
    ++next_man;
  }
}

} // namespace odin
} // namespace valhalla
