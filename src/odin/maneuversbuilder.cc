#include <iostream>
#include <stdexcept>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>

#include "odin/maneuversbuilder.h"

using namespace valhalla::midgard;

namespace valhalla {
namespace odin {

ManeuversBuilder::ManeuversBuilder(EnhancedTripPath* etp)
    : trip_path_(etp) {
}

std::list<Maneuver> ManeuversBuilder::Build() {
  // Create the maneuvers
  std::list<Maneuver> maneuvers = Produce();

  // Combine maneuvers
  Combine(maneuvers);

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
    //LOG_TRACE(std::string("  prev_edge=") + (prev_edge ? prev_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  prev2curr_turn_degree=") + std::to_string(
            GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading())));
    auto* node = trip_path_->GetEnhancedNode(i);
    for (size_t y = 0; y < node->GetIntersectingEdgesCount(); ++y) {
      auto* intersecting_edge = node->GetIntersectingEdge(y);
      LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
      LOG_TRACE(std::string("    prev2int_turn_degree=") + std::to_string(
              GetTurnDegree(prev_edge->end_heading(), intersecting_edge->begin_heading())));
    }
    uint32_t right_count;
    uint32_t right_similar_count;
    uint32_t left_count;
    uint32_t left_similar_count;
    node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
        right_count,
        right_similar_count,
        left_count,
        left_similar_count);
    LOG_TRACE(std::string("    right_count=") + std::to_string(right_count)
        + std::string("    left_count=") + std::to_string(left_count));
    LOG_TRACE(std::string("    right_similar_count=") + std::to_string(right_similar_count)
        + std::string("    left_similar_count=") + std::to_string(left_similar_count));
    //LOG_TRACE(std::string("  next_edge=") + (next_edge ? next_edge->ToString() : "NONE"));
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
  LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
  auto* node = trip_path_->GetEnhancedNode(0);
  for (size_t y = 0; y < node->GetIntersectingEdgesCount(); ++y) {
    auto* intersecting_edge = node->GetIntersectingEdge(y);
    LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
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
      StreetNames common_base_names = curr_man->street_names()
          .FindCommonBaseNames(next_man->street_names());

      // Combine current internal maneuver with next maneuver
      if (curr_man->internal_intersection() && (curr_man != next_man)) {
        curr_man = CombineInternalManeuver(maneuvers, prev_man, curr_man,
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
      else if ((next_man->begin_relative_direction()
          == Maneuver::RelativeDirection::kKeepStraight)
          && !common_base_names.empty()) {
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

  // Set maneuver type to 'none' so the type will be processed again
  next_man->set_type(TripDirections_Maneuver_Type_kNone);
  SetManeuverType(*(next_man));

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
  }

  // Internal_Intersection
  if (prev_edge->internal_intersection()) {
    maneuver.set_internal_intersection(true);
  }

  // TODO - what about street names; maybe check name flag
  UpdateManeuver(maneuver, node_index);
}

void ManeuversBuilder::UpdateManeuver(Maneuver& maneuver, int node_index) {

  // Set the begin and end shape index
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);

  // Street names
  // TODO - improve
  if (!maneuver.internal_intersection() && maneuver.street_names().empty()) {
    auto* names = maneuver.mutable_street_names();
    for (const auto& name : prev_edge->name()) {
      names->push_back(name);
    }
  }

  // Distance
  maneuver.set_distance(maneuver.distance() + prev_edge->length());

  // Time
  maneuver.set_time(
      maneuver.time() + GetTime(prev_edge->length(), prev_edge->speed()));

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

}

void ManeuversBuilder::FinalizeManeuver(Maneuver& maneuver, int node_index) {
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
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);
  if (prev_edge) {
    maneuver.set_turn_degree(
        GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading()));

    // Calculate and set the relative direction for the specified maneuver
    DetermineRelativeDirection(maneuver);
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
  // TripDirections_Maneuver_Type_kUturnRight
  // TripDirections_Maneuver_Type_kUturnLeft
  // TripDirections_Maneuver_Type_kStayStraight
  // TripDirections_Maneuver_Type_kStayRight
  // TripDirections_Maneuver_Type_kStayLeft

  // Process Internal Intersection
  if (maneuver.internal_intersection()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kNone);
    LOG_TRACE("INTERNAL_INTERSECTION");
  }
  // Process exit
  else if (maneuver.ramp()
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
    }LOG_TRACE("EXIT");
  }
  // Process on ramp
  else if (maneuver.ramp() && !prev_edge->IsHighway()) {
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
    }LOG_TRACE("RAMP");
  }
  // Process merge
  else if (curr_edge->IsHighway() && prev_edge->ramp()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kMerge);
    LOG_TRACE("MERGE");
  }
  // Process enter roundabout
  else if (maneuver.roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutEnter);
    LOG_TRACE("ROUNDABOUT_ENTER");
  }
  // Process exit roundabout
  else if (prev_edge->roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutExit);
    LOG_TRACE("ROUNDABOUT_EXIT");
  }
  // Process enter ferry
  else if (maneuver.ferry() || maneuver.rail_ferry()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kFerryEnter);
    LOG_TRACE("FERRY_ENTER");
  }
  // Process exit ferry
  else if (prev_edge->ferry() || prev_edge->rail_ferry()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kFerryExit);
    LOG_TRACE("FERRY_EXIT");
  }
  // Process simple direction
  else {
    SetSimpleDirectionalManeuverType(maneuver);
    LOG_TRACE("SIMPLE");
  }

}

void ManeuversBuilder::SetSimpleDirectionalManeuverType(Maneuver& maneuver) {
  uint32_t turn_degree = maneuver.turn_degree();
  if ((turn_degree > 349) || (turn_degree < 11)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kContinue);
  } else if ((turn_degree > 10) && (turn_degree < 45)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kSlightRight);
  } else if ((turn_degree > 44) && (turn_degree < 136)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRight);
  } else if ((turn_degree > 135) && (turn_degree < 181)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kSharpRight);
  } else if ((turn_degree > 180) && (turn_degree < 225)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kSharpLeft);
  } else if ((turn_degree > 224) && (turn_degree < 316)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kLeft);
  } else if ((turn_degree > 315) && (turn_degree < 350)) {
    maneuver.set_type(TripDirections_Maneuver_Type_kSlightLeft);
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
}

bool ManeuversBuilder::CanManeuverIncludePrevEdge(Maneuver& maneuver,
                                                  int node_index) {
  // TODO - fix it
  auto* prev_edge = trip_path_->GetPrevEdge(node_index);

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

  StreetNames prev_edge_names(prev_edge->name());

  // Process same names
  if (maneuver.street_names() == prev_edge_names) {
    return true;
  }

  // Process common base names
  StreetNames common_base_names = prev_edge_names.FindCommonBaseNames(
      maneuver.street_names());
  if (!common_base_names.empty()) {
    maneuver.set_street_names(std::move(common_base_names));
    return true;
  }

  return false;

}

void ManeuversBuilder::DetermineRelativeDirection(Maneuver& maneuver) {
  auto* prev_edge = trip_path_->GetPrevEdge(maneuver.begin_node_index());

  uint32_t right_count;
  uint32_t right_similar_count;
  uint32_t left_count;
  uint32_t left_similar_count;
  auto* node = trip_path_->GetEnhancedNode(maneuver.begin_node_index());
  // TODO driveable
  node->CalculateRightLeftIntersectingEdgeCounts(prev_edge->end_heading(),
                                                 right_count,
                                                 right_similar_count,
                                                 left_count,
                                                 left_similar_count);

  Maneuver::RelativeDirection relative_direction =
      ManeuversBuilder::DetermineRelativeDirection(maneuver.turn_degree());
  maneuver.set_begin_relative_direction(relative_direction);

  if ((right_similar_count == 0) && (left_similar_count > 0)
      && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
    maneuver.set_begin_relative_direction(
        Maneuver::RelativeDirection::kKeepRight);
  } else if ((right_similar_count > 0) && (left_similar_count == 0)
      && (relative_direction == Maneuver::RelativeDirection::kKeepStraight)) {
    maneuver.set_begin_relative_direction(
        Maneuver::RelativeDirection::kKeepLeft);

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

}
}

