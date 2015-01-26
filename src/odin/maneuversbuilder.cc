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

#ifdef LOGGING_LEVEL_INFO
    auto* prev_edge = trip_path_->GetPrevEdge(i);
    auto* curr_edge = trip_path_->GetCurrEdge(i);
    auto* next_edge = trip_path_->GetNextEdge(i);
    LOG_TRACE("---------------------------------------------");
    LOG_TRACE(std::to_string(i) + ":  ");
    LOG_TRACE(std::string("  prev_edge=") + (prev_edge ? prev_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  curr_edge=") + (curr_edge ? curr_edge->ToString() : "NONE"));
    LOG_TRACE(std::string("  prev_curr_turn_degree=") + std::to_string(
        GetTurnDegree(prev_edge->end_heading(), curr_edge->begin_heading())));
    auto* node = trip_path_->GetEnhancedNode(i);
    for (size_t y = 0; y < node->GetIntersectingEdgesCount(); ++y) {
      auto* intersecting_edge = node->GetIntersectingEdge(y);
      LOG_TRACE(std::string("    intersectingEdge=") + intersecting_edge->ToString());
      LOG_TRACE(std::string("    prev_int_turn_degree=") + std::to_string(
          GetTurnDegree(prev_edge->end_heading(), intersecting_edge->begin_heading())));
    }
    LOG_TRACE(std::string("  next_edge=") + (next_edge ? next_edge->ToString() : "NONE"));
#endif

    if (CanManeuverIncludePrevEdge(maneuvers.front(), i)) {
      UpdateManeuver(maneuvers.front(), i);
    } else {
      // Finalize current maneuver
      FinalizeManeuver(maneuvers.front(), i);

      // Initialize new maneuver
      maneuvers.emplace_front(Maneuver());
      InitializeManeuver(maneuvers.front(), i);
    }
  }

  // Process the Start maneuver
  CreateStartManeuver(maneuvers.front());

  return maneuvers;
}

void ManeuversBuilder::Combine(std::list<Maneuver>& maneuvers) {
  // TODO - implement
}

void ManeuversBuilder::CreateDestinationManeuver(Maneuver& maneuver) {
  int nodeIndex = trip_path_->GetLastNodeIndex();

  // Set the destination maneuver type
  // TODO - side of street
  maneuver.set_type(TripDirections_Maneuver_Type_kDestination);

  // Set the begin and end node index
  maneuver.set_begin_node_index(nodeIndex);
  maneuver.set_end_node_index(nodeIndex);

  // Set the begin and end shape index
  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);
  maneuver.set_begin_shape_index(prevEdge->end_shape_index());
  maneuver.set_end_shape_index(prevEdge->end_shape_index());

}

void ManeuversBuilder::CreateStartManeuver(Maneuver& maneuver) {
  int nodeIndex = 0;

  // Set the start maneuver type
  // TODO - side of street
  maneuver.set_type(TripDirections_Maneuver_Type_kStart);

  FinalizeManeuver(maneuver, nodeIndex);
}

void ManeuversBuilder::InitializeManeuver(Maneuver& maneuver, int nodeIndex) {

  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);

  // Set the end heading
  maneuver.set_end_heading(prevEdge->end_heading());

  // Set the end node index
  maneuver.set_end_node_index(nodeIndex);

  // Set the end shape index
  maneuver.set_end_shape_index(prevEdge->end_shape_index());

  // Ramp
  if (prevEdge->ramp()) {
    maneuver.set_ramp(true);
  }

  // Ferry
  if (prevEdge->ferry()) {
    maneuver.set_ferry(true);
  }

  // Rail Ferry
  if (prevEdge->rail_ferry()) {
    maneuver.set_rail_ferry(true);
  }

  // Roundabout
  if (prevEdge->roundabout()) {
    maneuver.set_roundabout(true);
  }

  // TODO - what about street names; maybe check name flag
  UpdateManeuver(maneuver, nodeIndex);
}

void ManeuversBuilder::UpdateManeuver(Maneuver& maneuver, int nodeIndex) {

  // Set the begin and end shape index
  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);

  // Street names
  // TODO - improve
  if (maneuver.street_names().empty()) {
    auto* names = maneuver.mutable_street_names();
    for (const auto& name : prevEdge->name()) {
      names->push_back(name);
    }
  } else {
    // TODO
  }

  // Distance
  maneuver.set_distance(maneuver.distance() + prevEdge->length());

  // Time
  maneuver.set_time(
      maneuver.time() + GetTime(prevEdge->length(), prevEdge->speed()));

  // Portions Toll
  if (prevEdge->toll()) {
    maneuver.set_portions_toll(true);
  }

  // Portions unpaved
  if (prevEdge->unpaved()) {
    maneuver.set_portions_unpaved(true);
  }

  // Portions highway
  if (prevEdge->IsHighway()) {
    maneuver.set_portions_highway(true);
  }

}

void ManeuversBuilder::FinalizeManeuver(Maneuver& maneuver, int nodeIndex) {
  auto* currEdge = trip_path_->GetCurrEdge(nodeIndex);

  // if possible, set the turn degree
  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);
  if (prevEdge) {
    maneuver.set_turn_degree(
        GetTurnDegree(prevEdge->end_heading(), currEdge->begin_heading()));
  }

  // Set the maneuver type
  SetManeuverType(maneuver, nodeIndex);

  // Begin street names
  // TODO

  // Set begin cardinal direction
  maneuver.set_begin_cardinal_direction(
      DetermineCardinalDirection(currEdge->begin_heading()));

  // Set the begin heading
  maneuver.set_begin_heading(currEdge->begin_heading());

  // Set the begin node index
  maneuver.set_begin_node_index(nodeIndex);

  // Set the begin shape index
  maneuver.set_begin_shape_index(currEdge->begin_shape_index());

}

void ManeuversBuilder::SetManeuverType(Maneuver& maneuver, int nodeIndex) {
  // If the type is already set then just return
  if (maneuver.type() != TripDirections_Maneuver_Type_kNone) {
    return;
  }

  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);
  auto* currEdge = trip_path_->GetCurrEdge(nodeIndex);

  // TODO - iterate and expand; "Stay"

  // Process exit
  if (maneuver.ramp() && prevEdge->IsHighway()) {
    // TODO - calculate relative S/R/L on R/L roads
    maneuver.set_type(TripDirections_Maneuver_Type_kExitRight);
    LOG_TRACE("EXIT");
  }
  // Process on ramp
  else if (maneuver.ramp() && !prevEdge->IsHighway()) {
    // TODO - calculate relative S/R/L on R/L roads
    maneuver.set_type(TripDirections_Maneuver_Type_kRampRight);
    LOG_TRACE("RAMP");
  }
  // Process merge
  else if (currEdge->IsHighway() && prevEdge->ramp()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kMerge);
    LOG_TRACE("MERGE");
  }
  // Process enter roundabout
  else if (maneuver.roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutEnter);
    LOG_TRACE("ROUNDABOUT_ENTER");
  }
  // Process exit roundabout
  else if (prevEdge->roundabout()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kRoundaboutExit);
    LOG_TRACE("ROUNDABOUT_EXIT");
  }
  // Process enter ferry
  else if (maneuver.ferry() || maneuver.rail_ferry()) {
    maneuver.set_type(TripDirections_Maneuver_Type_kFerryEnter);
    LOG_TRACE("FERRY_ENTER");
  }
  // Process exit ferry
  else if (prevEdge->ferry() || prevEdge->rail_ferry()) {
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
                                                  int nodeIndex) {
  // TODO - fix it
  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);

  /////////////////////////////////////////////////////////////////////////////
  // Process ramps
  if (maneuver.ramp() && !prevEdge->ramp()) {
    return false;
  }
  if (prevEdge->ramp() && !maneuver.ramp()) {
    return false;
  }
  // TODO - more logic with exit signs
  if (maneuver.ramp() && prevEdge->ramp()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process ferries
  if (maneuver.ferry() && !prevEdge->ferry()) {
    return false;
  }
  if (prevEdge->ferry() && !maneuver.ferry()) {
    return false;
  }
  if (maneuver.ferry() && prevEdge->ferry()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process rail ferries
  if (maneuver.rail_ferry() && !prevEdge->rail_ferry()) {
    return false;
  }
  if (prevEdge->rail_ferry() && !maneuver.rail_ferry()) {
    return false;
  }
  if (maneuver.rail_ferry() && prevEdge->rail_ferry()) {
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Process roundabouts
  if (maneuver.roundabout() && !prevEdge->roundabout()) {
    return false;
  }
  if (prevEdge->roundabout() && !maneuver.roundabout()) {
    return false;
  }
  if (maneuver.roundabout() && prevEdge->roundabout()) {
    return true;
  }

  StreetNames prev_edge_names(prevEdge->name());

  // Process same names
  if (maneuver.street_names() == prev_edge_names) {
    return true;
  }

  // Process common names
  StreetNames common_street_names = maneuver.street_names()
      .FindCommonStreetNames(prev_edge_names);
  if (!common_street_names.empty()) {
    maneuver.set_street_names(std::move(common_street_names));
    return true;
  }

  return false;

}

}
}

