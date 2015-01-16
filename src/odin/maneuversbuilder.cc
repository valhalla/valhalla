#include <iostream>
#include <stdexcept>

#include "odin/maneuversbuilder.h"

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
    // GDG
    auto* prevEdge = trip_path_->GetPrevEdge(i);
    auto* currEdge = trip_path_->GetCurrEdge(i);
    auto* nextEdge = trip_path_->GetNextEdge(i);
    std::cout << i << ":  ";
    if (prevEdge)
      std::cout
          << "prevEdge="
          << ((prevEdge->name_size() > 0) ? prevEdge->name(0) : "unnamed");
    else
      std::cout << "prevEdge=NONE";

    if (currEdge)
      std::cout
          << "  | currEdge="
          << ((currEdge->name_size() > 0) ? currEdge->name(0) : "unnamed");
    else
      std::cout << "  | currEdge=NONE";

    if (nextEdge)
      std::cout << "  | nextEdge="
                << ((nextEdge->name_size() > 0) ? nextEdge->name(0) : "unnamed")
                << std::endl;
    else
      std::cout << "  | nextEdge=NONE" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;

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

  FinalizeManeuver(maneuver, nodeIndex);

  // Set the start maneuver type
  // TODO - side of street
  maneuver.set_type(TripDirections_Maneuver_Type_kStart);

}

void ManeuversBuilder::InitializeManeuver(Maneuver& maneuver, int nodeIndex) {

  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);

  // Set the end heading
  maneuver.set_end_heading(prevEdge->end_heading());

  // Set the end node index
  maneuver.set_end_node_index(nodeIndex);

  // Set the end shape index
  maneuver.set_end_shape_index(prevEdge->end_shape_index());

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
      names->emplace_back(StreetName(name));
    }
  } else {
    // TODO
  }

  // Distance
  maneuver.set_distance(maneuver.distance() + prevEdge->length());

  // Time
  // TODO

  // Ramp
  if (prevEdge->ramp()) {
    maneuver.set_ramp(true);
  }

  // Portions Toll
  if (prevEdge->toll()) {
    maneuver.set_portions_toll(true);
  }

  // Portions unpaved
  if (prevEdge->unpaved()) {
    maneuver.set_portions_unpaved(true);
  }

}

void ManeuversBuilder::FinalizeManeuver(Maneuver& maneuver, int nodeIndex) {
  auto* currEdge = trip_path_->GetCurrEdge(nodeIndex);

  // Set the maneuver type
  SetManeuverType(maneuver, nodeIndex);

  // Begin street names
  // TODO

  // Set begin cardinal direction
  // TODO
  //maneuver.set_begin_cardinal_direction(TODO);

  // Set the begin heading
  maneuver.set_begin_heading(currEdge->begin_heading());

  // Set the begin node index
  maneuver.set_begin_node_index(nodeIndex);

  // Set the begin shape index
  maneuver.set_begin_shape_index(currEdge->begin_shape_index());

}

void ManeuversBuilder::SetManeuverType(Maneuver& maneuver, int nodeIndex) {
  // TODO - fix it
  maneuver.set_type(TripDirections_Maneuver_Type_kContinue);
}

bool ManeuversBuilder::CanManeuverIncludePrevEdge(Maneuver& maneuver,
                                                  int nodeIndex) {
  // TODO - fix it
  auto* prevEdge = trip_path_->GetPrevEdge(nodeIndex);

  // TODO - add to streetnames
  StreetNames new_names;
  for (const auto& name : prevEdge->name()) {
    new_names.push_back(StreetName(name));
  }

  if (new_names == maneuver.street_names()) {
    return true;
  }

  return false;

}

}
}

