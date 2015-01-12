#include <iostream>

#include "odin/directionsbuilder.h"

namespace valhalla {
namespace odin {

DirectionsBuilder::DirectionsBuilder() {
}

namespace {
std::string GetStreetName(std::vector<std::string>& maneuver_names) {
  std::string street_name;
  if (maneuver_names.empty()) {
    street_name = "unnamed road";
  } else {
    for (const auto& name : maneuver_names) {
      if (!street_name.empty()) {
        street_name += "/";
      }
      street_name += name;
    }
  }
  return street_name;
}

}

void DirectionsBuilder::BuildSimple(const TripPath& trip_path) {
  // TODO - temps for initial end to end test
  std::vector<std::vector<std::string>> maneuver_names_;
  std::vector<float> maneuver_distance_;

  for (const auto& node : trip_path.node()) {
    // Get outbound edges from node
    const auto& edges = node.edge();
    // Get path edge
    const auto& edge = edges.Get(0);
    // Get the names on path edge
    const auto& names = edge.name();
    std::vector<std::string> new_names;
    for (const auto name : names) {
      new_names.push_back(name);
    }
    if (maneuver_names_.empty()) {
      maneuver_names_.push_back(new_names);
      maneuver_distance_.push_back(0.0f);
    }

    if (maneuver_names_.back() == new_names) {
      maneuver_distance_.back() += edge.length();
    } else {
      maneuver_names_.push_back(new_names);
      maneuver_distance_.push_back(edge.length());
    }
  }

  float totalDistance = 0.0f;
  for (size_t i = 0, n = maneuver_names_.size(); i < n; i++) {
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Take " << GetStreetName(maneuver_names_[i]) << " for "
        << maneuver_distance_[i] << " km" << std::endl;
    totalDistance += maneuver_distance_[i];
  }
    std::cout << "==============================================" << std::endl;
    std::cout << "Total distance: " << totalDistance << " km" << std::endl;
}

void DirectionsBuilder::Build(const TripPath& trip_path) {
  // Create maneuvers
  // TODO

  // Combine maneuvers
  // TODO

  // Create the narrative
  // TODO

  // Return trip directions
  // TODO
}

}
}
