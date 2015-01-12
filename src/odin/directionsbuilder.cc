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

TripDirections DirectionsBuilder::BuildSimple(const TripPath& trip_path) {
  // TODO - temps for initial end to end test
  std::vector<std::vector<std::string>> maneuver_names;
  std::vector<float> maneuver_distance;

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
    if (maneuver_names.empty()) {
      maneuver_names.push_back(new_names);
      maneuver_distance.push_back(0.0f);
    }

    if (maneuver_names.back() == new_names) {
      maneuver_distance.back() += edge.length();
    } else {
      maneuver_names.push_back(new_names);
      maneuver_distance.push_back(edge.length());
    }
  }

  TripDirections trip_directions;
  for (size_t i = 0, n = maneuver_names.size(); i < n; i++) {
    auto* maneuver = trip_directions.add_maneuver();
    maneuver->set_type(TripDirections_Type_kContinue);
    maneuver->set_length(maneuver_distance[i]);
    maneuver->set_text_instruction("Take " + GetStreetName(maneuver_names[i]));
    for (const auto& name : maneuver_names[i]) {
      maneuver->add_street_name(name);
    }
  }

  return trip_directions;
}

TripDirections DirectionsBuilder::Build(const TripPath& trip_path) {
  // Create maneuvers
  // TODO

  // Combine maneuvers
  // TODO

  // Create the narrative
  // TODO

  // Return trip directions
  TripDirections trip_directions;
  return trip_directions;
}

}
}
