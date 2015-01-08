#include <iostream>

#include "odin/narrativebuilder.h"

namespace valhalla {
namespace odin {

NarrativeBuilder::NarrativeBuilder(TripPath& trip_path)
    : trip_path_(trip_path) {
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

void NarrativeBuilder::Build() {
  for (const auto& node : trip_path_.node()) {
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

  for (size_t i = 0, n = maneuver_names_.size(); i < n; i++) {
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "Take " << GetStreetName(maneuver_names_[i]) << " for "
        << maneuver_distance_[i] << " km" << std::endl;
  }
}
}
}
