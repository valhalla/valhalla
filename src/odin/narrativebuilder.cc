#include <iostream>

#include "odin/narrativebuilder.h"

namespace valhalla {
namespace odin {

NarrativeBuilder::NarrativeBuilder(TripPath& trip_path)
    : trip_path_(trip_path) {
}

void NarrativeBuilder::Build() {
  for (const auto& node : trip_path_.node()) {
    // Get outbound edges from node
    const auto& edges = node.edge();
    // Get path edge
    const auto& edge = edges.Get(0);
    // Get the names on path edge
    const auto& names = edge.name();
    std::cout << "----------------------------------------------" << std::endl;
    for (const auto& name : names) {
      std::cout << "name=" << name << std::endl;
    }
    std::cout << "length=" << edge.length() << std::endl;
  }
}
}
}
