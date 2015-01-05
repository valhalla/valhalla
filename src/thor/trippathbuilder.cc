#include <ostream>
#include <iostream>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/edgeinfo.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace thor {

// Default constructor
TripPathBuilder::TripPathBuilder() {
}

// Destructor
TripPathBuilder::~TripPathBuilder() {
}

// For now just find the length of the path!
// TODO - alter the return to be trip path structure.
float TripPathBuilder::Build(GraphReader& graphreader,
                             const std::vector<GraphId>& pathedges) {
  float length = 0.0f;
  std::vector<std::string> names;
  for (const auto& edge : pathedges) {
    GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);
    length += directededge->length();
    // GDG - rm later
    std::cout << __FILE__ << ":" << __LINE__ << " | edgedataoffset="
              << directededge->edgedataoffset() << std::endl;
    names = graphtile->GetNames(directededge->edgedataoffset(), names);
    for (auto& name : names) {
      std::cout << "   name=" << name << std::endl;
    }
    const std::shared_ptr<EdgeInfo> edgeinfo = graphtile->edgeinfo(
        directededge->edgedataoffset());
    // TODO - rm later
    edgeinfo->ToOstream();
  }
  return length;
}

}
}
