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
  const DirectedEdge* directededge;
  const EdgeInfo* edgeinfo;
  for (const auto& edge : pathedges) {
    directededge = graphreader.GetGraphTile(edge)->directededge(edge);
    length += directededge->length();
    edgeinfo = graphreader.GetGraphTile(edge)->edgeinfo(directededge->edgedataoffset());
    // TODO - rm later
    edgeinfo->ToOstream();
  }
  return length;
}

}
}
