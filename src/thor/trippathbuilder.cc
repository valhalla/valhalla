#include <ostream>
#include <iostream>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/edgeinfo.h>

using namespace valhalla::baldr;
using namespace valhalla::odin;

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
// TODO - probably need the location information passed in - to
// add to the TripPath
TripPath TripPathBuilder::Build(GraphReader& graphreader,
                             const std::vector<GraphId>& pathedges) {
  // TripPath is a protocol buffer that contains information about the
  // trip
  TripPath trip_path;

  // TODO - support multiple legs
  TripPath_Leg* trip_leg = trip_path.add_leg();

  std::vector<std::string> names;
  for (const auto& edge : pathedges) {
    GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);

    // Add a node. TODO - where do we get its attributes?
    TripPath_Node* trip_node = trip_leg->add_node();

    // Add edge to the trip node and set its attributes
    TripPath_Edge* trip_edge = trip_node->add_edge();
    trip_edge->set_length(directededge->length());

    names = graphtile->GetNames(directededge->edgedataoffset(), names);
    for (auto& name : names) {
      trip_edge->add_name(name);
    }

    // TODO - add other connected edges to the node...

/**
    // TODO - rm later
    std::cout << "-------------------------------------------------------"
              << std::endl;
    names = graphtile->GetNames(directededge->edgedataoffset(), names);
    for (auto& name : names) {
      std::cout << "   name=" << name << std::endl;
    }
    const std::shared_ptr<EdgeInfo> edgeinfo = graphtile->edgeinfo(
        directededge->edgedataoffset());
    // TODO - rm later
    //edgeinfo->ToOstream();
*/
  }
  return trip_path;
}

}
}
