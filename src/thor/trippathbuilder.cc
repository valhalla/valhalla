#include <ostream>
#include <iostream>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/midgard/pointll.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
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
// TODO - probably need the location information passed in - to
// add to the TripPath
void TripPathBuilder::Build(
    GraphReader& graphreader, const std::vector<GraphId>& pathedges,
    TripPath& trip_path) {
  // TripPath is a protocol buffer that contains information about the
  // trip

  std::vector<PointLL> trip_shape;

  std::vector<std::string> names;
  for (const auto& edge : pathedges) {
    GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);

    // Add a node. TODO - where do we get its attributes?
    TripPath_Node* trip_node = trip_path.add_node();

    // Add edge to the trip node and set its attributes
    TripPath_Edge* trip_edge = trip_node->add_edge();

    names = graphtile->GetNames(directededge->edgedataoffset(), names);
    for (const auto& name : names) {
      trip_edge->add_name(name);
    }

    trip_edge->set_length(directededge->length());

    trip_edge->set_speed(directededge->speed());

    const std::shared_ptr<EdgeInfo> edgeinfo = graphtile->edgeinfo(
        directededge->edgedataoffset());

    bool is_reverse = false;

    if (directededge->forward()) { //Edge is in the forward direction.

      if (directededge->forwardaccess() && directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kBoth);
      else if (directededge->forwardaccess() && !directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kForward);
      else if (!directededge->forwardaccess() && directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kBackward);
      else
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kNone);

    } else { //Edge is in the reverse direction.  We must flip everything.

      if (directededge->forwardaccess() && directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kBoth);
      else if (!directededge->forwardaccess() && directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kForward);
      else if (directededge->forwardaccess() && !directededge->reverseaccess())
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kBackward);
      else
        trip_edge->set_driveability(
            TripPath_Driveability::TripPath_Driveability_kNone);

      //Add shape in the reverse direction.
      is_reverse = true;
    }

    trip_edge->set_begin_shape_index(trip_shape.size());

    if(is_reverse)
      trip_shape.insert(trip_shape.end(), edgeinfo->shape().rbegin() + (trip_shape.size() ? 1 : 0), edgeinfo->shape().rend());
    else
      trip_shape.insert(trip_shape.end(), edgeinfo->shape().begin() + (trip_shape.size() ? 1 : 0), edgeinfo->shape().end());

    trip_edge->set_end_shape_index(trip_shape.size());

    trip_edge->set_ramp(directededge->link());

    trip_edge->set_toll(directededge->toll());

    trip_edge->set_begin_heading(PointLL::HeadingAlongPolyline(edgeinfo->shape(),30));

    trip_edge->set_end_heading(PointLL::HeadingAtEndOfPolyline(edgeinfo->shape(),30));
    
  }

  //encode shape and add to trip path.
  std::string encoded_shape_;

  if (trip_shape.size())
    encoded_shape_ = encode<std::vector<PointLL> >(trip_shape);

  trip_path.set_shape(encoded_shape_);

}

}
}
