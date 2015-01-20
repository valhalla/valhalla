#include <ostream>
#include <iostream>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/midgard/pointll.h>

namespace valhalla {
namespace thor {

// Meters offset from start/end of shape for finding heading
constexpr float kMetersOffsetForHeading =30.0f;

// Default constructor
TripPathBuilder::TripPathBuilder() {
}

// Destructor
TripPathBuilder::~TripPathBuilder() {
}

// For now just find the length of the path!
// TODO - probably need the location information passed in - to
// add to the TripPath
TripPath TripPathBuilder::Build(GraphReader& graphreader,
                                const std::vector<GraphId>& pathedges) {
  // TripPath is a protocol buffer that contains information about the trip
  TripPath trip_path;

  // TODO - what about the first node? Probably should pass it in?
  uint32_t shortcutcount = 0;
  const NodeInfo* nodeinfo = nullptr;

  // Iterate through path edges
  uint32_t prior_opp_index;
  std::vector<PointLL> trip_shape;
  for (const auto& edge : pathedges) {
    GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);

    // Skip transition edges
    if (directededge->trans_up() || directededge->trans_down()) {
      // TODO - remove debug stuff later.
      if (directededge->trans_up()) {
        std::cout << "Transition up!" << std::endl;
      } else {
        std::cout << "Transition down" << std::endl;
      }
      // Get the end node (needed for connected edges at next iteration).
      GraphId endnode = directededge->endnode();
      nodeinfo = graphreader.GetGraphTile(endnode)->node(
                      directededge->endnode());
      // TODO - how do we find the opposing edge to the incoming edge
      // on the prior level?
      prior_opp_index = kMaxEdgesPerNode + 1;
      continue;
    }

    // Add a node to the trip path and set its attributes.
    // TODO - What to do on the 1st node of the path (since we just have
    // a list of path edges).
    TripPath_Node* trip_node = trip_path.add_node();
    //if (nodeinfo != nullptr) {
    // TODO:  Add the trip_node for exits.
    //}

    // Add edge to the trip node and set its attributes
    TripPath_Edge* trip_edge = AddTripEdge(directededge, trip_node, graphtile);

    // Get the shape and set shape indexes (directed edge forward flag
    // determines whether shape is traversed forward or reverse).
    std::unique_ptr<const EdgeInfo> edgeinfo = graphtile->edgeinfo(
            directededge->edgedataoffset());
    trip_edge->set_begin_shape_index(trip_shape.size());
    if (directededge->forward()) {
      trip_shape.insert(trip_shape.end(), edgeinfo->shape().begin() +
              (trip_shape.size() ? 1 : 0), edgeinfo->shape().end());

      trip_edge->set_begin_heading(
          std::round(
              PointLL::HeadingAlongPolyline(edgeinfo->shape(), kMetersOffsetForHeading)));
      trip_edge->set_end_heading(
          std::round(
              PointLL::HeadingAtEndOfPolyline(edgeinfo->shape(), kMetersOffsetForHeading)));

    } else {

      trip_shape.insert(
          trip_shape.end(),
          edgeinfo->shape().rbegin() + (trip_shape.size() ? 1 : 0),
          edgeinfo->shape().rend());

      trip_edge->set_begin_heading(
          std::round(
              fmod(
                  (PointLL::HeadingAtEndOfPolyline(edgeinfo->shape(),
                               kMetersOffsetForHeading) + 180.0f), 360)));

      trip_edge->set_end_heading(
          std::round(
              fmod(
                  (PointLL::HeadingAlongPolyline(edgeinfo->shape(),
                               kMetersOffsetForHeading) + 180.0f), 360)));
    }
    trip_edge->set_end_shape_index(trip_shape.size());

    // Add connected edges. Do this after the first trip edge is added
    //
    //Our path is from 1 to 2 to 3 (nodes) to ... n nodes.
    //Each letter represents the edge info.
    //So at node 2, we will store the edge info for D first and then
    //the edge info for B, C, E, F, and G (order not important.)  We need to make sure
    //that we don't store the edge info from A and D again.  Also, do not store transition edges.
    //
    //     (X)    (3)   (X)
    //       \\   ||   //
    //      C \\ D|| E//
    //         \\ || //
    //      B   \\||//   F
    // (X)======= (2) ======(X)
    //            ||\\
    //          A || \\ G
    //            ||  \\
    //            (1)  (X)
    if (nodeinfo != nullptr) {
      // Get the first edge from the node
      uint32_t edgeid = nodeinfo->edge_index();
      const DirectedEdge* connectededge = graphtile->directededge(
                      nodeinfo->edge_index());
      for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                  i++, connectededge++, edgeid++) {
        // Skip the edge on the path and the incoming edge (this is the one
        // with prior_opp_index from this node). Skip any transition
        // edge.
        if (edgeid == edge.id() || i == prior_opp_index ||
            connectededge->trans_up() || connectededge->trans_down()) {
          continue;
        }
        AddTripEdge(connectededge, trip_node, graphtile);
      }
    }

    // Get the nodeinfo at the end node (may be in a different tile).
    GraphId endnode = directededge->endnode();
    nodeinfo = graphreader.GetGraphTile(endnode)->node(
                    directededge->endnode());

    // Save the index of the opposing directed edge at the end node
    prior_opp_index = directededge->opp_index();
  }

/** TODO - remove debug later
  std::cout << "Took " << shortcutcount << " shortcut edges out of " <<
      pathedges.size() << " edges" << std::endl;  **/

  // Encode shape and add to trip path.
  std::string encoded_shape_;
  if (trip_shape.size())
    encoded_shape_ = encode<std::vector<PointLL>>(trip_shape);
  trip_path.set_shape(encoded_shape_);

  //hand it back
  return trip_path;
}

// Add a trip edge to the trip node and set its attributes
TripPath_Edge* TripPathBuilder::AddTripEdge(const DirectedEdge* directededge,
                                            TripPath_Node* trip_node,
                                            GraphTile* graphtile) {
  TripPath_Edge* trip_edge = trip_node->add_edge();

  // Get the edgeinfo and list of names - add to the trip edge.
  std::unique_ptr<const EdgeInfo> edgeinfo = graphtile->edgeinfo(
     directededge->edgedataoffset());
  std::vector<std::string> names = graphtile->GetNames(edgeinfo);
  for (const auto& name : names) {
    trip_edge->add_name(name);
  }

  // Set speed and length
  trip_edge->set_length(directededge->length() * 0.001f);  // Convert to km
  trip_edge->set_speed(directededge->speed());

  // Test whether edge is traversed forward or reverse and set driveability
  if (directededge->forward()) {
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
  } else {
    // Reverse driveability
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
  }

  trip_edge->set_ramp(directededge->link());
  trip_edge->set_toll(directededge->toll());

  return trip_edge;
}

}
}
