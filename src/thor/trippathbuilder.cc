#include <ostream>
#include <iostream>

#include "thor/trippathbuilder.h"

#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/midgard/pointll.h>

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
TripPath TripPathBuilder::Build(GraphReader& graphreader, const std::vector<GraphId>& pathedges) {
  // TripPath is a protocol buffer that contains information about the trip
  TripPath trip_path;

  uint32_t shortcutcount = 0;

  const NodeInfo* nodeinfo = nullptr;
  const GraphTile* endNodeTile = nullptr;

  std::vector<PointLL> trip_shape;

  for (const auto& edge : pathedges) {
    GraphTile* graphtile = graphreader.GetGraphTile(edge);
    const DirectedEdge* directededge = graphtile->directededge(edge);

    // Skip transition edges
    if (directededge->trans_up()) {
//      std::cout << "Transition up!" << std::endl;
      continue;
    }
    if (directededge->trans_down()) {
//      std::cout << "Transition down" << std::endl;
      continue;
    }

    // Add a node. What to do on the 1st node of the path (since we just have
    // a list of path edges).
    //if (nodeinfo != nullptr) {
    // TODO:  Add the trip_node for exits.
    //}

    TripPath_Node* trip_node = trip_path.add_node();

    // Add edge to the trip node and set its attributes
    TripPath_Edge* trip_edge = AddTripEdge(directededge, trip_node, graphtile);

    std::vector<uint32_t> addedEdgeInfo;
    addedEdgeInfo.emplace_back(directededge->edgedataoffset());

    // Print out the node lat,lng
    GraphTile* tile = graphreader.GetGraphTile(directededge->endnode());
    const NodeInfo* nodeinfo = tile->node(directededge->endnode());

    // Test whether edge is traversed forward or reverse and set driveability
    bool is_reverse = false;
    if (directededge->forward()) { //Edge is in the forward direction.
      is_reverse = true;
    }

    const std::shared_ptr<EdgeInfo> edgeinfo = graphtile->edgeinfo(
        directededge->edgedataoffset());

    // Add shape and set shape indexes
    trip_edge->set_begin_shape_index(trip_shape.size());
    if (is_reverse) {
      trip_shape.insert(trip_shape.end(), edgeinfo->shape().rbegin() +
              (trip_shape.size() ? 1 : 0), edgeinfo->shape().rend());
    } else {
      trip_shape.insert(trip_shape.end(), edgeinfo->shape().begin() +
              (trip_shape.size() ? 1 : 0), edgeinfo->shape().end());
    }
    trip_edge->set_end_shape_index(trip_shape.size());

    // Add connected edges. Do this after the first trip edge is added
    //
    //Our path is from 1 to 2 to 3 to ...
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

    GraphId edgeid = GraphId(edge.tileid(), edge.level(),
                             nodeinfo->edge_index());
    const DirectedEdge* connectededge = graphtile->directededge(
                nodeinfo->edge_index());

    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, connectededge++, edgeid++) {

      // Skip the edge on the path and the incoming edge. Skip any transition
      // edge. TODO - Skip the opposing incoming edge (based on the prior edge)
      if (edgeid == edge || connectededge->trans_up() ||
          connectededge->trans_down() || (addedEdgeInfo.end() !=
          std::find(addedEdgeInfo.begin(),addedEdgeInfo.end(),connectededge->edgedataoffset()))){
        continue;
      }

      addedEdgeInfo.emplace_back(connectededge->edgedataoffset());

      AddTripEdge(connectededge, trip_node, endNodeTile);

    }

    // Get the end node (begin node of the next edge)
    endNodeTile = graphreader.GetGraphTile(directededge->endnode());
    nodeinfo = endNodeTile->node(directededge->endnode());

 /** DEBUG
    // TODO - remove or create debug output...
    std::cout << "  Edge:" << edge.tileid() << "," <<
          static_cast<uint32_t>(edge.level()) << "," <<  edge.id() <<
       " Use=" << static_cast<uint32_t>(directededge->use()) <<
       " Length=" << directededge->length() <<
       " Link=" << directededge->link() <<
       " Shortcut=" << directededge->shortcut() <<
       " Superseded=" << directededge->superseded() << std::endl;

    if (names.size() > 0) {
      std::cout << "  Names: ";
      for (const auto& name : names) {
        std::cout << name << " / ";
      }
      std::cout << std::endl;
    }
    if (directededge->shortcut())
      shortcutcount++;

    std::cout << "LL = " << nodeinfo->latlng().lat() << "," <<
        nodeinfo->latlng().lng() << " EdgeCount= " << nodeinfo->edge_count() <<
        std::endl;
  **/
  }

/**
  // TODO - remove debug later
  std::cout << "Took " << shortcutcount << " shortcut edges out of " <<
      pathedges.size() << " edges" << std::endl;
**/

  // Encode shape and add to trip path.
  std::string encoded_shape_;
  if (trip_shape.size())
    encoded_shape_ = encode<std::vector<PointLL>>(trip_shape);
  trip_path.set_shape(encoded_shape_);

  //hand it back
  return trip_path;
}

TripPath_Edge* TripPathBuilder::AddTripEdge(const DirectedEdge* directededge, TripPath_Node* trip_node,
                                            const GraphTile* graphtile) {

  // Add edge to the trip node and set its attributes
  TripPath_Edge* trip_edge = trip_node->add_edge();

  // Get the edgeinfo and list of names
  std::unique_ptr<const EdgeInfo> edgeinfo = graphtile->edgeinfo(
     directededge->edgedataoffset());
   std::vector<std::string> names = graphtile->GetNames(edgeinfo);

  for (const auto& name : names) {
    trip_edge->add_name(name);
  }

  // Set speed and length
  trip_edge->set_length(directededge->length());
  trip_edge->set_speed(directededge->speed());

  // Test whether edge is traversed forward or reverse and set driveability
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
  }

  trip_edge->set_ramp(directededge->link());
  trip_edge->set_toll(directededge->toll());
  trip_edge->set_begin_heading(PointLL::HeadingAlongPolyline(edgeinfo->shape(),30));
  trip_edge->set_end_heading(PointLL::HeadingAtEndOfPolyline(edgeinfo->shape(),30));

  return trip_edge;
}

}
}
