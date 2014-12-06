#include "graphbuilder.h"

using namespace valhalla::geo;

namespace valhalla{
namespace mjolnir{

void GraphBuilder::node_callback(uint64_t osmid, double lng, double lat,
                   const Tags &/*tags*/) {
  nodes.insert(std::make_pair(osmid, OSMNode((float)lat, (float)lng)));
  nodecount++;

//   if (nodecount % 10000 == 0) std::cout << nodecount << " nodes" << std::endl;
}

void GraphBuilder::way_callback(uint64_t osmid, const Tags &tags,
                  const std::vector<uint64_t> &refs) {
  // Do not add ways with < 2 nodes. Log error or add to a problem list
  if (refs.size() < 2) {
//      std::cout << "ERROR - way " << osmid << " with < 2 nodes" << std::endl;
    return;
  }

  // Add the way if it has a highway tag.
  // There are other tags that correspond to the street network,
  // however for simplicity, we don't manage them
  if (tags.find("highway") != tags.end()) {
    OSMWay w(osmid);
    w.nodelist_ = refs;

    // TODO: Read more properties!!!

    ways_.push_back(w);
//      if (ways.size() % 100000 == 0)
//        std::cout << ways.size() << " ways" << std::endl;
  }
}

void GraphBuilder::relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/,
                       const References & /*refs*/) {
  relationcount++;
}

void GraphBuilder::PrintCounts() {
  std::cout << "Read and parsed " << nodecount << " nodes, " << ways_.size()
            << " ways and " << relationcount << " relations" << std::endl;
}


// Once all the ways and nodes are read, we compute how many times a node
// is used. This allows us to detect intersections (nodes in a graph).
void GraphBuilder::SetNodeUses() {
  // Iterate through the ways
  for (auto way : ways_) {
    // Iterate through the nodes that make up the way
    for (auto node : way.nodelist_) {
      nodes[node].uses++;
    }
    // make sure that the last node is considered as an extremity
    // TODO - could avoid this by altering ConstructEdges
    nodes[way.nodelist_.back()].uses++;
  }
}

// Construct edges in the graph
void GraphBuilder::ConstructEdges() {
  // Iterate through the OSM ways
  uint64_t source, target, current;
  for (auto way : ways_) {
    Edge edge;
    current = way.nodelist_[0];
    OSMNode& node = nodes[current];
    edge.sourcenode_ = current;
    edge.latlngs_.push_back(node.latlng_);
    for (size_t i = 1, n = way.nodelist_.size(); i < n; ++i) {
      current = way.nodelist_[i];
      // If a node is used more than once, it is an intersection, hence it's
      // a node of the road network graph
      node = nodes[current];
      edge.sourcenode_ = current;
      edge.latlngs_.push_back(node.latlng_);
      if (node.uses > 1) {
        edge.targetnode_ = current;
        edges_.push_back(edge);

        // Start a new edge
        edge.sourcenode_ = current;
      }
    }
  }
  std::cout << "Constructed " << edges_.size() << " edges" << std::endl;
}

}
}
