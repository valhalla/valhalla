#include "loki/search.h"

using namespace valhalla::baldr;

namespace {

PathLocation NodeSearch(const Location& location, GraphReader& reader) {

  //grab the tile the lat, lon is in
  GraphTile* tile = reader.GetGraphTile(location.latlng_);

  //we couldn't find any data for this region
  //TODO: be smarter about this either in loki or in baldr cache
  if(!tile)
    throw std::runtime_error("No data found for location");

  //a place to keep track of which node is closest to our location
  const NodeInfo* closest = tile->node(0);
  float sqdist =  closest->latlng().DistanceSquared(location.latlng_);

  //for each node
  for(size_t node_index = 1; node_index < tile->header()->nodecount(); ++node_index) {
    //if this is closer then its better
    const NodeInfo* node = tile->node(node_index);
    float node_sqdist = node->latlng().DistanceSquared(location.latlng_);
    if(node_sqdist < sqdist) {
      sqdist = node_sqdist;
      closest = node;
    }
  }

  //now that we have a node we can pass back all the edges leaving it
  PathLocation correlated(location);
  correlated.CorrelateVertex(closest->latlng());
  for(size_t edge_index = 0; edge_index < closest->edge_count(); ++edge_index) {
    //const DirectedEdge* edge = tile->directededge(edge_index);
    GraphId id(tile->id());
    id.Set(id.tileid(), id.level(), closest->edge_index() + edge_index);
    //TODO: is there something we can do to set distance along as either 0 or 1 depending on direction?
    correlated.CorrelateEdge(id, 0);
  }

  //if we found nothing that is no good..
  if(correlated.edges().size() == 0)
    throw std::runtime_error("Unable to find any edges connected to this location");

  //give it back
  return correlated;
}

PathLocation EdgeSearch(const Location& location, GraphReader& reader) {
  //TODO:
  throw std::runtime_error("Edge correlated searches are not yet supported");
}

}

namespace valhalla {
namespace loki {

PathLocation Search(const Location& location, GraphReader& reader, const SearchStrategy strategy) {
  if(strategy == SearchStrategy::EDGE)
    return EdgeSearch(location, reader);
  return NodeSearch(location, reader);
}

}
}
