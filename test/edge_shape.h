#ifndef VALHALLA_EDGE_SHAPE_H
#define VALHALLA_EDGE_SHAPE_H

#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "midgard/encoded.h"

using namespace valhalla;

std::string get_shape(baldr::GraphReader& graphreader, const valhalla::baldr::GraphId& edgeid) {
  const baldr::GraphTile* t_debug = graphreader.GetGraphTile(edgeid);
  const baldr::DirectedEdge* directedEdge = t_debug->directededge(edgeid);
  auto shape = t_debug->edgeinfo(directedEdge->edgeinfo_offset()).shape();
  if (!directedEdge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  return encode(shape);
}

#endif // VALHALLA_EDGE_SHAPE_H
