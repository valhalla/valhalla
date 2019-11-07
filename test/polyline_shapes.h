#ifndef VALHALLA_POLYLINE_SHAPES_H
#define VALHALLA_POLYLINE_SHAPES_H
#endif // VALHALLA_POLYLINE_SHAPES_H

#include <string>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/midgard/encoded.h>

namespace test {
std::string get_shape(valhalla::baldr::GraphReader& graphreader,
                      const valhalla::baldr::GraphId& edgeid) {
  const valhalla::baldr::GraphTile* t_debug = graphreader.GetGraphTile(edgeid);
  const valhalla::baldr::DirectedEdge* directedEdge = t_debug->directededge(edgeid);
  auto shape = t_debug->edgeinfo(directedEdge->edgeinfo_offset()).shape();
  if (!directedEdge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  return valhalla::midgard::encode(shape);
}
} // namespace test