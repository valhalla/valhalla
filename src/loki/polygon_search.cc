#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <iostream>
#include <math.h>

#include "loki/node_search.h"
#include "loki/polygon_search.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "proto/options.pb.h"

namespace bg = boost::geometry;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {} // namespace

namespace valhalla {
namespace loki {

void edges_in_rings(const multi_ring_t& rings) {

  auto tiles = vb::TileHierarchy::levels().back().tiles;
  const uint8_t bin_level = vb::TileHierarchy::levels().back().level;

  std::unordered_map<int32_t, std::unordered_set<unsigned short>> intersection;
  // loop over rings and collect all bins:
  // - inspect all tiles & bins which are in between the ones returned by intersection (per row) ->
  // look at Tiles.Intersect()
  //    - if the tile's/bin's center (or other point) is inside the polygon, the whole tile/bin is
  //    inside and consequently all edges
  // - the intersected tiles/bins have to be fully intersected to find the edges which are affected
  // in the end we'll have two sets of bins:
  //    - one set containing all bins which are fully inside the polygon(s)
  //    - one set which needs further handling to find all edges intersecting the polygons
  for (auto ring : rings) {
    auto line_intersected = tiles.Intersect(ring);
    std::cout << "No of intersected tiles: " << std::to_string(line_intersected.size()) << std::endl;
    for (auto& tile : line_intersected) {
      std::cout << "Tile: " << std::to_string(tile.first)
                << ", Num bins: " << std::to_string(tile.second.size()) << std::endl;
    }
  }

  // Then we can continue to try and make an edge's shape a boost.geometry to intersect it with a
  // polygon Finally return this set of edges to loki worker
}

multi_ring_t PBFToRings(const google::protobuf::RepeatedPtrField<Options::AvoidPolygon>& rings_pbf) {
  multi_ring_t rings;
  for (const auto& ring_pbf : rings_pbf) {
    ring_bg_t new_ring;
    for (const auto& coord : ring_pbf.coords()) {
      new_ring.push_back({coord.ll().lng(), coord.ll().lat()});
    }
    rings.push_back(new_ring);
  }
  return rings;
}

double GetAvoidArea(const multi_ring_t& rings) {
  double area;
  for (const auto& ring : rings) {
    area += bg::area(ring, bg::strategy::area::geographic<>());
  }
  return area;
}
} // namespace loki
} // namespace valhalla
