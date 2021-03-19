#pragma once

// we type alias the tile pointers as they are ubiquitous throughout the library

#include <boost/intrusive_ptr.hpp>

namespace valhalla {
namespace baldr {
class GraphTile;
using graph_tile_ptr = boost::intrusive_ptr<const GraphTile>;
} // namespace baldr
} // namespace valhalla
using graph_tile_ptr = boost::intrusive_ptr<const valhalla::baldr::GraphTile>;
