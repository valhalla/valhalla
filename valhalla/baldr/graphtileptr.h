#pragma once

// we type alias the tile pointers as they are ubiquitous throughout the library

#include <boost/intrusive_ptr.hpp>
#include <boost/smart_ptr/intrusive_ref_counter.hpp>
#include <memory>

namespace valhalla {
namespace baldr {
class GraphTile;
#ifdef ENABLE_THREAD_SAFE_TILE_REF_COUNT
using graph_tile_ptr = std::shared_ptr<const GraphTile>;
#else
using graph_tile_ptr = boost::intrusive_ptr<const GraphTile>;
#endif
} // namespace baldr
} // namespace valhalla

using graph_tile_ptr = valhalla::baldr::graph_tile_ptr;
