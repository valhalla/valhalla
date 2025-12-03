#pragma once

// we type alias the tile pointers as they are ubiquitous throughout the library

#ifdef ENABLE_THREAD_SAFE_TILE_REF_COUNT
#include <memory>
#else
#include <boost/intrusive_ptr.hpp>
#endif

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
