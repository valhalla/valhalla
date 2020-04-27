#ifndef VALHALLA_BALDR_GRAPHMEMORY_H_
#define VALHALLA_BALDR_GRAPHMEMORY_H_

#include <memory>

namespace valhalla {
namespace baldr {

// A holder struct for memory owned by the GraphTile.
class GraphMemory {
protected:
  GraphMemory() = default;

public:
  virtual ~GraphMemory() = default;

  char* data;
  size_t size;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_GRAPHMEMORY_H_
