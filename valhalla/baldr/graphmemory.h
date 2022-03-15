#pragma once

namespace valhalla {
namespace baldr {

// A holder struct for memory owned by the GraphTile.
class GraphMemory {
protected:
  GraphMemory() = default;

public:
  virtual ~GraphMemory() = default;

  const char* data;
  size_t size;
};

} // namespace baldr
} // namespace valhalla
