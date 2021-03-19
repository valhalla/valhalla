#pragma once

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

class VectorGraphMemory final : public GraphMemory {
public:
  VectorGraphMemory(std::vector<char>&& memory) : memory_(std::move(memory)) {
    data = const_cast<char*>(memory_.data());
    size = memory_.size();
  }

private:
  const std::vector<char> memory_;
};

} // namespace baldr
} // namespace valhalla
