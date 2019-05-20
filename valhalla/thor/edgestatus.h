#ifndef VALHALLA_THOR_EDGESTATUS_H_
#define VALHALLA_THOR_EDGESTATUS_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>

namespace valhalla {
namespace thor {

// Edge label status
enum class EdgeSet : uint8_t {
  kUnreached = 0, // Unreached - not yet encountered in search
  kPermanent = 1, // Permanent - shortest path to this edge has been found
  kTemporary = 2  // Temporary - edge has been encountered but there could
                  //   still be a shorter path to this edge. This edge will
                  //   be "adjacent" to an edge that is permanently labeled.
};

// Store the edge label status and its index in the EdgeLabels list
struct EdgeStatusInfo {
  uint32_t index_ : 28;
  uint32_t set_ : 4;

  EdgeStatusInfo() : index_(0), set_(0) {
  }

  EdgeStatusInfo(const EdgeSet set, const uint32_t index) {
    set_ = static_cast<uint32_t>(set);
    index_ = index;
  }

  uint32_t index() const {
    return index_;
  }

  EdgeSet set() const {
    return static_cast<EdgeSet>(set_);
  }
};

/**
 * Class to define / lookup the status and index of an edge in the edge label
 * list during shortest path algorithms. This method stores status info for
 * edges within arrays for each tile. This allows the path algorithms to get
 * a pointer to the first edge status and iterate that pointer over sequential
 * edges. This reduces the number of map lookups.
 */
class EdgeStatus {
public:
  /**
   * Destructor. Delete any allocated EdgeStatusInfo arrays.
   */
  ~EdgeStatus() {
    clear();
  }

  /**
   * Clear the EdgeStatusInfo arrays and the edge status map.
   */
  void clear() {
    // Delete any allocated arrays for tiles within the map.
    for (auto& iter : edgestatus_) {
      delete[] iter.second;
    }
    edgestatus_.clear();
  }

  /**
   * Set the status of a directed edge given its GraphId.
   * @param  edgeid   GraphId of the directed edge to set.
   * @param  set      Label set for this directed edge.
   * @param  index    Index of the edge label.
   * @param  tile     Graph tile of the directed edge.
   */
  void Set(const baldr::GraphId& edgeid,
           const EdgeSet set,
           const uint32_t index,
           const baldr::GraphTile* tile) {
    auto p = edgestatus_.find(edgeid.tile_value());
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()] = {set, index};
    } else {
      // Tile is not in the map. Add an array of EdgeStatusInfo, sized to
      // the number of directed edges in the specified tile.
      auto inserted = edgestatus_.emplace(edgeid.tile_value(),
                                          new EdgeStatusInfo[tile->header()->directededgecount()]);
      inserted.first->second[edgeid.id()] = {set, index};
    }
  }

  /**
   * Update the status (set) of a directed edge given its GraphId.
   * This method assumes that the edge id has already been encountered.
   * @param  edgeid   GraphId of the directed edge to set.
   * @param  set      Label set for this directed edge.
   */
  void Update(const baldr::GraphId& edgeid, const EdgeSet set) {
    const auto p = edgestatus_.find(edgeid.tile_value());
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()].set_ = static_cast<uint32_t>(set);
    } else {
      throw std::runtime_error("EdgeStatus Update on edge not previously set");
    }
  }

  /**
   * Get the status info of a directed edge given its GraphId.
   * @param   edgeid  GraphId of the directed edge.
   * @return  Returns edge status info.
   */
  EdgeStatusInfo Get(const baldr::GraphId& edgeid) const {
    const auto p = edgestatus_.find(edgeid.tile_value());
    return (p == edgestatus_.end()) ? EdgeStatusInfo() : p->second[edgeid.id()];
  }

  /**
   * Get a pointer to the edge status info of a directed edge. Since directed
   * edges are stored sequentially from a node this reduces the number of
   * lookups by edgeid.
   * @param   edgeid  GraphId of the directed edge.
   * @param   tile    Graph tile of the directed edge.
   * @return  Returns a pointer to edge status info for this edge.
   */
  EdgeStatusInfo* GetPtr(const baldr::GraphId& edgeid, const baldr::GraphTile* tile) {
    const auto p = edgestatus_.find(edgeid.tile_value());
    if (p != edgestatus_.end()) {
      return &p->second[edgeid.id()];
    } else {
      // Tile is not in the map. Add an array of EdgeStatusInfo, sized to
      // the number of directed edges in the specified tile.
      auto inserted = edgestatus_.emplace(edgeid.tile_value(),
                                          new EdgeStatusInfo[tile->header()->directededgecount()]);
      return &(inserted.first->second)[edgeid.id()];
    }
  }

private:
  // Edge status - keys are the tile Ids (level and tile Id) and the
  // values are dynamically allocated arrays of EdgeStatusInfo (sized
  // based on the directed edge count within the tile).
  std::unordered_map<uint32_t, EdgeStatusInfo*> edgestatus_;
};

} // namespace thor
} // namespace valhalla

#endif // VALHALLA_THOR_EDGESTATUS_H_
