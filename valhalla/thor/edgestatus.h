#pragma once

#include <unordered_map>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>

// handy macro for shifting the 7bit path index value so that it can be or'd with the tile/level id
#define SHIFT_path_id(x) (static_cast<uint32_t>(x) << 25u)

namespace valhalla {
namespace thor {

// Edge label status
enum class EdgeSet : uint8_t {
  kUnreachedOrReset = 0, // Unreached - not yet encountered in search _or_ encountered but
                         // reset due to encountering a complex restriction:
                         // https://github.com/valhalla/valhalla/issues/2103
  kPermanent = 1,        // Permanent - shortest path to this edge has been found
  kTemporary = 2,        // Temporary - edge has been encountered but there could
                         //   still be a shorter path to this edge. This edge will
                         //   be "adjacent" to an edge that is permanently labeled.
  kSkipped = 3           // Skipped - edge has been encountered but was thrown out
                         // of consideration.
};

// Store the edge label status and its index in the EdgeLabels list
struct EdgeStatusInfo {
  uint32_t index_ : 28;
  uint32_t set_ : 4;

  EdgeStatusInfo() : index_(0), set_(0) {
  }

  EdgeStatusInfo(const EdgeSet set, const uint32_t index)
      : index_(index), set_(static_cast<uint32_t>(set)) {
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
   * Default constructor.
   */
  EdgeStatus() = default;

  // in order no to delete objects twice in destructor we should explicitly
  // forbid copying
  EdgeStatus(const EdgeStatus&) = delete;
  EdgeStatus& operator=(const EdgeStatus&) = delete;
  EdgeStatus(EdgeStatus&&) = default;
  EdgeStatus& operator=(EdgeStatus&&) = default;

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
   * @param  edgeid      GraphId of the directed edge to set.
   * @param  set         Label set for this directed edge.
   * @param  index       Index of the edge label.
   * @param  tile        Graph tile of the directed edge.
   * @param  path_id     Identifies which path the edge status belongs to when tracking multiple paths
   *                     valid ids are from 0 to 127 (since we only have 7 bits free)
   */
  void Set(const baldr::GraphId& edgeid,
           const EdgeSet set,
           const uint32_t index,
           const graph_tile_ptr& tile,
           const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()] = {set, index};
    } else {
      // Tile is not in the map. Add an array of EdgeStatusInfo, sized to
      // the number of directed edges in the specified tile.
      auto inserted = edgestatus_.emplace(edgeid.tile_value() | SHIFT_path_id(path_id),
                                          new EdgeStatusInfo[tile->header()->directededgecount()]);
      inserted.first->second[edgeid.id()] = {set, index};
    }
  }

  /**
   * Update the status (set) of a directed edge given its GraphId.
   * This method assumes that the edge id has already been encountered.
   * @param  edgeid      GraphId of the directed edge to set.
   * @param  set         Label set for this directed edge.
   * @param  path_id     Identifies which path the edge status belongs to when tracking multiple paths
   *                     valid ids are from 0 to 127 (since we only have 7 bits free)
   */
  void Update(const baldr::GraphId& edgeid, const EdgeSet set, const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    const auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    if (p != edgestatus_.end()) {
      p->second[edgeid.id()].set_ = static_cast<uint32_t>(set);
    } else {
      throw std::runtime_error("EdgeStatus Update on edge not previously set");
    }
  }

  /**
   * Get the status info of a directed edge given its GraphId.
   * @param   edgeid     GraphId of the directed edge.
   * @param  path_id     Identifies which path the edge status belongs to when tracking multiple paths
   *                     valid ids are from 0 to 127 (since we only have 7 bits free)
   * @return  Returns edge status info.
   */
  EdgeStatusInfo Get(const baldr::GraphId& edgeid, const uint8_t path_id = 0) const {
    assert(path_id <= baldr::kMaxMultiPathId);
    const auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    return (p == edgestatus_.end()) ? EdgeStatusInfo() : p->second[edgeid.id()];
  }

  /**
   * Get a pointer to the edge status info of a directed edge. Since directed
   * edges are stored sequentially from a node this reduces the number of
   * lookups by edgeid.
   * @param   edgeid     GraphId of the directed edge.
   * @param   tile       Graph tile of the directed edge.
   * @param  path_id     Identifies which path the edge status belongs to when tracking multiple paths
   *                     valid ids are from 0 to 127 (since we only have 7 bits free)
   * @return  Returns a pointer to edge status info for this edge.
   */
  EdgeStatusInfo*
  GetPtr(const baldr::GraphId& edgeid, const graph_tile_ptr& tile, const uint8_t path_id = 0) {
    assert(path_id <= baldr::kMaxMultiPathId);
    const auto p = edgestatus_.find(edgeid.tile_value() | SHIFT_path_id(path_id));
    if (p != edgestatus_.end()) {
      return &p->second[edgeid.id()];
    } else {
      // Tile is not in the map. Add an array of EdgeStatusInfo, sized to
      // the number of directed edges in the specified tile.
      auto inserted = edgestatus_.emplace(edgeid.tile_value() | SHIFT_path_id(path_id),
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
