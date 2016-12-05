#ifndef VALHALLA_THOR_EDGESTATUS_H_
#define VALHALLA_THOR_EDGESTATUS_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace thor {

// Default size to reserve for the EdgeStatus unordered map
constexpr uint32_t kDefaultEdgeStatusSize = 2000000;

// Edge label status
enum class EdgeSet : uint8_t {
  kUnreached = 0,   // Unreached - not yet encountered in search
  kPermanent = 1,   // Permanent - shortest path to this edge has been found
  kTemporary = 2    // Temporary - edge has been encountered but there could
                    //   still be a shorter path to this edge. This edge will
                    //   be "adjacent" to an edge that is permanently labeled.
};

// Store the edge label status and its index in the EdgeLabels list
struct EdgeStatusInfo {
  struct SetAndIndex {
    uint32_t index : 28;
    uint32_t set   : 4;
  };
  SetAndIndex status;

  EdgeStatusInfo() {
    status.set   = static_cast<uint32_t>(EdgeSet::kUnreached);
    status.index = 0;
  }

  EdgeStatusInfo(const EdgeSet set, const uint32_t index) {
    status.set   = static_cast<uint32_t>(set);
    status.index = index;
  }

  uint32_t index() const {
    return status.index;
  }

  EdgeSet set() const {
    return static_cast<EdgeSet>(status.set);
  }
};

/**
 * Class to define / lookup the status and index of an edge in the edge label
 * list during shortest path algorithms.
 */
class EdgeStatus {
 public:
  /**
   * Constructor given an initial size.
   * @param  sz  Size to reserve for edgestatus unordered map.
   */
  EdgeStatus(const uint32_t sz = kDefaultEdgeStatusSize) {
    edgestatus_.reserve(sz);
  }

  /**
   * Initialize the status to unreached for all edges.
   */
  void Init() {
    edgestatus_.clear();
  }

  /**
   * Set the status of a directed edge given its GraphId.
   * @param  edgeid   GraphId of the directed edge to set.
   * @param  set      Label set for this directed edge.
   * @param  index    Index of the edge label.
   */
  void Set(const baldr::GraphId& edgeid, const EdgeSet set,
           const uint32_t index) {
    edgestatus_[edgeid.value] = { set, index };
  }

  /**
   * Update the status of a directed edge given its GraphId.
   * @param  edgeid   GraphId of the directed edge to set.
   * @param  set      Label set for this directed edge.
   */
  void Update(const baldr::GraphId& edgeid, const EdgeSet set) {
    edgestatus_[edgeid.value].status.set = static_cast<uint32_t>(set);
  }

  /**
   * Get the status info of a directed edge given its GraphId.
   * @param   edgeid  GraphId of the directed edge.
   * @return  Returns edge status info.
   */
  EdgeStatusInfo Get(const baldr::GraphId& edgeid) const {
    auto p = edgestatus_.find(edgeid.value);
    return (p == edgestatus_.end()) ? EdgeStatusInfo() : p->second;
  }

 private:
  // Map to store the status and index of GraphIds that have been encountered.
  // Any unreached edges are not added to the map.
  std::unordered_map<uint64_t, EdgeStatusInfo> edgestatus_;
};

}
}

#endif  // VALHALLA_THOR_EDGESTATUS_H_
