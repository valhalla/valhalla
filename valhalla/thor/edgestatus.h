#ifndef VALHALLA_THOR_EDGESTATUS_H_
#define VALHALLA_THOR_EDGESTATUS_H_

#include <unordered_map>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace thor {

enum EdgeStatusType {
  kUnreached = 0,   // Unreached - not yet encountered in search
  kPermanent = 1,   // Permanent - shortest path to this edge has been found
  kTemporary = 2    // Temporary - edge has been encountered but there could
                    //   still be a shorter path to this edge. This edge will
                    //   be "adjacent" to an edge that is permanently labeled.
};

/**
 * Class to define / lookup the status of an edge during the shortest path
 * algorithm.
 */
class EdgeStatus {
 public:
  EdgeStatus();

  virtual ~EdgeStatus() {
  }

  /**
   * Initialize the status to unreached for all edges.
   */
  void Init();

  /**
   * Set the status of a directed edge given its GraphId.
   * @param  edgeid   GraphId of the directed edge to set.
   * @param  status   Label status for this directed edge.
   */
  void Set(const baldr::GraphId& edgeid, const EdgeStatusType status);

  /**
   * Get the status of a directed edge given its GraphId.
   */
  EdgeStatusType Get(const baldr::GraphId& edgeid) const;

 private:
  // Map to store the status of GraphIds that have been encountered.
  // Any unreached edges are not added to the map.
  std::unordered_map<uint64_t, EdgeStatusType> edgestatus_;
};

}
}

#endif  // VALHALLA_THOR_EDGESTATUS_H_
