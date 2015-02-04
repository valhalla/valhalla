#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <valhalla/proto/trippath.pb.h>

namespace valhalla {
namespace odin {

class EnhancedTripPath;
class EnhancedTripPath_Edge;
class EnhancedTripPath_Node;

class EnhancedTripPath : public TripPath {
 public:

  EnhancedTripPath_Node* GetEnhancedNode(const int node_index);

  EnhancedTripPath_Edge* GetPrevEdge(const int node_index, int delta = 1);

  EnhancedTripPath_Edge* GetCurrEdge(const int node_index);

  EnhancedTripPath_Edge* GetNextEdge(const int node_index, int delta = 1);

  bool IsValidNodeIndex(int node_index) const;

  bool IsFirstNodeIndex(int node_index) const;

  bool IsLastNodeIndex(int node_index) const;

  int GetLastNodeIndex() const;

 protected:
  EnhancedTripPath();

};

class EnhancedTripPath_Edge : public TripPath_Edge {
 public:

  bool IsUnnamed() const;

  bool IsHighway() const;

  std::string ToString() const;

 protected:
  EnhancedTripPath_Edge();

};

class EnhancedTripPath_Node : public TripPath_Node {
 public:
  bool last_node() const;
  void set_last_node(bool last_node);

  bool HasIntersectingEdges() const;

  size_t GetIntersectingEdgesCount() const;

  EnhancedTripPath_Edge* GetIntersectingEdge(size_t index);

  void CalculateRightLeftIntersectingEdgeCounts(
      uint32_t from_heading, uint32_t& right_count,
      uint32_t& right_similar_count, uint32_t& left_count,
      uint32_t& left_similar_count) const;

 protected:
  EnhancedTripPath_Node();

  static bool IsSimilarTurnDegree(uint32_t path_turn_degree,
                                  uint32_t intersecting_turn_degree,
                                  bool is_right,
                                  uint32_t turn_degree_threshold = 30);

  bool last_node_;

};

}
}

#endif  // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

