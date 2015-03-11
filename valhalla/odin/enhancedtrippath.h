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
  EnhancedTripPath() = delete;

  EnhancedTripPath_Node* GetEnhancedNode(const int node_index);

  EnhancedTripPath_Edge* GetPrevEdge(const int node_index, int delta = 1);

  EnhancedTripPath_Edge* GetCurrEdge(const int node_index);

  EnhancedTripPath_Edge* GetNextEdge(const int node_index, int delta = 1);

  bool IsValidNodeIndex(int node_index) const;

  bool IsFirstNodeIndex(int node_index) const;

  bool IsLastNodeIndex(int node_index) const;

  int GetLastNodeIndex() const;

};

class EnhancedTripPath_Edge : public TripPath_Edge {
 public:
  EnhancedTripPath_Edge() = delete;

  bool IsUnnamed() const;

  bool IsHighway() const;

  std::string ToString() const;

  std::string ToParameterString() const;

 protected:

  std::string ListToString(
      const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const;

  std::string ListToParameterString(
      const ::google::protobuf::RepeatedPtrField<::std::string>& string_list) const;

};

class EnhancedTripPath_IntersectingEdge : public TripPath_IntersectingEdge {
 public:

  EnhancedTripPath_IntersectingEdge() = delete;

  std::string ToString() const;

};

class EnhancedTripPath_Node : public TripPath_Node {
 public:
  EnhancedTripPath_Node() = delete;

  bool HasIntersectingEdges() const;

  EnhancedTripPath_IntersectingEdge* GetIntersectingEdge(size_t index);

  void CalculateRightLeftIntersectingEdgeCounts(
      uint32_t from_heading, uint32_t& right_count,
      uint32_t& right_similar_count, uint32_t& left_count,
      uint32_t& left_similar_count) const;

  std::string ToString() const;

};

}
}

#endif  // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

