#include <valhalla/baldr/pathlocation.h>
#include <valhalla/meili/routing.h>

namespace valhalla {
namespace meili {

class SearchTree {
 public:
  void AddLabel(const Label& label);
  TreeIterator<std::unordered_map<int, Label>> GetPath(const baldr::PathLocation& location);
 private:
  std::unordered_map<int, Label> tree_;
};

/**
 * This class implements Single Source Shortest Path search algorithm. The class
 * takes a single source (PathLocation) as input, and search destinations as
 * needed. when searching for destinations, it expands from the source until the
 * destination is found or the tree is exhausted.
 */
class SSSP {
 public:
  /**
   * construct a SSSP instance.
   */
  SSSP(baldr::GraphReader& graphreader,
       const baldr::PathLocation& source,
       sif::cost_ptr_t costing,
       const float turn_cost_table[181],
       float max_dist,
       float max_time)
      : graphreader_(graphreader),
        source_(source),
        costing_(costing),
        turn_cost_table_(turn_cost_table),
        max_dist_(max_dist),
        max_time_(max_time),
        tree_(max_dist),
        path_end_(RoutePathIterator(&tree_)) {}

  /**
   * Return the source.
   */
  const baldr::PathLocation& source() const
  { return source_; }

  /**
   * Return the path end iterator. This iterator is used to check if the path is
   * terminated.
   */
  const RoutePathIterator& path_end() const
  { return path_end_; }

  /**
   * Search the destination in the search tree. If the destination is found,
   * return the path iterator which starts from the destination, otherwise the end
   * iterator (see path_end()).
   */
  RoutePathIterator GetPath(const baldr::PathLocation& destination) const;

  /**
   * Search the destination in the search tree. If not found, expand the search
   * tree until the destination is found in the tree or the expansion size is
   * exceeded (see AllowExpansion). If the destination is found, return the path
   * iterator which starts from the destination.
   */
  RoutePathIterator SearchPath(const baldr::PathLocation& destination);

 private:
  // Expand along edges from this node. This method has to be
  // set-up to be called recursively (for transition edges) so we set up a
  // function reference.
  void ExpandFromNode(const Label& label,
                      const uint32_t label_idx,
                      const baldr::GraphId& nodeid,
                      const bool from_transition = false);

  void ExpandFromEdge(const Label& label, uint32_t label_idx);

  /**
   * Test if the expansion is allowd given the current cost.
   */
  bool AllowExpansion(const sif::Cost& cost) const
  {
    return (max_dist_ < 0 || cost.cost < max_dist_) &&
           (max_time_ < 0 || cost.secs < max_time_);
  }

  void SetOrigin(const baldr::PathLocation& origin);

  baldr::GraphReader& graphreader_;

  const baldr::PathLocation source_;

  const sif::cost_ptr_t costing_;

  /**
   * A pointer pointing to an array of 180. turn_cost_table_[N] precents the
   * turn penally at degree N.
   */
  const float* turn_cost_table_;

  /**
   * The max allowed distance.
   */
  float max_dist_;

  /**
   * The max allowed time.
   */
  float max_time_;

  /**
   * The search tree, in which each
   */
  LabelSet tree_;

  /**
   * End path iterator.
   */
  const RoutePathIterator path_end_;
};

}
}
