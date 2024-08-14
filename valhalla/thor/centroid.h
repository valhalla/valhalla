#pragma once

#include <cstdint>
#include <vector>

#include <valhalla/midgard/util.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/dijkstras.h>
#include <valhalla/thor/pathinfo.h>

namespace valhalla {
namespace thor {

/*
 * This is a simple structure which stores the edges at which paths have intersected (ie potential
 * centroid). The structure tracks both directions of the edge in one record so as to reduce the
 * number of intersections to track. It also stores a mask of which paths have intersected this edge.
 * To actually recover the path you need to look up the edges in the edgestatus/labelset.
 */
struct PathIntersection {
  /**
   * Simple constructor initializing all edge labels for every location to invalid
   * @param edge_id   The id used for the hash and label retrieval
   * @param opp_id    The id used for the hash and label retrieval
   * @param location_count  the number of paths we are tracking
   */
  PathIntersection(uint64_t edge_id, uint64_t opp_id, uint8_t location_count);

  /**
   * Updates the intersection with the label of the path that found it
   * @param path_id  the index of the path who has reached this edge
   * @return whether or not all paths have converged here
   */
  bool AddPath(uint8_t path_id) const;

  /**
   * Returns true if the path id in question has a shortest path to this intersection
   * @param path_id   the id of the path in question
   * @return true if the path converged to this point
   */
  bool HasConverged(uint8_t path_id) const;

  /**
   * Equality operator for hashed containers to resolve hash collisions
   * @param i  the other intersection to compare against this one
   * @return true if i is equal to this intersection
   */
  bool operator==(const PathIntersection& i) const;

  // instead of having two full records for an edge and for its opposing edge we store one using the
  // lesser of the two ids to make the tracking deterministic. this assumes that a centroid is equally
  // accessible from either side of an edge (not strictly true)
  uint64_t edge_id_;

  // each locations label index for the shortest path to this intersection from that location
  // currently we only support 127 paths at the same time so we use bit fields to mark which
  // edges have a shortest path to this particular edge/intersection, this would be more flexible
  // with std::vector<bool> which does masking internally but its unclear if checking for all bits
  // being set
  mutable uint64_t lower_mask_;
  mutable uint64_t upper_mask_;
};
} // namespace thor
} // namespace valhalla

// Extend the standard namespace to know how to hash this object
namespace std {
template <> struct hash<valhalla::thor::PathIntersection> {
  inline size_t operator()(const valhalla::thor::PathIntersection& i) const {
    // the edge id is always unique so its good enough
    return static_cast<size_t>(i.edge_id_);
  }
};
} // namespace std

namespace valhalla {
namespace thor {

/**
 * TODO: explain this better and more accurately, the claim about minimum isnt quite accurate
 * A best first (dijkstras) path algorithm which given a set of locations, will find the set of paths
 * from those locations to an intersection point (centroid) such that the cost of each path to the
 * intersection point is minimized among all paths. Rephrased, the algorithm finds the point in the
 * graph at which all paths from all locations converge and each path could not reach that convergence
 * point for any lower cost. From the perspective of a single location, the algorithm finds the
 * cheapest path to a point in the graph where all other paths meet
 */
class Centroid : public thor::Dijkstras {
public:
  /**
   * Returns a path for each location to a common intersection point (centroid) of all locations paths
   * such that each path is the shortest path to that common intersection point
   *
   * @param expansion_type  Which type of expansion to do, forward/reverse/mulitmodal
   * @param api             The locations from which the path finding originates
   * @param reader          Graph reader to provide access to graph primitives
   * @param costings        Per mode costing objects
   * @param mode            The mode specifying which costing to use
   * @param centroid        The location where all paths meet
   * @return                The list of paths, one per location, to the centroid as well as the
   * centroid
   */
  std::vector<std::vector<PathInfo>> Expand(const ExpansionType& expansion_type,
                                            valhalla::Api& api,
                                            baldr::GraphReader& reader,
                                            const sif::mode_costing_t& costings,
                                            const sif::TravelMode mode,
                                            valhalla::Location& centroid);

  /**
   * Resets internal state before the next call
   */
  virtual void Clear() override;

protected:
  /**
   * This callback is used to notify the child class of an edge who has been reached. We only care
   * about edges who have been settled which means we can completely ignore this
   */
  virtual void ExpandingNode(baldr::GraphReader&,
                             graph_tile_ptr,
                             const baldr::NodeInfo*,
                             const sif::EdgeLabel&,
                             const sif::EdgeLabel*) override {
    // we dont care about reached edges
  }

  /**
   * This callback is used to notify the child class of an edge who is settled meaning we have found
   * the shortest path to it. We want to check if other locations have found a shortest path to this
   * edge and remember which ones
   *
   * @param reader      used for accessing graph primitives
   * @param pred        predecessor label of the edge who has just been settled (shortest path found)
   * @param route_type  enum of forward/reverse/multimodal
   * @return returns an enumeration that signals the main loop to either continue the search, prune
   * this search path but continue the search or to stop/terminate the search completely
   */
  virtual thor::ExpansionRecommendation ShouldExpand(baldr::GraphReader& reader,
                                                     const sif::EdgeLabel& pred,
                                                     const thor::ExpansionType route_type) override;

  /**
   * Tell the expansion how many labels to expect and how many buckets to use
   *
   * @param bucket_count            impacts the number of buckets in the double bucket queue
   * @param edge_label_reservation  an estimate of the total number of edgelabels for this expansion
   */
  virtual void GetExpansionHints(uint32_t& bucket_count,
                                 uint32_t& edge_label_reservation) const override;

  /**
   * Walks back the labels in the label set for a each location to recover its path to the centroid
   *
   * @param locations   The location from which each path was seeded
   * @param labels      The edge labels used to recover the path
   * @param reader      used for accessing graph primitives
   * @param centroid    The location where all paths meet
   * @return The list of paths, one for each location, to the centroid
   */
  template <typename label_container_t>
  std::vector<std::vector<PathInfo>>
  FormPaths(const ExpansionType& expansion_type,
            const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
            const label_container_t& labels,
            baldr::GraphReader& reader,
            valhalla::Location& centroid) const;

  // the key is the edge id and the value is the label indices for each location
  // we store both directions of the edge to avoid strange uturns at the centroid
  std::unordered_set<PathIntersection> intersections_;

  // track the best intersection so far so we can return partial results
  PathIntersection best_intersection_{baldr::kInvalidGraphId, baldr::kInvalidGraphId,
                                      baldr::kMaxMultiPathId};

  // number of paths we are tracking
  uint8_t location_count_;
};

} // namespace thor
} // namespace valhalla
