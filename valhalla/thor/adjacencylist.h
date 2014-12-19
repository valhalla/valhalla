#ifndef VALHALLA_THOR_ADJACENCYLIST_H_
#define VALHALLA_THOR_ADJACENCYLIST_H_

#include <vector>
#include <list>

#include "thor/edgelabel.h"

namespace valhalla {
namespace thor {

/**
 * Adjacency list support. Uses a bucket sort implementation for performance.
 * An "overflow" bucket is maintained to allow reduced memory use - costs
 * outside the current bucket "range" get placed into the overflow bucket and
 * are moved into the low-level buckets as needed.
 * @author  David W. Nesbitt
 */
class AdjacencyList {
 public:
  /**
   * Constructor given a minimum cost, a range of costs held within the
   * bucket sort, and a bucket size. All costs above mincost + range are
   * stored in an "overflow" bucket.
   * @param mincost    Minimum sort cost (based on A* heuristic). Used
   *                   to create the initial range for bucket sorting.
   * @param range      Cost (sort cost) range for double buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   */
  AdjacencyList(const float mincost, const float range,
                const float bucketsize);

  /**
   * Destructor.
   */
  virtual ~AdjacencyList();

  /**
   * Clear all edge labels from from the adjacency list
   */
  void Clear();

  /**
   * Adds an edge label to the sorted list. Adds to the appropriate
   * bucket given its sort cost. If the sortcost is greater than maxcost_
   * the edge label is placed in the overflow bucket. If the sortcost is <
   * the current bucket cost then the edge label is placed at the front of
   * the current bucket (this prevents underflow).
   * @param   edgelabel  Edge label to add to the adjacency list.
   */
  void Add(EdgeLabel* edgelabel);

  /**
   * The specified edge label now has a smaller cost.  Reorders it in the
   * sorted bucket list.
   * @param   edgelabel  Edge label (directed edge) to reorder in the
   *                     adjacency list
   * @param   previouscost Previous cost.
   */
  void DecreaseCost(EdgeLabel* edgelabel, const float previouscost);

  /**
   * Removes the lowest cost edge label from the sorted list.
   * @return  Pointer to the edge label with lowest cost.
   */
  EdgeLabel* Remove();

 private:
  float bucketrange_;  // Total range of costs in lower level buckets
  float bucketcount_;  // Number of buckets
  float bucketsize_;   // Bucket size (range of costs in same bucket)
  float mincost_;      // Minimum cost within the low level buckets
  float maxcost_;      // Above this goes into overflow bucket.
  float currentcost_;  // Current cost.

  // Low level buckets
  std::vector<std::list<EdgeLabel*> > buckets_;

  // Current bucket in the list
  std::vector<std::list<EdgeLabel*> >::iterator currentbucket_;

  // Overflow bucket
  std::list<EdgeLabel*> overflowbucket_;

  // Make the default constructor private to force use of one with args
  AdjacencyList();

  // Returns the bucket given the cost
  std::list<EdgeLabel*>& Bucket(const float cost);

  // Empties the overflow bucket by placing the edge labels into the
  // low level buckets.
  void EmptyOverflow();
};

}
}

#endif  // VALHALLA_THOR_ADJACENCYLIST_H_

