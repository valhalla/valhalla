#ifndef VALHALLA_THOR_ADJACENCYLIST_H_
#define VALHALLA_THOR_ADJACENCYLIST_H_

#include <vector>
#include <deque>

#include <valhalla/sif/edgelabel.h>

namespace valhalla {
namespace thor {

/**
 * Adjacency list support. Uses a bucket sort implementation for performance.
 * An "overflow" bucket is maintained to allow reduced memory use - costs
 * outside the current bucket "range" get placed into the overflow bucket and
 * are moved into the low-level buckets as needed. The adjacency list stores
 * indexes into a list (vector) of labels where complete cost and predecessor
 * information are stored. The adjacency list simply provides a fast sorting
 * method.
 */
class AdjacencyList {
 public:
  /**
   * Constructor given a minimum cost, a range of costs held within the
   * bucket sort, and a bucket size. All costs above mincost + range are
   * stored in an "overflow" bucket.
   * @param mincost    Minimum sort cost (based on A* heuristic). Used
   *                   to create the initial range for bucket sorting.
   * @param range      Cost (sort cost) range for low-level buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   *                   Must be an integer value.
   */
  AdjacencyList(const float mincost, const float range,
                const uint32_t bucketsize);

  /**
   * Destructor.
   */
  virtual ~AdjacencyList();

  /**
   * Clear all labels from from the adjacency list.
   */
  void Clear();

  /**
   * Adds a label index to the sorted list. Adds it to the appropriate bucket
   * given the sort cost. If the sortcost is greater than maxcost_ the label
   * is placed in the overflow bucket. If the sortcost is < the current bucket
   * cost then the label is placed at the front of the current bucket (this
   * prevents underflow).
   * @param   label     Label index to add to the adjacency list.
   * @param   sortcost  Sort cost for this label.
   */
  void Add(const uint32_t label, const float sortcost);

  /**
   * The specified label index now has a smaller cost.  Reorders it in the
   * sorted bucket list.
   * @param  label  Label index to reorder in the adjacency list
   * @param  newsortcost  New sort cost.
   * @param  previouscost Previous sort cost.
   */
  void DecreaseCost(const uint32_t label,
                    const float newsortcost,
                    const float previouscost);

  /**
   * Removes the lowest cost label index from the sorted list.
   * @param  edgelabels  List of labels (for use when emptying
   *                         overflow buckets).
   * @return  Returns the label index of the lowest cost label. Returns
   *             kInvalidLabel if the adjacency list is empty.
   */
  uint32_t Remove(const std::vector<sif::EdgeLabel>& edgelabels);

 private:
  float bucketrange_;  // Total range of costs in lower level buckets
  float bucketcount_;  // Number of buckets
  float bucketsize_;   // Bucket size (range of costs in same bucket)
  float inv_;          // 1/bucketsize (so we can avoid division)
  float mincost_;      // Minimum cost within the low level buckets
  float maxcost_;      // Above this goes into overflow bucket.
  float currentcost_;  // Current cost.

  // Low level buckets
  std::vector<std::deque<uint32_t>> buckets_;

  // Current bucket in the deque
  std::vector<std::deque<uint32_t>>::iterator currentbucket_;

  // Overflow bucket
  std::deque<uint32_t> overflowbucket_;

  /**
   * Returns the bucket given the cost.
   * @param  cost  Sort cost.
   * @return Returns the bucket that the cost lies within.
   */
  std::deque<uint32_t>& Bucket(const float cost);

  /**
   * Empties the overflow bucket by placing the label indexes into the
   * low level buckets.
   * @param  edgelabels  List of all edge labels (required to get costs).
   */
  void EmptyOverflow(const std::vector<sif::EdgeLabel>& edgelabels);
};

}
}

#endif  // VALHALLA_THOR_ADJACENCYLIST_H_

