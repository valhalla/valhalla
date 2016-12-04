#ifndef VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_
#define VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_

#include <vector>
#include <deque>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

constexpr uint32_t kInvalidLabel = std::numeric_limits<uint32_t>::max();

/**
 * A callable element which returns the cost for a label.
 */
using LabelCost = std::function<float (const uint32_t label)>;

/**
 * Double Bucket Queue. Contains a bucket sort implementation for performance.
 * An "overflow" bucket is maintained to allow reduced memory use. Costs
 * outside the current bucket "range" get placed into the overflow bucket and
 * are moved into the low-level buckets as needed. Each bucket stores label
 * indexes into external data.
 */
class DoubleBucketQueue {
 public:
  /**
   * Constructor given a minimum cost, a range of costs held within the
   * bucket sort, and a bucket size. All costs above mincost + range are
   * stored in an "overflow" bucket.
   * @param mincost    Minimum cost. Used to create the initial range for
   *                   bucket sorting.
   * @param range      Cost range for low-level buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   *                   Must be an integer value.
   * @param labelcost  Functor to get a cost given a label index.
   */
  DoubleBucketQueue(const float mincost, const float range,
                    const uint32_t bucketsize, const LabelCost& labelcost);

  /**
   * Destructor.
   */
  virtual ~DoubleBucketQueue();

  /**
   * Clear all labels from the low-level buckets and the overflow buckets.
   */
  void clear();

  /**
   * Adds a label index to the bucketed sort. Adds it to the appropriate bucket
   * given the cost. If the cost is greater than maxcost_ the label
   * is placed in the overflow bucket. If the cost is < the current bucket
   * cost then the label is placed in the current bucket to prevent underflow.
   * @param   label  Label index to add to the adjacency list.
   * @param   cost   Cost for this label.
   */
  void add(const uint32_t label, const float cost) {
    get_bucket(cost).push_back(label);
  }

  /**
   * The specified label index now has a smaller cost.  Reorders it in the
   * sorted bucket list.
   * @param  label        Label index to reorder.
   * @param  newcost      New sort cost.
   * @param  previouscost Previous cost.
   */
  void decrease(const uint32_t label, const float newcost,
                const float previouscost);

  /**
   * Removes the lowest cost label index from the sorted buckets.
   * @return  Returns the label index of the lowest cost label. Returns
   *          kInvalidLabel if the buckets are empty.
   */
  uint32_t pop();

 private:
  float bucketrange_;  // Total range of costs in lower level buckets
  float bucketcount_;  // Number of buckets
  float bucketsize_;   // Bucket size (range of costs in same bucket)
  float inv_;          // 1/bucketsize (so we can avoid division)
  float mincost_;      // Minimum cost within the low level buckets
  float maxcost_;      // Above this goes into overflow bucket
  float currentcost_;  // Current cost

  // Low level buckets
  std::vector<std::deque<uint32_t>> buckets_;

  // Current bucket
  std::vector<std::deque<uint32_t>>::iterator currentbucket_;

  // Overflow bucket
  std::deque<uint32_t> overflowbucket_;

  // Cost function to get cost given the label index.
  LabelCost labelcost_;

  /**
   * Returns the bucket given the cost.
   * @param  cost  Cost.
   * @return Returns the bucket that the cost lies within.
   */
  std::deque<uint32_t>& get_bucket(const float cost) {
    return (cost < currentcost_) ? *currentbucket_ :
             (cost < maxcost_) ?
               buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)] :
               overflowbucket_;
  }

  /**
   * Empties the overflow bucket by placing the label indexes into the
   * low level buckets.
   */
  void empty_overflow();
};

}
}

#endif  // VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_

