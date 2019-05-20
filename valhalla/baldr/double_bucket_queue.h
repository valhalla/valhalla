#ifndef VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_
#define VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

constexpr uint32_t kInvalidLabel = std::numeric_limits<uint32_t>::max();

/**
 * A callable element which returns the cost for a label.
 */
using LabelCost = std::function<float(const uint32_t label)>;

// Bucket type and bucket list type.
using bucket_t = std::vector<uint32_t>;
using buckets_t = std::vector<bucket_t>;

/**
 * Double Bucket Queue - a form of priority queue. Contains a bucket sort
 * implementation for performance. An "overflow" bucket is maintained to allow
 * reduced memory use. Costs outside the current bucket "range" get placed
 * into the overflow bucket and are moved into the low-level buckets as
 * needed. Each bucket stores label indexes into external data.
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
  DoubleBucketQueue(const float mincost,
                    const float range,
                    const uint32_t bucketsize,
                    const LabelCost& labelcost) {
    // We need at least a bucketsize of 1 or more
    if (bucketsize < 1) {
      throw std::runtime_error("Bucketsize must be 1 or greater");
    }

    // We need at least a bucketrange of something larger than 0
    if (range <= 0.f) {
      throw std::runtime_error("Bucketrange must be greater than 0");
    }

    // Adjust min cost to be the start of a bucket
    uint32_t c = static_cast<uint32_t>(mincost);
    currentcost_ = (c - (c % bucketsize));
    mincost_ = currentcost_;
    bucketrange_ = range;
    bucketsize_ = static_cast<float>(bucketsize);
    inv_ = 1.0f / bucketsize_;

    // Set the maximum cost (above this goes into the overflow bucket)
    maxcost_ = mincost_ + bucketrange_;

    // Allocate the low-level buckets
    size_t bucketcount = (range / bucketsize_) + 1;
    buckets_.resize(bucketcount);

    // Set the current bucket to the lowest cost low level bucket
    currentbucket_ = buckets_.begin();

    // Set the cost function.
    labelcost_ = labelcost;
  }

  /**
   * Destructor.
   */
  virtual ~DoubleBucketQueue() {
    clear();
  }

  /**
   * Clear all labels from the low-level buckets and the overflow buckets.
   */
  void clear() {
    // Empty the overflow bucket and each bucket
    overflowbucket_.clear();
    while (currentbucket_ != buckets_.end()) {
      currentbucket_->clear();
      currentbucket_++;
    }

    // Reset current bucket and cost
    currentcost_ = mincost_;
    currentbucket_ = buckets_.begin();
  }

  /**
   * Adds a label index to the bucketed sort. Adds it to the appropriate bucket
   * given the cost. If the cost is greater than maxcost_ the label
   * is placed in the overflow bucket. If the cost is < the current bucket
   * cost then the label is placed in the current bucket to prevent underflow.
   * @param   label  Label index to add to the queue.
   */
  void add(const uint32_t label) {
    get_bucket(labelcost_(label)).push_back(label);
  }

  /**
   * The specified label index now has a smaller cost.  Reorders it in the
   * sorted bucket list. Uses the labelcost_ function to get the bucket that
   * the label is currently within.
   * @param  label        Label index to reorder.
   * @param  newcost      New sort cost.
   */
  void decrease(const uint32_t label, const float newcost) {
    // Get the buckets of the previous and new costs. Nothing needs to be done
    // if old cost and the new cost are in the same buckets.
    bucket_t& prevbucket = get_bucket(labelcost_(label));
    bucket_t& newbucket = get_bucket(newcost);
    if (prevbucket != newbucket) {
      // Add label to newbucket and remove from previous bucket
      newbucket.push_back(label);
      prevbucket.erase(std::remove(prevbucket.begin(), prevbucket.end(), label));
    }
  }

  /**
   * Removes the lowest cost label index from the sorted buckets.
   * @return  Returns the label index of the lowest cost label. Returns
   *          kInvalidLabel if the buckets are empty.
   */
  uint32_t pop() {
    if (empty()) {
      // No labels found in the low-level buckets.
      if (overflowbucket_.empty()) {
        // Return an invalid label if no labels are in the overflow buckets.
        // Reset currentbucket to the last bucket - in case another access of
        // adjacency list is done.
        currentbucket_--;
        return kInvalidLabel;
      } else {
        // Move labels from the overflow bucket to the low level buckets.
        // Return invalid label if still empty.
        empty_overflow();
        if (empty()) {
          return kInvalidLabel;
        }
      }
    }

    // Return label from lowest non-empty bucket
    uint32_t label = currentbucket_->back();
    currentbucket_->pop_back();
    return label;
  }

private:
  float bucketrange_; // Total range of costs in lower level buckets
  float bucketsize_;  // Bucket size (range of costs in same bucket)
  float inv_;         // 1/bucketsize (so we can avoid division)
  float mincost_;     // Minimum cost within the low level buckets
  float maxcost_;     // Above this goes into overflow bucket
  float currentcost_; // Current cost

  // Low level buckets
  buckets_t buckets_;

  // Current bucket
  buckets_t::iterator currentbucket_;

  // Overflow bucket
  bucket_t overflowbucket_;

  // Cost function to get cost given the label index.
  LabelCost labelcost_;

  /**
   * Returns the bucket given the cost.
   * @param  cost  Cost.
   * @return Returns the bucket that the cost lies within.
   */
  bucket_t& get_bucket(const float cost) {
    return (cost < currentcost_)
               ? *currentbucket_
               : (cost < maxcost_) ? buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)]
                                   : overflowbucket_;
  }

  /**
   * Increments currentbucket_in the low-level buckets until a non-empty
   * bucket is found.
   * @return  Returns true if the low-level buckets are all empty.
   */
  bool empty() {
    while (currentbucket_ != buckets_.end() && currentbucket_->empty()) {
      currentbucket_++;
      currentcost_ += bucketsize_;
    }
    return currentbucket_ == buckets_.end();
  }

  /**
   * Empties the overflow bucket by placing the label indexes into the
   * low level buckets.
   */
  void empty_overflow() {
    // Get the minimum label so we can figure out where the new range should be
    auto itr =
        std::min_element(overflowbucket_.begin(), overflowbucket_.end(),
                         [this](uint32_t a, uint32_t b) { return labelcost_(a) < labelcost_(b); });

    // If there is actually stuff to move
    if (itr != overflowbucket_.end()) {

      // Adjust cost range so smallest element is in the buckets_
      float min = labelcost_(*itr);
      mincost_ += (std::floor((min - mincost_) / bucketrange_)) * bucketrange_;

      // Avoid precision issues
      if (mincost_ > min) {
        mincost_ -= bucketrange_;
      } else if (mincost_ + bucketrange_ < min) {
        mincost_ += bucketrange_;
      }
      maxcost_ = mincost_ + bucketrange_;

      // Move elements within the range from overflow to buckets
      bucket_t tmp;
      for (const auto& label : overflowbucket_) {
        // Get the cost (using the label cost function)
        float cost = labelcost_(label);
        if (cost < maxcost_) {
          buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)].push_back(label);
        } else {
          tmp.push_back(label);
        }
      }

      // Add any labels that lie outside the new range back to overflow bucket
      overflowbucket_ = std::move(tmp);
    }

    // Reset current cost and bucket to beginning of low level buckets
    currentcost_ = mincost_;
    currentbucket_ = buckets_.begin();
  }
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_DOUBLE_BUCKET_QUEUE_H_
