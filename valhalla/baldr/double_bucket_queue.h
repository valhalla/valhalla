#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/util.h>
#include <vector>

namespace valhalla {
namespace baldr {

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
template <typename label_t> class DoubleBucketQueue final {
public:
  /**
   * Default c-tor creates empty object that needs to be initialized with `reuse` method
   */
  DoubleBucketQueue() {
    reuse(0.f, 1.f, 1, nullptr);
  }

  /**
   * Constructor given a minimum cost, a range of costs held within the
   * bucket sort, and a bucket size. All costs above mincost + range are
   * stored in an "overflow" bucket.
   * @param mincost    Minimum cost. Used to create the initial range for
   *                   bucket sorting.
   * @param range      Cost range for low-level buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   *                   Must be an integer value.
   * @param labelcontainer  Container of labels with sortcosts.
   */
  DoubleBucketQueue(const float mincost,
                    const float range,
                    const uint32_t bucketsize,
                    const std::vector<label_t>* labelcontainer) {
    reuse(mincost, range, bucketsize, labelcontainer);
  }

  DoubleBucketQueue(DoubleBucketQueue&&) = default;
  DoubleBucketQueue& operator=(DoubleBucketQueue&&) = default;
  DoubleBucketQueue(const DoubleBucketQueue&) = delete;
  DoubleBucketQueue& operator=(const DoubleBucketQueue&) = delete;

  /**
   * The same as c-tor, but without buffers reallocation. Before call this
   * method you should clean up the current state (call `clear`).
   * @param mincost    Minimum cost. Used to create the initial range for
   *                   bucket sorting.
   * @param range      Cost range for low-level buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   *                   Must be an integer value.
   * @param labelcontainer  Container of labels with sortcosts.
   */
  void reuse(const float mincost,
             const float range,
             const uint32_t bucketsize,
             const std::vector<label_t>* labelcontainer) {
    labelcontainer_ = labelcontainer;
    // We need at least a bucketsize of 1 or more
    if (bucketsize < 1) {
      throw std::runtime_error("Bucketsize must be 1 or greater");
    }

    // We need at least a bucketrange of something larger than 0
    if (range <= 0.f) {
      throw std::runtime_error("Bucketrange must be greater than 0");
    }

    // Adjust min cost to be the start of a bucket
    const uint32_t c = static_cast<uint32_t>(mincost);
    currentcost_ = (c - (c % bucketsize));
    mincost_ = currentcost_;
    bucketrange_ = range;
    bucketsize_ = static_cast<float>(bucketsize);
    inv_ = 1.0f / bucketsize_;

    // Set the maximum cost (above this goes into the overflow bucket)
    maxcost_ = mincost_ + bucketrange_;

    // Allocate the low-level buckets
    const size_t bucketcount = (range / bucketsize_) + 1;
    buckets_.resize(bucketcount);

    // Set the current bucket to the lowest cost low level bucket
    currentbucket_ = buckets_.begin();
  }

  /**
   * Clear all labels from the low-level buckets and the overflow buckets and deallocates buckets
   * memory.
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
    get_bucket((*labelcontainer_)[label].sortcost()).push_back(label);
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
    bucket_t& prevbucket = get_bucket((*labelcontainer_)[label].sortcost());
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
        --currentbucket_;
        return baldr::kInvalidLabel;
      } else {
        // Move labels from the overflow bucket to the low level buckets.
        // Return invalid label if still empty.
        empty_overflow();
        if (empty()) {
          return baldr::kInvalidLabel;
        }
      }
    }

    // Return label from lowest non-empty bucket
    const uint32_t label = currentbucket_->back();
    currentbucket_->pop_back();
    return label;
  }

private:
  float bucketrange_; // Total range of costs in lower level buckets
  float bucketsize_;  // Bucket size (range of costs in same bucket)
  float inv_;         // 1/bucketsize (so we can avoid division)
  double mincost_;    // Minimum cost within the low level buckets
  float maxcost_;     // Above this goes into overflow bucket
  float currentcost_; // Current cost

  // Low level buckets
  buckets_t buckets_;

  // Current bucket
  buckets_t::iterator currentbucket_;

  // Overflow bucket
  bucket_t overflowbucket_;

  // Access to a container of labels to get cost given the label index.
  const std::vector<label_t>* labelcontainer_;

  /**
   * Returns the bucket given the cost.
   * @param  cost  Cost.
   * @return Returns the bucket that the cost lies within.
   */
  bucket_t& get_bucket(const float cost) {
    return (cost < currentcost_) ? *currentbucket_
           : (cost < maxcost_)   ? buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)]
                                 : overflowbucket_;
  }

  /**
   * Increments currentbucket_in the low-level buckets until a non-empty
   * bucket is found.
   * @return  Returns true if the low-level buckets are all empty.
   */
  bool empty() {
    while (currentbucket_ != buckets_.end() && currentbucket_->empty()) {
      ++currentbucket_;
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
                         [this](uint32_t a, uint32_t b) {
                           return (*labelcontainer_)[a].sortcost() < (*labelcontainer_)[b].sortcost();
                         });

    // If there is actually stuff to move
    if (itr != overflowbucket_.end()) {

      // Adjust cost range so smallest element is in the buckets_
      float min = (*labelcontainer_)[*itr].sortcost();
      mincost_ += (std::floor((min - mincost_) / bucketrange_)) * bucketrange_;

      // Avoid precision issues
      if (mincost_ > min) {
        mincost_ -= bucketrange_;
      } else if (mincost_ + bucketrange_ < min) {
        mincost_ += bucketrange_;
      }
      maxcost_ = mincost_ + bucketrange_;

      // Move elements within the range from overflow to buckets
      auto minLabelsIt =
          std::remove_if(overflowbucket_.begin(), overflowbucket_.end(), [this](const auto label) {
            float cost = (*labelcontainer_)[label].sortcost();
            if (cost < maxcost_) {
              buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)].push_back(label);
              return true;
            }
            return false;
          });
      // Remove labels with min costs from overflow bucket
      overflowbucket_.erase(minLabelsIt, overflowbucket_.end());
    }

    // Reset current cost and bucket to beginning of low level buckets
    currentcost_ = mincost_;
    currentbucket_ = buckets_.begin();
  }
};

} // namespace baldr
} // namespace valhalla
