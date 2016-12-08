#include "baldr/double_bucket_queue.h"

namespace valhalla {
namespace baldr {

// Constructor given a minimum cost, a range of costs held within the
// bucket sort, and a bucket size. All costs above mincost + range are
// stored in an "overflow" bucket.
DoubleBucketQueue::DoubleBucketQueue(const float mincost, const float range,
          const uint32_t bucketsize, const LabelCost& labelcost) {
  // Adjust min cost to be the start of a bucket
  uint32_t c = static_cast<uint32_t>(mincost);
  currentcost_ = (c - (c % bucketsize));
  mincost_ = currentcost_;
  bucketrange_ = range;
  bucketsize_ = static_cast<float>(bucketsize);
  inv_ = 1.0f / bucketsize_;

  // Set the maximum cost (above this goes into the overflow bucket)
  maxcost_ = mincost + bucketrange_;

  // Allocate the low-level buckets
  bucketcount_ = (range / bucketsize_) + 1;
  buckets_.resize(bucketcount_);

  // Set the current bucket to the lowest cost low level bucket
  currentbucket_ = buckets_.begin();

  // Set the cost function.
  labelcost_ = labelcost;
}

// Destructor
DoubleBucketQueue::~DoubleBucketQueue() {
  clear();
}

// Clear all labels from the low-level buckets and the overflow buckets.
void DoubleBucketQueue::clear() {
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

// The specified label now has a smaller cost.  Reorders it in the sorted list
void DoubleBucketQueue::decrease(const uint32_t label, const float newcost,
                                 const float previouscost) {
  // Get the buckets of the previous and new costs. Nothing needs to be done
  // if old cost and the new cost are in the same buckets.
  auto& prevbucket = get_bucket(previouscost);
  auto& newbucket  = get_bucket(newcost);
  if (prevbucket != newbucket) {
    // Remove the label index from the old bucket and add to end of newbucket
    for (auto it = prevbucket.begin(); it != prevbucket.end(); ++it) {
      if (*it == label) {
        prevbucket.erase(it);
        break;
      }
    }
    newbucket.push_back(label);
  }
}

// Remove the label with the lowest cost
uint32_t DoubleBucketQueue::pop() {
  const auto nextlabel = [this]() {
    uint32_t label = currentbucket_->front();
    currentbucket_->pop_front();
    return label;
  };

  // Return a label from lowest non-empty bucket.
  for ( ; currentbucket_ != buckets_.end(); currentbucket_++,
          currentcost_ += bucketsize_) {
    if (!currentbucket_->empty()) {
      return nextlabel();
    }
  }

  // No labels found in the low-level buckets. Return an invalid label if no
  // labels are in the overflow buckets
  if (overflowbucket_.empty()) {
    // Reset currentbucket to the last bucket - in case another access of
    // adjacency list is done
    currentbucket_--;
    return kInvalidLabel;
  }

  // Move labels from the overflow bucket to the low level buckets. Then find
  // smallest bucket that is not empty and set it as the currentbucket and
  // return its first label.
  empty_overflow();
  for (currentbucket_ = buckets_.begin(); currentbucket_ != buckets_.end();
           currentbucket_++, currentcost_ += bucketsize_) {
    if (!currentbucket_->empty()) {
      return nextlabel();
    }
  }
  return kInvalidLabel;
}

// Empties the overflow bucket by placing the labels into the
// low level buckets.
void DoubleBucketQueue::empty_overflow() {
  bool found = false;
  std::vector<uint32_t> tmp;
  while (!found && !overflowbucket_.empty()) {
    // Adjust cost range
    mincost_ += bucketrange_;
    maxcost_ += bucketrange_;
    currentcost_ = mincost_;

    tmp.clear();
    while (!overflowbucket_.empty()) {
      uint32_t label = overflowbucket_.front();
      overflowbucket_.pop_front();

      // Get the cost (using the label cost function)
      float cost = labelcost_(label);
      if (cost < maxcost_) {
        buckets_[static_cast<uint32_t>((cost-mincost_)*inv_)].push_back(label);
        found = true;
      } else {
        tmp.push_back(label);
      }
    }

    // Clear overflow and add any labels that lie outside the new range
    overflowbucket_.clear();
    for (auto label : tmp) {
      overflowbucket_.push_back(label);
    }
  }
}

}
}

