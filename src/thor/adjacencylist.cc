#include "thor/adjacencylist.h"

using namespace valhalla::sif;

namespace valhalla {
namespace thor {

// Constructor with bucket sizes and range.
AdjacencyList::AdjacencyList(const float mincost,
                             const float range,
                             const uint32_t bucketsize) {
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
}

// Destructor
AdjacencyList::~AdjacencyList() {
  Clear();
}

// Clear all label indexes from from the adjacency list.
void AdjacencyList::Clear() {
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

// Add a label index to the adjacency list. Adds it to the appropriate bucket
// based on the sort cost.
void AdjacencyList::Add(const uint32_t label, const float sortcost) {
  Bucket(sortcost).push_back(label);
}

// The specified label now has a smaller cost.  Reorders it in the sorted list
void AdjacencyList::DecreaseCost(const uint32_t label,
                                 const float newsortcost,
                                 const float previouscost) {
  // Get the buckets of the previous and new costs. Nothing needs to be done
  // if old cost and the new cost are in the same buckets.
  auto& prevbucket = Bucket(previouscost);
  auto& newbucket  = Bucket(newsortcost);
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
uint32_t AdjacencyList::Remove(const std::vector<EdgeLabel>& edgelabels) {
  // If the current bucket is not empty return the label off the front
  if (!currentbucket_->empty()) {
    uint32_t label = currentbucket_->front();
    currentbucket_->pop_front();
    return label;
  }

  // If current bucket is empty increment until a non-empty low-level
  // bucket is found. If we get to the end of the regular buckets
  // we move labels from the overflow buckets into the regular buckets.
  for ( ; currentbucket_ != buckets_.end(); currentbucket_++) {
    // If current bucket is not empty get the first label off the list
    if (!currentbucket_->empty()) {
      uint32_t label = currentbucket_->front();
      currentbucket_->pop_front();
      return label;
    }
    currentcost_ += bucketsize_;
  }

  // Return an invalid label if no labels are in the overflow buckets
  if (overflowbucket_.empty()) {
    // Reset currentbucket to the last bucket - in case another access of
    // adjacency list is done
    currentbucket_--;
    return kInvalidLabel;
  }

  // Move labels from the overflow bucket to the low level buckets. Then find
  // smallest bucket that is not empty and set it as the currentbucket
  EmptyOverflow(edgelabels);
  for (currentbucket_ = buckets_.begin(); currentbucket_ != buckets_.end();
       currentbucket_++) {
    // If current bucket is not empty return the first label off the list
    if (!currentbucket_->empty()) {
      uint32_t label = currentbucket_->front();
      currentbucket_->pop_front();
      return label;
    }
    currentcost_ += bucketsize_;
  }
  return kInvalidLabel;
}

// Returns the bucket given the cost
std::deque<uint32_t>& AdjacencyList::Bucket(const float cost) {
  if (cost < currentcost_) {
    return *currentbucket_;
  } else {
    return (cost < maxcost_) ?
        buckets_[static_cast<uint32_t>((cost - mincost_) * inv_)] :
        overflowbucket_;
  }
}

// Empties the overflow bucket by placing the labels into the
// low level buckets.
void AdjacencyList::EmptyOverflow(const std::vector<EdgeLabel>& edgelabels) {
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
      float cost = edgelabels[label].sortcost();
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

