#include "thor/adjacencylist.h"

namespace valhalla {
namespace thor {

// Default constructor
AdjacencyList::AdjacencyList()
    : bucketrange_(0.0f),
      bucketcount_(0.0f),
      bucketsize_(0.0f),
      mincost_(0.0f),
      maxcost_(0.0f),
      currentcost_(0.0f) {
}

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

  // Set the maximum cost (above this goes into the overflow bucket)
  maxcost_ = mincost + bucketrange_;

  // Allocate the low-level buckets
  bucketcount_ = (range / bucketsize_) + 1;
  buckets_.resize(bucketcount_);
  currentbucket_ = buckets_.begin();

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
  if (sortcost < currentcost_) {
    currentbucket_->emplace_front(label);
  } else {
    Bucket(sortcost).emplace_back(label);
  }
}

// The specified label now has a smaller cost.  Reorders it in the sorted list
void AdjacencyList::DecreaseCost(const uint32_t label,
                                 const float newsortcost,
                                 const float previouscost) {
  // Get the bucket the label index currently is in. Protect against
  // previous cost less than current cost
  std::list<uint32_t>& previousbucket =
      (previouscost < currentcost_) ?
          Bucket(currentcost_) : Bucket(previouscost);

  // If less than current cost add to the front of the current bucket
  if (newsortcost < currentcost_) {
    // Remove the label index from the old bucket and push it on the
    // front of the current (so it is the next edge processed)
    previousbucket.remove(label);
    currentbucket_->emplace_front(label);
    return;
  }

  // If the old cost and the new cost are in the same bucket just return
  std::list<uint32_t>& newbucket = Bucket(newsortcost);
  if (previousbucket == newbucket)
    return;

  // Remove the label index from the old bucket and add it to the new bucket
  previousbucket.remove(label);
  newbucket.emplace_back(label);
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
  if (overflowbucket_.empty())
    return kInvalidLabel;

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
std::list<uint32_t>& AdjacencyList::Bucket(const float cost) {
  return (cost < maxcost_) ?
      buckets_[(uint32_t)((cost - mincost_) / bucketsize_)] :
      overflowbucket_;
}

// Empties the overflow bucket by placing the labels into the
// low level buckets.
void AdjacencyList::EmptyOverflow(const std::vector<EdgeLabel>& edgelabels) {
  bool found = false;
  float cost;
  uint32_t label;
  std::vector<uint32_t> tmp;
  while (!found && !overflowbucket_.empty()) {
    // Adjust cost range
    mincost_ += bucketrange_;
    maxcost_ += bucketrange_;
    currentcost_ = mincost_;

    tmp.clear();
    while (!overflowbucket_.empty()) {
      label = overflowbucket_.front();
      overflowbucket_.pop_front();
      cost = edgelabels[label].sortcost();
      if (cost < maxcost_) {
        buckets_[((cost - mincost_) / bucketsize_)].emplace_back(label);
        found = true;
      } else {
        tmp.emplace_back(label);
      }
    }

    // Clear overflow and add any labels that lie outside the new range
    overflowbucket_.clear();
    for (auto label : tmp) {
      overflowbucket_.emplace_back(label);
    }
  }
}

}
}

