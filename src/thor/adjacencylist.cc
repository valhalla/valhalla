#include "thor/adjacencylist.h"

namespace valhalla {
namespace thor {

// Default constructor
AdjacencyList::AdjacencyList()
    : bucketrange_(0),
      bucketcount_(0),
      bucketsize_(0),
      mincost_(0),
      maxcost_(0),
      currentcost_(0) {
}

// Constructor with bucket sizes and range.
AdjacencyList::AdjacencyList(const unsigned int mincost,
                             const unsigned int range,
                             const unsigned int bucketsize) {
  mincost_ = mincost;
  currentcost_ = mincost_;
  bucketrange_ = range;
  bucketsize_ = bucketsize;

  // Set the maximum cost (above this goes into the overflow bucket)
  maxcost_ = mincost + bucketrange_;

  // Allocate the low-level buckets
  bucketcount_ = (range / bucketsize_);
  buckets_.resize(bucketcount_);
  currentbucket_ = buckets_.begin();

  // Set the current bucket to the lowest cost low level bucket
  currentbucket_ = buckets_.begin();
}

// Destructor
AdjacencyList::~AdjacencyList() {
  Clear();
}

// Clear all edge labels from from the adjacency list. Deletes the
// remaining allocated edge labels.
void AdjacencyList::Clear() {
  // Empty the overflow bucket first for efficiency
  EdgeLabel* elem;
  while (!overflowbucket_.empty()) {
    elem = overflowbucket_.front();
    overflowbucket_.pop_front();
    delete elem;
  }

  // Delete edge labels from the low-level buckets
  if (currentbucket_ != buckets_.end()) {
    while ((elem = Remove()) != nullptr)
      delete elem;
  }
}

// Add an edge label to the adjacency list. Adds it to the appropriate bucket
// based on its sort cost.
void AdjacencyList::Add(EdgeLabel* edgelabel) {
  if (edgelabel->sortcost() < currentcost_) {
    currentbucket_->push_front(edgelabel);
  } else {
    Bucket(edgelabel->sortcost()).push_back(edgelabel);
  }
}

/**
 * The specified edge label now has a smaller cost.  Reorders it in
 * the sorted list.
 * @param   edgelabel  Directed edge to reorder in the adjacency list
 * @param   previouscost Previous cost.
 */
void AdjacencyList::DecreaseCost(EdgeLabel* edgelabel,
                                 const unsigned int previouscost) {
  // Get the bucket the edge label currently is in. Protect against
  // previous cost less than current cost
  std::list<EdgeLabel*>& previousbucket =
      (previouscost < currentcost_) ?
          Bucket(currentcost_) : Bucket(previouscost);

  // If less than current cost add to the front of the current bucket
  unsigned int cost = edgelabel->sortcost();
  if (cost < currentcost_) {
    // Remove the edge label from the old bucket and push it on the
    // front of the current (so it is the next edge processed)
    previousbucket.remove(edgelabel);
    currentbucket_->push_front(edgelabel);
    return;
  }

  // If the old cost and the new cost are in the same bucket just return
  std::list<EdgeLabel*>& newbucket = Bucket(cost);
  if (previousbucket == newbucket)
    return;

  // Remove the edge label from the old bucket and add to the new bucket
  previousbucket.remove(edgelabel);
  newbucket.push_back(edgelabel);
}

/**
 * Removes the lowest cost edge label from the sorted list.
 * @return  Pointer to the edge label with lowest cost.
 */
EdgeLabel* AdjacencyList::Remove() {
  // If the current bucket is not empty return the edge label off the front
  if (!currentbucket_->empty()) {
    EdgeLabel* edgelabel = currentbucket_->front();
    currentbucket_->pop_front();
    return edgelabel;
  }

  // If current bucket is empty increment until a non-empty low-level
  // bucket is found. If we get to the end of the regular buckets
  // we move edge labels from the overflow buckets into the regular buckets.
  for ( ; currentbucket_ != buckets_.end(); currentbucket_++) {
    // If the current bucket is not empty get the first edge label off the list
    if (!currentbucket_->empty()) {
      EdgeLabel* edgelabel = currentbucket_->front();
      currentbucket_->pop_front();
      return edgelabel;
    }
    currentcost_ += bucketsize_;
  }

  // If no edge labels are in the overflow buckets, there are no more entries
  if (overflowbucket_.empty())
    return nullptr;

  // No edge labels in the low level buckets.  Move edge labels from the
  // overflow bucket to the low level buckets. Then find smallest bucket
  // that is not empty
  EmptyOverflow();
  for (currentbucket_ = buckets_.begin(); currentbucket_ != buckets_.end();
      currentbucket_++) {
    // If current bucket is not empty get the first edge label off the list
    if (!currentbucket_->empty()) {
      EdgeLabel* edgelabel = currentbucket_->front();
      currentbucket_->pop_front();
      return edgelabel;
    }
    currentcost_ += bucketsize_;
  }
  return nullptr;
}

// Returns the bucket given the cost
std::list<EdgeLabel*>& AdjacencyList::Bucket(const unsigned int cost) {
  return
      (cost < maxcost_) ?
          buckets_[((cost - mincost_) / bucketsize_)] : overflowbucket_;
}

// Empties the overflow bucket by placing the edgelabels into the
// low level buckets.
void AdjacencyList::EmptyOverflow() {
  bool found = false;
  unsigned int cost;
  EdgeLabel* edgelabel;
  std::vector<EdgeLabel*> tmp;
  while (!found && !overflowbucket_.empty()) {
    // Adjust cost range
    mincost_ += bucketrange_;
    maxcost_ += bucketrange_;
    currentcost_ = mincost_;

    tmp.clear();
    while (!overflowbucket_.empty()) {
      edgelabel = overflowbucket_.front();
      overflowbucket_.pop_front();
      cost = edgelabel->sortcost();
      if (cost < maxcost_) {
        buckets_[((cost - mincost_) / bucketsize_)].push_back(edgelabel);
        found = true;
      } else {
        tmp.push_back(edgelabel);
      }
    }

    // Clear overflow and add any edge labels that lie outside the new range
    overflowbucket_.clear();
    for (auto edgelabel : tmp) {
      overflowbucket_.push_back(edgelabel);
    }
  }
}

}
}

