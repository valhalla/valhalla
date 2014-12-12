#include "thor/adjacencylist.h"

namespace valhalla {
namespace thor {

/**
 * Constructor.
 */
template<class T>
AdjacencyList<T>::AdjacencyList()
    : bucketrange_(0),
      bucketcount_(0),
      bucketsize_(0),
      mincost_(0),
      maxcost_(0),
      currentcost_ (0) {
}

/**
 * Constructor given a minimum cost. May be needed if a bucket sort
 * implementation is used (to save memory).
 * @param mincost    Minimum cost (based on A* heuristic) in a route. Used
 *                   to create the initial range for bucket sorting.
 * @param range      Cost range for double buckets.
 * @param bucketsize Bucket size (range of costs within same bucket).
 */
template<class T>
AdjacencyList<T>::AdjacencyList(const unsigned int mincost,
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

  // Clear the overflow bucket
  overflowbucket_.clear();
}

// Destructor
template<class T>
AdjacencyList<T>::~AdjacencyList() {
  // Delete all elements in the list - empty the overflow bucket first
  // for efficiency
  AdjacencyList<T> elem;
  while (!overflowbucket_.empty()) {
    elem = overflowbucket_.pop_front();
    delete elem;
  }
  while ((elem = Remove()) != nullptr)
    delete elem;
}

/**
 * Adds an element to the sorted list. Adds the element to the appropriate
 * bucket given its cost. If the cost is greater than maxcost_ the element
 * is placed in the overflow bucket. If the cost is < the current bucket
 * cost then the element is placed at the front of the current bucket
 * (this prevents underflow).
 * @param   element  Element to add to the adjacency list.
 * @param   cost     Sort cost for this element.
 */
template<class T>
void AdjacencyList<T>::Add(T* element, const unsigned int cost) {
  if (cost < currentcost_) {
    currentbucket_->push_front(element);
  } else {
    Bucket(cost)->push_back(element);
  }
}

/**
 * The specified element now has a smaller cost.  Reorders it in the sorted list.
 * @param   element  Directed link to reorder in the adjacency list
 * @param   previouscost Previous cost.
 */
template<class T>
void AdjacencyList<T>::DecreaseCost(T* element,
                                    const unsigned int previouscost) {
  // Get the bucket the element currently is in. Protect against
  // previous cost less than current cost
  typename std::list<T*>::iterator previousbucket =
      (previouscost < currentcost_) ?
          Bucket(currentcost_) : Bucket(previouscost);

  // If less than current cost add to the front of the current bucket
  unsigned int cost = element->SortCost();
  if (cost < currentcost_) {
    // Remove the element from the old bucket and push it on the
    // front of the current (so it is the next link processed)
    previousbucket->remove(element);
    currentbucket_->push_front(element);
    return;
  }

  // If the old cost and the new cost are in the same bucket just return
  std::list<T*>& newBucket = Bucket(cost);
  if (previousbucket == newBucket)
    return;

  // Remove the element from the old bucket and add to the new bucket
  previousbucket->remove(element);
  newBucket->push_back(element);
}

/**
 * Removes the lowest cost element from the sorted list.
 * @return  Pointer to the element with lowest cost.
 */
template<class T>
T* AdjacencyList<T>::Remove() {
  // If the current bucket is not empty return the element off the front
  if (!currentbucket_->empty()) {
    return currentbucket_->pop_front();
  }

  // If current bucket is empty increment until a non-empty low-level
  // bucket is found. If we get to the end of the regular buckets
  // we move elements from the overflow buckets into the regular buckets.
  for (; currentbucket_ != buckets_.end(); currentbucket_++) {
    // If the current bucket is not empty get the first element off the list
    if (!currentbucket_->empty())
      return currentbucket_->pop_front();

    currentcost_ += bucketsize_;
  }

  // If no elements are in the overflow buckets, there are no more entries
  if (overflowbucket_.empty())
    return 0;

  // No elements in the low level buckets.  Move elements from the
  // overflow bucket to the low level buckets. Then find smallest bucket
  // that is not empty
  EmptyOverflow();
  for (currentbucket_ = buckets_.begin(); currentbucket_ != buckets_.end();
      currentbucket_++) {
    // If the current bucket is not empty get the first element off the list
    if (!currentbucket_->empty())
      return currentbucket_->pop_front();

    currentcost_ += bucketsize_;
  }
  return nullptr;
}

// Returns the bucket given the cost
template<class T>
typename std::list<T*>::iterator AdjacencyList<T>::Bucket(
    const unsigned int cost) {
  return
      (cost < maxcost_) ?
          &buckets_[((cost - mincost_) / bucketsize_)] : &overflowbucket_;
}

// Empties the overflow bucket by placing the elements into the
// low level buckets.
template<class T>
void AdjacencyList<T>::EmptyOverflow() {
  bool found = false;
  unsigned int cost;
  T* element;
  std::vector<T*> tmp;
  while (!found && !overflowbucket_.empty()) {
    // Adjust cost range
    mincost_ += bucketrange_;
    maxcost_ += bucketrange_;
    currentcost_ = mincost_;

    tmp.clear();
    while (!overflowbucket_.empty()) {
      element = overflowbucket_.pop_front();
      cost = element->SortCost();
      if (cost < maxcost_) {
        buckets_[((cost - mincost_) / bucketsize_)].push_back(element);
        found = true;
      } else {
        tmp.push_back(element);
      }
    }

    // Clear overflow and add any elements that lie outside the new range
    overflowbucket_.clear();
    if (tmp.size() > 0) {
      for (auto element : tmp) {
        overflowbucket_.push_back(*element);
      }
    }
  }
}

}
}

