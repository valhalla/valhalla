#ifndef VALHALLA_THOR_ADJACENCYLIST_H_
#define VALHALLA_THOR_ADJACENCYLIST_H_

#include <vector>
#include <list>

namespace valhalla {
namespace thor {

/**
 * Adjacency list support. Uses a bucket sort implementation for performance.
 * @author  David W. Nesbitt
 */
template<class T>
class AdjacencyList {
 public:
  /**
   * Constructor.
   */
  AdjacencyList();

  /**
   * Constructor given a minimum cost. May be needed if a bucket sort
   * implementation is used (to save memory).
   * @param mincost    Minimum cost (based on A* heuristic) in a route. Used
   *                   to create the initial range for bucket sorting.
   * @param range      Cost range for double buckets.
   * @param bucketsize Bucket size (range of costs within same bucket).
   */
  AdjacencyList(const unsigned int mincost, const unsigned int range,
                const unsigned int bucketsize);

  /**
   * Destructor.
   */
  virtual ~AdjacencyList();

  /**
   * Adds an element to the sorted list. Adds the element to the appropriate
   * bucket given its cost. If the cost is greater than maxcost_ the element
   * is placed in the overflow bucket. If the cost is < the current bucket
   * cost then the element is placed at the front of the current bucket
   * (this prevents underflow).
   * @param   element  Element to add to the adjacency list.
   * @param   cost     Sort cost for this element.
   */
  void Add(T* element, const unsigned int cost);

  /**
   * The specified element now has a smaller cost.  Reorders it in the sorted list.
   * @param   element  Directed link to reorder in the adjacency list
   * @param   previouscost Previous cost.
   */
  void DecreaseCost(T* element, const unsigned int previouscost);

  /**
   * Removes the lowest cost element from the sorted list.
   * @return  Pointer to the element with lowest cost.
   */
  T* Remove();

 private:
  unsigned int bucketrange_;  // Total range of costs in lower level buckets
  unsigned int bucketcount_;  // Number of buckets
  unsigned int bucketsize_;   // Bucket size (range of costs in same bucket)
  unsigned int mincost_;      // Minimum cost within the low level buckets
  unsigned int maxcost_;      // Above this goes into overflow bucket.
  unsigned int currentcost_;

  // Low level buckets
  std::vector<std::list<T*> > buckets_;

  typename std::list<T*>::iterator it;

  // Current bucket in the list
  typename std::vector<std::list<T*> >::iterator currentbucket_;

  // Overflow bucket
  std::list<T*> overflowbucket_;

  // Returns the bucket given the cost
  typename std::list<T*>::iterator Bucket(const unsigned int cost);

  // Empties the overflow bucket by placing the elements into the
  // low level buckets.
  void EmptyOverflow();
};

}
}

#endif  // VALHALLA_THOR_ADJACENCYLIST_H_

