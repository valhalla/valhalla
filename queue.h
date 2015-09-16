// -*- mode: c++ -*-

#include <unordered_map>

#include <boost/heap/fibonacci_heap.hpp>


template <typename T>
class LabelInterface
{
 public:
  using id_type = T;
  virtual T id() const = 0;
  virtual double sortcost() const = 0;
};

template <typename T>
bool operator>(const LabelInterface<T>& lhs, const LabelInterface<T>& rhs) {
  return lhs.sortcost() > rhs.sortcost();
}

template <typename T>
bool operator<(const LabelInterface<T>& lhs, const LabelInterface<T>& rhs) {
  return lhs.sortcost() < rhs.sortcost();
}

template <typename T>
bool operator==(const LabelInterface<T>& lhs, const LabelInterface<T>& rhs) {
  return lhs.sortcost() == rhs.sortcost();
}


// Shortest-path-specific priority queue
// (A friendly wrapper of boost::heap::fibonacci_heap)
template <typename T>  // T must be type of LabelInterface
class SPQueue
{
 public:
  // Min heap
  using Heap = boost::heap::fibonacci_heap<T, boost::heap::compare<std::greater<T>>>;

  ~SPQueue() {
    clear();
  }

  void push(const T& label) {
    typename T::id_type id = label.id();
    auto handler_itr = handlers_.find(id);
    if (handler_itr == handlers_.end()) {
      handlers_[id] = heap_.push(label);
    } else {
      auto handler = handler_itr->second;
      // Update it if it has lower cost
      if (label < *handler) {
        heap_.update(handler, label);
      }
    }
  }

  void pop() {
    auto top = heap_.top();
    assert(handlers_.find(top.id()) != handlers_.end());
    handlers_.erase(top.id());
    assert(handlers_.find(top.id()) == handlers_.end());
    heap_.pop();
  }

  inline const T& top() const {
    return heap_.top();
  }

  inline bool empty() const {
    assert(heap_.empty() == handlers_.empty());
    return heap_.empty();
  }

  inline void clear() {
    heap_.clear();
    handlers_.clear();
  }

  inline typename Heap::size_type size() const {
    assert(handlers_.size() == heap_.size());
    return heap_.size();
  }

 protected:
  Heap heap_;
  std::unordered_map<typename T::id_type, typename Heap::handle_type> handlers_;
};
