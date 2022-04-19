// -*- mode: c++ -*-
#ifndef MMP_PRIORITY_QUEUE_H_
#define MMP_PRIORITY_QUEUE_H_

#include <unordered_map>

#include <boost/heap/fibonacci_heap.hpp>

// Shortest-path-specific priority queue
// (A friendly wrapper of boost::heap::fibonacci_heap)
template <typename T> class SPQueue {
public:
  // Min heap
  using Heap = boost::heap::fibonacci_heap<T, boost::heap::compare<std::greater<T>>>;

  ~SPQueue() {
    clear();
  }

  void push(const T& label) {
    const auto& id = label.id();
    const auto it = handlers_.find(id);
    if (it == handlers_.end()) {
      handlers_.emplace(id, heap_.push(label));
    } else if (label < *(it->second)) {
      heap_.update(it->second, label);
    }
  }

  void pop() {
    const auto& top = heap_.top();
    handlers_.erase(top.id());
    heap_.pop();
  }

  const T& top() const {
    return heap_.top();
  }

  bool empty() const {
    return heap_.empty();
  }

  void clear() {
    heap_.clear();
    handlers_.clear();
  }

  typename Heap::size_type size() const {
    return heap_.size();
  }

protected:
  Heap heap_;

  std::unordered_map<typename T::id_type, typename Heap::handle_type> handlers_;
};

#endif // MMP_PRIORITY_QUEUE_H_
