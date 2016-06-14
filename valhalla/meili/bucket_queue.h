// -*- mode: c++ -*-
#ifndef MMP_BUCKET_QUEUE_H_
#define MMP_BUCKET_QUEUE_H_

#include <vector>
#include <unordered_map>
#include <algorithm>


namespace valhalla {

namespace meili {


template<typename key_t, key_t invalid_key>
class BucketQueue
{
 public:
  using size_type = typename std::vector<std::vector<key_t>>::size_type;

  BucketQueue(size_type count, float size = 1.f)
      : bucket_count_(count),
        bucket_size_(size),
        top_(0),
        costmap_(),
        buckets_()
  {
    if (bucket_size_ <= 0.f) {
      throw std::invalid_argument("expect bucket size to be positive");
    }
    buckets_.reserve(bucket_count_);
  }

  bool add(const key_t& key, float cost)
  {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    const auto idx = bucket_idx(cost);
    if (idx < bucket_count_) {
      const auto it = costmap_.emplace(key, cost);
      if (!it.second) {
        throw std::runtime_error("the key " + std::to_string(key) + " exists in the queue,"
                                 " you probably should call the decrease method");
      }

      if (buckets_.size() <= idx) {
        buckets_.resize(idx + 1);
      }
      buckets_[idx].push_back(key);

      // Update top cursor
      if (idx < top_) {
        top_ = idx;
      }

      return true;
    }

    return false;
  }

  void decrease(const key_t& key, float cost)
  {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    const auto it = costmap_.find(key);
    if (it == costmap_.end()) {
      throw std::runtime_error("the key " + std::to_string(key) + " to decrease doesn't exist in the queue,"
                               " you probably should call the add method");
    }

    if (cost < it->second) {
      const auto old_idx = bucket_idx(it->second);
      const auto idx = bucket_idx(cost);
      if (old_idx < idx) {
        throw std::logic_error("invalid cost: " + std::to_string(cost) + " (old value is " + std::to_string(it->second) + ")");
      }

      // Remove the key from the old bucket
      auto& old_bucket = buckets_[old_idx];
      const auto old_key = std::find(old_bucket.begin(), old_bucket.end(), key);
      if (old_key == old_bucket.end()) {
        throw std::logic_error("the key " + std::to_string(key) + " in the cost map was failed to add to the bukcet");
      }
      old_bucket.erase(old_key);

      // Add the key to the new bucket
      buckets_[idx].push_back(key);
      // Update the cost
      it->second = cost;

      // Update top cursor
      if (idx < top_) {
        top_ = idx;
      }
    } else {
      throw std::runtime_error("the cost " + std::to_string(cost)
                               + " is not less than the cost (" + std::to_string(it->second)
                               + ") associated with the key (" + std::to_string(key)
                               + ") you requested to decrease ");
    }
  }

  float cost(const key_t& key) const
  {
    const auto it = costmap_.find(key);
    return it == costmap_.end()? -1.f : it->second;
  }

  key_t pop()
  {
    if (empty()) {
      return invalid_key;
    }

    const auto key = buckets_[top_].back();
    buckets_[top_].pop_back();
    costmap_.erase(key);
    return key;
  }

  bool empty() const
  {
    while (top_ < buckets_.size() && buckets_[top_].empty()) {
      top_++;
    }
    return top_ >= buckets_.size();
  }

  size_type size() const
  { return costmap_.size(); }

  void clear()
  {
    buckets_.clear();
    costmap_.clear();
    top_ = 0;
  }

 private:

  size_type bucket_count_;

  float bucket_size_;

  mutable size_type top_;

  std::unordered_map<key_t, float> costmap_;

  std::vector<std::vector<key_t>> buckets_;

  size_type bucket_idx(float cost) const
  { return static_cast<size_type>(cost / bucket_size_); }
};


}
}


#endif // MMP_BUCKET_QUEUE_H_
