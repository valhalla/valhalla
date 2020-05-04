#pragma once
#include <algorithm>
#include <cstring>
#include <utility>
#include <vector>

namespace valhalla {
namespace midgard {

// Simple implemtation of a map-like structure that stores data in a flat list.
// To be used when values are small and the list is very short.
// Works when all your data will fit in L1 cache and it's faster to just loop
// over everything than it is to fetch fragmented records from main memory.
template <typename KeyT, typename ValueT>
struct SmallFlatMap : public std::vector<std::pair<KeyT, ValueT>> {

  using typename std::vector<std::pair<KeyT, ValueT>>::iterator;
  using std::vector<std::pair<KeyT, ValueT>>::vector;
  using std::vector<std::pair<KeyT, ValueT>>::begin;
  using std::vector<std::pair<KeyT, ValueT>>::end;
  using std::vector<std::pair<KeyT, ValueT>>::emplace_back;
  using std::vector<std::pair<KeyT, ValueT>>::back;

  iterator find(const KeyT& key) {
    return std::find_if(begin(), end(), [&key](const auto& pair) { return pair.first == key; });
  }

  ValueT& operator[](const KeyT& key) {
    auto result = find(key);
    if (result == end()) {
      emplace_back(key, ValueT());
      return back().second;
    }
    return result->second;
  }
};
} // namespace mjolnir
} // namespace valhalla