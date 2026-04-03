#ifndef VALHALLA_MIDGARD_CONST_MAP_H_
#define VALHALLA_MIDGARD_CONST_MAP_H_

#include <algorithm>
#include <array>
#include <stdexcept>

namespace valhalla {
namespace midgard {

// Compile-time sorted flat map. Sorts entries at compile time via consteval constructor,
// uses binary search for O(log N) lookup at runtime. Zero static initialization cost.
template <size_t Size, class Key, class Value>
class ConstFlatMap : public std::array<std::pair<Key, Value>, Size> {
public:
  consteval explicit ConstFlatMap(const std::array<std::pair<Key, Value>, Size>& vals) noexcept {
    std::copy(vals.begin(), vals.end(), this->begin());
    std::sort(this->begin(), this->end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  consteval explicit ConstFlatMap(const std::pair<Key, Value> (&vals)[Size]) noexcept {
    std::copy_n(vals, Size, this->begin());
    std::sort(this->begin(), this->end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  [[nodiscard]] constexpr auto find(const Key& key) const {
    auto it = std::lower_bound(this->begin(), this->end(), key,
                               [](const auto& a, const auto& b) { return a.first < b; });
    if (it == this->end() || it->first != key)
      return this->end();
    return it;
  }

  [[nodiscard]] constexpr const Value& at(const Key& key) const {
    auto it = find(key);
    if (it == this->end())
      throw std::out_of_range("ConstFlatMap::at");
    return it->second;
  }
};

// Deduction guides
template <class Key, class Value, size_t Size>
ConstFlatMap(const std::array<std::pair<Key, Value>, Size>&) -> ConstFlatMap<Size, Key, Value>;

template <class Key, class Value, size_t Size>
ConstFlatMap(const std::pair<Key, Value> (&)[Size]) -> ConstFlatMap<Size, Key, Value>;

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MIDGARD_CONST_MAP_H_
