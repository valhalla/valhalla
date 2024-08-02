#ifndef VALHALLA_MEILI_STATE_H_
#define VALHALLA_MEILI_STATE_H_

#include <cstdint>
#include <limits>

namespace valhalla {
namespace meili {

constexpr uint32_t kInvalidTime = std::numeric_limits<uint32_t>::max();

union StateId {
public:
  using Time = uint32_t;
  using Id = uint32_t;

  StateId() = default;
  explicit StateId(Time time, Id id) : fields_({time, id}) {
  }
  Time time() const {
    return fields_.time;
  }
  Id id() const {
    return fields_.id;
  }
  bool IsValid() const {
    return fields_.time != kInvalidTime;
  }
  bool operator==(const StateId& rhs) const {
    return value_ == rhs.value_;
  }
  bool operator!=(const StateId& rhs) const {
    return value_ != rhs.value_;
  }
  uint64_t value() const {
    return value_;
  }

private:
  struct {
    uint64_t time : 32;
    uint64_t id : 32;
  } fields_{kInvalidTime, 0};

  uint64_t value_;
};
} // namespace meili
} // namespace valhalla

// Extend the standard namespace to know how to hash StateIds
namespace std {
template <> struct hash<valhalla::meili::StateId> {
  std::size_t operator()(const valhalla::meili::StateId& stateid) const {
    return static_cast<size_t>(stateid.value());
  }
};

template <> struct hash<std::pair<valhalla::meili::StateId, valhalla::meili::StateId>> {
  std::size_t
  operator()(const std::pair<valhalla::meili::StateId, valhalla::meili::StateId>& couple) const {
    auto seed = static_cast<size_t>(couple.first.value());
    return static_cast<size_t>(couple.second.value()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
};
} // namespace std

#endif // VALHALLA_MEILI_STATE_H_
