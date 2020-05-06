#pragma once
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/tripcommon.pb.h>

namespace valhalla {
namespace loki {
namespace util {
inline midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

} // namespace util
} // namespace loki
} // namespace valhalla