#include "loki/node_search.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;

namespace {
} // anonymous namespace

namespace valhalla {
namespace loki {

std::vector<baldr::GraphId>
nodes_in_bbox(const midgard::AABB2<midgard::PointLL> &bbox, baldr::GraphReader& reader) {
  std::vector<baldr::GraphId> nodes;
  return nodes;
}

}
}
