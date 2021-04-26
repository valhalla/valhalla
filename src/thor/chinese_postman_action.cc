#include "midgard/util.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

void thor_worker_t::chinese_postman(Api& request) {
  std::cout << "thor_worker_t::chinese_postman" << std::endl;
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "thor_worker_t::isochrones");

  parse_locations(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  auto* co = options.mutable_costing_options(options.costing());
  google::protobuf::RepeatedPtrField<::valhalla::ChinesePostmanEdge> edges = co->chinese_edges();

  // User specified edges to route with percent along (for avoiding PathEdges of locations)
  std::unordered_map<baldr::GraphId, float> chinese_edges_;
  int i = 0;

  // Add chinese edges to internal set
  for (auto& edge : co->chinese_edges()) {
    chinese_edges_.insert({GraphId(edge.id()), edge.percent_along()});
  }
}

} // namespace thor
} // namespace valhalla
