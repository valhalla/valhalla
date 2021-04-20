#include "baldr/datetime.h"
#include "baldr/rapidjson_utils.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/logging.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace {
midgard::PointLL to_ll(const valhalla::Location& l) {
  return midgard::PointLL{l.ll().lng(), l.ll().lat()};
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                    float matrix_max_distance) {
  // see if any locations pairs are unreachable or too far apart
  for (auto source = locations.begin(); source != locations.end() - 1; ++source) {
    for (auto target = source + 1; target != locations.end(); ++target) {
      // check if distance between latlngs exceed max distance limit
      auto path_distance = to_ll(*source).Distance(to_ll(*target));
      if (path_distance > matrix_max_distance) {
        throw valhalla_exception_t{154};
      };
    }
  }
}
} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_chinese_postman(Api& request) {
  std::cout << "Calling init_chinese_postman" << std::endl;
  auto& options = *request.mutable_options();

  // strip off unused information
  parse_locations(options.mutable_locations());
  if (options.locations_size() < 1) {
    throw valhalla_exception_t{120};
  };
  for (auto& l : *options.mutable_locations()) {
    l.clear_heading();
  }

  parse_costing(request);
}
void loki_worker_t::chinese_postman(Api& request) {
  std::cout << "Calling loki_worker_t::chinese_postman" << std::endl;
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request, "loki_worker_t::chinese_postman");

  init_chinese_postman(request);
  auto& options = *request.mutable_options();

}

} // namespace loki
} // namespace valhalla
