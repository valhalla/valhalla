#include "loki/search.h"
#include "loki/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

void loki_worker_t::init_locate(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations(), request);
  if (request.options().locations_size() < 1)
    throw valhalla_exception_t{120};

  parse_costing(request, true);
}

std::string loki_worker_t::locate(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // correlate the various locations to the underlying graph
  init_locate(request);
  auto* options = request.mutable_options();
  auto* locations = options->mutable_locations();
  const auto& costing_name = Costing_Enum_Name(options->costing_type());
  if (costing_name == "auto_pedestrian") {
    if (locations->size() > 2) {
      throw valhalla_exception_t{150, "for auto_pedestrian: " + std::to_string(locations->size())};
    }
    google::protobuf::RepeatedPtrField<Location> start_loc(locations->begin(),
                                                           locations->begin() + 1);
    search_.search(start_loc, mode_costing[static_cast<size_t>(mode)]);
    google::protobuf::RepeatedPtrField<Location> end_loc(locations->begin() + 1,
                                                         locations->begin() + 2);
    search_.search(end_loc, mode_costing[static_cast<size_t>(mode)]);
    // merge them again
    locations->at(0).CopyFrom(start_loc.at(0));
    locations->at(1).CopyFrom(end_loc.at(0));
  } else {
    search_.search(*locations, mode_costing[static_cast<size_t>(mode)]);
  }
  return tyr::serializeLocate(request, *reader);
}

} // namespace loki
} // namespace valhalla
