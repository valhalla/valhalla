#include "midgard/util.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

std::string thor_worker_t::isochrones(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  auto& options = *request.mutable_options();
  adjust_scores(options);
  auto costing = parse_costing(request);

  // name of the metric (time/distance, value, color)
  std::vector<GriddedData<2>::contour_interval_t> contours;
  for (const auto& contour : options.contours()) {
    if (contour.has_time_case()) {
      contours.emplace_back(0, contour.time(), "time", contour.color());
    }
    if (contour.has_distance_case()) {
      contours.emplace_back(1, contour.distance(), "distance", contour.color());
    }
  }

  // If no generalization is requested an optimal factor is computed (based on the isotile grid size).
  if (!options.has_generalize_case()) {
    options.set_generalize(kOptimalGeneralization);
  }

  // get the raster
  bool reverse = options.reverse() || options.date_time_type() == valhalla::Options::arrive_by;
  auto expansion_type = costing == "multimodal" || costing == "transit"
                            ? ExpansionType::multimodal
                            : (reverse ? ExpansionType::reverse : ExpansionType::forward);
  auto grid = isochrone_gen.Expand(expansion_type, request, *reader, mode_costing, mode);

  // e.g. in case of /expansion request
  if (options.action() == Options_Action_expansion)
    return "";

  // we have parallel vectors of contour properties and the actual geojson features
  // this method sorts the contour specifications by metric (time or distance) and then by value
  // with the largest values coming first. eg (60min, 30min, 10min, 40km, 10km)
  auto isolines =
      grid->GenerateContours(contours, options.polygons(), options.denoise(), options.generalize());

  // make the final json
  std::string ret = tyr::serializeIsochrones(request, contours, isolines, options.polygons(),
                                             options.show_locations());

  return ret;
}

} // namespace thor
} // namespace valhalla
