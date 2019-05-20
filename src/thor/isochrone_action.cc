#include "thor/worker.h"

#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

std::string thor_worker_t::isochrones(valhalla_request_t& request) {
  parse_locations(request);
  auto costing = parse_costing(request);

  std::vector<float> contours;
  std::unordered_map<float, std::string> colors;
  for (const auto& contour : request.options.contours()) {
    contours.push_back(contour.time());
    colors[contours.back()] = contour.color();
  }

  // If generalize is not provided then an optimal factor is computed
  // (based on the isotile grid size).
  if (!request.options.has_generalize()) {
    request.options.set_generalize(kOptimalGeneralization);
  }

  // get the raster
  // Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
  // Cost (including penalties) is used when adding to the adjacency list but the elapsed
  // time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
  // where there has been a higher cost that might still be marked in the isochrone
  auto grid = (costing == "multimodal" || costing == "transit")
                  ? isochrone_gen.ComputeMultiModal(*request.options.mutable_locations(),
                                                    contours.back() + 10, *reader, mode_costing, mode)
                  : isochrone_gen.Compute(*request.options.mutable_locations(), contours.back() + 10,
                                          *reader, mode_costing, mode);

  // turn it into geojson
  auto isolines = grid->GenerateContours(contours, request.options.polygons(),
                                         request.options.denoise(), request.options.generalize());

  return tyr::serializeIsochrones<PointLL>(request, isolines, request.options.polygons(), colors,
                                           request.options.show_locations());
}

} // namespace thor
} // namespace valhalla
