#include "thor/worker.h"

#include "tyr/serializers.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace thor {

std::string thor_worker_t::isochrones(Api& request) {
  parse_locations(request);
  auto costing = parse_costing(request);
  auto& options = *request.mutable_options();

  std::vector<float> contours_time, contours_distance;
  // Colors will be duplicated, but times and distances might have the same value
  std::unordered_map<const IsoMetrics, std::unordered_map<float, std::string>> colors;

  for (const auto& contour : options.contours()) {
    contours_time.push_back(contour.time());
    colors[IsoMetrics::kTime][contour.time()] = contour.color();
    contours_distance.push_back(contour.distance());
    colors[IsoMetrics::kDistance][contour.distance()] = contour.color();
  }
  // sort the contour vectors in descending order
  std::sort(contours_time.begin(), contours_time.end(), std::greater<float>());
  std::sort(contours_distance.begin(), contours_distance.end(), std::greater<float>());
  /*
  for (float c : contours_time) {
    printf("Contour time: %f\n", c);
  }
  for (float c : contours_distance) {
    printf("Contour distance %f\n", c);
  }
  */
  // If generalize is not provided then an optimal factor is computed
  // (based on the isotile grid size).
  if (!options.has_generalize()) {
    options.set_generalize(kOptimalGeneralization);
  }

  // get the raster
  // Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
  // Cost (including penalties) is used when adding to the adjacency list but the elapsed
  // time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
  // where there has been a higher cost that might still be marked in the isochrone
  auto grid =
      (costing == "multimodal" || costing == "transit")
          ? isochrone_gen.ComputeMultiModal(*options.mutable_locations(),
                                            contours_time.back() + 10.0f,
                                            contours_distance.back() + 10.0f, *reader, mode_costing,
                                            mode)
          : isochrone_gen.Compute(*options.mutable_locations(), contours_time.back() + 10.0f,
                                  contours_distance.back() + 10.0f, *reader, mode_costing, mode);

  // hold isochrones and isodistances in one map
  std::map<const IsoMetrics, const tyr::contours_t> isolines;

  // turn it into geojson
  isolines.emplace(IsoMetrics::kTime,
                   grid->GenerateContours(contours_time,
                                          [](const time_distance_t& td) { return td.sec; },
                                          options.polygons(), options.denoise(),
                                          options.generalize()));
  isolines.emplace(IsoMetrics::kDistance,
                   grid->GenerateContours(contours_distance,
                                          [](const time_distance_t& td) { return td.sec; },
                                          options.polygons(), options.denoise(),
                                          options.generalize()));

  return tyr::serializeIsochrones(request, isolines, options.polygons(), colors,
                                  options.show_locations());
}

} // namespace thor
} // namespace valhalla
