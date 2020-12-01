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

  // std::vector<time_distance_t> time_distances;
  std::vector<float> contours_time, contours_distance;
  std::unordered_map<float, std::string> colors;

  for (const auto& contour : options.contours()) {
    // time_distances.push_back({contour.time() * 60, contour.distance()});
    contours_time.push_back(contour.time());
    contours_distance.push_back(contour.distance());
    colors[contour.time()] = contour.color();
    // colors[contour.distance()] = contour.color();
  }

  // sort the contour vectors
  std::sort(contours_time.begin(), contours_time.end());
  std::sort(contours_distance.begin(), contours_distance.end());
  for (float c : contours_time) {
    printf("Contour time: %f\n", c);
  }
  for (float c : contours_distance) {
    printf("Contour distance %f\n", c);
  }

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

  // turn it into geojson
  auto isolines =
      grid->GenerateContours(contours_time, [](const time_distance_t& td) { return td.sec; },
                             options.polygons(), options.denoise(), options.generalize());
  auto isolines_dist =
      grid->GenerateContours(contours_distance, [](const time_distance_t& td) { return td.dist; },
                             options.polygons(), options.denoise(), options.generalize());

  return tyr::serializeIsochrones(request, isolines_dist, options.polygons(), colors,
                                  options.show_locations());
}

} // namespace thor
} // namespace valhalla
