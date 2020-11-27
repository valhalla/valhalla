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

  // TODO: we have to check what type of contour it was time or distance and push to different vectors
  // of each
  std::vector<float> contours, dist_contours;
  std::unordered_map<float, std::string> colors;
  for (const auto& contour : options.contours()) {
    // if(contour.has_time())
    contours.push_back(contour.time());
    /*else
      dist_contours.push_back(contour.dist());*/
    colors[contours.back()] = contour.color();
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
  auto grid = (costing == "multimodal" || costing == "transit")
                  ? isochrone_gen.ComputeMultiModal(*options.mutable_locations(),
                                                    contours.back() + 10, *reader, mode_costing, mode)
                  : isochrone_gen.Compute(*options.mutable_locations(), contours.back() + 10, *reader,
                                          mode_costing, mode);

  // turn it into geojson
  auto isolines = grid->GenerateContours(contours, [](const float& secs) { return secs; },
                                         options.polygons(), options.denoise(), options.generalize());

  // TODO: if they had any distance contours call GenerateContours again but with the distance
  // contours
  /*auto dist_isolines = grid->GenerateContours(dist_contours, [](const time_distance_t& v) { return
     v.dist; }, options.polygons(), options.denoise(), options.generalize());*/
  return tyr::serializeIsochrones(request, isolines, options.polygons(), colors,
                                  options.show_locations());
}

} // namespace thor
} // namespace valhalla
