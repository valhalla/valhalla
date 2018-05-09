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
  for (const auto& contour :
       rapidjson::get<rapidjson::Value::ConstArray>(request.document, "/contours")) {
    contours.push_back(rapidjson::get<float>(contour, "/time"));
    colors[contours.back()] = rapidjson::get<std::string>(contour, "/color", "");
  }
  auto polygons = rapidjson::get<bool>(request.document, "/polygons", false);
  auto denoise =
      std::max(std::min(rapidjson::get<float>(request.document, "/denoise", 1.f), 1.f), 0.f);

  // Get the generalization factor (in meters). If none is provided then
  // an optimal factor is computed (based on the isotile grid size).
  auto generalize = rapidjson::get<float>(request.document, "/generalize", kOptimalGeneralization);

  // get the raster
  // Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
  // Cost (including penalties) is used when adding to the adjacency list but the elapsed
  // time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
  // where there has been a higher cost that might still be marked in the isochrone
  auto grid =
      (costing == "multimodal" || costing == "transit")
          ? isochrone_gen.ComputeMultiModal(*request.options.mutable_locations(),
                                            contours.back() + 10, reader, mode_costing, mode)
          : isochrone_gen.Compute(*request.options.mutable_locations(), contours.back() + 10,
                                  reader, mode_costing, mode);

  // turn it into geojson
  auto isolines = grid->GenerateContours(contours, polygons, denoise, generalize);

  auto showLocations = rapidjson::get<bool>(request.document, "/show_locations", false);
  return tyr::serializeIsochrones<PointLL>(request, isolines, polygons, colors, showLocations);
}

} // namespace thor
} // namespace valhalla
