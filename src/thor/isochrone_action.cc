#include "thor/worker.h"

#include "baldr/json.h"
#include "baldr/geojson.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
  namespace thor {

    json::MapPtr thor_worker_t::isochrones(const boost::property_tree::ptree &request) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      parse_locations(request);
      auto costing = parse_costing(request);

      std::vector<float> contours;
      std::unordered_map<float, std::string> colors;
      for(const auto& contour : request.get_child("contours")) {
        contours.push_back(contour.second.get<float>("time"));
        colors[contours.back()] = contour.second.get<std::string>("color", "");
      }
      auto polygons = request.get<bool>("polygons", false);
      auto denoise = std::max(std::min(request.get<float>("denoise", 1.f), 1.f), 0.f);

      // Get the generalization factor (in meters). If none is provided then
      // an optimal factor is computed (based on the isotile grid size).
      auto generalize = request.get<float>("generalize", kOptimalGeneralization);

      //get the raster
      //Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
      //Cost (including penalties) is used when adding to the adjacency list but the elapsed
      //time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
      //where there has been a higher cost that might still be marked in the isochrone
      auto grid = (costing == "multimodal" || costing == "transit") ?
        isochrone_gen.ComputeMultiModal(correlated, contours.back()+10, reader, mode_costing, mode) :
        isochrone_gen.Compute(correlated, contours.back()+10, reader, mode_costing, mode);

      //turn it into geojson
      auto isolines = grid->GenerateContours(contours, polygons, denoise, generalize);

      auto showLocations = request.get<bool>("show_locations", false);
      auto geojson = (showLocations) ? baldr::json::to_geojson<PointLL>(isolines, polygons, colors, correlated)
                                     : baldr::json::to_geojson<PointLL>(isolines, polygons, colors);

      auto id = request.get_optional<std::string>("id");
      if(id)
        geojson->emplace("id", *id);

      return geojson;
    }

  }
}
