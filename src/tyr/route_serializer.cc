#include <cstdint>
#include <functional>
#include <sstream>
#include <vector>

#include "midgard/encoded.h"
#include "midgard/util.h"
#include "odin/util.h"
#include "route_serializer_osrm.cc"
#include "route_serializer_valhalla.cc"
#include "tyr/serializers.h"

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/trippath.pb.h>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::odin;
using namespace valhalla::tyr;
using namespace std;

namespace {

/**
 * Returns GPX formatted route responses given the legs of the route
 * @param  legs  The legs of the route
 * @return the gpx string
 */
std::string pathToGPX(const std::list<odin::TripPath>& legs) {
  // start the gpx, we'll use 6 digits of precision
  std::stringstream gpx;
  gpx << std::setprecision(6) << std::fixed;
  gpx << R"(<?xml version="1.0" encoding="UTF-8" standalone="no"?><gpx version="1.1" creator="libvalhalla"><metadata/>)";

  // for each leg
  for (const auto& leg : legs) {
    // decode the shape for this leg
    auto wpts = midgard::decode<std::vector<PointLL>>(leg.shape());

    // throw the shape points in as way points
    // TODO: add time to each, need transition time at nodes
    for (const auto& wpt : wpts) {
      gpx << R"(<wpt lon=")" << wpt.first << R"(" lat=")" << wpt.second << R"("></wpt>)";
    }

    // throw the intersections in as route points
    // TODO: add time to each, need transition time at nodes
    gpx << "<rte>";
    uint64_t last_id = -1;
    for (const auto& node : leg.node()) {
      // if this isnt the last node we want the begin shape index of the edge
      size_t shape_idx = wpts.size() - 1;
      if (node.has_edge()) {
        last_id = node.edge().way_id();
        shape_idx = node.edge().begin_shape_index();
      }

      // output this intersection (note that begin and end points may not be intersections)
      const auto& rtept = wpts[shape_idx];
      gpx << R"(<rtept lon=")" << rtept.first << R"(" lat=")" << rtept.second << R"(">)"
          << "<name>" << last_id << "</name></rtept>";
    }
    gpx << "</rte>";
  }

  // give it back as a string
  gpx << "</gpx>";
  return gpx.str();
}

} // namespace

namespace valhalla {
namespace tyr {

std::string serializeDirections(const valhalla_request_t& request,
                                std::list<TripPath>& path_legs,
                                const std::list<TripDirections>& directions_legs) {
  // serialize them
  switch (request.options.format()) {
    case DirectionsOptions_Format_osrm:
      return osrm_serializers::serialize(request.options, path_legs, directions_legs);
    case DirectionsOptions_Format_gpx:
      return pathToGPX(path_legs);
    case DirectionsOptions_Format_json:
      return valhalla_serializers::serialize(request.options, directions_legs);
    default:
      throw;
  }
}

} // namespace tyr
} // namespace valhalla
