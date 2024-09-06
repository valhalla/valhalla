#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <string>
#include <unordered_map>
#include <vector>

#include <valhalla/baldr/attributes_controller.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/meili/match_result.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/proto/api.pb.h>
#include <valhalla/proto_conversions.h>
#include <valhalla/tyr/actor.h>

namespace valhalla {
namespace tyr {

/**
 * Turn path and directions into a route that one can follow
 */
std::string serializeDirections(Api& request);

/**
 * Turn a time distance matrix into json that one can look up location pair results from
 */
std::string serializeMatrix(Api& request);

/**
 * Turn grid data contours into geojson
 *
 * @param grid_contours    the contours generated from the grid
 * @param colors           the #ABC123 hex string color used in geojson fill color
 */
std::string serializeIsochrones(Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                const std::shared_ptr<const midgard::GriddedData<2>>& isogrid);
/**
 * Write GeoJSON from expansion pbf
 */
std::string serializeExpansion(Api& request, const std::string& algo);

/**
 * Turn heights and ranges into a height response
 *
 * @param request  The original request
 * @param heights  The actual height at each shape point
 * @param ranges   The distances between each point. If this is empty no ranges are serialized
 */
std::string serializeHeight(const Api& request,
                            const std::vector<double>& heights,
                            const std::vector<double>& ranges = {});

/**
 * Turn some correlated points on the graph into info about those locations
 *
 * @param request      The original request
 * @param locations    The input locations
 * @param projections  The correlated locations
 * @param reader       A graph reader to get at each correlated points info
 */
std::string
serializeLocate(const Api& request,
                const std::vector<baldr::Location>& locations,
                const std::unordered_map<baldr::Location, baldr::PathLocation>& projections,
                baldr::GraphReader& reader);

/**
 * Turn a list of locations into a list of locations with a bool that says whether transit tiles are
 * near by
 *
 * @param request    The original request
 * @param locations  The input locations
 * @param found      Which locations had transit
 */
std::string serializeTransitAvailable(const Api& request,
                                      const std::vector<baldr::Location>& locations,
                                      const std::unordered_set<baldr::Location>& found);

/**
 * Turn trip paths and the match results of each into attributes based on the filter specified
 *
 * @param request     The original request
 * @param controller  The filter for what attributes should be serialized
 * @param results     The vector of trip paths and match results for each match found
 */
std::string serializeTraceAttributes(
    Api& request,
    const baldr::AttributesController& controller,
    std::vector<std::tuple<float, float, std::vector<meili::MatchResult>>>& results);

/**
 * Turn proto with status information into json
 * @param request  the proto request with status info attached
 * @return json string
 */
std::string serializeStatus(Api& request);

// Return a JSON array of OpenLR 1.5 line location references for each edge of a map matching
// result. For the time being, result is only non-empty for auto costing requests.
void route_references(baldr::json::MapPtr& route_json,
                      const TripRoute& route,
                      const Options& options);

void openlr(const valhalla::Api& api, int route_index, rapidjson::writer_wrapper_t& writer);

/**
 * Turns the pbf into bytes omitting the fields specified in the request options pbf field selector
 * @param request  The protobuf object which will be serialized
 * @return the bytes representing the protobuf object
 */
std::string serializePbf(Api& request);

/**
 * @brief Turns warnings into json
 * @param request The protobuf warnings object
 * @return json string
 */
void serializeWarnings(const valhalla::Api& api, rapidjson::writer_wrapper_t& writer);
baldr::json::ArrayPtr serializeWarnings(const valhalla::Api& api);

/**
 * Turns a line into a GeoJSON LineString geometry.
 *
 * @param shape  The points making up the line.
 * @returns The GeoJSON geometry of the LineString
 */
baldr::json::MapPtr geojson_shape(const std::vector<midgard::PointLL> shape);

// Elevation serialization support

/**
 * Update elevations where consecutive bridges or tunnels occur. This does a linear interpolation of
 * elevation from just prior to first bridge/tunnel to just after the last bridge/tunnel. This is
 * done since elevation is stored for single edges and if a bridge/tunnel spans multiple edges then
 * the elevation can be incorrect.
 * @param elevation Elevation vector.
 * @param bridges   List of bridges/tunnels - the start and end index into the elevation list for
 * each.
 */
inline void update_bridge_elevations(std::vector<float>& elevation,
                                     std::vector<std::pair<uint32_t, uint32_t>>& bridges) {
  for (const auto& bridge : bridges) {
    // Validate elevation indexes. Get elevation before and after the bridge/tunnel and
    // form delta elevation for linear interpolation.
    int32_t n = bridge.second - bridge.first;
    if (n > 0 && bridge.first < elevation.size() && bridge.second < elevation.size()) {
      float start_elevation = bridge.first > 0 ? elevation[bridge.first - 1] : elevation[0];
      float end_elevation =
          bridge.second < elevation.size() - 1 ? elevation[bridge.second + 1] : elevation.back();
      float delta = (end_elevation - start_elevation) / n;
      float elev = start_elevation + delta;
      for (uint32_t i = bridge.first; i <= bridge.second; ++i) {
        elevation[i] = elev;
        elev += delta;
      }
    }
  }
}

/**
 * Convenience method to get elevation along a path from TripLeg. Samples
 * the edge elevation information at the specified interval.
 * @param  path_leg  Trip path for this route leg.
 * @param  interval  Elevation sampling interval (meters) along the path.
 * @param  start_distance  Optional start distance along the path. This is used when
 *                         matched and unmatched sections so that a consistent sampling
 *                         interval is preserved.
 * @return  Returns an array of elevation postings along the path.
 */
inline std::vector<float>
get_elevation(const TripLeg& path_leg, const float interval, const float start_distance = 0.0f) {
  // Store the first elevation if start_distance == 0
  std::vector<float> elevation;
  auto first_elevation = path_leg.node(0).edge().elevation(0);
  if (start_distance == 0.0f) {
    elevation.push_back(first_elevation);
  }

  uint32_t bridge_edge_count = 0;
  uint32_t first_bridge_index;
  float distance = 0.0f;             // Distance from prior elevation posting
  float remaining = -start_distance; // How much of the current edge interval remains
  float prior_elevation = first_elevation;
  std::vector<std::pair<uint32_t, uint32_t>> bridges;
  for (const auto& node : path_leg.node()) {
    // Get the edge on the path, the starting elevation and sampling interval
    auto path_edge = node.edge();
    float edge_interval = path_edge.elevation_sampling_interval();

    // Identify consecutive bridge/tunnel edges and store elevation indexes at start and end.
    if (path_edge.bridge() || path_edge.tunnel()) {
      if (bridge_edge_count == 0) {
        first_bridge_index = elevation.size();
      }
      ++bridge_edge_count;
    } else {
      // Not a bridge/tunnel. Add a bridge elevation pair if more than 1 consecutive bridge/tunnel
      // exists prior to this edge. Make sure elevation size > 0 - may have very short tunnel edges
      // after an unmatched section (no new elevation added).
      if (bridge_edge_count > 1 && elevation.size() > 0) {
        bridges.emplace_back(first_bridge_index, elevation.size() - 1);
      }
      bridge_edge_count = 0;
    }

    // Iterate through the edge elevation (skip the first)
    for (int32_t i = 1; i < path_edge.elevation_size(); ++i) {
      auto elev = path_edge.elevation(i);

      // Update distance from prior elevation posting
      distance = interval - remaining;
      remaining += edge_interval;

      // Store elevation while distance remaining > interval
      while (remaining > interval) {
        // Linear interpolation between prior elevation
        float p1 = distance / edge_interval;
        elevation.push_back(prior_elevation * (1.0f - p1) + elev * p1);
        remaining -= interval;
        distance += interval;
      }

      // Update prior elevation
      prior_elevation = elev;
    }
  }

  // Store the last elevation
  elevation.push_back(prior_elevation);

  // Update elevations along consecutive bridges/tunnels. Update bridges if
  // last edge was a bridge (and edge before was also a bridge).
  if (bridge_edge_count > 1) {
    bridges.emplace_back(first_bridge_index, elevation.size() - 1);
  }
  update_bridge_elevations(elevation, bridges);

  return elevation;
}

} // namespace tyr
} // namespace valhalla

namespace osrm {

/*
 * Serialize a location into a osrm waypoint
 * http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
 */
valhalla::baldr::json::MapPtr
waypoint(const valhalla::Location& location, bool is_tracepoint = false, bool is_optimized = false);

/*
 * Serialize locations into osrm waypoints
 */
valhalla::baldr::json::ArrayPtr
waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
          bool tracepoints = false);
valhalla::baldr::json::ArrayPtr waypoints(const valhalla::Trip& locations);
valhalla::baldr::json::ArrayPtr intermediate_waypoints(const valhalla::TripLeg& leg);

void serializeIncidentProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer,
                                 const valhalla::IncidentsTile::Metadata& incident_metadata,
                                 const int begin_shape_index,
                                 const int end_shape_index,
                                 const std::string& road_class,
                                 const std::string& key_prefix);

} // namespace osrm

#endif //__VALHALLA_TYR_SERVICE_H__
