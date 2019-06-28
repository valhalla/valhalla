#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <iostream>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace tyr {

/**
 * Turn path and directions into a route that one can follow
 */
std::string serializeDirections(Api& request);

/**
 * Turn a time distance matrix into json that one can look up location pair results from
 */
std::string serializeMatrix(const Api& request,
                            const std::vector<thor::TimeDistance>& time_distances,
                            double distance_scale);

/**
 * Turn grid data contours into geojson
 *
 * @param grid_contours    the contours generated from the grid
 * @param colors           the #ABC123 hex string color used in geojson fill color
 */
template <class coord_t>
std::string
serializeIsochrones(const Api& request,
                    const typename midgard::GriddedData<coord_t>::contours_t& grid_contours,
                    bool polygons = true,
                    const std::unordered_map<float, std::string>& colors = {},
                    bool show_locations = false);

/**
 * Turn heights and ranges into a height response
 *
 * @param request  The original request
 * @param heights  The actual height at each shape point
 * @param ranges   The distances between each point. If this is empty no ranges are serialized
 */
std::string serializeHeight(const Api& request,
                            const std::vector<double>& heights,
                            const std::vector<float>& ranges = {});

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
    const Api& request,
    const thor::AttributesController& controller,
    std::vector<std::tuple<float, float, std::vector<thor::MatchResult>>>& results);

} // namespace tyr
} // namespace valhalla

namespace osrm {

/*
 * Serialize a location into a osrm waypoint
 * http://project-osrm.org/docs/v5.5.1/api/#waypoint-object
 */
valhalla::baldr::json::MapPtr waypoint(const valhalla::Location& location,
                                       bool is_tracepoint = false,
                                       const bool is_optimized = false,
                                       const int64_t waypoint_index = -1);

/*
 * Serialize locations into osrm waypoints
 */
valhalla::baldr::json::ArrayPtr
waypoints(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
          bool tracepoints = false);
valhalla::baldr::json::ArrayPtr waypoints(const valhalla::Trip& locations);

} // namespace osrm

#endif //__VALHALLA_TYR_SERVICE_H__
