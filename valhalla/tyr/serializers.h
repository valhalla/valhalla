#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <list>
#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>

#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/worker.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/route.pb.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/thor/attributes_controller.h>
#include <valhalla/thor/match_result.h>
#include <valhalla/tyr/actor.h>


namespace valhalla {
  namespace tyr {

    /**
     * Turn path and directions into a route that one can follow
     */
    std::string serializeDirections(const valhalla_request_t& request,
        const std::list<odin::TripPath>& path_legs,
        const std::list<odin::TripDirections>& directions_legs);

    /**
     * Turn a time distance matrix into json that one can look up location pair results from
     */
    std::string serializeMatrix(const valhalla_request_t& request,
        const std::vector<thor::TimeDistance>& time_distances, double distance_scale);

    /**
     * Turn grid data contours into geojson
     *
     * @param grid_contours    the contours generated from the grid
     * @param colors           the #ABC123 hex string color used in geojson fill color
     */
    template <class coord_t>
    std::string serializeIsochrones(const valhalla_request_t& request, const typename midgard::GriddedData<coord_t>::contours_t& grid_contours,
        bool polygons = true, const std::unordered_map<float, std::string>& colors = {}, bool show_locations = false);

    /**
     * Turn heights and ranges into a height response
     *
     * @param request  The original request
     * @param heights  The actual height at each shape point
     * @param ranges   The distances between each point. If this is empty no ranges are serialized
     */
    std::string serializeHeight(const valhalla_request_t& request,
        const std::vector<double>& heights, std::vector<float> ranges = {});

    /**
     * Turn some correlated points on the graph into info about those locations
     *
     * @param request      The original request
     * @param locations    The input locations
     * @param projections  The correlated locations
     * @param reader       A graph reader to get at each correlated points info
     */
    std::string serializeLocate(const valhalla_request_t& request, const std::vector<baldr::Location>& locations,
        const std::unordered_map<baldr::Location, baldr::PathLocation>& projections, baldr::GraphReader& reader);

    /**
     * Turn a list of locations into a list of locations with a bool that says whether transit tiles are near by
     *
     * @param request    The original request
     * @param locations  The input locations
     * @param found      Which locations had transit
     */
    std::string serializeTransitAvailable(const valhalla_request_t& request, const std::vector<baldr::Location>& locations,
        const std::unordered_set<baldr::Location>& found);

    /**
     * Turn trip paths and the match results of each into attributes based on the filter specified
     *
     * @param request     The original request
     * @param controller  The filter for what attributes should be serialized
     * @param results     The vector of trip paths and match results for each match found
     */
    std::string serializeTraceAttributes(const valhalla_request_t& request, const thor::AttributesController& controller,
        std::vector<std::tuple<float, float, std::vector<thor::MatchResult>, odin::TripPath>>& results);

    /**
     * Transfers the JSON route information returned from a route request into
     * the Route proto object passed in by reference.
     * @param json_route   The route information to be parsed as JSON
     * @param proto_route  The protobuf object that will hold the information
     *                     from the JSON string
     */
    void jsonToProtoRoute(const std::string& json_route, Route& proto_route);
  }
}

namespace osrm {

  /*
   *
   */
  valhalla::baldr::json::MapPtr waypoint(const valhalla::odin::Location& location, bool tracepoint = false,
      const bool optimized = false, const uint32_t waypoint_index = 0);

  /*
   *
   */
  valhalla::baldr::json::ArrayPtr waypoints(const google::protobuf::RepeatedPtrField<valhalla::odin::Location>& locations, bool tracepoints = false);

}


#endif //__VALHALLA_TYR_SERVICE_H__
