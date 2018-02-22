#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <list>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <iostream>
#include <vector>

#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/gridded_data.h>
#include <valhalla/worker.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/route.pb.h>
#include <valhalla/thor/costmatrix.h>
#include <valhalla/tyr/actor.h>

namespace valhalla {
  namespace tyr {

    /**
     * Turn path and directions into a route that one can follow
     *
     * @param
     */
    std::string serializeDirections(const odin::DirectionsOptions& directions_options,
        const std::list<odin::TripPath>& path_legs,
        const std::list<odin::TripDirections>& directions_legs);

    /**
     * Turn a time distance matrix into json that one can look up location pair results from
     *
     * @param
     */
    std::string serializeMatrix(const odin::DirectionsOptions& options, const std::vector<baldr::PathLocation>& sources,
        const std::vector<baldr::PathLocation>& targets, const std::vector<thor::TimeDistance>& time_distances, double distance_scale);

    /**
     * Turn grid data contours into geojson
     *
     * @param grid_contours    the contours generated from the grid
     * @param colors           the #ABC123 hex string color used in geojson fill color
     */
    template <class coord_t>
    std::string serializeIsochrones(const odin::DirectionsOptions& options, const typename midgard::GriddedData<coord_t>::contours_t& grid_contours,
        bool polygons = true, const std::unordered_map<float, std::string>& colors = {}, const std::vector<baldr::PathLocation>& locations = {});

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


#endif //__VALHALLA_TYR_SERVICE_H__
