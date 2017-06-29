#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <list>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <iostream>

#include <valhalla/worker.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/route.pb.h>
#include <valhalla/tyr/actor.h>

namespace valhalla {
  namespace tyr {

    baldr::json::MapPtr serializeDirections(ACTION_TYPE action, const boost::property_tree::ptree& request, const std::list<odin::TripDirections>& directions_legs);

    /**
     * Transfers the JSON route information returned from a route request into
     * the Route proto object passed in by reference.
     * @param json_route   The route information to be parsed as JSON
     * @param proto_route  The protobuf object that will hold the information
     *                     from the JSON string
     */
    void jsonToProtoRoute (const std::string& json_route, Route& proto_route);

  }
}


#endif //__VALHALLA_TYR_SERVICE_H__
