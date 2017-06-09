#ifndef VALHALLA_TYR_UTIL_H
#define VALHALLA_TYR_UTIL_H

#include <string>
#include <iostream>

#include "proto/route.pb.h"

namespace valhalla {
namespace tyr {

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

#endif // VALHALLA_TYR_UTIL_H
