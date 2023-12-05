#ifndef VALHALLA_TYR_ROUTE_SERIALIZER_OSRM_
#define VALHALLA_TYR_ROUTE_SERIALIZER_OSRM_
#pragma once

#include "proto_conversions.h"

namespace osrm_serializers {
/*
OSRM output is described in: http://project-osrm.org/docs/v5.5.1/api/
{
    "code":"Ok"
    "waypoints": [{ }, { }...],
    "routes": [
        {
            "geometry":"....."
            "distance":xxx.y
            "duration":yyy.z
            "legs":[
                {
                    "steps":[
                        "intersections":[
                        ]
                        "geometry":" "
                        "maneuver":{
                        }
                    ]
                }
            ]
        },
        ...
    ]
}
*/

// Serialize route response in OSRM compatible format.
// Inputs are:
//     directions options
//     TripLeg protocol buffer
//     DirectionsLeg protocol buffer
std::string serialize(valhalla::Api& api);

} // namespace osrm_serializers

#endif // VALHALLA_TYR_ROUTE_SERIALIZER_OSRM_
