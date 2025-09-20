#ifndef VALHALLA_TYR_ROUTE_SERIALIZER_VALHALLA_
#define VALHALLA_TYR_ROUTE_SERIALIZER_VALHALLA_
#pragma once

#include "proto/api.pb.h"

namespace valhalla_serializers {

std::string serialize(valhalla::Api& api);

} // namespace valhalla_serializers

#endif // VALHALLA_TYR_ROUTE_SERIALIZER_VALHALLA_
