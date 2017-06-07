#include "valhalla/tyr/util.h"

#include "proto/route.pb.h"
#include "baldr/rapidjson_utils.h"

namespace valhalla {
namespace tyr {

void jsonRouteToProtoRoute (const std::string& json_route, Route& proto_route) {
  rapidjson::Document d;
  d.Parse (json_route.c_str());

  auto json_trip = GetOptionalFromRapidJson<rapidjson::Value::Object> (d, "/trip");
  if (!json_trip) {
    std::cout << "Empty" << std::endl;
  } else {
    std::cout << "Not Empty" << std::endl;
  }

  if (proto_route.has_trip())
    proto_route.clear_trip();

  Route::Trip* route_trip = proto_route.mutable_trip();



}

}
}
