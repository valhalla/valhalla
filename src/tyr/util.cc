#include <string>
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
    return;
  }

  if (proto_route.has_trip())
    proto_route.clear_trip();

  Route::Trip* route_trip = proto_route.mutable_trip();

  auto language = json_trip->FindMember("language");//GetOptionalFromRapidJson<std::string> (d, "/trip/language");
  if (language != json_trip->end()) {
    route_trip->set_language(language->value.GetString());
  }

  std::cout << "route language: " << *route_trip->mutable_language() << std::endl;

}

}
}
