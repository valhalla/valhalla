#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <iostream>

#include <google/protobuf/util/json_util.h>

#include "proto/route.pb.h"
#include "proto/navigator.pb.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "baldr/json.h"
#include "baldr/location.h"
#include "baldr/errorcode_util.h"
#include "tyr/navigator.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace tyr {

Navigator::Navigator(const std::string& route_json_str) {
  google::protobuf::util::JsonStringToMessage(route_json_str, &route_);
  if (route_.has_trip() && (route_.trip().legs_size() > 0)
      && route_.trip().legs(0).has_shape()) {
    shape_ = midgard::decode<std::vector<PointLL> >(
        route_.trip().legs(0).shape());
  }
}

const Route& Navigator::route() const {
  return route_;
}

NavigationStatus Navigator::OnLocationChanged(const FixLocation& fix_location) {
  NavigationStatus nav_status;

  SnapToRoute(fix_location, nav_status);

  // TODO real code
  nav_status.set_route_state(NavigationStatus_RouteState_kPreTransition);
  nav_status.set_lat(40.042519);
  nav_status.set_lon(-76.299171);
  nav_status.set_leg_index(0);
  nav_status.set_maneuver_index(0);
  return nav_status;
}

void Navigator::SnapToRoute(const FixLocation& fix_location,
    NavigationStatus& nav_status) {
  // TODO check threshold
  PointLL fix_pt = PointLL(fix_location.lon(), fix_location.lat());
  auto closest = fix_pt.ClosestPoint(shape_);
  PointLL closest_ll = std::get<0>(closest);
  std::cout << "LL=" << closest_ll.lat() << "," << closest_ll.lng() << " | distance=" << std::get<1>(closest) << " | index=" << std::get<2>(closest) << std::endl;
  nav_status.set_route_state(NavigationStatus_RouteState_kTracking);
  nav_status.set_lat(closest_ll.lat());
  nav_status.set_lon(closest_ll.lng());
  // TODO Set leg and maneuver index
}



}
}

