#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>

#include <google/protobuf/util/json_util.h>

#include "proto/route.pb.h"
#include "midgard/logging.h"
#include "baldr/json.h"
#include "baldr/location.h"
#include "baldr/errorcode_util.h"
#include "tyr/navigator.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace tyr {

Navigator::Navigator(const std::string& route_json_str) {
  google::protobuf::util::JsonStringToMessage(route_json_str, &route_);
}

const valhalla::Route& Navigator::route() const {
  return route_;
}

void Navigator::OnLocationChanged(const Location& location) {
  // TODO
}

void Navigator::SnapToRoute() {
  // TODO
}



}
}

