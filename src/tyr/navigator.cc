#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <iostream>
#include <vector>

#include <google/protobuf/util/json_util.h>

#include "proto/route.pb.h"
#include "proto/navigator.pb.h"
#include "midgard/constants.h"
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
  set_route(route_json_str);
}

const Route& Navigator::route() const {
  return route_;
}

void Navigator::set_route(const std::string& route_json_str) {
  google::protobuf::util::JsonStringToMessage(route_json_str, &route_);
  leg_index_ = 0;
  maneuver_index_ = 0;
  SetUnits();
  SetShapeLengthTime();

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

void Navigator::SetUnits() {
  if (route_.has_trip() && (route_.trip().has_units())
      && route_.trip().units() == "miles") {
    kilometer_units_ = false;
  } else {
    kilometer_units_ = true;
  }
}

bool Navigator::HasKilometerUnits() const {
  return kilometer_units_;
}

void Navigator::SetShapeLengthTime() {
  if (route_.has_trip() && (route_.trip().legs_size() > 0)
      && route_.trip().legs(leg_index_).has_shape()) {
    shape_ = midgard::decode<std::vector<PointLL> >(
        route_.trip().legs(leg_index_).shape());

    // Set the remaining leg length
    float km_length = 0.0f;
    float total_remaining_leg_length = 0.0f;
    remaining_leg_lengths_.resize(shape_.size());
    int i = (remaining_leg_lengths_.size() - 1);
    remaining_leg_lengths_[i--] = total_remaining_leg_length;
    if (remaining_leg_lengths_.size() > 1) {
      for (; i >= 0; --i) {
        km_length = (shape_[i].Distance(shape_[i+1]) * midgard::kKmPerMeter);
        total_remaining_leg_length += (HasKilometerUnits() ? km_length : (km_length * midgard::kMilePerKm));
        remaining_leg_lengths_[i] = total_remaining_leg_length;
      }
    }
  } else {
    shape_.resize(0);
    remaining_leg_lengths_.resize(0);
  }
  current_shape_index_ = 0;
}

bool Navigator::IsDestinationShapeIndex(size_t idx) const {
  return (idx == (shape_.size() - 1));
}

size_t Navigator::FindManeuverIndex(size_t begin_search_index,
    size_t shape_index) const {

  // Check for destination shape index and return destination maneuver index
  if (IsDestinationShapeIndex(shape_index))
    return (route_.trip().legs(leg_index_).maneuvers_size() - 1);

  // Loop over maneuvers - starting at specified maneuver index and return
  // the maneuver index that contains the specified shape index
  for (size_t i = begin_search_index; i < route_.trip().legs(leg_index_).maneuvers_size(); ++i) {
    auto& maneuver = route_.trip().legs(leg_index_).maneuvers(i);
    if ((shape_index >= maneuver.begin_shape_index()) && (shape_index < maneuver.end_shape_index()))
      return i;
  }
  // If not found, return specified maneuver index
  return begin_search_index;
}

void Navigator::SnapToRoute(const FixLocation& fix_location,
    NavigationStatus& nav_status) {
  // TODO check threshold

  // Find closest point and set current shape index
  PointLL fix_pt = PointLL(fix_location.lon(), fix_location.lat());
  auto closest = fix_pt.ClosestPoint(shape_, current_shape_index_);

  // GDG - rm
  std::cout << std::endl << "--------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "LL=" << std::get<0>(closest).lat() << "," << std::get<0>(closest).lng() << " | distance=" << std::get<1>(closest) << " | index=" << std::get<2>(closest) << std::endl;

  // If off route then return invalid route state
  if (std::get<1>(closest) > kOffRouteThreshold) {
    nav_status.set_route_state(NavigationStatus_RouteState_kInvalid);
    return;
  }

  // If not off route then set closest point and current shape index
  PointLL closest_ll = std::get<0>(closest);
  current_shape_index_ = std::get<2>(closest);

  // If approximately equal to the next shape point then snap to it and set flag
  bool snapped_to_shape_point = false;
  if (!IsDestinationShapeIndex(current_shape_index_)
      && closest_ll.ApproximatelyEqual(shape_.at(current_shape_index_ + 1))) {
    // Increment current shape index
    ++current_shape_index_;
    // Set at shape point flag
    snapped_to_shape_point = true;
  }

  // Set remaining index
  size_t remaining_index = 0;
  if (snapped_to_shape_point || IsDestinationShapeIndex(current_shape_index_))
    remaining_index = current_shape_index_;
  else
    remaining_index = (current_shape_index_ + 1);

  // Calculate partial length, if needed
  float partial_length = 0.0f;
  if (!snapped_to_shape_point && !IsDestinationShapeIndex(current_shape_index_)) {
    partial_length = (closest_ll.Distance(shape_.at(remaining_index)) * midgard::kKmPerMeter);
    // Convert to miles, if needed
    if (!HasKilometerUnits())
      partial_length = (partial_length * midgard::kMilePerKm);
  }

  // Set the maneuver index and maneuver end shape index
  maneuver_index_ = FindManeuverIndex(maneuver_index_, current_shape_index_);
  uint32_t maneuver_end_shape_index = route_.trip().legs(leg_index_).maneuvers(maneuver_index_).end_shape_index();
  float remaining_leg_length = (remaining_leg_lengths_.at(remaining_index) + partial_length);

  // GDG - rm
  std::cout << "current_shape_index_=" << current_shape_index_ << " | remaining_index=" << remaining_index << " | maneuver_index_=" << maneuver_index_ << " | snapped_to_shape_point=" << (snapped_to_shape_point ? "true" : "false") << std::endl;
  std::cout << "remaining_leg_length=" << remaining_leg_length << " | remaining_leg_lengths_.at(remaining_index)=" << remaining_leg_lengths_.at(remaining_index) << " | partial_length=" << partial_length << " | remaining_maneuver_length=" << (remaining_leg_length - remaining_leg_lengths_.at(maneuver_end_shape_index)) << " | remaining_leg_lengths_.at(maneuver_end_shape_index)=" << remaining_leg_lengths_.at(maneuver_end_shape_index) << std::endl;

  // Populate navigation status
  nav_status.set_route_state(NavigationStatus_RouteState_kTracking);
  nav_status.set_lon(closest_ll.lng());
  nav_status.set_lat(closest_ll.lat());
  nav_status.set_leg_index(leg_index_);
  nav_status.set_remaining_leg_length(remaining_leg_length);
  nav_status.set_remaining_leg_time(0);
  nav_status.set_maneuver_index(maneuver_index_);
  nav_status.set_remaining_maneuver_length(remaining_leg_length - remaining_leg_lengths_.at(maneuver_end_shape_index));
  nav_status.set_remaining_maneuver_time(0);
}



}
}

