#include <cmath>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "proto/route.pb.h"
#include "proto/navigator.pb.h"
#include "midgard/constants.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "baldr/json.h"
#include "baldr/location.h"
#include "exception.h"
#include "tyr/serializers.h"
#include "tyr/navigator.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace tyr {

Navigator::Navigator() {
  route_state_ = NavigationStatus_RouteState_kInvalid;
  maneuver_index_ = 0;
  leg_index_ = 0;
  current_shape_index_ = 0;
  kilometer_units_ = true;
}

NavigationStatus Navigator::SetRoute(const std::string& route_json_str) {
  NavigationStatus nav_status;

  try {
    jsonToProtoRoute (route_json_str, route_);
  } catch (const std::runtime_error& e) {
    route_state_ = NavigationStatus_RouteState_kInvalid;
    nav_status.set_route_state(route_state_);
    return nav_status;
  }

  leg_index_ = 0;
  maneuver_index_ = 0;
  InitializeDistanceUnits();
  InitializeShapeLengthTime();
  InitializeUsedInstructions();
  route_state_ = NavigationStatus_RouteState_kInitialized;

  nav_status.set_route_state(route_state_);
  return nav_status;
}

NavigationStatus Navigator::OnLocationChanged(const FixLocation& fix_location) {
  NavigationStatus nav_status;

  // If route state is invalid then just return
  if (route_state_ == NavigationStatus_RouteState_kInvalid) {
    nav_status.set_route_state(route_state_);
    return nav_status;
  }

  // Set the previous route state prior to snapping to route
  NavigationStatus_RouteState prev_route_state = route_state_;

  // Snap the fix location to the route and update nav_status
  nav_status = SnapToRoute(fix_location);

  size_t curr_instruction_index = maneuver_index_;
  size_t next_instruction_index = (maneuver_index_ + 1);
  float alert_length = 0.0f;

  // Only process a valid route state
  if (nav_status.route_state() != NavigationStatus_RouteState_kInvalid) {

    //////////////////////////////////////////////////////////////////////////
    // If destination maneuver index then mark as complete
    if (IsDestinationManeuverIndex(maneuver_index_)) {
      // Set route state
      route_state_ = NavigationStatus_RouteState_kComplete;
      nav_status.set_route_state(route_state_);
    }

    //////////////////////////////////////////////////////////////////////////
    // If start maneuver index and instruction has not been used
    // and starting navigation and close to origin
    // then set route state to kPreTransition
    else if (IsStartManeuverIndex(maneuver_index_)
        && !(std::get<kPreTransition>(used_instructions_.at(curr_instruction_index)))
        && StartingNavigation(prev_route_state, route_state_)
        && OnRouteLocationCloseToOrigin(nav_status)) {
      // Set route state
      route_state_ = NavigationStatus_RouteState_kPreTransition;
      nav_status.set_route_state(route_state_);

      // Set the instruction maneuver index for the start maneuver
      nav_status.set_instruction_maneuver_index(curr_instruction_index);

      // Mark that the pre-transition was used
      std::get<kPreTransition>(used_instructions_.at(curr_instruction_index)) = true;
    }

    //////////////////////////////////////////////////////////////////////////
    // else if pre-transition instruction has not been used
    // and route location is a pre-transition
    // then set route state to kPreTransition
    else if (!(std::get<kPreTransition>(used_instructions_.at(next_instruction_index)))
        && (GetRemainingManeuverTime(fix_location, nav_status)
            <= GetPreTransitionThreshold(next_instruction_index))) {

      // Set route state
      route_state_ = NavigationStatus_RouteState_kPreTransition;
      nav_status.set_route_state(route_state_);

      // Set the instruction maneuver index for the next maneuver
      nav_status.set_instruction_maneuver_index(next_instruction_index);

      // Mark the next pre-transition, the next final transition alert,
      // and the current post transition were used
      std::get<kPreTransition>(used_instructions_.at(next_instruction_index)) = true;
      std::get<kFinalTransitionAlert>(used_instructions_.at(next_instruction_index)) = true;
      std::get<kPostTransition>(used_instructions_.at(curr_instruction_index)) = true;
    }

    //////////////////////////////////////////////////////////////////////////
    // else if initial transition alert instruction has not been used
    // and route location is an initial transition alert
    // then set route state to kTransitionAlert
    else if (!(std::get<kInitialTransitionAlert>(used_instructions_.at(next_instruction_index)))
        && IsInitialTransitionAlert(fix_location, nav_status, alert_length)) {
      // Set route state
      route_state_ = NavigationStatus_RouteState_kTransitionAlert;
      nav_status.set_route_state(route_state_);

      // Set the instruction maneuver index for the next maneuver
      nav_status.set_instruction_maneuver_index(next_instruction_index);

      // Set the transition alert length
      nav_status.set_transition_alert_length(alert_length);

      // Mark the next initial transition alert and the current post transition were used
      std::get<kInitialTransitionAlert>(used_instructions_.at(next_instruction_index)) = true;
      std::get<kPostTransition>(used_instructions_.at(curr_instruction_index)) = true;
    }

    //////////////////////////////////////////////////////////////////////////
    // else if final transition alert instruction has not been used
    // and route location is a final transition alert
    // and alert is not close to the pre-transition
    // then set route state to kTransitionAlert
    else if (!(std::get<kFinalTransitionAlert>(used_instructions_.at(next_instruction_index)))
        && IsFinalTransitionAlert(fix_location, nav_status, alert_length)
        && !IsAlertCloseToPre(fix_location, nav_status, next_instruction_index)) {
      // Set route state
      route_state_ = NavigationStatus_RouteState_kTransitionAlert;
      nav_status.set_route_state(route_state_);

      // Set the instruction maneuver index for the next maneuver
      nav_status.set_instruction_maneuver_index(next_instruction_index);

      // Set the transition alert length
      nav_status.set_transition_alert_length(alert_length);

      // Mark the next final transition alert and the current post transition were used
      std::get<kFinalTransitionAlert>(used_instructions_.at(next_instruction_index)) = true;
      std::get<kPostTransition>(used_instructions_.at(curr_instruction_index)) = true;
    }

    //////////////////////////////////////////////////////////////////////////
    // if post instruction has not been used
    // and route location is post transition
    // then set route state to kPostTransition
    else if (!(std::get<kPostTransition>(used_instructions_.at(curr_instruction_index)))
        && IsPostTransition(fix_location, nav_status)) {
      // Set route state
      route_state_ = NavigationStatus_RouteState_kPostTransition;
      nav_status.set_route_state(route_state_);

      // Set the instruction maneuver index for the current maneuver
      nav_status.set_instruction_maneuver_index(curr_instruction_index);

      // Mark that the post-transition was used
      std::get<kPostTransition>(used_instructions_.at(curr_instruction_index)) = true;
    }

  }
  return nav_status;
}

void Navigator::InitializeDistanceUnits() {
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

void Navigator::InitializeShapeLengthTime() {
  if (route_.has_trip() && (route_.trip().legs_size() > 0)
      && route_.trip().legs(leg_index_).has_shape()) {
    shape_ = midgard::decode<std::vector<PointLL> >(
        route_.trip().legs(leg_index_).shape());

    // Create maneuver speeds in units per second
    maneuver_speeds_.clear();
    // TODO: rm or make trace
    std::cout << std::endl << "SPEED ======================================" << std::endl;
    size_t zzz = 0;
    for (const auto& maneuver : route_.trip().legs(leg_index_).maneuvers()) {
      // Calculate speed in units per second - protect against divide by zero
      float time = (maneuver.time() == 0) ? 0.000028f : maneuver.time();
      float speed = (maneuver.length()/time);
      // TODO: rm or make trace
      std::cout << "index=" << zzz++ << " | maneuver.length()=" << maneuver.length() << " | maneuver.time()=" << maneuver.time() << " | time=" << time << " | speed(units/sec)=" << speed << " | speed(units/hour)=" << (speed*3600) << std::endl;
      maneuver_speeds_.emplace_back(speed);

    }

    // Initialize the remaining leg length and time
    float km_length = 0.0f;
    float length = 0.0f;
    float total_remaining_leg_length = 0.0f;
    uint32_t total_remaining_leg_time = 0;

    // Resize vector for current shape
    remaining_leg_values_.resize(shape_.size());

    // Initialize indexes and set destination shape values
    size_t maneuver_speed_index = (maneuver_speeds_.size() - 1);
    int i = (remaining_leg_values_.size() - 1);
    remaining_leg_values_[i--] = {total_remaining_leg_length, total_remaining_leg_time};

    // Process shape to set remaining length and time
    if (remaining_leg_values_.size() > 1) {
      for (; i >= 0; --i) {
        // Determine length between shape points and convert to appropriate units
        km_length = (shape_[i].Distance(shape_[i+1]) * midgard::kKmPerMeter);
        length = (HasKilometerUnits() ? km_length : (km_length * midgard::kMilePerKm));

        // Find the maneuver index that corresponds to the shape index
        maneuver_speed_index = RfindManeuverIndex(maneuver_speed_index, i);

        // Update the total remaining values
        total_remaining_leg_length += length;
        total_remaining_leg_time += static_cast<uint32_t>(round(length/maneuver_speeds_.at(maneuver_speed_index)));

        remaining_leg_values_[i] = {total_remaining_leg_length, total_remaining_leg_time};
      }
    }
  } else {
    shape_.resize(0);
    remaining_leg_values_.resize(0);
  }
  current_shape_index_ = 0;
}

void Navigator::InitializeUsedInstructions() {
  used_instructions_.clear();
  for (size_t i = 0; i < route_.trip().legs(leg_index_).maneuvers_size(); ++i) {
    used_instructions_.emplace_back(false, false, false, false);
  }
}

bool Navigator::IsDestinationShapeIndex(size_t idx) const {
  return (idx == (shape_.size() - 1));
}

bool Navigator::IsStartManeuverIndex(size_t idx) const {
  return (idx == 0);
}

bool Navigator::IsDestinationManeuverIndex(size_t idx) const {
  return (idx == (route_.trip().legs(leg_index_).maneuvers_size() - 1));
}

size_t Navigator::FindManeuverIndex(size_t begin_search_index,
    size_t shape_index) const {

  // Set the destination maneuver index - since destination maneuver is a special case
  size_t destination_maneuver_index =
      (route_.trip().legs(leg_index_).maneuvers_size() - 1);

  // Validate the begin_search_index
  if ((route_.trip().legs(leg_index_).maneuvers_size() == 0)
      || (begin_search_index > destination_maneuver_index))
    throw valhalla_exception_t{502};

  // Check for destination shape index and return destination maneuver index
  if (IsDestinationShapeIndex(shape_index))
    return destination_maneuver_index;

  // Loop over maneuvers - starting at specified maneuver index and return
  // the maneuver index that contains the specified shape index
  for (size_t i = begin_search_index; i < destination_maneuver_index; ++i) {
    const auto& maneuver = route_.trip().legs(leg_index_).maneuvers(i);
    if ((shape_index >= maneuver.begin_shape_index())
        && (shape_index < maneuver.end_shape_index()))
      return i;
  }
  // If not found, throw exception
  throw valhalla_exception_t{502};
}

size_t Navigator::RfindManeuverIndex(size_t rbegin_search_index,
    size_t shape_index) const {

  size_t maneuver_count = route_.trip().legs(leg_index_).maneuvers_size();

  // Set the destination maneuver index - since destination maneuver is a special case
  size_t destination_maneuver_index = (maneuver_count - 1);

  // Validate the rbegin_search_index
  if ((maneuver_count == 0) || (rbegin_search_index > destination_maneuver_index))
    throw valhalla_exception_t { 502 };

  // Check for destination shape index and rbegin search index
  // if so, return destination maneuver index
  if (IsDestinationShapeIndex(shape_index)
      && (destination_maneuver_index == rbegin_search_index))
    return destination_maneuver_index;

  // Loop over maneuvers in reverse - starting at specified maneuver index
  // and return the maneuver index that contains the specified shape index
  for (size_t i = rbegin_search_index; i < maneuver_count; --i) {
    const auto& maneuver = route_.trip().legs(leg_index_).maneuvers(i);
    if ((shape_index >= maneuver.begin_shape_index()) && (shape_index < maneuver.end_shape_index()))
      return i;
  }
  // If not found, throw exception
  throw valhalla_exception_t{502};
}

NavigationStatus Navigator::SnapToRoute(const FixLocation& fix_location) {
  NavigationStatus nav_status;

  // Find the closest point on the route that corresponds to the fix location
  PointLL fix_pt = PointLL(fix_location.lon(), fix_location.lat());
  auto closest = fix_pt.ClosestPoint(shape_, current_shape_index_);

  // If the fix point distance from route is greater than the off route threshold
  // then return invalid route state
  if (std::get<kClosestPointDistance>(closest) > kOffRouteThreshold) {
    route_state_ = NavigationStatus_RouteState_kInvalid;
    nav_status.set_route_state(route_state_);
    return nav_status;
  }

  // If not off route then set closest point and current shape index
  PointLL closest_ll = std::get<kClosestPoint>(closest);
  current_shape_index_ = std::get<kClosestPointSegmentIndex>(closest);

  // If approximately equal to the next shape point then snap to it and set flag
  bool snapped_to_shape_point = false;
  if (!IsDestinationShapeIndex(current_shape_index_)
      && closest_ll.ApproximatelyEqual(shape_.at(current_shape_index_ + 1))) {
    // Increment current shape index
    ++current_shape_index_;
    // Set at shape point flag
    snapped_to_shape_point = true;
  }

  // Set the remaining index
  size_t remaining_index = 0;
  if (snapped_to_shape_point || IsDestinationShapeIndex(current_shape_index_))
    remaining_index = current_shape_index_;
  else
    remaining_index = (current_shape_index_ + 1);

  // Calculate the partial length, if needed
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

  // Set the remaining leg length and time values
  float remaining_leg_length = (remaining_leg_values_.at(remaining_index).first
      + partial_length);
  uint32_t remaining_leg_time = (remaining_leg_values_.at(remaining_index).second
      + static_cast<uint32_t>(round(
          partial_length / maneuver_speeds_.at(maneuver_index_))));

  // Populate navigation status
  route_state_ = NavigationStatus_RouteState_kTracking;
  nav_status.set_route_state(route_state_);
  nav_status.set_lon(closest_ll.lng());
  nav_status.set_lat(closest_ll.lat());
  nav_status.set_leg_index(leg_index_);
  nav_status.set_remaining_leg_length(remaining_leg_length);
  nav_status.set_remaining_leg_time(remaining_leg_time);
  nav_status.set_maneuver_index(maneuver_index_);
  nav_status.set_remaining_maneuver_length(remaining_leg_length - remaining_leg_values_.at(maneuver_end_shape_index).first);
  nav_status.set_remaining_maneuver_time(remaining_leg_time - remaining_leg_values_.at(maneuver_end_shape_index).second);

#ifdef LOGGING_LEVEL_TRACE
  // Output to help build unit tests
  std::cout << "  maneuver_index = " << nav_status.maneuver_index() << ";" << std::setprecision(9) << std::endl
            << "  instruction_index = maneuver_index;" << std::endl
            << "  TryRouteOnLocationChanged(nav," << std:: endl
            << "      GetFixLocation(" << fix_location.lon() << "f, " << fix_location.lat() << "f, " << fix_location.time() << ", " << fix_location.speed()  << "f),"  << std::endl
            << "      GetNavigationStatus(NavigationStatus_RouteState_kTracking," << std::endl
            << "          " << nav_status.lon() << "f, " << nav_status.lat() << "f, leg_index, " << nav_status.remaining_leg_length() << "f, " << nav_status.remaining_leg_time() << "," << std::endl
            << "          maneuver_index, " << nav_status.remaining_maneuver_length() << "f, " << nav_status.remaining_maneuver_time() << "));" << std::endl;
#endif
//#ifdef LOGGING_LEVEL_TRACE
//  // Output to help build unit tests
//  std::cout << std::endl << "//----------------------------------------------------------------------" << std::setprecision(9) << std::endl
//            << "      GetFixLocation(" << fix_location.lon() << "f, " << fix_location.lat() << "f, " << (remaining_leg_values_.at(0).second - nav_status.remaining_leg_time()) << "),"  << std::endl
//            << "      GetNavigationStatus(NavigationStatus_RouteState_kTracking," << std::endl
//            << "          " << nav_status.lon() << "f, " << nav_status.lat() << "f, leg_index, " << nav_status.remaining_leg_length() << "f, " << nav_status.remaining_leg_time() << "," << std::endl
//            << "          maneuver_index, " << nav_status.remaining_maneuver_length() << "f, " << nav_status.remaining_maneuver_time() << ", instruction_index));" << std::endl;
//#endif


  return nav_status;
}

bool Navigator::StartingNavigation(
    const NavigationStatus_RouteState& prev_route_state,
    const NavigationStatus_RouteState& curr_route_state) const {
  return ((prev_route_state == NavigationStatus_RouteState_kInitialized)
      && (curr_route_state == NavigationStatus_RouteState_kTracking));
}

bool Navigator::OnRouteLocationCloseToOrigin(
    const NavigationStatus& nav_status) const {
  if ((remaining_leg_values_.size() > 0)
      && nav_status.has_remaining_leg_length()) {
    float meters = UnitsToMeters(
        remaining_leg_values_.at(0).first - nav_status.remaining_leg_length());
    return (meters <= kOnRouteCloseToOriginThreshold);
  }
  return false;
}

float Navigator::UnitsToMeters(float units) const {
  float km_length = 0.0f;
  if (HasKilometerUnits())
    km_length = units;
  else
    km_length = units * midgard::kKmPerMile;

  return (km_length * kMetersPerKm);
}

size_t Navigator::GetWordCount(const std::string& instruction) const {
  size_t word_count = 0;
  std::string::const_iterator pos = instruction.begin();
  std::string::const_iterator end = instruction.end();

  while (pos != end)
  {
    // Skip over space, white space, and punctuation
    while (pos != end
        && ((*pos == ' ') || std::isspace(*pos) || std::ispunct(*pos)))
      ++pos;

    // Word found - increment
    word_count += (pos != end);

    // Skip over letters in word
    while (pos != end
        && ((*pos != ' ') && (!std::isspace(*pos)) && (!std::ispunct(*pos))))
      ++pos;
  }
  return word_count;
}

uint32_t Navigator::GetSpentManeuverTime(const FixLocation& fix_location,
    const NavigationStatus& nav_status) const {
  // speed in meters per second
  float speed = 0.0f;
  if (fix_location.has_speed()) {
    speed = fix_location.speed();
  }

  uint32_t maneuver_begin_shape_index = route_.trip().legs(leg_index_).maneuvers(maneuver_index_).begin_shape_index();

  // Use speed if user is moving to calculate spent maneuver time
  if (speed > kMinSpeedThreshold) {
    return static_cast<uint32_t>(round(
        UnitsToMeters(remaining_leg_values_.at(maneuver_begin_shape_index).first - nav_status.remaining_leg_length()) / speed));
  }

  return (remaining_leg_values_.at(maneuver_begin_shape_index).second - nav_status.remaining_leg_time());
}

uint32_t Navigator::GetRemainingManeuverTime(const FixLocation& fix_location,
    const NavigationStatus& nav_status) const {
  // speed in meters per second
  float speed = 0.0f;
  if (fix_location.has_speed()) {
    speed = fix_location.speed();
  }

  // Use speed if user is moving to calculate remaining maneuver time
  if (speed > kMinSpeedThreshold) {
    return static_cast<uint32_t>(round(
        UnitsToMeters(nav_status.remaining_maneuver_length()) / speed));
  }

  return nav_status.remaining_maneuver_time();
}

uint32_t Navigator::GetPreTransitionThreshold(size_t instruction_index) const {
  const auto& maneuver = route_.trip().legs(leg_index_).maneuvers(instruction_index);

  // TODO: do we need to adjust time for destination?

  float adjustment_factor = 1.0f;
  // Set adjustment factor to reduce time if instruction is multi-cue
  // TODO: may want to isolate adjustment to be based on next maneuver phrase
  if (maneuver.verbal_multi_cue()){
    adjustment_factor = 0.75f;
  }

  return (kPreTransitionBaseThreshold
      + (static_cast<uint32_t>(round(
          GetWordCount(maneuver.verbal_pre_transition_instruction())
              / kWordsPerSecond * adjustment_factor))));
}

bool Navigator::IsAlertCloseToPre(const FixLocation& fix_location,
    const NavigationStatus& nav_status, size_t instruction_index) const {
  const auto& maneuver = route_.trip().legs(leg_index_).maneuvers(instruction_index);

  // TODO handle the transition alert pre-phrase of "In 500 feet..."
  int remaining_time_after_alert = (GetRemainingManeuverTime(fix_location, nav_status)
      - (static_cast<uint32_t>(round(GetWordCount(maneuver.verbal_transition_alert_instruction()) / kWordsPerSecond)))
      - kAlertPreTimeDelta);

  return (remaining_time_after_alert < GetPreTransitionThreshold(instruction_index));
}

bool Navigator::IsTimeWithinBounds(uint32_t time, uint32_t lower_bound,
    uint32_t upper_bound) const {
  return ((time > lower_bound) && (time < upper_bound));
}

bool Navigator::IsLengthWithinBounds(float length, float lower_bound,
    float upper_bound) const {
  return ((length > lower_bound) && (length < upper_bound));
}

bool Navigator::IsPostTransition(const FixLocation& fix_location,
    const NavigationStatus& nav_status) const {

  const auto& maneuver = route_.trip().legs(leg_index_).maneuvers(maneuver_index_);

  // If the maneuver has a verbal post transition instruction
  // and the maneuver is NOT a verbal multi-cue
  // and the fix location is within the post instruction bounds
  // then return true
  if (maneuver.has_verbal_post_transition_instruction()
      && !maneuver.has_verbal_multi_cue()
      && (IsTimeWithinBounds(GetSpentManeuverTime(fix_location, nav_status),
          kPostTransitionLowerBound, kPostTransitionUpperBound))) {
    return true;
  }
  return false;
}

bool Navigator::IsInitialTransitionAlert(const FixLocation& fix_location,
    const NavigationStatus& nav_status, float& alert_length) const {

  size_t curr_instruction_index = maneuver_index_;
  size_t next_instruction_index = (maneuver_index_ + 1);

  // Verify that the current maneuver is not a destination maneuver
  // and the next maneuver has a transition alert instruction
  // and allow for post transition time
  if (!IsDestinationManeuverIndex(curr_instruction_index)
      && route_.trip().legs(leg_index_).maneuvers(next_instruction_index).has_verbal_transition_alert_instruction()
      && (GetSpentManeuverTime(fix_location, nav_status) > kPostTransitionLowerBound)) {

    ///////////////////////////////////////////////////////////////////////////
    // Validate initial long current maneuver length
    // and fix speed OR maneuver speed
    // and location prior to next maneuver
    if ((route_.trip().legs(leg_index_).maneuvers(curr_instruction_index).length()
        > GetInitialLongTransitionAlertMinManeuverLength())
        && ((fix_location.has_speed()
            && (fix_location.speed() > kInitialLongTransitionAlertMinSpeed)) // ~62.6 MPH
            || (!fix_location.has_speed()
                && (UnitsToMeters(nav_status.remaining_maneuver_length())
                    / nav_status.remaining_maneuver_time()
                    > kInitialLongTransitionAlertMinSpeed)))
        && IsLengthWithinBounds(nav_status.remaining_maneuver_length(),
            GetInitialLongTransitionAlertLowerLength(),
            GetInitialLongTransitionAlertUpperLength())) {
      alert_length = GetInitialLongTransitionAlertLength();
      return true;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Validate initial short current maneuver length
    // and fix speed OR maneuver speed
    // and location prior to next maneuver
    else if ((route_.trip().legs(leg_index_).maneuvers(curr_instruction_index).length()
        > GetInitialShortTransitionAlertMinManeuverLength())
        && ((fix_location.has_speed()
            && (fix_location.speed() > kInitialShortTransitionAlertMinSpeed)) // ~40.3 MPH
            || (!fix_location.has_speed()
                && (UnitsToMeters(nav_status.remaining_maneuver_length())
                    / nav_status.remaining_maneuver_time()
                    > kInitialShortTransitionAlertMinSpeed)))
        && IsLengthWithinBounds(nav_status.remaining_maneuver_length(),
            GetInitialShortTransitionAlertLowerLength(),
            GetInitialShortTransitionAlertUpperLength())) {
      alert_length = GetInitialShortTransitionAlertLength();
      return true;
    }
  }
  return false;
}

float Navigator::GetInitialLongTransitionAlertLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialLongTransitionAlertImperialLength;
  }
  // Return metric value
  return kInitialLongTransitionAlertMetricLength;
}

float Navigator::GetInitialLongTransitionAlertLowerLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialLongTransitionAlertLowerImperialLength;
  }
  // Return metric value
  return kInitialLongTransitionAlertLowerMetricLength;
}

float Navigator::GetInitialLongTransitionAlertUpperLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialLongTransitionAlertUpperImperialLength;
  }
  // Return metric value
  return kInitialLongTransitionAlertUpperMetricLength;
}

float Navigator::GetInitialLongTransitionAlertMinManeuverLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialLongTransitionAlertMinManeuverImperialLength;
  }
  // Return metric value
  return kInitialLongTransitionAlertMinManeuverMetricLength;
}

float Navigator::GetInitialShortTransitionAlertLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialShortTransitionAlertImperialLength;
  }
  // Return metric value
  return kInitialShortTransitionAlertMetricLength;
}

float Navigator::GetInitialShortTransitionAlertLowerLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialShortTransitionAlertLowerImperialLength;
  }
  // Return metric value
  return kInitialShortTransitionAlertLowerMetricLength;
}

float Navigator::GetInitialShortTransitionAlertUpperLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialShortTransitionAlertUpperImperialLength;
  }
  // Return metric value
  return kInitialShortTransitionAlertUpperMetricLength;
}

float Navigator::GetInitialShortTransitionAlertMinManeuverLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kInitialShortTransitionAlertMinManeuverImperialLength;
  }
  // Return metric value
  return kInitialShortTransitionAlertMinManeuverMetricLength;
}

bool Navigator::IsFinalTransitionAlert(const FixLocation& fix_location,
    const NavigationStatus& nav_status, float& alert_length) const {

  size_t curr_instruction_index = maneuver_index_;
  size_t next_instruction_index = (maneuver_index_ + 1);

  // Verify that the current maneuver is not a destination maneuver
  // and the next maneuver has a transition alert instruction
  // and allow for post transition time
  if (!IsDestinationManeuverIndex(curr_instruction_index)
      && route_.trip().legs(leg_index_).maneuvers(next_instruction_index).has_verbal_transition_alert_instruction()
      && (GetSpentManeuverTime(fix_location, nav_status) > kPostTransitionLowerBound)) {

    ///////////////////////////////////////////////////////////////////////////
    // Validate final long current maneuver length
    // and fix speed OR maneuver speed
    // and location prior to next maneuver
    if ((route_.trip().legs(leg_index_).maneuvers(curr_instruction_index).length()
        > GetFinalLongTransitionAlertMinManeuverLength())
        && ((fix_location.has_speed()
            && (fix_location.speed() > kFinalLongTransitionAlertMinSpeed)) // ~62.6 MPH
            || (!fix_location.has_speed()
                && (UnitsToMeters(nav_status.remaining_maneuver_length())
                    / nav_status.remaining_maneuver_time()
                    > kFinalLongTransitionAlertMinSpeed)))
        && IsLengthWithinBounds(nav_status.remaining_maneuver_length(),
            GetFinalLongTransitionAlertLowerLength(),
            GetFinalLongTransitionAlertUpperLength())) {
      alert_length = GetFinalLongTransitionAlertLength();
      return true;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Validate final medium current maneuver length
    // and fix speed OR maneuver speed
    // and location prior to next maneuver
    else if ((route_.trip().legs(leg_index_).maneuvers(curr_instruction_index).length()
        > GetFinalMediumTransitionAlertMinManeuverLength())
        && ((fix_location.has_speed()
            && (fix_location.speed() > kFinalMediumTransitionAlertMinSpeed)) // ~22.4 MPH
            || (!fix_location.has_speed()
                && (UnitsToMeters(nav_status.remaining_maneuver_length())
                    / nav_status.remaining_maneuver_time()
                    > kFinalMediumTransitionAlertMinSpeed)))
        && IsLengthWithinBounds(nav_status.remaining_maneuver_length(),
            GetFinalMediumTransitionAlertLowerLength(),
            GetFinalMediumTransitionAlertUpperLength())) {
      alert_length = GetFinalMediumTransitionAlertLength();
      return true;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Validate location prior to next maneuver
    else if (IsLengthWithinBounds(nav_status.remaining_maneuver_length(),
        GetFinalShortTransitionAlertLowerLength(),
        GetFinalShortTransitionAlertUpperLength())) {
      alert_length = GetFinalShortTransitionAlertLength();
      return true;
    }
  }
  return false;
}

float Navigator::GetFinalLongTransitionAlertLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalLongTransitionAlertImperialLength;
  }
  // Return metric value
  return kFinalLongTransitionAlertMetricLength;
}

float Navigator::GetFinalLongTransitionAlertLowerLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalLongTransitionAlertLowerImperialLength;
  }
  // Return metric value
  return kFinalLongTransitionAlertLowerMetricLength;
}

float Navigator::GetFinalLongTransitionAlertUpperLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalLongTransitionAlertUpperImperialLength;
  }
  // Return metric value
  return kFinalLongTransitionAlertUpperMetricLength;
}

float Navigator::GetFinalLongTransitionAlertMinManeuverLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalLongTransitionAlertMinManeuverImperialLength;
  }
  // Return metric value
  return kFinalLongTransitionAlertMinManeuverMetricLength;
}

float Navigator::GetFinalMediumTransitionAlertLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalMediumTransitionAlertImperialLength;
  }
  // Return metric value
  return kFinalMediumTransitionAlertMetricLength;
}

float Navigator::GetFinalMediumTransitionAlertLowerLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalMediumTransitionAlertLowerImperialLength;
  }
  // Return metric value
  return kFinalMediumTransitionAlertLowerMetricLength;
}

float Navigator::GetFinalMediumTransitionAlertUpperLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalMediumTransitionAlertUpperImperialLength;
  }
  // Return metric value
  return kFinalMediumTransitionAlertUpperMetricLength;
}

float Navigator::GetFinalMediumTransitionAlertMinManeuverLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalMediumTransitionAlertMinManeuverImperialLength;
  }
  // Return metric value
  return kFinalMediumTransitionAlertMinManeuverMetricLength;
}

float Navigator::GetFinalShortTransitionAlertLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalShortTransitionAlertImperialLength;
  }
  // Return metric value
  return kFinalShortTransitionAlertMetricLength;
}

float Navigator::GetFinalShortTransitionAlertLowerLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalShortTransitionAlertLowerImperialLength;
  }
  // Return metric value
  return kFinalShortTransitionAlertLowerMetricLength;
}

float Navigator::GetFinalShortTransitionAlertUpperLength() const {
  // If imperial units
  if (!HasKilometerUnits()) {
    // Return imperial value
    return kFinalShortTransitionAlertUpperImperialLength;
  }
  // Return metric value
  return kFinalShortTransitionAlertUpperMetricLength;
}

}
}

