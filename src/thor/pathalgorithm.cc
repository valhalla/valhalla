#include "thor/pathalgorithm.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "sif/edgelabel.h"

#include <chrono>
#include <ctime>

namespace dt = valhalla::baldr::DateTime;
namespace sc = std::chrono;

namespace valhalla {
namespace thor {

// helper function to initialize the object from a location
TimeInfo
TimeInfo::make(valhalla::Location& location, baldr::GraphReader& reader, int default_timezone_index) {
  // No time to to track
  if (!location.has_date_time()) {
    return {false};
  }

  // Find the first edge whose end node has a valid timezone index and keep it
  int timezone_index = 0;
  for (const auto& pe : location.path_edges()) {
    const baldr::GraphTile* tile = nullptr;
    const auto* edge = reader.directededge(baldr::GraphId(pe.graph_id()), tile);
    timezone_index = reader.GetTimezone(edge->endnode(), tile);
    if (timezone_index != 0)
      break;
  }

  // Set the origin timezone to be the timezone at the end node use this for timezone changes
  if (timezone_index == 0) {
    // Don't use the provided one if its not valid
    if (!dt::get_tz_db().from_index(default_timezone_index)) {
      default_timezone_index = baldr::DateTime::get_tz_db().to_index("Etc/UTC");
    }
    LOG_WARN("No timezone for location using default");
    timezone_index = default_timezone_index;
  }
  const auto* tz = dt::get_tz_db().from_index(timezone_index);

  // Get the current local time and set it if its a current time route
  // NOTE: we set to minute resolution to match the input format
  const auto now_date =
      date::make_zoned(tz, sc::time_point_cast<sc::seconds>(
                               sc::time_point_cast<sc::minutes>(sc::system_clock::now())));
  if (location.date_time() == "current") {
    std::ostringstream iso_dt;
    iso_dt << date::format("%FT%R", now_date);
    location.set_date_time(iso_dt.str());
  }

  // Convert the requested time into seconds from epoch
  date::local_seconds parsed_date;
  try {
    parsed_date = dt::get_formatted_date(location.date_time(), true);
  } catch (...) {
    LOG_ERROR("Could not parse provided date_time: " + location.date_time());
    return {false};
  }
  const auto then_date = date::make_zoned(tz, parsed_date);
  uint64_t local_time = date::to_utc_time(then_date.get_sys_time()).time_since_epoch().count();

  // What second of the week is this (for historical traffic lookup)
  auto second_of_week = dt::second_of_week(local_time, tz);

  // When is this route with respect to now this will let us appropriately use current flow traffic
  int64_t seconds_from_now = (then_date.get_local_time() - now_date.get_local_time()).count();

  // Make a valid object
  return {true, timezone_index, local_time, second_of_week, seconds_from_now};
}

// offset all the initial time info to reflect the progress along the route to this point
TimeInfo TimeInfo::operator+(Offset offset) const {
  if (!valid)
    return *this;

  // if the timezone changed we need to account for that offset as well
  int tz_diff = 0;
  if (offset.timezone_index != timezone_index) {
    tz_diff =
        dt::timezone_diff(local_time + offset.seconds, dt::get_tz_db().from_index(timezone_index),
                          dt::get_tz_db().from_index(offset.timezone_index));
  }

  // offset the local time and second of week by the amount traveled to this label
  uint64_t lt = local_time + static_cast<uint64_t>(offset.seconds) + tz_diff;
  uint32_t sw = second_of_week + static_cast<uint32_t>(offset.seconds) + tz_diff;
  if (sw > valhalla::midgard::kSecondsPerWeek) {
    sw -= valhalla::midgard::kSecondsPerWeek;
  }

  // return the shifted object, notice that seconds from now is only useful for
  // date_time type == current
  return {valid, offset.timezone_index, lt, sw,
          seconds_from_now + static_cast<int64_t>(offset.seconds)};
}

// offset all the initial time info to reflect the progress along the route to this point
TimeInfo TimeInfo::operator-(Offset offset) const {
  if (!valid)
    return *this;

  // if the timezone changed we need to account for that offset as well
  int tz_diff = 0;
  if (offset.timezone_index != timezone_index) {
    tz_diff =
        dt::timezone_diff(local_time + offset.seconds, dt::get_tz_db().from_index(timezone_index),
                          dt::get_tz_db().from_index(offset.timezone_index));
  }

  // offset the local time and second of week by the amount traveled to this label
  uint64_t lt =
      local_time - static_cast<uint64_t>(offset.seconds) + tz_diff; // dont route near the epoch
  int64_t sw = static_cast<int64_t>(second_of_week) - static_cast<int64_t>(offset.seconds) + tz_diff;
  if (sw < 0) {
    sw = valhalla::midgard::kSecondsPerWeek + sw;
  }

  // return the shifted object, notice that seconds from now is negative, this could be useful if
  // we had the ability to arrive_by current time but we dont for the moment
  return {valid, offset.timezone_index, lt, static_cast<uint32_t>(sw),
          seconds_from_now - static_cast<int64_t>(offset.seconds)};
}

} // namespace thor
} // namespace valhalla
