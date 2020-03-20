#include "thor/pathalgorithm.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"
#include "sif/edgelabel.h"

#include <chrono>
#include <ctime>

namespace dt = valhalla::baldr::DateTime;

namespace valhalla {
namespace thor {

// helper function to initialize the object from a location
TimeInfo TimeInfo::make(valhalla::Location& location, baldr::GraphReader& reader) {
  // no time to to track
  if (!location.has_date_time()) {
    return {false};
  }

  // get the timezone of the first edge's end node that we can
  int timezone_index = 0;
  for (const auto& pe : location.path_edges()) {
    const baldr::GraphTile* tile = nullptr;
    const auto* edge = reader.directededge(baldr::GraphId(pe.graph_id()), tile);
    timezone_index = reader.GetTimezone(edge->endnode(), tile);
    if (timezone_index != 0)
      break;
  }

  // Set the origin timezone to be the timezone at the end node
  if (timezone_index == 0) {
    LOG_ERROR("No timezone for location using default");
    timezone_index = 1;
  }
  const auto* tz = dt::get_tz_db().from_index(timezone_index);

  // Set the time for the current time route
  bool current = false;
  const auto date = date::make_zoned(tz, std::chrono::system_clock::now());
  if (location.date_time() == "current") {
    current = true;
    std::ostringstream iso_dt;
    iso_dt << date::format("%FT%R", date);
    location.set_date_time(iso_dt.str());
  }

  // Set route start time (seconds from epoch)
  uint64_t local_time = 0;
  try {
    local_time = dt::seconds_since_epoch(location.date_time(), tz);
  } catch (...) {
    LOG_ERROR("Could not get epoch seconds for date_time: " + location.date_time());
    return {false};
  }

  // Set seconds from beginning of the week
  std::tm t = dt::iso_to_tm(location.date_time());
  if (t.tm_year == 0) {
    LOG_ERROR("Could not parse date_time: " + location.date_time());
    return {false};
  }
  std::mktime(&t);
  auto second_of_week = t.tm_wday * valhalla::midgard::kSecondsPerDay +
                        t.tm_hour * valhalla::midgard::kSecondsPerHour + t.tm_sec;

  // TODO: compute the offset from now for this location whether negative or positive
  int64_t seconds_from_now = 0;

  // construct the info
  return {true, timezone_index, local_time, second_of_week, current, seconds_from_now};
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
  return {valid,   offset.timezone_index,
          lt,      sw,
          current, seconds_from_now + static_cast<int64_t>(offset.seconds)};
}

// offset all the initial time info to reflect the progress along the route to this point
TimeInfo TimeInfo::operator-(Offset offset) const {
  // if (!valid)
  //  return *this;

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
  return {valid,   offset.timezone_index,
          lt,      static_cast<uint32_t>(sw),
          current, seconds_from_now - static_cast<int64_t>(offset.seconds)};
}

} // namespace thor
} // namespace valhalla
