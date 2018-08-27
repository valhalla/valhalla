#include <algorithm>
#include <bitset>
#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "baldr/datetime2.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "baldr/tz_private.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include "date_time_northamerica.h"
#include "date_time_zonespec.h"

using namespace valhalla::baldr;

namespace {

const date::local_seconds pivot_date_ = DateTime2::get_formatted_date(kPivotDate2);
}

namespace date {
namespace detail {
struct undocumented {
  explicit undocumented() = default;
};
} // namespace detail
} // namespace date

namespace date {
struct tzdb_list::undocumented_helper {
  static void push_front(tzdb_list& db_list, tzdb* tzdb) noexcept {
    db_list.push_front(tzdb);
  }
};
} // namespace date

namespace valhalla {
namespace baldr {
namespace DateTime2 {

tz_db_t::tz_db_t() : db(get_tzdb()) {
  // load up the tz data
  for (const auto& z : db.zones) {
    names.push_back(z.name());
  }
}

size_t tz_db_t::to_index(const std::string& zone) const {
  auto it = std::find(names.cbegin(), names.cend(), zone);
  if (it == names.cend()) {
    return 0;
  }
  return (it - names.cbegin()) + 1;
}

const date::time_zone* tz_db_t::from_index(size_t index) const {
  if (index < 1 || index > names.size()) {
    return nullptr;
  }
  return &db.zones[index - 1];
}

static std::unique_ptr<date::tzdb> init_tzdb();

const date::tzdb& get_tzdb() {
  return get_tzdb_list().front();
}

static date::tzdb_list create_tzdb() {
  date::tzdb_list tz_db;
  date::tzdb_list::undocumented_helper::push_front(tz_db, init_tzdb().release());
  return tz_db;
}

date::tzdb_list& get_tzdb_list() {
  static date::tzdb_list tz_db = create_tzdb();
  return tz_db;
}

const tz_db_t& get_tz_db() {
  // thread safe static initialization of global singleton
  static const tz_db_t tz_db;
  return tz_db;
}

static std::unique_ptr<date::tzdb> init_tzdb() {
  std::string line;
  bool continue_zone = false;
  std::unique_ptr<date::tzdb> db(new date::tzdb);

  std::string tz_data(date_time_northamerica, date_time_northamerica + date_time_northamerica_len);
  std::stringstream ss(tz_data);

  //  CONSTDATA char* const files[] = {"africa",       "antarctica",   "asia",    "australasia",
  //                                   "backward",     "etcetera",     "europe",  "pacificnew",
  //                                   "northamerica", "southamerica", "systemv", "leapseconds"};

  //  for (const auto& filename : files) {
  //  std::ifstream infile(path + filename);
  //  while (infile) {
  while (std::getline(ss, line)) {
    if (!line.empty() && line[0] != '#') {
      std::istringstream in(line);
      std::string word;
      in >> word;
      if (word == "Rule") {
        db->rules.push_back(date::detail::Rule(line));
        continue_zone = false;
      } else if (word == "Link") {
        db->links.push_back(date::link(line));
        continue_zone = false;
      } else if (word == "Leap") {
        db->leaps.push_back(date::leap(line, date::detail::undocumented{}));
        continue_zone = false;
      } else if (word == "Zone") {
        db->zones.push_back(date::time_zone(line, date::detail::undocumented{}));
        continue_zone = true;
      } else if (line[0] == '\t' && continue_zone) {
        db->zones.back().add(line);
      } else {
        std::cerr << line << '\n';
      }
    }
  }
  //  }
  std::sort(db->rules.begin(), db->rules.end());
  //  date::detail::Rule::split_overlaps(db->rules);
  std::sort(db->zones.begin(), db->zones.end());
  db->zones.shrink_to_fit();
  std::sort(db->links.begin(), db->links.end());
  db->links.shrink_to_fit();
  std::sort(db->leaps.begin(), db->leaps.end());
  db->leaps.shrink_to_fit();

  //#ifdef _WIN32
  //  std::string mapping_file = get_install() + folder_delimiter + "windowsZones.xml";
  //  db->mappings = load_timezone_mappings_from_xml_file(mapping_file);
  //  sort_zone_mappings(db->mappings);
  //#endif // _WIN32

  return db;
}

// get a formatted date.  date in the format of 2016-11-06T01:00
date::local_seconds get_formatted_date(const std::string& date) {
  std::istringstream in{date};
  date::local_seconds tp;
  in >> date::parse("%FT%R", tp);
  return tp;
}

// get a local_date_time with support for dst.  Assumes that we are moving
// forward in time.  e.g. depart at
// 2016-11-06T02:00 ---> 2016-11-06T01:00
date::zoned_seconds get_ldt(const date::local_seconds& d, const date::time_zone* tz) {
  // need try catch
  date::zoned_time<std::chrono::seconds> zt = date::make_zoned(tz, d, date::choose::latest);
  return zt;
}

// Get the number of days that have elapsed from the pivot date for the inputed date.
// date_time is in the format of 2015-05-06T08:00
uint32_t days_from_pivot_date(const date::local_seconds& date_time) {
  if (date_time <= pivot_date_) {
    return 0;
  }
  return static_cast<uint32_t>(date::floor<date::days>(date_time - pivot_date_).count());
}

// Get the current iso date and time.
std::string iso_date_time(const date::time_zone* time_zone) {
  std::ostringstream iso_date_time;
  const auto date = date::make_zoned(time_zone, std::chrono::system_clock::now());
  iso_date_time << date::format("%FT%R%z", date);
  std::string iso_date = iso_date_time.str();
  iso_date.insert(19, 1, ':');
  return iso_date;
}

// Get the seconds since epoch time is already adjusted based on TZ
uint64_t seconds_since_epoch(const std::string& date_time, const date::time_zone* time_zone) {
  if (date_time.empty()) {
    return 0;
  }
  const auto d = get_formatted_date(date_time);
  const auto utc = date::to_utc_time(get_ldt(d, time_zone).get_sys_time()); // supports leap sec.
  return static_cast<uint64_t>(utc.time_since_epoch().count());
}

// Get the difference between two timezones using the current time (seconds from epoch
// so that DST can be take into account). Returns the difference in seconds.
int timezone_diff(const bool is_depart_at,
                  const uint64_t seconds,
                  const date::time_zone* origin_tz,
                  const date::time_zone* dest_tz) {

  if (!origin_tz || !dest_tz || origin_tz == dest_tz) {
    return 0;
  }
  std::chrono::seconds dur(seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);

  const auto origin = date::make_zoned(origin_tz, tp);
  const auto dest = date::make_zoned(dest_tz, tp);

  auto duration = std::chrono::duration_cast<std::chrono::seconds>(origin.get_local_time() -
                                                                   dest.get_local_time());
  if (origin.get_info().offset < dest.get_info().offset) {
    return abs(duration.count());
  } else {
    return -1 * abs(duration.count());
  }
}

std::string seconds_to_date(const uint64_t seconds, const date::time_zone* tz) {

  std::string iso_date;
  if (seconds == 0 || !tz) {
    return iso_date;
  }

  std::chrono::seconds dur(seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  std::ostringstream iso_date_time;

  const auto date = date::make_zoned(tz, tp);
  iso_date_time << date::format("%FT%R%z", date);
  iso_date = iso_date_time.str();
  iso_date.insert(19, 1, ':');
  return iso_date;
}

// Get the date from seconds and timezone.
void seconds_to_date(const bool is_depart_at,
                     const uint64_t origin_seconds,
                     const uint64_t dest_seconds,
                     const date::time_zone* origin_tz,
                     const date::time_zone* dest_tz,
                     std::string& iso_origin,
                     std::string& iso_dest) {
  iso_origin = seconds_to_date(origin_seconds, origin_tz);
  iso_dest = seconds_to_date(dest_seconds, dest_tz);
}

// Get the dow mask
// date_time is in the format of 2015-05-06T08:00
uint32_t day_of_week_mask(const std::string& date_time) {
  date::local_seconds date;
  date = get_formatted_date(date_time);
  if (date < pivot_date_) {
    return kDOWNone;
  }
  auto ld = date::floor<date::days>(date);
  uint8_t wd = (date::weekday{ld} - date::Sunday).count();

  switch (wd) {
    case 0:
      return kSunday;
      break;
    case 1:
      return kMonday;
      break;
    case 2:
      return kTuesday;
      break;
    case 3:
      return kWednesday;
      break;
    case 4:
      return kThursday;
      break;
    case 5:
      return kFriday;
      break;
    case 6:
      return kSaturday;
      break;
  }
  return kDOWNone;
}

// add x seconds to a date_time and return a ISO date_time string.
// date_time is in the format of 2015-05-06T08:00
std::string
get_duration(const std::string& date_time, const uint32_t seconds, const date::time_zone* tz) {

  date::local_seconds date;
  date = get_formatted_date(date_time);
  if (date < pivot_date_)
    return "";

  std::chrono::seconds dur(seconds_since_epoch(date_time, tz) + seconds);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  std::ostringstream iso_date_time;

  const auto origin = date::make_zoned(tz, tp);
  iso_date_time << date::format("%FT%R%z %Z", origin);
  std::string iso_date = iso_date_time.str();
  iso_date.insert(19, 1, ':');
  return iso_date;
}

// does this date fall in the begin and end date range?
bool is_restricted(const bool type,
                   const uint8_t begin_hrs,
                   const uint8_t begin_mins,
                   const uint8_t end_hrs,
                   const uint8_t end_mins,
                   const uint8_t dow,
                   const uint8_t begin_week,
                   const uint8_t begin_month,
                   const uint8_t begin_day_dow,
                   const uint8_t end_week,
                   const uint8_t end_month,
                   const uint8_t end_day_dow,
                   const uint64_t current_time,
                   const date::time_zone* time_zone) {
  bool dow_in_range = true;
  bool dt_in_range = false;

  // auto begin_date = date::year_month_day(date::year(2002),date::month(3),date::day(2));

  // date::time_of_day()
  std::chrono::minutes b_td = std::chrono::hours(0);
  std::chrono::minutes e_td = std::chrono::hours(23) + std::chrono::minutes(59);

  std::chrono::seconds dur(current_time);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);

  const auto in_local_time = date::make_zoned(time_zone, tp);
  auto date = date::floor<date::days>(tp);
  auto d = date::year_month_day(date);
  std::chrono::minutes td = date::make_time(tp - date).minutes(); // Yields time_of_day type

  // try {
  boost::gregorian::date begin_date, end_date;

  // we have dow
  if (dow) {

    uint8_t wd = (date::weekday{date} - date::Sunday).count();
    uint8_t local_dow = 0;
    switch (wd) {
      case 0:
        local_dow = kSunday;
        break;
      case 1:
        local_dow = kMonday;
        break;
      case 2:
        local_dow = kTuesday;
        break;
      case 3:
        local_dow = kWednesday;
        break;
      case 4:
        local_dow = kThursday;
        break;
      case 5:
        local_dow = kFriday;
        break;
      case 6:
        local_dow = kSaturday;
        break;
      default:
        return false; // should never happen
        break;
    }
    dow_in_range = (dow & local_dow);
  }

  uint8_t b_month = begin_month;
  uint8_t e_month = end_month;
  uint8_t b_day_dow = begin_day_dow;
  uint8_t e_day_dow = end_day_dow;
  uint8_t b_week = begin_week;
  uint8_t e_week = end_week;

  if (type == kNthDow && begin_week && !begin_day_dow && !begin_month) { // Su[-1]
    b_month = unsigned(d.month());
  }
  if (type == kNthDow && end_week && !end_day_dow && !end_month) { // Su[-1]
    e_month = unsigned(d.month());
  }

  if (type == kNthDow && begin_week && !begin_day_dow && !begin_month && !end_week && !end_day_dow &&
      !end_month) { // only Su[-1] set in begin.
    // First Sunday of every month only.
    e_month = b_month;
    b_day_dow = e_day_dow = dow;
    e_week = b_week;
  } else if (type == kYMD && (b_month && e_month) &&
             (!b_day_dow && !e_day_dow)) { // Sep-Jun We 08:15-08:45

    b_day_dow = 1;
    date::year_month_day e_d = date::year_month_day(d.year(), date::month(e_month), date::day(1));
    e_day_dow = unsigned((date::year_month(e_d.year(), e_d.month()) / date::last).day());
  }

  // month only
  if (type == kYMD && (b_month && e_month) && (!b_day_dow && !e_day_dow && !b_week && !b_week) &&
      b_month == e_month) {

    dt_in_range = (b_month <= unsigned(d.month()) && unsigned(d.month()) <= e_month);

    if (begin_hrs || begin_mins || end_hrs || end_mins) {
      b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
      e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);
    }

    dt_in_range = (dt_in_range && (b_td <= td && td <= e_td));
    return (dow_in_range && dt_in_range);
  }
  /*
      } else if (type == kYMD && b_month && b_day_dow) {

        uint32_t e_year = d.year(), b_year = d.year();
        if (b_month == e_month) {
          if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
            e_year = d.year() + 1;
          }
        } else if (b_month > e_month) { // Oct 10 - Mar 3
          if (b_month > unsigned(d.month())) {
            b_year = d.year() - 1;
          } else {
            e_year = d.year() + 1;
          }
        }

        begin_date = date::year_month_day(b_year, b_month, b_day_dow);
        end_date = date::year_month_day(e_year, e_month, e_day_dow);

      } else if (type == kNthDow && b_month && b_day_dow && e_month &&
                 e_day_dow) { // kNthDow types can have a mix of ymd and nthdow. (e.g. Dec Su[-1]-Mar
                              // 3 Sat 15:00-17:00)

        uint32_t e_year = d.year(), b_year = d.year();
        if (b_month == e_month) {
          if (b_day_dow > e_day_dow) { // Mar 15 - Mar 1
            e_year = d.year() + 1;
          }
        } else if (b_month > e_month) { // Oct 10 - Mar 3
          if (b_month > unsigned(d.month())) {
            b_year = d.year() - 1;
          } else {
            e_year = d.year() + 1;
          }
        }

        if (b_week && b_week <= 5) { // kNthDow
          boost::gregorian::nth_day_of_the_week_in_month
              nthdow(static_cast<boost::gregorian::nth_day_of_the_week_in_month::week_num>(b_week),
                     b_day_dow - 1, b_month);
          begin_date = nthdow.get_date(b_year);
        } else { // YMD
          begin_date = date::year_month_day(b_year, b_month, b_day_dow);
        }

        if (e_week && e_week <= 5) { // kNthDow
          boost::gregorian::nth_day_of_the_week_in_month
              nthdow(static_cast<boost::gregorian::nth_day_of_the_week_in_month::week_num>(e_week),
                     e_day_dow - 1, e_month);
          end_date = nthdow.get_date(e_year);
        } else {                                                         // YMD
          end_date = date::year_month_day(e_year, e_month, e_day_dow); // Dec 5 to Mar 3
        }
      } else { // do we have just time?

        if (begin_hrs || begin_mins || end_hrs || end_mins) {
          b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
          e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);

          if (begin_hrs > end_hrs) { // 19:00 - 06:00
            dt_in_range = !(e_td <= td && td <= b_td);
          } else {
            dt_in_range = (b_td <= td && td <= e_td);
          }
        }
        return (dow_in_range && dt_in_range);
      }

      if (begin_hrs || begin_mins || end_hrs || end_mins) {
        b_td = std::chrono::hours(begin_hrs) + std::chrono::minutes(begin_mins);
        e_td = std::chrono::hours(end_hrs) + std::chrono::minutes(end_mins);
      }

      ///date::local_days{date::mar/13/2016} + std::chrono::hours{2} + b_td.

      auto b_in_local_time = date::make_zoned(time_zone,date::local_days(begin_date.year_month_day())
   + b_td); auto e_in_local_time =
   date::make_zoned(time_zone,date::local_days(end_date.year_month_day()) + e_td);

      dt_in_range = (b_in_local_time <= in_local_time &&
                     in_local_time <= e_in_local_time);

      bool time_in_range = false;

      if (begin_hrs > end_hrs) { // 19:00 - 06:00
        time_in_range = !(e_td <= td && td <= b_td);
      } else {
        time_in_range = (b_td <= td && td <= e_td);
      }

      dt_in_range = (dt_in_range && time_in_range);
   // } catch (std::exception& e) {}
  */
  return (dow_in_range && dt_in_range);
}

} // namespace DateTime2
} // namespace baldr
} // namespace valhalla
