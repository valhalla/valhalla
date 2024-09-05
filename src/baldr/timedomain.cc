#include "baldr/timedomain.h"

#include <iomanip>
#include <sstream>

namespace {
// Format daterange.dow mask into human-readable format. Week starts from Sunday in the mask
// - 0b1000001 => Su,Sa
// - 0b0111110 => Mo-Fr
// - 0b0101110 => Mo-We,Fr
void format_dow(uint64_t dow_mask, std::ostringstream& ss) {
  const std::string days[] = {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"};

  // state machine that helps us to understand the range
  bool prev = false;
  bool preprev = false;
  // flag to track do we need a comma or not
  bool empty = true;

  for (int i = 0; i < 7; ++i) {
    const bool curr = dow_mask & (1 << i);

    if (curr && !prev) {
      // range start
      if (!empty) {
        ss << ',';
      }
      ss << days[i];
      empty = false;
    }

    if (!curr && prev && preprev) {
      // we passed the range end - print the previous day
      ss << '-' << days[i - 1];
    }

    preprev = prev;
    prev = curr;
  }

  // handle range ending after loop
  if (prev && preprev) {
    // we passed the range end - print the previous day
    ss << '-' << days[6];
  }
}

std::string month_name(valhalla::baldr::MONTH month) {
  using valhalla::baldr::MONTH;
  static const std::unordered_map<MONTH, std::string> month_name = {
      {MONTH::kJan, "Jan"}, {MONTH::kFeb, "Feb"}, {MONTH::kMar, "Mar"}, {MONTH::kApr, "Apr"},
      {MONTH::kMay, "May"}, {MONTH::kJun, "Jun"}, {MONTH::kJul, "Jul"}, {MONTH::kAug, "Aug"},
      {MONTH::kSep, "Sep"}, {MONTH::kOct, "Oct"}, {MONTH::kNov, "Nov"}, {MONTH::kDec, "Dec"},
  };
  auto it = month_name.find(month);
  if (it != month_name.end()) {
    return it->second;
  }
  return "";
}

std::string dow_name(valhalla::baldr::DOW dow) {
  using valhalla::baldr::DOW;
  switch (dow) {
    case DOW::kSunday:
      return "Su";
    case DOW::kMonday:
      return "Mo";
    case DOW::kTuesday:
      return "Tu";
    case DOW::kWednesday:
      return "We";
    case DOW::kThursday:
      return "Th";
    case DOW::kFriday:
      return "Fr";
    case DOW::kSaturday:
      return "Sa";
    default:
      return "";
  }
}
} // namespace

namespace valhalla {
namespace baldr {

std::string TimeDomain::to_string() const {
  std::ostringstream ss;

  // Handle daterange.begin_day_dow and daterange.end_day_dow type, day of the
  // month or Nth weekday
  bool need_space = false;
  if (daterange.type == kYMD) { // kYMD - use days of the month
    if (daterange.begin_month != 0) {
      ss << month_name(static_cast<MONTH>(daterange.begin_month));
      if (daterange.begin_day_dow != 0) {
        ss << ' ' << static_cast<int>(daterange.begin_day_dow);
      }
      need_space = true;
    }
    if (daterange.end_month != 0 && (daterange.end_month != daterange.begin_month ||
                                     daterange.end_day_dow != daterange.begin_day_dow)) {
      if (daterange.end_month != daterange.begin_month || daterange.end_day_dow != 0) {
        ss << '-';
      }
      if (daterange.end_month != 0) {
        ss << month_name(static_cast<MONTH>(daterange.end_month));
        if (daterange.end_day_dow != 0) {
          ss << ' ' << static_cast<int>(daterange.end_day_dow);
        }
        need_space = true;
      }
    }
  } else { // daterange.kNthDow - use Nth weekday
    if (daterange.begin_month != 0) {
      ss << month_name(static_cast<MONTH>(daterange.begin_month)) << ' ';
    }

    if (daterange.begin_week) {
      // Mo[1] or Su[-1]
      // 5 annotates the last week of the month
      const int begin_nth_week =
          (daterange.begin_week != 5) ? static_cast<int>(daterange.begin_week) : -1;
      ss << dow_name(static_cast<DOW>(daterange.begin_day_dow)) << '[' << begin_nth_week << ']';
    } else {
      // Regular date, like March 3-...
      ss << static_cast<int>(daterange.begin_day_dow);
    }

    if (daterange.end_day_dow != 0 && (daterange.end_day_dow != daterange.begin_day_dow ||
                                       daterange.end_week != daterange.begin_week ||
                                       daterange.end_month != daterange.begin_month)) {
      ss << '-';
      if (daterange.end_month != daterange.begin_month) {
        ss << month_name(static_cast<MONTH>(daterange.end_month)) << ' ';
      }

      if (daterange.end_week) {
        // Mo[1] or Su[-1]
        // 5 annotates the last week of the month
        const int end_nth_week =
            (daterange.end_week != 5) ? static_cast<int>(daterange.end_week) : -1;
        ss << dow_name(static_cast<DOW>(daterange.end_day_dow)) << '[' << end_nth_week << ']';
      } else {
        // Regular date, like ...-March 3
        ss << static_cast<int>(daterange.end_day_dow);
      }
    }
    need_space = true;
  }

  if (daterange.dow != 0 && daterange.dow != 0b1111111) {
    if (need_space) {
      ss << ' ';
    }
    format_dow(daterange.dow, ss);
    need_space = true;
  }

  // Handle time range
  if (daterange.begin_hrs != 0 || daterange.end_hrs != 0 || daterange.begin_mins != 0 ||
      daterange.end_mins != 0) {
    if (need_space) {
      ss << ' ';
    }
    ss << std::setw(2) << std::setfill('0') << static_cast<int>(daterange.begin_hrs) << ':'
       << std::setw(2) << std::setfill('0') << static_cast<int>(daterange.begin_mins);
    if (daterange.end_hrs != 0 || daterange.end_mins != 0) {
      ss << '-' << std::setw(2) << std::setfill('0') << static_cast<int>(daterange.end_hrs) << ':'
         << std::setw(2) << std::setfill('0') << static_cast<int>(daterange.end_mins);
    }
  }

  return ss.str();
}

} // namespace baldr
} // namespace valhalla
