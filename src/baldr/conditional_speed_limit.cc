#include "baldr/conditional_speed_limit.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"

#include <iomanip>
#include <sstream>

namespace {
// Format dow mask into human-readable format. Week starts from Sunday in the mask
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

using namespace valhalla::baldr;
std::string month_name(MONTH month) {
  switch (month) {
    case MONTH::kJan:
      return "Jan";
    case MONTH::kFeb:
      return "Feb";
    case MONTH::kMar:
      return "Mar";
    case MONTH::kApr:
      return "Apr";
    case MONTH::kMay:
      return "May";
    case MONTH::kJun:
      return "Jun";
    case MONTH::kJul:
      return "Jul";
    case MONTH::kAug:
      return "Aug";
    case MONTH::kSep:
      return "Sep";
    case MONTH::kOct:
      return "Oct";
    case MONTH::kNov:
      return "Nov";
    case MONTH::kDec:
      return "Dec";
    default:
      return "";
  }
}

std::string dow_name(DOW dow) {
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

std::string ConditionalSpeedLimit::condition_str() const {
  std::ostringstream ss;

  const TimeDomain td(condition());

  // Handle begin_day_dow and end_day_dow type, day of the month or Nth weekday
  bool need_space = false;
  if (td.type() == kYMD) { // kYMD - use days of the month
    if (td.begin_month() != 0) {
      ss << month_name(static_cast<MONTH>(td.begin_month()));
      if (td.begin_day_dow() != 0) {
        ss << ' ' << static_cast<int>(td.begin_day_dow());
      }
      need_space = true;
    }
    if (td.end_month() != 0 &&
        (td.end_month() != td.begin_month() || td.end_day_dow() != td.begin_day_dow())) {
      if (td.end_month() != td.begin_month() || td.end_day_dow() != 0) {
        ss << '-';
      }
      if (td.end_month() != 0) {
        ss << month_name(static_cast<MONTH>(td.end_month()));
        if (td.end_day_dow() != 0) {
          ss << ' ' << static_cast<int>(td.end_day_dow());
        }
        need_space = true;
      }
    }
  } else { // kNthDow - use Nth weekday
    if (td.begin_month() != 0) {
      ss << month_name(static_cast<MONTH>(td.begin_month())) << ' ';
    }
    if (td.begin_week() != 0) {
      ss << static_cast<int>(td.begin_week()) << ' ';
    }
    ss << dow_name(static_cast<DOW>(td.begin_day_dow()));
    if (td.end_day_dow() != 0 &&
        (td.end_day_dow() != td.begin_day_dow() || td.end_week() != td.begin_week() ||
         td.end_month() != td.begin_month())) {
      ss << '-';
      if ((td.end_week() != 0)) {
        ss << static_cast<int>(td.end_week()) << ' ';
      }
      ss << dow_name(static_cast<DOW>(td.end_day_dow()));
    }
    need_space = true;
  }

  if (td.dow()) {
    if (need_space) {
      ss << ' ';
    }
    format_dow(td.dow(), ss);
    need_space = true;
  }

  // Handle time range
  if (td.begin_hrs() != 0 || td.end_hrs() != 0 || td.begin_mins() != 0 || td.end_mins() != 0) {
    if (need_space) {
      ss << ' ';
    }
    ss << std::setw(2) << std::setfill('0') << static_cast<int>(td.begin_hrs()) << ':' << std::setw(2)
       << std::setfill('0') << static_cast<int>(td.begin_mins());
    if (td.end_hrs() != 0 || td.end_mins() != 0) {
      ss << '-' << std::setw(2) << std::setfill('0') << static_cast<int>(td.end_hrs()) << ':'
         << std::setw(2) << std::setfill('0') << static_cast<int>(td.end_mins());
    }
  }

  return ss.str();
}

} // namespace baldr
} // namespace valhalla
