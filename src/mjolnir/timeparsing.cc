#include <ctime>
#include <regex>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "midgard/logging.h"
#include "mjolnir/timeparsing.h"

using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {
// Dec Su[-1]-Mar 3 => Dec#Su#[5]-Mar#3
const std::pair<std::regex, std::string> kBeginWeekdayOfTheMonth =
    {std::regex(
         "(?:(January|February|March|April|May|June|July|"
         "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
         "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
         "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\])-"
         "(?:(January|February|March|April|May|June|July|August|September|October|November"
         "|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|Nov|Dec)) (\\d{1,2}))",
         std::regex_constants::icase),
     "$1#$2#$3-$4#$5"};

// Mar 3-Dec Su[-1] => Mar#3-Dec#Su#[5]
const std::pair<std::regex, std::string> kEndWeedkayOfTheMonth =
    {std::regex(
         "(?:(January|February|March|April|May|June|July|August|"
         "September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Sept|Oct|"
         "Nov|Dec)) (\\d{1,2})-(?:(January|February|March|April|May|June|July|"
         "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
         "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
         "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]\\])"
         ")",
         std::regex_constants::icase),
     "$1#$2-$3#$4#$5"};

// Dec Su[-1] => Dec#Su#[5]
const std::pair<std::regex, std::string> kWeekdayOfTheMonth =
    {std::regex("(?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec)) (?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
                "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|Su)(\\[-?[0-9]"
                "\\]))",
                std::regex_constants::icase),
     "$1#$2#$3"};

// Mon[-1] => Mon#[5], Tue[2] => Tue#[2]
const std::pair<std::regex, std::string> kWeekdayOfEveryMonth =
    {std::regex("(?:(Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|"
                "Sunday|Mon|Mo|Tues|Tue|Tu|Weds|Wed|We|Thurs|Thur|Th|Fri|Fr|Sat|Sa|Sun|"
                "Su)(\\[-?[0-9]\\]))",
                std::regex_constants::icase),
     "$1#$2"};

// Feb 16-Oct 15 09:00-18:30 => Feb#16-Oct#15 09:00-18:30
const std::pair<std::regex, std::string> kMonthDay =
    {std::regex("(?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec)) (\\d{1,2})",
                std::regex_constants::icase),
     "$1#$2"};

// Feb 2-14 => Feb#2-Feb#14
const std::pair<std::regex, std::string> kRangeWithinMonth =
    {std::regex("(?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec)) (\\d{1,2})-(\\d{1,2})",
                std::regex_constants::icase),
     "$1#$2-$1#$3"};

// Nov - Mar => Nov-Mar
const std::pair<std::regex, std::string> kMonthRange =
    {std::regex("(?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec)) - (?:(January|February|March|April|May|June|July|"
                "August|September|October|November|December|Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|"
                "Sep|Sept|Oct|Nov|Dec))",
                std::regex_constants::icase),
     "$1-$2"};

// fifth is the equivalent of last week in month (-1)
const std::pair<std::regex, std::string> kLastWeekday = {std::regex("\\[-1\\]"), "[5]"};

std::vector<std::string> GetTokens(const std::string& tag_value, char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value,
                          std::bind(std::equal_to<char>(), delim, std::placeholders::_1),
                          boost::algorithm::token_compress_on);
  return tokens;
}

bool RegexFound(const std::string& source, const std::regex& regex) {
  auto begin = std::sregex_iterator(source.begin(), source.end(), regex);
  auto end = std::sregex_iterator();
  return std::distance(begin, end);
}

std::string FormatCondition(const std::string& source,
                            const std::pair<std::regex, std::string>& regex_pattern) {
  return std::regex_replace(source, regex_pattern.first, regex_pattern.second);
}

} // namespace

namespace valhalla {
namespace mjolnir {

// get the dow mask from the provided string.  try to handle most inputs
uint8_t get_dow_mask(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" || str == "SUN" || str == "SU") {
    return kSunday;

  } else if (str == "MONDAY" || str == "MON" || str == "MO") {
    return kMonday;

  } else if (str == "TUESDAY" || str == "TUES" || str == "TUE" || str == "TU") {
    return kTuesday;

  } else if (str == "WEDNESDAY" || str == "WEDS" || str == "WED" || str == "WE") {
    return kWednesday;

  } else if (str == "THURSDAY" || str == "THURS" || str == "THUR" || str == "TH") {
    return kThursday;

  } else if (str == "FRIDAY" || str == "FRI" || str == "FR") {
    return kFriday;

  } else if (str == "SATURDAY" || str == "SAT" || str == "SA") {
    return kSaturday;
  }
  return kDOWNone;
}

// get the dow from the provided string.  try to handle most inputs
DOW get_dow(const std::string& dow) {

  std::string str = dow;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "SUNDAY" || str == "SUN" || str == "SU") {
    return DOW::kSunday;

  } else if (str == "MONDAY" || str == "MON" || str == "MO") {
    return DOW::kMonday;

  } else if (str == "TUESDAY" || str == "TUES" || str == "TUE" || str == "TU") {
    return DOW::kTuesday;

  } else if (str == "WEDNESDAY" || str == "WEDS" || str == "WED" || str == "WE") {
    return DOW::kWednesday;

  } else if (str == "THURSDAY" || str == "THURS" || str == "THUR" || str == "TH") {
    return DOW::kThursday;

  } else if (str == "FRIDAY" || str == "FRI" || str == "FR") {
    return DOW::kFriday;

  } else if (str == "SATURDAY" || str == "SAT" || str == "SA") {
    return DOW::kSaturday;
  }
  return DOW::kNone;
}

// Get the month from the input string.Try to handle most inputs
MONTH get_month(const std::string& month) {

  std::string str = month;
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  str.erase(boost::remove_if(str, boost::is_any_of(":")), str.end());

  if (str == "JANUARY" || str == "JAN") {
    return MONTH::kJan;

  } else if (str == "FEBRUARY" || str == "FEB") {
    return MONTH::kFeb;

  } else if (str == "MARCH" || str == "MAR") {
    return MONTH::kMar;

  } else if (str == "APRIL" || str == "APR") {
    return MONTH::kApr;

  } else if (str == "MAY") {
    return MONTH::kMay;

  } else if (str == "JUNE" || str == "JUN") {
    return MONTH::kJun;

  } else if (str == "JULY" || str == "JUL") {
    return MONTH::kJul;

  } else if (str == "AUGUST" || str == "AUG") {
    return MONTH::kAug;

  } else if (str == "SEPTEMBER" || str == "SEP" || str == "SEPT") {
    return MONTH::kSep;

  } else if (str == "OCTOBER" || str == "OCT") {
    return MONTH::kOct;

  } else if (str == "NOVEMBER" || str == "NOV") {
    return MONTH::kNov;

  } else if (str == "DECEMBER" || str == "DEC") {
    return MONTH::kDec;
  }
  return MONTH::kNone;
}

std::vector<uint64_t> get_time_range(const std::string& str) {

  std::vector<uint64_t> time_domains;

  TimeDomain timedomain(0);
  // rm ()
  try {
    std::string condition = str;
    condition.erase(boost::remove_if(condition, boost::is_any_of("()")), condition.end());

    // rm white space at both ends
    boost::algorithm::trim(condition);

    // Holidays and school hours skip for now
    if (boost::algorithm::starts_with(condition, "PH") ||
        boost::algorithm::starts_with(condition, "SH")) {
      return time_domains;
    }

    // Dec Su[-1]-Mar 3 => Dec#Su#[5]-Mar#3
    if (RegexFound(condition, kBeginWeekdayOfTheMonth.first)) {
      condition = FormatCondition(condition, kBeginWeekdayOfTheMonth);
      condition = FormatCondition(condition, kLastWeekday);
    } else {

      // Mar 3-Dec Su[-1] => Mar#3-Dec#Su#[5]
      if (RegexFound(condition, kEndWeedkayOfTheMonth.first)) {
        condition = FormatCondition(condition, kEndWeedkayOfTheMonth);
        condition = FormatCondition(condition, kLastWeekday);
      } else {

        // Dec Su[-1] => Dec#Su#[5]
        if (RegexFound(condition, kWeekdayOfTheMonth.first)) {
          condition = FormatCondition(condition, kWeekdayOfTheMonth);
          condition = FormatCondition(condition, kLastWeekday);
        } else {

          // Mon[-1] => Mon#[5], Tue[2] => Tue#[2]
          if (RegexFound(condition, kWeekdayOfEveryMonth.first)) {
            condition = FormatCondition(condition, kWeekdayOfEveryMonth);
            condition = FormatCondition(condition, kLastWeekday);
          } else {

            // Feb 16-Oct 15 09:00-18:30 => Feb#16-Oct#15 09:00-18:30
            if (RegexFound(condition, kMonthDay.first)) {
              condition = FormatCondition(condition, kMonthDay);
            } else {

              // Feb 2-14 => Feb#2-Feb#14
              if (RegexFound(condition, kRangeWithinMonth.first)) {
                condition = FormatCondition(condition, kRangeWithinMonth);
              } else {

                // Nov - Mar => Nov-Mar
                if (RegexFound(condition, kMonthRange.first)) {
                  condition = FormatCondition(condition, kMonthRange);
                }
              }
            }
          }
        }
      }
    }

    std::size_t found = condition.find(",PH");
    if (found != std::string::npos)
      condition.erase(found, 3);

    found = condition.find("PH,");
    if (found != std::string::npos)
      condition.erase(found, 3);

    std::vector<std::string> months_dow_times = GetTokens(condition, ' ');

    if (months_dow_times.size() == 1 && condition.find('#') != std::string::npos &&
        std::count(condition.begin(), condition.end(), '#') == 1) {
      months_dow_times = GetTokens(condition, '#');
    }

    if (months_dow_times.size() == 1) {
      // no dow just times
      // 06:00-11:00,17:00-19:00
      if (months_dow_times.at(0).find('-') != std::string::npos &&
          months_dow_times.at(0).find(':') != std::string::npos) {

        std::vector<std::string> times = GetTokens(months_dow_times.at(0), ',');
        // is this data looking good enough to try to process?
        if (times.size()) {

        } else {
          return time_domains;
        }

        // multiple times are saved as multiple restrictions
        for (const auto& t : times) {

          std::vector<std::string> on_off = GetTokens(t, '-');

          // do we have an hour on and hour off?
          if (on_off.size() == 2) {

            // process the hour on
            std::size_t found = on_off.at(0).find(':');
            if (found == std::string::npos) {
              return time_domains;
            }

            std::stringstream stream(on_off.at(0));
            uint32_t hour, min;

            stream >> hour;
            stream.ignore();
            stream >> min;

            timedomain.set_begin_hrs(hour);
            timedomain.set_begin_mins(min);

            // process the hour off
            found = on_off.at(1).find(':');
            if (found == std::string::npos) {
              return time_domains;
            }

            stream.str("");
            stream.clear();
            stream.str(on_off.at(1));

            stream >> hour;
            stream.ignore();
            stream >> min;

            timedomain.set_end_hrs(hour);
            timedomain.set_end_mins(min);

            time_domains.push_back(timedomain.td_value());
          }
        }
        return time_domains;
      }
    }
    // Mo-Fr 06:00-11:00,17:00-19:00; Sa 03:30-19:00
    // Apr-Sep: Mo-Fr 09:00-13:00,14:00-18:00; Apr-Sep: Sa 10:00-13:00
    // Mo,We,Th,Fr 12:00-18:00; Sa-Su 12:00-17:00
    // Feb#16-Oct#15 09:00-18:30; Oct#16-Nov#15: 09:00-17:30; Nov#16-Feb#15: 09:00-16:30
    // and etc.
    for (auto& mdt : months_dow_times) {
      // rm white space at both ends
      boost::algorithm::trim(mdt);

      std::vector<std::string> months_dow;
      bool is_range = false;
      bool is_date = false;
      bool is_nth_week = false;
      bool ends_nth_week = false;

      if (mdt.find(',') != std::string::npos) {
        months_dow = GetTokens(mdt, ',');
      } else if (mdt.find('-') != std::string::npos) {
        months_dow = GetTokens(mdt, '-');
        is_range = true;

        if (months_dow.size() && mdt.find('#') != std::string::npos &&
            mdt.find('[') != std::string::npos && mdt.find(']') != std::string::npos) {
          is_date = true;
          is_nth_week = true;

          std::vector<std::string> tmp, result;
          for (auto& md : months_dow) {
            tmp = GetTokens(md, '#');
            result.insert(std::end(result), std::begin(tmp), std::end(tmp));
          }
          months_dow = result;
        }
        // Feb#16-Oct#15
        else if (months_dow.size() && mdt.find('#') != std::string::npos) {
          is_date = true;
          std::vector<std::string> tmp, result;
          for (auto& md : months_dow) {
            tmp = GetTokens(md, '#');
            result.insert(std::end(result), std::begin(tmp), std::end(tmp));
          }
          months_dow = result;
        }
        // Dec Su[-1] Su-Sa 15:00-17:00
      } else if (mdt.find('#') != std::string::npos && mdt.find('[') != std::string::npos &&
                 mdt.find(']') != std::string::npos) {
        is_date = true;
        is_nth_week = true;
        months_dow = GetTokens(mdt, '#');
      } else if (mdt.find('#') != std::string::npos) { // May#15
        is_date = true;
        months_dow = GetTokens(mdt, '#');
      } else {
        months_dow.push_back(mdt); // just one day: Th or month
      }

      // dealing with months?
      if (get_month(months_dow.at(0)) != MONTH::kNone) {
        for (auto& md : months_dow) {

          // Feb#16-Oct#15
          if (months_dow.size() == 4 && is_date && is_range) {
            timedomain.set_type(kYMD);
            timedomain.set_begin_month(static_cast<uint8_t>(get_month(months_dow.at(0))));
            timedomain.set_begin_day_dow(std::stoi(months_dow.at(1)));

            timedomain.set_end_month(static_cast<uint8_t>(get_month(months_dow.at(2))));
            timedomain.set_end_day_dow(std::stoi(months_dow.at(3)));

            break;
          } // May 16-31
          else if (months_dow.size() == 3 && is_date && is_range) {
            timedomain.set_type(kYMD);
            timedomain.set_begin_month(static_cast<uint8_t>(get_month(months_dow.at(0))));
            timedomain.set_begin_day_dow(std::stoi(months_dow.at(1)));

            timedomain.set_end_month(timedomain.begin_month());
            timedomain.set_end_day_dow(std::stoi(months_dow.at(2)));
            break;
          }
          // Apr-Sep or May 15
          else if (months_dow.size() == 2) {

            timedomain.set_begin_month(static_cast<uint8_t>(get_month(months_dow.at(0))));
            baldr::MONTH month = get_month(months_dow.at(1));

            if (month != MONTH::kNone) {
              timedomain.set_type(kYMD);
              timedomain.set_end_month(static_cast<uint8_t>(month));
            } else if (is_date) { // May 15
              timedomain.set_type(kYMD);
              timedomain.set_begin_day_dow(std::stoi(months_dow.at(1)));
              timedomain.set_end_month(timedomain.begin_month());
              timedomain.set_end_day_dow(timedomain.begin_day_dow());
            } else {
              return time_domains;
            }

            break;
          } else if (months_dow.size() == 1) { // May
            timedomain.set_type(kYMD);
            timedomain.set_begin_month(static_cast<uint8_t>(get_month(months_dow.at(0))));
            timedomain.set_end_month(static_cast<uint8_t>(get_month(months_dow.at(0))));
            break;
          } else if (is_nth_week) { // Oct Su[-1]-Mar Su[4] Su 09:00-16:00
            if (get_month(md) != MONTH::kNone) {

              timedomain.set_type(kNthDow);
              if (timedomain.begin_month() == 0) {

                // assume the restriction is the entire week.
                timedomain.set_dow(kAllDaysOfWeek);

                timedomain.set_begin_month(static_cast<uint8_t>(get_month(md)));
                // assume no range.  Dec Su[-1] Su-Sa 15:00-17:00 starts on the last week
                // in Dec and ends in the last week in Dec
                if (!is_range) {
                  timedomain.set_end_month(timedomain.begin_month());
                }
              } else {
                timedomain.set_end_month(static_cast<uint8_t>(get_month(md)));

                if (is_range && is_date &&
                    md != months_dow.at(months_dow.size() - 1)) { // Dec Su[-1]-Mar 3 Sat

                  if (months_dow.at(months_dow.size() - 1).find('[') == std::string::npos) {
                    timedomain.set_end_day_dow(std::stoi(months_dow.at(months_dow.size() - 1)));
                    break;
                  } else {
                    ends_nth_week = true;
                  }
                }
              }

            } else if (get_dow(md) != DOW::kNone) {

              if (timedomain.begin_day_dow() == 0) {
                timedomain.set_begin_day_dow(static_cast<uint8_t>(get_dow(md)));
              } else {
                timedomain.set_end_day_dow(static_cast<uint8_t>(get_dow(md)));
              }

            } else if (md.find('[') != std::string::npos && md.find(']') != std::string::npos) {
              md.erase(boost::remove_if(md, boost::is_any_of("[]")), md.end());

              if (timedomain.begin_week() == 0 && !ends_nth_week) {
                timedomain.set_begin_week(std::stoi(md));
                // assume no range.  Dec Su[-1] Su-Sa 15:00-17:00 starts on the last week
                // in Dec and ends in the last week in Dec
                if (!is_range) {
                  timedomain.set_end_week(timedomain.begin_week());
                }
              } else {
                timedomain.set_end_week(std::stoi(md));
              }
            } else if (is_date && is_range && timedomain.begin_month() != 0 &&
                       timedomain.end_month() == 0) { // Mar 3-Dec Su[-1] Sat
              timedomain.set_begin_day_dow(std::stoi(md));
            }
          }
        }
      }
      // dealing with dow
      else if (get_dow(months_dow.at(0)) != DOW::kNone) {
        // Mo,We,Th,Fr
        if (!is_range) {
          // wipe out assumption that this restriction is for the entire week.
          if (timedomain.type() == kNthDow) {
            timedomain.set_dow(0);
          }

          for (auto& md : months_dow) {
            timedomain.set_dow(timedomain.dow() + get_dow_mask(md));
          }

          if (months_dow_times.size() == 2) {
            std::string week = months_dow_times.at(1);
            // Su[1] every 1st Sunday of every month.
            if (week.find('[') != std::string::npos && week.find(']') != std::string::npos) {
              timedomain.set_type(kNthDow);
              week.erase(boost::remove_if(week, boost::is_any_of("[]")), week.end());
              timedomain.set_begin_week(std::stoi(week));
              break;
            }
          }
          // Mo-Fr
        } else if (months_dow.size() == 2) {
          // wipe out assumption that this restriction is for the entire week.
          if (timedomain.type() == kNthDow) {
            timedomain.set_dow(0);
          }

          uint8_t b_index = static_cast<uint8_t>(get_dow(months_dow.at(0)));
          uint8_t e_index = static_cast<uint8_t>(get_dow(months_dow.at(1)));

          if (b_index > e_index) { // Th - Tu

            while (b_index <= static_cast<uint8_t>(DOW::kSaturday)) {
              timedomain.set_dow(timedomain.dow() + (1 << (b_index - 1)));
              b_index++;
            }
            b_index = static_cast<uint8_t>(DOW::kSunday);
          }

          while (b_index <= e_index) {
            timedomain.set_dow(timedomain.dow() + (1 << (b_index - 1)));
            b_index++;
          }

        } else {
          return time_domains;
        }
      } else {

        std::vector<std::string> on_off;

        for (const auto& time : months_dow) {
          // is this data looking good enough to try to process?
          if (time.find('-') != std::string::npos && time.find(':') != std::string::npos) {

            // multiple times are saved as multiple restrictions
            on_off = GetTokens(time, '-');

          } else if (is_range && months_dow.size() == 2) {
            on_off.insert(std::end(on_off), std::begin(months_dow), std::end(months_dow));
          } else {
            continue;
          }

          // do we have an hour on and hour off?
          if (on_off.size() == 2) {

            // process the hour on
            std::size_t found = on_off.at(0).find(':');
            if (found == std::string::npos) {
              return time_domains;
            }

            std::stringstream stream(on_off.at(0));
            uint32_t hour, min;

            stream >> hour;
            stream.ignore();
            stream >> min;

            timedomain.set_begin_hrs(hour);
            timedomain.set_begin_mins(min);

            // process the hour off
            found = on_off.at(1).find(':');
            if (found == std::string::npos) {
              return time_domains;
            }

            stream.str("");
            stream.clear();
            stream.str(on_off.at(1));

            stream >> hour;
            stream.ignore();
            stream >> min;

            timedomain.set_end_hrs(hour);
            timedomain.set_end_mins(min);

            time_domains.push_back(timedomain.td_value());
          }
        }
      }
    }

    // no time.
    if (time_domains.size() == 0 && timedomain.td_value()) {
      time_domains.push_back(timedomain.td_value());
    }
  } catch (const std::invalid_argument& arg) {
    LOG_INFO("invalid_argument thrown for condition " + str);
  } catch (const std::out_of_range& oor) {
    LOG_INFO("out_of_range thrown for condition: " + str);
  } catch (const std::runtime_error& oor) {
    // TODO deal with these.  For now toss.
    // LOG_INFO("runtime_error thrown for condition: " + str);
  }
  return time_domains;
}

} // namespace mjolnir
} // namespace valhalla
