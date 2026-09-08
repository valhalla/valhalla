#include "mjolnir/timeparsing.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "mjolnir/util.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

// A single lexical piece of a condition. Tokenizing first keeps the grammar rules readable
// and tolerant to the free form spacing and punctuation mappers use.
enum class TokenKind : uint8_t {
  kMonth,   // value is 1 (January) to 12 (December)
  kWeekday, // value is 1 (Sunday) to 7 (Saturday), as baldr::DOW
  kNumber,  // a bare number, e.g. a day of the month
  kYear,    // four digits in [1900, 3000), makes a date range temporary
  kTime,    // value is minutes since midnight
  kNth,     // [n] or [-n], the nth weekday of a month, negative counts from the end
  kDash,
  kComma,
  kSemicolon,
  kPlus,
  kAlways,   // 24/7
  kHoliday,  // PH or SH
  kSunEvent, // sunrise, sunset, dawn or dusk
  kOff,      // off or closed
  kNoise,    // filler words like "and", skipped entirely
  kUnknown,  // anything else, e.g. a `weight>3.5` qualifier, skipped within a rule
  kEnd,
};

struct Token {
  TokenKind kind;
  int32_t value;
};

// Month and weekday names with common mapper variations, holidays and keywords
const std::unordered_map<std::string_view, Token> kWords = {
    {"jan", {TokenKind::kMonth, 1}},        {"january", {TokenKind::kMonth, 1}},
    {"feb", {TokenKind::kMonth, 2}},        {"february", {TokenKind::kMonth, 2}},
    {"mar", {TokenKind::kMonth, 3}},        {"march", {TokenKind::kMonth, 3}},
    {"apr", {TokenKind::kMonth, 4}},        {"april", {TokenKind::kMonth, 4}},
    {"may", {TokenKind::kMonth, 5}},        {"jun", {TokenKind::kMonth, 6}},
    {"june", {TokenKind::kMonth, 6}},       {"jul", {TokenKind::kMonth, 7}},
    {"july", {TokenKind::kMonth, 7}},       {"aug", {TokenKind::kMonth, 8}},
    {"august", {TokenKind::kMonth, 8}},     {"sep", {TokenKind::kMonth, 9}},
    {"sept", {TokenKind::kMonth, 9}},       {"september", {TokenKind::kMonth, 9}},
    {"oct", {TokenKind::kMonth, 10}},       {"october", {TokenKind::kMonth, 10}},
    {"nov", {TokenKind::kMonth, 11}},       {"november", {TokenKind::kMonth, 11}},
    {"dec", {TokenKind::kMonth, 12}},       {"december", {TokenKind::kMonth, 12}},

    {"su", {TokenKind::kWeekday, 1}},       {"sun", {TokenKind::kWeekday, 1}},
    {"sunday", {TokenKind::kWeekday, 1}},   {"mo", {TokenKind::kWeekday, 2}},
    {"mon", {TokenKind::kWeekday, 2}},      {"monday", {TokenKind::kWeekday, 2}},
    {"tu", {TokenKind::kWeekday, 3}},       {"tue", {TokenKind::kWeekday, 3}},
    {"tues", {TokenKind::kWeekday, 3}},     {"tuesday", {TokenKind::kWeekday, 3}},
    {"we", {TokenKind::kWeekday, 4}},       {"wed", {TokenKind::kWeekday, 4}},
    {"weds", {TokenKind::kWeekday, 4}},     {"wednesday", {TokenKind::kWeekday, 4}},
    {"th", {TokenKind::kWeekday, 5}},       {"thu", {TokenKind::kWeekday, 5}},
    {"thur", {TokenKind::kWeekday, 5}},     {"thurs", {TokenKind::kWeekday, 5}},
    {"thursday", {TokenKind::kWeekday, 5}}, {"fr", {TokenKind::kWeekday, 6}},
    {"fri", {TokenKind::kWeekday, 6}},      {"friday", {TokenKind::kWeekday, 6}},
    {"sa", {TokenKind::kWeekday, 7}},       {"sat", {TokenKind::kWeekday, 7}},
    {"saturday", {TokenKind::kWeekday, 7}},

    {"ph", {TokenKind::kHoliday, 0}},       {"sh", {TokenKind::kHoliday, 0}},
    {"off", {TokenKind::kOff, 0}},          {"closed", {TokenKind::kOff, 0}},
    {"sunrise", {TokenKind::kSunEvent, 0}}, {"sunset", {TokenKind::kSunEvent, 0}},
    {"dawn", {TokenKind::kSunEvent, 0}},    {"dusk", {TokenKind::kSunEvent, 0}},
    {"and", {TokenKind::kNoise, 0}},
};

// Recognizes a word of the condition regardless of its case
Token word_token(std::string_view word) {
  // longest recognized word is "wednesday"
  char buffer[10];
  if (word.size() >= sizeof(buffer)) {
    return {TokenKind::kUnknown, 0};
  }
  std::transform(word.begin(), word.end(), buffer, [](char c) { return std::tolower(c); });

  auto found = kWords.find(std::string_view(buffer, word.size()));
  return found != kWords.end() ? found->second : Token{TokenKind::kUnknown, 0};
}

std::vector<Token> tokenize(std::string_view str) {
  std::vector<Token> tokens;
  size_t i = 0;
  const size_t n = str.size();
  while (i < n) {
    const char c = str[i];
    // parens and stray colons carry no meaning
    if (c == ' ' || c == '\t' || c == '(' || c == ')' || c == ':') {
      ++i;
    } else if (static_cast<unsigned char>(c) >= 0x80) {
      // a byte outside ascii opens a utf-8 sequence and mappers use a handful of them: en dash,
      // em dash and the minus sign all stand for a dash, a non breaking space for a space
      if (str.compare(i, 3, "\xE2\x80\x93") == 0 || str.compare(i, 3, "\xE2\x80\x94") == 0 ||
          str.compare(i, 3, "\xE2\x88\x92") == 0) {
        tokens.push_back({TokenKind::kDash, 0});
        i += 3;
      } else if (str.compare(i, 2, "\xC2\xA0") == 0) {
        i += 2;
      } else {
        tokens.push_back({TokenKind::kUnknown, 0});
        ++i;
      }
    } else if (c == '-') {
      tokens.push_back({TokenKind::kDash, 0});
      ++i;
    } else if (c == ',') {
      tokens.push_back({TokenKind::kComma, 0});
      ++i;
    } else if (c == ';' || c == '|') {
      // a `||` fallback rule is close enough to a rule of its own
      tokens.push_back({TokenKind::kSemicolon, 0});
      ++i;
    } else if (c == '+') {
      tokens.push_back({TokenKind::kPlus, 0});
      ++i;
    } else if (c == '"') { // a quoted comment carries nothing to parse
      i = str.find('"', i + 1);
      i = (i == std::string_view::npos) ? n : i + 1;
    } else if (c == '&' && str.compare(i, 6, "&quot;") == 0) {
      i = str.find("&quot;", i + 6);
      i = (i == std::string_view::npos) ? n : i + 6;
    } else if (c == '[') {
      // [n] or [-n]
      size_t j = i + 1;
      bool negative = (j < n && str[j] == '-');
      j += negative;
      int32_t value = 0;
      size_t digits = 0;
      while (j < n && std::isdigit(static_cast<unsigned char>(str[j])) && digits < 2) {
        value = value * 10 + (str[j] - '0');
        ++j, ++digits;
      }
      if (digits > 0 && j < n && str[j] == ']') {
        tokens.push_back({TokenKind::kNth, negative ? -value : value});
        i = j + 1;
      } else {
        // an unreadable group, e.g. Su[1,-1]. Drop it whole, its pieces must not read as
        // selectors of their own
        const size_t close = str.find(']', i + 1);
        i = (close != std::string_view::npos && close - i <= 8) ? close + 1 : i + 1;
        tokens.push_back({TokenKind::kUnknown, 0});
      }
    } else if (std::isdigit(static_cast<unsigned char>(c))) {
      int32_t value = 0;
      size_t digits = 0;
      while (i < n && std::isdigit(static_cast<unsigned char>(str[i])) && digits < 6) {
        value = value * 10 + (str[i] - '0');
        ++i, ++digits;
      }
      if (i < n && str[i] == ':' && i + 1 < n &&
          std::isdigit(static_cast<unsigned char>(str[i + 1]))) {
        // hh:mm
        int32_t minutes = 0;
        size_t mm_digits = 0;
        ++i;
        while (i < n && std::isdigit(static_cast<unsigned char>(str[i])) && mm_digits < 2) {
          minutes = minutes * 10 + (str[i] - '0');
          ++i, ++mm_digits;
        }
        // a 12 hour clock, e.g. 04:00pm. A suffix that can't apply to the hour is only dropped
        if (i + 1 < n && (str[i] == 'a' || str[i] == 'A' || str[i] == 'p' || str[i] == 'P') &&
            (str[i + 1] == 'm' || str[i + 1] == 'M')) {
          if (value >= 1 && value <= 12) {
            value = (value % 12) + ((str[i] == 'p' || str[i] == 'P') ? 12 : 0);
          }
          i += 2;
        }
        if (value <= 48 && minutes <= 59) {
          tokens.push_back({TokenKind::kTime, value * 60 + minutes});
        } else {
          tokens.push_back({TokenKind::kUnknown, 0});
        }
      } else if (value == 24 && i + 1 < n && str[i] == '/' && str[i + 1] == '7' &&
                 (i + 2 == n || !std::isdigit(static_cast<unsigned char>(str[i + 2])))) {
        tokens.push_back({TokenKind::kAlways, 0});
        i += 2;
      } else if (digits == 4 && value >= 1900 && value < 3000) {
        tokens.push_back({TokenKind::kYear, value});
      } else {
        tokens.push_back({TokenKind::kNumber, value});
      }
    } else if (std::isalpha(static_cast<unsigned char>(c))) {
      size_t j = i;
      while (j < n && std::isalpha(static_cast<unsigned char>(str[j]))) {
        ++j;
      }
      const Token token = word_token(str.substr(i, j - i));
      if (token.kind != TokenKind::kNoise) {
        tokens.push_back(token);
      }
      i = j;
    } else {
      tokens.push_back({TokenKind::kUnknown, 0});
      ++i;
    }
  }
  return tokens;
}

// dow mask bit for a DOW value: Sunday (1) -> kSunday (1), Saturday (7) -> kSaturday (64)
uint8_t dow_mask_bit(int32_t dow) {
  return 1 << (dow - 1);
}

// A rule either parses in full, uses something TimeDomain can't represent (years, sun
// events, semantic inversion with "off"), or is not a valid time condition at all
enum class RuleResult : uint8_t { kOk, kUnsupported, kFailed };

// The parse position within the token stream, shared by the rule parsing functions below
struct TokenCursor {
  const std::vector<Token>& tokens;
  size_t pos = 0;

  TokenKind kind(size_t ahead = 0) const {
    return pos + ahead < tokens.size() ? tokens[pos + ahead].kind : TokenKind::kEnd;
  }

  int32_t value() const {
    return tokens[pos].value;
  }

  int32_t eat() {
    return tokens[pos++].value;
  }
};

bool at_rule_end(const TokenCursor& t) {
  return t.kind() == TokenKind::kSemicolon || t.kind() == TokenKind::kComma ||
         t.kind() == TokenKind::kEnd;
}

bool part_of_selector(TokenKind kind) {
  return kind == TokenKind::kWeekday || kind == TokenKind::kMonth || kind == TokenKind::kNumber ||
         kind == TokenKind::kYear || kind == TokenKind::kNth || kind == TokenKind::kHoliday ||
         kind == TokenKind::kComma || kind == TokenKind::kDash;
}

// A selector followed by times of its own, by 24/7 or by an `off` is a rule of its own, e.g. the Sa
// of `Mo-Fr 18:00-11:00 AND Sa 00:00-10:00`, while the Mo-Fr of `08:00-20:00 Mo-Fr` states the
// weekdays of the rule that precedes it
bool starts_another_rule(const TokenCursor& t) {
  size_t ahead = 0;
  while (part_of_selector(t.kind(ahead))) {
    ++ahead;
  }
  const TokenKind kind = t.kind(ahead);
  return kind == TokenKind::kTime || kind == TokenKind::kAlways || kind == TokenKind::kOff;
}

// consume the rest of the rule, the next one starts after a semicolon
void skip_rule(TokenCursor& t) {
  while (t.kind() != TokenKind::kSemicolon && t.kind() != TokenKind::kEnd) {
    ++t.pos;
  }
}

// A date point within a range: either a day of the month or the nth weekday of it
struct DatePoint {
  int32_t month = 0;
  int32_t day = 0;
  int32_t weekday = 0;
  int32_t week = 0;
  int32_t year = 0;
};

struct DateRange {
  DatePoint begin;
  DatePoint end;
};

struct TimeRange {
  int32_t begin = 0;
  int32_t end = 0;
};

// The weekday selector of a rule, e.g. Mo-Fr or Su[1]
struct Weekdays {
  uint8_t dow = 0;
  uint8_t week = 0;     // nth weekday of every month, from a Su[1] style selector
  bool nth_dow = false; // whether that selector was there, [1] and no week are both 1
};

bool starts_dates(const TokenCursor& t, size_t ahead = 0) {
  return t.kind(ahead) == TokenKind::kMonth ||
         (t.kind(ahead) == TokenKind::kYear && t.kind(ahead + 1) == TokenKind::kMonth) ||
         // a day in front of its month, e.g. 26 November
         (t.kind(ahead) == TokenKind::kNumber && t.kind(ahead + 1) == TokenKind::kMonth);
}

// Tokens a TimeDomain can't hold: a qualifier on top of the time, e.g. `weight>3.5 AND
// 20:00-06:00`, or a holiday. Skipping them leaves the rule covering more than the tag says
bool is_ignorable(const TokenCursor& t, size_t ahead = 0) {
  const TokenKind kind = t.kind(ahead);
  return kind == TokenKind::kUnknown || kind == TokenKind::kNumber || kind == TokenKind::kHoliday;
}

RuleResult parse_date_point(TokenCursor& t, DatePoint& point, bool expect_month) {
  // a year is written on either side of the date, e.g. 2025 Feb 15 or Feb 15 2025
  if (expect_month) {
    if (t.kind() == TokenKind::kYear) {
      point.year = t.eat();
    }
    // the day comes in front of its month too, e.g. 26 November
    if (t.kind() == TokenKind::kNumber && t.kind(1) == TokenKind::kMonth) {
      if (t.value() < 1 || t.value() > 31) {
        return RuleResult::kFailed;
      }
      point.day = t.eat();
    }
    if (t.kind() != TokenKind::kMonth) {
      return RuleResult::kFailed;
    }
    point.month = t.eat();
    if (point.day != 0) {
      return RuleResult::kOk;
    }
  }
  if (t.kind() == TokenKind::kNumber) {
    if (t.value() < 1 || t.value() > 31) {
      return RuleResult::kFailed;
    }
    point.day = t.eat();
    if (t.kind() == TokenKind::kYear) {
      point.year = t.eat();
    }
  } else if (t.kind() == TokenKind::kWeekday && t.kind(1) == TokenKind::kNth) {
    point.weekday = t.eat();
    const int32_t nth = t.eat();
    if (nth >= 1 && nth <= 5) {
      point.week = nth;
    } else if (nth == -1) { // the last week of the month
      point.week = 5;
    } else {
      return RuleResult::kUnsupported;
    }
  }
  return RuleResult::kOk;
}

RuleResult parse_date_range(TokenCursor& t, DateRange& range) {
  RuleResult result = parse_date_point(t, range.begin, true);
  if (result != RuleResult::kOk) {
    return result;
  }

  if (t.kind() == TokenKind::kDash) {
    ++t.pos;
    if (starts_dates(t)) {
      result = parse_date_point(t, range.end, true);
    } else if (t.kind() == TokenKind::kNumber && range.begin.day != 0) {
      // a range within one month, e.g. May 16-31
      result = parse_date_point(t, range.end, false);
      range.end.month = range.begin.month;
    } else {
      return RuleResult::kFailed;
    }
    if (result != RuleResult::kOk) {
      return result;
    }
  } else {
    // a single point spans onto itself, e.g. Dec or May 15 or Dec Su[-1]
    range.end.month = range.begin.month;
    range.end.day = range.begin.day;
    range.end.week = range.begin.week;
  }

  // TimeDomain repeats a range every year, so a year on both ends bounds it to an interval it
  // cannot hold: a multi year closure stored yearly is even open for the months in between
  if (range.begin.year != 0 && range.end.year != 0) {
    return RuleResult::kUnsupported;
  }
  return RuleResult::kOk;
}

RuleResult parse_dates(TokenCursor& t, std::vector<DateRange>& dates) {
  while (true) {
    DateRange range;
    const RuleResult result = parse_date_range(t, range);
    if (result != RuleResult::kOk) {
      return result;
    }
    dates.push_back(range);

    // a list of dates, e.g. Jan 01,Apr 19,Dec 25 or Jan 07-Jul 14,Sep 01-Dec 19
    if (t.kind() == TokenKind::kComma && starts_dates(t, 1)) {
      ++t.pos;
      continue;
    }
    return RuleResult::kOk;
  }
}

RuleResult parse_weekdays(TokenCursor& t, Weekdays& weekdays) {
  while (true) {
    const int32_t first = t.eat();
    weekdays.dow |= dow_mask_bit(first);

    if (t.kind() == TokenKind::kNth) {
      // Su[1] is the first Sunday of every month
      const int32_t nth = t.eat();
      if (nth == -1) {
        weekdays.week = 5;
      } else if (nth >= 1 && nth <= 5) {
        weekdays.week = nth;
      } else {
        return RuleResult::kUnsupported;
      }
      weekdays.nth_dow = true;
    } else if (t.kind() == TokenKind::kDash) {
      ++t.pos;
      if (t.kind() != TokenKind::kWeekday) {
        return RuleResult::kFailed;
      }
      int32_t from = first;
      const int32_t to = t.eat();
      if (from > to) { // Th-Tu wraps around the end of the week
        for (; from <= 7; ++from) {
          weekdays.dow |= dow_mask_bit(from);
        }
        from = 1;
      }
      for (; from <= to; ++from) {
        weekdays.dow |= dow_mask_bit(from);
      }
    }

    // a holiday in the list has no dates to resolve, but it must not end the list either
    while (t.kind() == TokenKind::kComma && t.kind(1) == TokenKind::kHoliday) {
      t.pos += 2;
    }
    // a list mixes single days and ranges, e.g. Mo-Tu,Th-Fr
    if (t.kind() == TokenKind::kComma && t.kind(1) == TokenKind::kWeekday) {
      ++t.pos;
      continue;
    }
    return RuleResult::kOk;
  }
}

RuleResult parse_times(TokenCursor& t, std::vector<TimeRange>& times) {
  while (true) {
    const int32_t begin = t.eat();
    int32_t end;
    if (t.kind() == TokenKind::kDash) {
      ++t.pos;
      if (t.kind() == TokenKind::kSunEvent) {
        return RuleResult::kUnsupported;
      }
      if (t.kind() != TokenKind::kTime) {
        return RuleResult::kFailed;
      }
      end = t.eat();
    } else if (t.kind() == TokenKind::kPlus) {
      // an open ended time lasts until the end of the day
      ++t.pos;
      end = 24 * 60;
    } else {
      return RuleResult::kFailed;
    }
    // an hour past midnight belongs to the next day, e.g. 20:00-26:00 is 20:00-02:00. A range
    // longer than a day cannot repeat daily, which is all TimeDomain can do
    if (end - begin > 24 * 60) {
      return RuleResult::kUnsupported;
    }
    times.push_back({begin % (24 * 60), end % (24 * 60)});

    // ranges are usually comma separated, but a bare space between them happens too
    if (t.kind() == TokenKind::kTime) {
      continue;
    }
    if (t.kind() == TokenKind::kComma && t.kind(1) == TokenKind::kTime) {
      ++t.pos;
      continue;
    }
    return RuleResult::kOk;
  }
}

// TimeDomain holds one date and one time range, so a rule naming several of either becomes one
// restriction per combination
void emit(const Weekdays& weekdays,
          const DateRange& date,
          const std::vector<TimeRange>& times,
          std::vector<uint64_t>& time_domains) {
  const bool nth_date = date.begin.weekday != 0 || date.end.weekday != 0;

  TimeDomain td(0);
  // the type has to be set first, it decides how the day fields are validated
  if (nth_date || weekdays.nth_dow) {
    td.set_type(kNthDow);
  }
  // an nth weekday range with no weekday selector of its own covers the entire week
  td.set_dow(nth_date && weekdays.dow == 0 ? kAllDaysOfWeek : weekdays.dow);
  td.set_begin_month(date.begin.month);
  td.set_end_month(date.end.month);
  if (nth_date) {
    td.set_begin_day_dow(date.begin.weekday != 0 ? date.begin.weekday : date.begin.day);
    td.set_end_day_dow(date.end.weekday != 0 ? date.end.weekday : date.end.day);
    td.set_begin_week(date.begin.week);
    td.set_end_week(date.end.week);
  } else {
    td.set_begin_day_dow(date.begin.day);
    td.set_end_day_dow(date.end.day);
    td.set_begin_week(weekdays.week);
  }

  if (times.empty()) {
    time_domains.push_back(td.td_value());
    return;
  }
  for (const TimeRange& time : times) {
    TimeDomain range = td;
    range.set_begin_hrs(time.begin / 60);
    range.set_begin_mins(time.begin % 60);
    range.set_end_hrs(time.end / 60);
    range.set_end_mins(time.end % 60);
    time_domains.push_back(range.td_value());
  }
}

RuleResult parse_rule(TokenCursor& t, std::vector<uint64_t>& time_domains) {
  // a rule about public or school holidays alone has no dates to resolve, drop it in one piece.
  // Next to other selectors a holiday is merely ignored, see is_ignorable
  if (t.kind() == TokenKind::kHoliday &&
      !(t.kind(1) == TokenKind::kComma && t.kind(2) == TokenKind::kWeekday)) {
    skip_rule(t);
    return RuleResult::kOk;
  }

  if (t.kind() == TokenKind::kSunEvent) {
    return RuleResult::kUnsupported;
  }

  Weekdays weekdays;
  std::vector<DateRange> dates;
  std::vector<TimeRange> times;
  // dates and weekdays come in either order, e.g. Su,Mo Jul 16-Sep 04
  while (true) {
    RuleResult result;
    if (starts_dates(t)) {
      result = parse_dates(t, dates);
    } else if (t.kind() == TokenKind::kWeekday) {
      result = parse_weekdays(t, weekdays);
    } else if (t.kind() == TokenKind::kComma && times.empty() &&
               (t.kind(1) == TokenKind::kTime ||
                (t.kind(1) == TokenKind::kWeekday && weekdays.dow == 0) ||
                (starts_dates(t, 1) && dates.empty()))) {
      // mappers also put a comma between the selectors of one rule, e.g. `Oct-Mar, 07:00-19:00`
      // or `Su, Jul-Aug`. Only a selector the rule still lacks can be meant that way
      ++t.pos;
      continue;
    } else if (is_ignorable(t) || (t.kind() == TokenKind::kComma && is_ignorable(t, 1))) {
      // a comma inside a run of them is part of the same list, e.g. `motorcar,moped Mo-Sa 07:30`
      ++t.pos;
      continue;
    } else {
      break;
    }
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // 24/7 always holds: alone it means the whole week, after a date range it adds nothing
  if (t.kind() == TokenKind::kAlways) {
    ++t.pos;
    if (dates.empty() && weekdays.dow == 0) {
      weekdays.dow = kAllDaysOfWeek;
    }
  }

  if (t.kind() == TokenKind::kTime) {
    const RuleResult result = parse_times(t, times);
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // mappers also write the selector behind the times, e.g. `08:00-20:00 Mo-Fr`. As with a comma
  // between selectors, only one the rule still lacks can be meant that way
  while (!starts_another_rule(t) && ((t.kind() == TokenKind::kWeekday && weekdays.dow == 0) ||
                                     (starts_dates(t) && dates.empty()))) {
    const RuleResult result =
        t.kind() == TokenKind::kWeekday ? parse_weekdays(t, weekdays) : parse_dates(t, dates);
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // "off" inverts the meaning of the rule, which the callers can't represent
  if (t.kind() == TokenKind::kOff) {
    return RuleResult::kUnsupported;
  }

  if (dates.empty() && times.empty() && weekdays.dow == 0) {
    return RuleResult::kFailed;
  }

  if (!at_rule_end(t)) {
    // trailing junk voids the rule unless the times have already been parsed: unquoted
    // free text comments are common enough to tolerate
    if (times.empty()) {
      return RuleResult::kFailed;
    }
    // a date, weekday or time here starts another rule, e.g. Mo-Fr 18:00-11:00 AND Sa
    // 00:00-10:00, anything else is a tail comment
    if (t.kind() != TokenKind::kWeekday && t.kind() != TokenKind::kMonth &&
        t.kind() != TokenKind::kTime) {
      skip_rule(t);
    }
  }

  if (dates.empty()) {
    // no date selector, so the rule spans every month, which a default DateRange already says
    emit(weekdays, DateRange(), times, time_domains);
  } else {
    for (const DateRange& date : dates) {
      emit(weekdays, date, times, time_domains);
    }
  }
  return RuleResult::kOk;
}

std::vector<uint64_t> parse_conditions(const std::vector<Token>& tokens) {
  std::vector<uint64_t> time_domains;
  TokenCursor t{tokens};
  while (t.kind() != TokenKind::kEnd) {
    if (t.kind() == TokenKind::kSemicolon || t.kind() == TokenKind::kComma) {
      ++t.pos; // an empty rule or a separator the previous rule stopped at
      continue;
    }
    const size_t rule_start = time_domains.size();
    RuleResult result;
    try {
      result = parse_rule(t, time_domains);
    } catch (...) {
      // TimeDomain setters throw on out of range values
      result = RuleResult::kFailed;
    }
    if (result != RuleResult::kOk) {
      time_domains.resize(rule_start);
      build_stats::get().increment(result == RuleResult::kFailed
                                       ? build_stats::kFailedOSMTimeRange
                                       : build_stats::kUnsupportedOSMTimeRange);
      skip_rule(t);
    }
  }
  return time_domains;
}

} // namespace

namespace valhalla {
namespace mjolnir {

std::vector<uint64_t> get_time_range(std::string_view str) {
  return parse_conditions(tokenize(str));
}

} // namespace mjolnir
} // namespace valhalla
