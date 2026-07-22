#include "mjolnir/timeparsing.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "mjolnir/util.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
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
  kNumber,  // a bare number, e.g. a day of the month or a year
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
  kUnknown,  // anything else, fails the rule it appears in
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
    // parens and stray colons carry no meaning, quoted comments are skipped entirely
    if (c == ' ' || c == '\t' || c == '(' || c == ')' || c == ':') {
      ++i;
    } else if (c == '"') {
      i = str.find('"', i + 1);
      i = (i == std::string_view::npos) ? n : i + 1;
    } else if (str.compare(i, 6, "&quot;") == 0) {
      i = str.find("&quot;", i + 6);
      i = (i == std::string_view::npos) ? n : i + 6;
    } else if (c == '-') {
      tokens.push_back({TokenKind::kDash, 0});
      ++i;
    } else if (str.compare(i, 3, "\xE2\x80\x93") == 0 || str.compare(i, 3, "\xE2\x80\x94") == 0 ||
               str.compare(i, 3, "\xE2\x88\x92") == 0) {
      // en dash, em dash and the minus sign are all dashes
      tokens.push_back({TokenKind::kDash, 0});
      i += 3;
    } else if (str.compare(i, 2, "\xC2\xA0") == 0) { // non breaking space
      i += 2;
    } else if (c == ',') {
      tokens.push_back({TokenKind::kComma, 0});
      ++i;
    } else if (c == ';') {
      tokens.push_back({TokenKind::kSemicolon, 0});
      ++i;
    } else if (c == '+') {
      tokens.push_back({TokenKind::kPlus, 0});
      ++i;
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
        tokens.push_back({TokenKind::kUnknown, 0});
        ++i;
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
        if (value <= 48 && minutes <= 59) {
          tokens.push_back({TokenKind::kTime, value * 60 + minutes});
        } else {
          tokens.push_back({TokenKind::kUnknown, 0});
        }
      } else if (value == 24 && i + 1 < n && str[i] == '/' && str[i + 1] == '7' &&
                 (i + 2 == n || !std::isdigit(static_cast<unsigned char>(str[i + 2])))) {
        tokens.push_back({TokenKind::kAlways, 0});
        i += 2;
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
};

RuleResult parse_date_point(TokenCursor& t, DatePoint& point, bool expect_month) {
  if (expect_month) {
    if (t.kind() != TokenKind::kMonth) {
      return RuleResult::kFailed;
    }
    point.month = t.eat();
  }
  if (t.kind() == TokenKind::kNumber) {
    if (t.value() >= 1000) { // a year
      return RuleResult::kUnsupported;
    }
    if (t.value() < 1 || t.value() > 31) {
      return RuleResult::kFailed;
    }
    point.day = t.eat();
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

RuleResult parse_dates(TokenCursor& t, TimeDomain& td) {
  DatePoint begin;
  RuleResult result = parse_date_point(t, begin, true);
  if (result != RuleResult::kOk) {
    return result;
  }

  DatePoint end;
  if (t.kind() == TokenKind::kDash) {
    ++t.pos;
    if (t.kind() == TokenKind::kMonth) {
      result = parse_date_point(t, end, true);
      if (result != RuleResult::kOk) {
        return result;
      }
    } else if (t.kind() == TokenKind::kNumber && begin.day != 0) {
      // a range within one month, e.g. May 16-31
      result = parse_date_point(t, end, false);
      if (result != RuleResult::kOk) {
        return result;
      }
      end.month = begin.month;
    } else {
      return RuleResult::kFailed;
    }
  } else {
    // a single point spans onto itself, e.g. Dec or May 15 or Dec Su[-1]
    end.month = begin.month;
    end.day = begin.day;
    end.week = begin.week;
  }

  if (begin.weekday != 0 || end.weekday != 0) {
    // the nth weekday of a month is its own domain type. Until a weekday selector says
    // otherwise, assume the restriction covers the entire week
    td.set_type(kNthDow);
    td.set_dow(kAllDaysOfWeek);
    td.set_begin_month(begin.month);
    td.set_begin_day_dow(begin.weekday != 0 ? begin.weekday : begin.day);
    td.set_begin_week(begin.week);
    td.set_end_month(end.month);
    td.set_end_day_dow(end.weekday != 0 ? end.weekday : end.day);
    td.set_end_week(end.week);
  } else {
    td.set_type(kYMD);
    td.set_begin_month(begin.month);
    td.set_begin_day_dow(begin.day);
    td.set_end_month(end.month);
    td.set_end_day_dow(end.day);
  }
  return RuleResult::kOk;
}

RuleResult parse_weekdays(TokenCursor& t, TimeDomain& td) {
  // wipe out the assumption that an nth weekday restriction covers the entire week
  uint8_t mask = (td.type() == kNthDow && td.dow() == kAllDaysOfWeek) ? 0 : td.dow();

  const int32_t first = t.eat();
  if (t.kind() == TokenKind::kNth) {
    // Su[1] is the first Sunday of every month
    const int32_t nth = t.eat();
    if (nth == -1) {
      td.set_begin_week(5);
    } else if (nth >= 1 && nth <= 5) {
      td.set_begin_week(nth);
    } else {
      return RuleResult::kUnsupported;
    }
    td.set_type(kNthDow);
    td.set_dow(mask | dow_mask_bit(first));
    return RuleResult::kOk;
  }

  if (t.kind() == TokenKind::kDash) {
    ++t.pos;
    if (t.kind() != TokenKind::kWeekday) {
      return RuleResult::kFailed;
    }
    int32_t from = first;
    const int32_t to = t.eat();
    if (from > to) { // Th-Tu wraps around the end of the week
      for (; from <= 7; ++from) {
        mask |= dow_mask_bit(from);
      }
      from = 1;
    }
    for (; from <= to; ++from) {
      mask |= dow_mask_bit(from);
    }
  } else {
    mask |= dow_mask_bit(first);
    // a list of days, holidays in it are ignored
    while (t.kind() == TokenKind::kComma &&
           (t.kind(1) == TokenKind::kWeekday || t.kind(1) == TokenKind::kHoliday)) {
      ++t.pos;
      if (t.kind() == TokenKind::kHoliday) {
        ++t.pos;
      } else {
        mask |= dow_mask_bit(t.eat());
      }
    }
  }
  // a holiday appended after a range is ignored too, e.g. Mo-Fr,PH
  while (t.kind() == TokenKind::kComma && t.kind(1) == TokenKind::kHoliday) {
    t.pos += 2;
  }
  td.set_dow(mask);
  return RuleResult::kOk;
}

RuleResult parse_times(TokenCursor& t,
                       const TimeDomain& td,
                       std::vector<uint64_t>& time_domains,
                       bool& emitted) {
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

    // every time range becomes its own restriction
    TimeDomain range = td;
    range.set_begin_hrs(begin / 60);
    range.set_begin_mins(begin % 60);
    range.set_end_hrs(end / 60);
    range.set_end_mins(end % 60);
    time_domains.push_back(range.td_value());
    emitted = true;

    if (t.kind() == TokenKind::kComma && t.kind(1) == TokenKind::kTime) {
      ++t.pos;
      continue;
    }
    return RuleResult::kOk;
  }
}

RuleResult parse_rule(TokenCursor& t, std::vector<uint64_t>& time_domains) {
  TimeDomain td(0);

  // public and school holidays can't be resolved into dates: a rule about them alone is
  // skipped in one piece, in a list with weekdays they are simply ignored
  if (t.kind() == TokenKind::kHoliday &&
      !(t.kind(1) == TokenKind::kComma && t.kind(2) == TokenKind::kWeekday)) {
    skip_rule(t);
    return RuleResult::kOk;
  }

  if (t.kind() == TokenKind::kSunEvent) {
    return RuleResult::kUnsupported;
  }

  // years of temporary restrictions don't fit into TimeDomain
  if (t.kind() == TokenKind::kNumber && t.value() >= 1000) {
    return RuleResult::kUnsupported;
  }

  if (t.kind() == TokenKind::kMonth) {
    const RuleResult result = parse_dates(t, td);
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // holidays listed in front of the weekdays are ignored, e.g. PH,Sat
  while (t.kind() == TokenKind::kHoliday && t.kind(1) == TokenKind::kComma &&
         t.kind(2) == TokenKind::kWeekday) {
    t.pos += 2;
  }

  while (t.kind() == TokenKind::kWeekday) {
    const RuleResult result = parse_weekdays(t, td);
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // 24/7 always holds: alone it means the whole week, after a date range it adds nothing
  if (t.kind() == TokenKind::kAlways) {
    ++t.pos;
    if (td.td_value() == 0) {
      td.set_dow(kAllDaysOfWeek);
    }
  }

  bool emitted = false;
  if (t.kind() == TokenKind::kTime) {
    const RuleResult result = parse_times(t, td, time_domains, emitted);
    if (result != RuleResult::kOk) {
      return result;
    }
  }

  // "off" inverts the meaning of the rule, which the callers can't represent
  if (t.kind() == TokenKind::kOff) {
    return RuleResult::kUnsupported;
  }

  if (!at_rule_end(t)) {
    // trailing junk voids the rule unless the times have already been parsed: unquoted
    // free text comments are common enough to tolerate
    if (!emitted) {
      return RuleResult::kFailed;
    }
    // a date, weekday or time here starts another rule, e.g. Mo-Fr 18:00-11:00 AND Sa
    // 00:00-10:00, anything else is a tail comment
    if (t.kind() != TokenKind::kWeekday && t.kind() != TokenKind::kMonth &&
        t.kind() != TokenKind::kTime) {
      skip_rule(t);
    }
  }

  if (!emitted) {
    if (td.td_value() == 0) {
      return RuleResult::kFailed;
    }
    time_domains.push_back(td.td_value());
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
