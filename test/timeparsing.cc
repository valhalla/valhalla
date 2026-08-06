#include "mjolnir/timeparsing.h"
#include "baldr/timedomain.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

// expected values are packed TimeDomain encodings, one per parsed rule of the condition
void TryConditionalRestrictions(const std::string& condition,
                                const std::vector<uint64_t>& expected_values) {
  EXPECT_EQ(get_time_range(condition), expected_values) << "Time domain " << condition;
}

struct DateTimePoint {
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
};

void TryConditionalRestrictions(const std::string& condition,
                                const uint32_t index,
                                const uint32_t type,
                                const uint32_t dow,
                                const DateTimePoint begin,
                                const DateTimePoint end) {

  std::vector<uint64_t> results = get_time_range(condition);

  TimeDomain res = TimeDomain(results.at(index));

  EXPECT_EQ(res.type(), type);
  EXPECT_EQ(res.dow(), dow);
  EXPECT_EQ(res.begin_month(), begin.month);
  EXPECT_EQ(res.begin_day_dow(), begin.day);
  EXPECT_EQ(res.begin_week(), begin.week);
  EXPECT_EQ(res.begin_hrs(), begin.hour);
  EXPECT_EQ(res.begin_mins(), begin.minute);
  EXPECT_EQ(res.end_month(), end.month);
  EXPECT_EQ(res.end_day_dow(), end.day);
  EXPECT_EQ(res.end_week(), end.week);
  EXPECT_EQ(res.end_hrs(), end.hour);
  EXPECT_EQ(res.end_mins(), end.minute);

  if (::testing::Test::HasFailure()) {
    std::cerr << "Time domain: " << condition << std::endl;
  }
}

} // namespace

// whole tag values go through get_time_range in one piece, exactly as production callers pass them
TEST(TimeParsing, TestConditionalRestrictions) {

  std::string str = "Mo-Fr 06:00-11:00,17:00-19:00;Sa 03:30-19:00";
  TryConditionalRestrictions(str, {23622321788, 40802193788, 40802435968});
  TryConditionalRestrictions(str, 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions(str, 1, 0, 62, {0, 0, 0, 17, 0}, {0, 0, 0, 19, 0});
  TryConditionalRestrictions(str, 2, 0, 64, {0, 0, 0, 3, 30}, {0, 0, 0, 19, 0});

  str = "Mo,We,Th,Fr 12:00-18:00; Sa-Su 12:00-17:00";
  TryConditionalRestrictions(str, {38654708852, 36507225218});
  TryConditionalRestrictions(str, 0, 0, 58, {0, 0, 0, 12, 0}, {0, 0, 0, 18, 0});
  TryConditionalRestrictions(str, 1, 0, 65, {0, 0, 0, 12, 0}, {0, 0, 0, 17, 0});

  str = "July 23-Aug 21 Sa 14:00-20:00;JUL 23-jUl 28 Fr,PH 10:00-20:00";
  TryConditionalRestrictions(str, {1512971146104448, 2001154308835904});
  TryConditionalRestrictions(str, 0, 0, 64, {7, 23, 0, 14, 0}, {8, 21, 0, 20, 0});
  TryConditionalRestrictions(str, 1, 0, 32, {7, 23, 0, 10, 0}, {7, 28, 0, 20, 0});

  str = "Apr-Sep Mo-Fr 09:00-13:00,14:00-18:00; Apr-Sep Sa 10:00-13:00";
  TryConditionalRestrictions(str, {39610337986940, 39621075406460, 39610337987200});
  TryConditionalRestrictions(str, 2, 0, 64, {4, 0, 0, 10, 0}, {9, 0, 0, 13, 0});

  str = "Apr-Sep: Monday-Fr 09:00-13:00,14:00-18:00; ApRil-Sept: Sa 10:00-13:00";
  TryConditionalRestrictions(str, {39610337986940, 39621075406460, 39610337987200});
  TryConditionalRestrictions(str, 2, 0, 64, {4, 0, 0, 10, 0}, {9, 0, 0, 13, 0});

  str = "06:00-11:00,17:00-19:45";
  TryConditionalRestrictions(str, {23622321664, 3133178646784});
  TryConditionalRestrictions(str, 0, 0, 0, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions(str, 1, 0, 0, {0, 0, 0, 17, 0}, {0, 0, 0, 19, 45});

  str = " Feb 16-Oct 15 09:00-18:30; Oct 16-Nov 15: 09:00-17:30; Nov 16-Feb 15: 09:00-16:30";
  TryConditionalRestrictions(str, {1101612002052352, 1106007905274112, 1066423339714816});
  TryConditionalRestrictions(str, 1, 0, 0, {10, 16, 0, 9, 0}, {11, 15, 0, 17, 30});
  TryConditionalRestrictions(str, 2, 0, 0, {11, 16, 0, 9, 0}, {02, 15, 0, 16, 30});

  str = "th 07:00-08:30; th-friday 06:00-09:30; May 15 09:00-11:30; May 07:00-08:30; May 16-31 "
        "11:00-13:30";
  TryConditionalRestrictions(str, {2078764173088, 2080911656544, 1079606730295552, 24068999350016,
                                   2205510940494592});
  TryConditionalRestrictions(str, 0, 0, 16, {0, 0, 0, 7, 0}, {0, 0, 0, 8, 30});
  TryConditionalRestrictions(str, 1, 0, 48, {0, 0, 0, 6, 0}, {0, 0, 0, 9, 30});
  TryConditionalRestrictions(str, 2, 0, 0, {5, 15, 0, 9, 0}, {5, 15, 0, 11, 30});
  TryConditionalRestrictions(str, 3, 0, 0, {5, 0, 0, 7, 0}, {5, 0, 0, 8, 30});
  TryConditionalRestrictions(str, 4, 0, 0, {5, 16, 0, 11, 0}, {5, 31, 0, 13, 30});

  str = "(Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50;Sep-Jun We 08:15-08:45,11:55-12:35)";
  TryConditionalRestrictions(str, {29497840232556, 29856470044524, 29497840232464, 28819235728144});
  TryConditionalRestrictions(str, 0, 0, 54, {9, 0, 0, 8, 15}, {6, 0, 0, 8, 45});
  TryConditionalRestrictions(str, 1, 0, 54, {9, 0, 0, 15, 20}, {6, 0, 0, 15, 50});
  TryConditionalRestrictions(str, 2, 0, 8, {9, 0, 0, 8, 15}, {6, 0, 0, 8, 45});
  TryConditionalRestrictions(str, 3, 0, 8, {9, 0, 0, 11, 55}, {6, 0, 0, 12, 35});

  // the holiday rules inside the paren groups are dropped, the date range rules survive
  str = "Oct Su[-1]-Mar th[4] (Su 09:00-16:00; PH 09:00-16:00);Mar Su[-1]-Oct Su[-1] (Su "
        "09:00-18:00; PH 09:00-18:00)";
  TryConditionalRestrictions(str, {9372272830712067, 11373388284561667});
  TryConditionalRestrictions(str, 0, 1, 1, {10, 1, 5, 9, 0}, {3, 5, 4, 16, 0});
  TryConditionalRestrictions(str, 1, 1, 1, {3, 1, 5, 9, 0}, {10, 1, 5, 18, 0});

  str = "Dec Fr[-1]-Jan Sa[3] Su,Sat 09:00-16:00, 15:00-17:00; Dec Su[-1] Su-Sa 15:00-17:00";
  TryConditionalRestrictions(str, {7252414455351683, 7252416602836867, 11311813490642943});
  TryConditionalRestrictions(str, 0, 1, 65, {12, 6, 5, 9, 0}, {1, 7, 3, 16, 0});
  TryConditionalRestrictions(str, 1, 1, 65, {12, 6, 5, 15, 0}, {1, 7, 3, 17, 0});
  TryConditionalRestrictions(str, 2, 1, 127, {12, 1, 5, 15, 0}, {12, 0, 5, 17, 0});

  str =
      "Sun 09:00-16:00; Su[1]; Dec; Dec Su[-1] 15:00-17:00; Dec Su[-1] Th 15:00-17:00;"
      "Dec Su[-1]; Dec Su[-1]-Mar 3 Sat;Mar 3-Dec Su[-1] Sat;Dec Su[-1]-Mar 3 Sat 15:00-17:00;"
      "Mar 3-Dec Su[-1] Sat 15:00-17:00; Mar 3-Dec Su[-1] Sat,PH 15:00-17:00; Mar 3-Dec Su[-1] PH,Sat 15:00-17:00";
  TryConditionalRestrictions(str, {34359740674, 268435459, 52776564424704, 11311813490642943,
                                   11311813490642721, 11311776983417087, 224301728923777,
                                   11382144397475969, 224338236149633, 11382180904701825,
                                   11382180904701825, 11382180904701825});
  TryConditionalRestrictions(str, 0, 0, 1, {0, 0, 0, 9, 0}, {0, 0, 0, 16, 0});
  TryConditionalRestrictions(str, 1, 1, 1, {0, 0, 1, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions(str, 2, 0, 0, {12, 0, 0, 0, 0}, {12, 0, 0, 0, 0});
  TryConditionalRestrictions(str, 3, 1, 127, {12, 1, 5, 15, 0}, {12, 0, 5, 17, 0});
  TryConditionalRestrictions(str, 4, 1, 16, {12, 1, 5, 15, 0}, {12, 0, 5, 17, 0});
  TryConditionalRestrictions(str, 5, 1, 127, {12, 1, 5, 0, 0}, {12, 0, 5, 0, 0});
  TryConditionalRestrictions(str, 6, 1, 64, {12, 1, 5, 0, 0}, {3, 3, 0, 0, 0});
  TryConditionalRestrictions(str, 7, 1, 64, {3, 3, 0, 0, 0}, {12, 1, 5, 0, 0});
  TryConditionalRestrictions(str, 8, 1, 64, {12, 1, 5, 15, 0}, {3, 3, 0, 17, 0});
  TryConditionalRestrictions(str, 9, 1, 64, {3, 3, 0, 15, 0}, {12, 1, 5, 17, 0});
  TryConditionalRestrictions(str, 10, 1, 64, {3, 3, 0, 15, 0}, {12, 1, 5, 17, 0});
  TryConditionalRestrictions(str, 11, 1, 64, {3, 3, 0, 15, 0}, {12, 1, 5, 17, 0});

  str = "Mon;Wed;Fr;Friday-Friday";
  TryConditionalRestrictions(str, {4, 16, 64, 64});
  TryConditionalRestrictions(str, 0, 0, 2, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions(str, 1, 0, 8, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions(str, 2, 0, 32, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions(str, 3, 0, 32, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});

  // invalid input, no start day is provided
  TryConditionalRestrictions("-Friday", {});

  str = "monday-friday 7:00-9:30,13:00-15:00";
  TryConditionalRestrictions(str, {2080911656828, 32212258172});
  TryConditionalRestrictions(str, 0, 0, 62, {0, 0, 0, 7, 0}, {0, 0, 0, 9, 30});
  TryConditionalRestrictions(str, 1, 0, 62, {0, 0, 0, 13, 0}, {0, 0, 0, 15, 0});

  // includes end of year
  str = "Jan 04-Jan 01 Mo-Sa;Jan 04-Jan 01 22:00-24:00;Jan 04-Jan 01";
  TryConditionalRestrictions(str, {74766824767740, 74766824773120, 74766824767488});

  // ranges without time
  str = "Mon-Friday;Mo,Wed;March-May;March 18-April 30";
  TryConditionalRestrictions(str, {124, 20, 21990234128384, 2128654663942144});
}

// A test case with exotic conditions extracted from `maxspeed:conditional` OSM field.
TEST(TimeParsing, TestConditionalMaxspeed) {
  TryConditionalRestrictions("(19:00-06:00)", 0, 0, 0, {0, 0, 0, 19, 0}, {0, 0, 0, 6, 0});
  TryConditionalRestrictions("(06:00-18:00)", 0, 0, 0, {0, 0, 0, 6, 0}, {0, 0, 0, 18, 0});

  std::string condition = "(07:00-09:00,13:00-16:00; SH off)";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0, {0, 0, 0, 7, 0}, {0, 0, 0, 9, 0});
  TryConditionalRestrictions(condition, 1, 0, 0, {0, 0, 0, 13, 0}, {0, 0, 0, 16, 0});

  TryConditionalRestrictions("Mo-Fr 19:00-07:00,Sa,Su", 0, 0, 62, {0, 0, 0, 19, 0}, {0, 0, 0, 7, 0});

  condition = "Mo-Fr 06:00-10:00,15:00-19:00 &quot;bij grote verkeersdrukte&quot;";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 10, 0});
  TryConditionalRestrictions(condition, 1, 0, 62, {0, 0, 0, 15, 0}, {0, 0, 0, 19, 0});

  condition = "(Mo, We, Th, Sa 07:00-15:00)";
  ASSERT_EQ(get_time_range(condition).size(), 1);
  TryConditionalRestrictions(condition, 0, 0, 0b01011010, {0, 0, 0, 7, 0}, {0, 0, 0, 15, 0});

  condition = "(Mo-Sa 07:00-20:00,07:00-20:00; Su 00:00-24:00; PH 00:00-24:00)";
  ASSERT_EQ(get_time_range(condition).size(), 3);
  TryConditionalRestrictions(condition, 0, 0, 126, {0, 0, 0, 7, 0}, {0, 0, 0, 20, 0});
  TryConditionalRestrictions(condition, 1, 0, 126, {0, 0, 0, 7, 0}, {0, 0, 0, 20, 0});
  TryConditionalRestrictions(condition, 2, 0, 1, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});

  TryConditionalRestrictions("Jun-Aug", 0, 0, 0, {6, 0, 0, 0, 0}, {8, 0, 0, 0, 0});
  TryConditionalRestrictions("(Nov - Mar)", 0, 0, 0, {11, 0, 0, 0, 0}, {3, 0, 0, 0, 0});

  TryConditionalRestrictions("(Apr 15-Oct 15 00:00-24:00)", 0, 0, 0, {4, 15, 0, 0, 0},
                             {10, 15, 0, 0, 0});
  TryConditionalRestrictions("(Aug 01-Jun 30 06:00-18:00)", 0, 0, 0, {8, 1, 0, 6, 0},
                             {6, 30, 0, 18, 0});
  TryConditionalRestrictions("(Aug 01-Jun 30 Mo-Fr 07:00-17:00; PH -1 day off; PH off)", 0, 0, 62,
                             {8, 1, 0, 7, 0}, {6, 30, 0, 17, 0});

  condition =
      "(Jan 01-Jun 15 Mo-Fr 07:00-18:00; PH -1 day off; PH off; Aug 15-Dec 31 Mo-Fr 00:00-24:00; PH -1 day off; PH off)";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 62, {1, 1, 0, 7, 0}, {6, 15, 0, 18, 0});
  TryConditionalRestrictions(condition, 1, 0, 62, {8, 15, 0, 0, 0}, {12, 31, 0, 0, 0});

  TryConditionalRestrictions("(Jun 1-Aug 31 00:00-24:00)", 0, 0, 0, {6, 1, 0, 0, 0},
                             {8, 31, 0, 0, 0});
}

// Mappers commonly put spaces around dashes or between a range and its times
TEST(TimeParsing, WhitespaceTolerance) {
  ASSERT_EQ(get_time_range("Mo - Fr 06:00 - 11:00").size(), 1);
  TryConditionalRestrictions("Mo - Fr 06:00 - 11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions("Mo-Fr 07:00 - 18:00", 0, 0, 62, {0, 0, 0, 7, 0}, {0, 0, 0, 18, 0});
  TryConditionalRestrictions("(Mar 20 - Jul 15)", 0, 0, 0, {3, 20, 0, 0, 0}, {7, 15, 0, 0, 0});
  TryConditionalRestrictions("08:00 - 18:00", 0, 0, 0, {0, 0, 0, 8, 0}, {0, 0, 0, 18, 0});
}

// ... or no space between the month and the day at all
TEST(TimeParsing, MissingSpaceAfterMonth) {
  ASSERT_EQ(get_time_range("Jul15 - Nov15").size(), 1);
  TryConditionalRestrictions("Jul15 - Nov15", 0, 0, 0, {7, 15, 0, 0, 0}, {11, 15, 0, 0, 0});
  TryConditionalRestrictions("Dec24-Dec26", 0, 0, 0, {12, 24, 0, 0, 0}, {12, 26, 0, 0, 0});
}

// ... or a unicode dash instead of the ascii one
TEST(TimeParsing, UnicodeDashes) {
  ASSERT_EQ(get_time_range("Mo–Fr 06:00-11:00").size(), 1); // en dash
  TryConditionalRestrictions("Mo–Fr 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
}

TEST(TimeParsing, FullDayNames) {
  TryConditionalRestrictions("Monday-Friday 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0},
                             {0, 0, 0, 11, 0});
  TryConditionalRestrictions("Wednesday 12:00-18:00", 0, 0, 8, {0, 0, 0, 12, 0}, {0, 0, 0, 18, 0});
}

// 24/7 means the condition always holds: an all week domain, not an unparsed condition
TEST(TimeParsing, AlwaysActive) {
  ASSERT_EQ(get_time_range("24/7").size(), 1);
  TryConditionalRestrictions("24/7", 0, 0, 127, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  // quoted comments are noise
  TryConditionalRestrictions("24/7 \"see https://example.com\"", 0, 0, 127, {0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0});
}

// an open ended time lasts until the end of the day, 24:00 is stored as 0
TEST(TimeParsing, OpenEndedTime) {
  ASSERT_EQ(get_time_range("18:00+").size(), 1);
  TryConditionalRestrictions("18:00+", 0, 0, 0, {0, 0, 0, 18, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions("Mo-Fr 20:00+", 0, 0, 62, {0, 0, 0, 20, 0}, {0, 0, 0, 0, 0});
}

// a comma can also separate whole rules, not only lists of days or times
TEST(TimeParsing, MultipleRulesInOneCondition) {
  const std::string condition = "Oct-Mar 18:00-08:00, Apr-Sep 21:00-07:00";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0, {10, 0, 0, 18, 0}, {3, 0, 0, 8, 0});
  TryConditionalRestrictions(condition, 1, 0, 0, {4, 0, 0, 21, 0}, {9, 0, 0, 7, 0});
}

// public holidays can't be resolved into dates but must not fail the rest of the rule
TEST(TimeParsing, HolidayTolerance) {
  TryConditionalRestrictions("Mo-Fr,PH 08:00-18:00", 0, 0, 62, {0, 0, 0, 8, 0}, {0, 0, 0, 18, 0});
  TryConditionalRestrictions("PH,Mo-Fr 08:00-18:00", 0, 0, 62, {0, 0, 0, 8, 0}, {0, 0, 0, 18, 0});
  EXPECT_TRUE(get_time_range("PH").empty());
  EXPECT_TRUE(get_time_range("PH off").empty());
}

// nth weekday of every month, with no month context
TEST(TimeParsing, NthWeekdayOfEveryMonth) {
  ASSERT_EQ(get_time_range("Mon[-1]").size(), 1); // the last Monday of every month
  TryConditionalRestrictions("Mon[-1]", 0, 1, 2, {0, 0, 5, 0, 0}, {0, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range("Tue[2]").size(), 1); // the second Tuesday of every month
  TryConditionalRestrictions("Tue[2]", 0, 1, 4, {0, 0, 2, 0, 0}, {0, 0, 0, 0, 0});
}

// range shapes without times attached
TEST(TimeParsing, BareRanges) {
  // a range within one month
  TryConditionalRestrictions("Feb 2-14", 0, 0, 0, {2, 2, 0, 0, 0}, {2, 14, 0, 0, 0});
  // a weekday range wrapping around the end of the week
  TryConditionalRestrictions("Th - Tu", 0, 0, 119, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  // nth weekday ranges on both ends
  TryConditionalRestrictions("Oct Su[-1]-Mar Su[4] Su 09:00-16:00", 0, 1, 1, {10, 1, 5, 9, 0},
                             {3, 1, 4, 16, 0});
}

// conditions that don't fit TimeDomain or are not about time must be rejected in one piece,
// without exceptions and without half parsed leftovers
TEST(TimeParsing, UnsupportedConditionsRejected) {
  // years of temporary construction closures don't fit into TimeDomain
  EXPECT_TRUE(get_time_range("2023 Jul 10-2023 Sep 04").empty());
  EXPECT_TRUE(get_time_range("(2026 May 04-2026 Jul 31)").empty());
  EXPECT_TRUE(get_time_range("2025 Jun 23- 2026 Dec 15").empty());
  // sun events can't be resolved into fixed hours at parse time
  EXPECT_TRUE(get_time_range("sunset-sunrise").empty());
  EXPECT_TRUE(get_time_range("Nov-Feb 08:00-dusk").empty());
  // seasons can't be resolved into a month range without knowing the hemisphere
  EXPECT_TRUE(get_time_range("summer").empty());
  EXPECT_TRUE(get_time_range("winter").empty());
  // vehicle and road state conditions are not about time at all
  EXPECT_TRUE(get_time_range("wet").empty());
  EXPECT_TRUE(get_time_range("weight>7.5").empty());
  EXPECT_TRUE(get_time_range("fuel=diesel AND emissions<euro_6").empty());
  // an unparsable or unsupported rule is dropped alone, the rest of the value still works
  const std::string broken = "qwerty; Sa 08:00-12:00; 2023 Jan-Nov";
  ASSERT_EQ(get_time_range(broken).size(), 1);
  TryConditionalRestrictions(broken, 0, 0, 64, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
