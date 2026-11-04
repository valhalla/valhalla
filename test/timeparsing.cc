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
  ASSERT_EQ(get_time_range("(19:00-06:00)").size(), 1);
  TryConditionalRestrictions("(19:00-06:00)", 0, 0, 0, {0, 0, 0, 19, 0}, {0, 0, 0, 6, 0});
  ASSERT_EQ(get_time_range("(06:00-18:00)").size(), 1);
  TryConditionalRestrictions("(06:00-18:00)", 0, 0, 0, {0, 0, 0, 6, 0}, {0, 0, 0, 18, 0});

  std::string condition = "(07:00-09:00,13:00-16:00; SH off)";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0, {0, 0, 0, 7, 0}, {0, 0, 0, 9, 0});
  TryConditionalRestrictions(condition, 1, 0, 0, {0, 0, 0, 13, 0}, {0, 0, 0, 16, 0});

  // the trailing `,Sa,Su` is a rule of its own with no time, i.e. all day
  condition = "Mo-Fr 19:00-07:00,Sa,Su";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 62, {0, 0, 0, 19, 0}, {0, 0, 0, 7, 0});
  TryConditionalRestrictions(condition, 1, 0, 0b1000001, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});

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

  ASSERT_EQ(get_time_range("Jun-Aug").size(), 1);
  TryConditionalRestrictions("Jun-Aug", 0, 0, 0, {6, 0, 0, 0, 0}, {8, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range("(Nov - Mar)").size(), 1);
  TryConditionalRestrictions("(Nov - Mar)", 0, 0, 0, {11, 0, 0, 0, 0}, {3, 0, 0, 0, 0});

  ASSERT_EQ(get_time_range("(Apr 15-Oct 15 00:00-24:00)").size(), 1);
  TryConditionalRestrictions("(Apr 15-Oct 15 00:00-24:00)", 0, 0, 0, {4, 15, 0, 0, 0},
                             {10, 15, 0, 0, 0});
  ASSERT_EQ(get_time_range("(Aug 01-Jun 30 06:00-18:00)").size(), 1);
  TryConditionalRestrictions("(Aug 01-Jun 30 06:00-18:00)", 0, 0, 0, {8, 1, 0, 6, 0},
                             {6, 30, 0, 18, 0});
  ASSERT_EQ(get_time_range("(Aug 01-Jun 30 Mo-Fr 07:00-17:00; PH -1 day off; PH off)").size(), 1);
  TryConditionalRestrictions("(Aug 01-Jun 30 Mo-Fr 07:00-17:00; PH -1 day off; PH off)", 0, 0, 62,
                             {8, 1, 0, 7, 0}, {6, 30, 0, 17, 0});

  condition =
      "(Jan 01-Jun 15 Mo-Fr 07:00-18:00; PH -1 day off; PH off; Aug 15-Dec 31 Mo-Fr 00:00-24:00; PH -1 day off; PH off)";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 62, {1, 1, 0, 7, 0}, {6, 15, 0, 18, 0});
  TryConditionalRestrictions(condition, 1, 0, 62, {8, 15, 0, 0, 0}, {12, 31, 0, 0, 0});

  ASSERT_EQ(get_time_range("(Jun 1-Aug 31 00:00-24:00)").size(), 1);
  TryConditionalRestrictions("(Jun 1-Aug 31 00:00-24:00)", 0, 0, 0, {6, 1, 0, 0, 0},
                             {8, 31, 0, 0, 0});
}

// Mappers commonly put spaces around dashes or between a range and its times
TEST(TimeParsing, WhitespaceTolerance) {
  ASSERT_EQ(get_time_range("Mo - Fr 06:00 - 11:00").size(), 1);
  TryConditionalRestrictions("Mo - Fr 06:00 - 11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  ASSERT_EQ(get_time_range("Mo-Fr 07:00 - 18:00").size(), 1);
  TryConditionalRestrictions("Mo-Fr 07:00 - 18:00", 0, 0, 62, {0, 0, 0, 7, 0}, {0, 0, 0, 18, 0});
  ASSERT_EQ(get_time_range("(Mar 20 - Jul 15)").size(), 1);
  TryConditionalRestrictions("(Mar 20 - Jul 15)", 0, 0, 0, {3, 20, 0, 0, 0}, {7, 15, 0, 0, 0});
  ASSERT_EQ(get_time_range("08:00 - 18:00").size(), 1);
  TryConditionalRestrictions("08:00 - 18:00", 0, 0, 0, {0, 0, 0, 8, 0}, {0, 0, 0, 18, 0});
}

// ... or no space between the month and the day at all
TEST(TimeParsing, MissingSpaceAfterMonth) {
  ASSERT_EQ(get_time_range("Jul15 - Nov15").size(), 1);
  TryConditionalRestrictions("Jul15 - Nov15", 0, 0, 0, {7, 15, 0, 0, 0}, {11, 15, 0, 0, 0});
  ASSERT_EQ(get_time_range("Dec24-Dec26").size(), 1);
  TryConditionalRestrictions("Dec24-Dec26", 0, 0, 0, {12, 24, 0, 0, 0}, {12, 26, 0, 0, 0});
}

// ... or a unicode dash instead of the ascii one
TEST(TimeParsing, UnicodeDashes) {
  ASSERT_EQ(get_time_range("Mo–Fr 06:00-11:00").size(), 1); // en dash
  TryConditionalRestrictions("Mo–Fr 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions("Mo—Fr 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions("Mo−Fr 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
  // and a non breaking space where a plain one belongs
  TryConditionalRestrictions("Mo-Fr\xC2\xA0"
                             "06:00-11:00",
                             0, 0, 62, {0, 0, 0, 6, 0}, {0, 0, 0, 11, 0});
}

TEST(TimeParsing, FullDayNames) {
  ASSERT_EQ(get_time_range("Monday-Friday 06:00-11:00").size(), 1);
  TryConditionalRestrictions("Monday-Friday 06:00-11:00", 0, 0, 62, {0, 0, 0, 6, 0},
                             {0, 0, 0, 11, 0});
  ASSERT_EQ(get_time_range("Wednesday 12:00-18:00").size(), 1);
  TryConditionalRestrictions("Wednesday 12:00-18:00", 0, 0, 8, {0, 0, 0, 12, 0}, {0, 0, 0, 18, 0});
}

// 24/7 means the condition always holds: an all week domain, not an unparsed condition
TEST(TimeParsing, AlwaysActive) {
  ASSERT_EQ(get_time_range("24/7").size(), 1);
  TryConditionalRestrictions("24/7", 0, 0, 127, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  // quoted comments are noise
  ASSERT_EQ(get_time_range("24/7 \"see https://example.com\"").size(), 1);
  TryConditionalRestrictions("24/7 \"see https://example.com\"", 0, 0, 127, {0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0});
}

// an open ended time lasts until the end of the day, 24:00 is stored as 0
TEST(TimeParsing, OpenEndedTime) {
  ASSERT_EQ(get_time_range("18:00+").size(), 1);
  TryConditionalRestrictions("18:00+", 0, 0, 0, {0, 0, 0, 18, 0}, {0, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range("Mo-Fr 20:00+").size(), 1);
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
  ASSERT_EQ(get_time_range("Mo-Fr,PH 08:00-18:00").size(), 1);
  TryConditionalRestrictions("Mo-Fr,PH 08:00-18:00", 0, 0, 62, {0, 0, 0, 8, 0}, {0, 0, 0, 18, 0});
  ASSERT_EQ(get_time_range("PH,Mo-Fr 08:00-18:00").size(), 1);
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
  ASSERT_EQ(get_time_range("Feb 2-14").size(), 1);
  TryConditionalRestrictions("Feb 2-14", 0, 0, 0, {2, 2, 0, 0, 0}, {2, 14, 0, 0, 0});
  // a weekday range wrapping around the end of the week
  ASSERT_EQ(get_time_range("Th - Tu").size(), 1);
  TryConditionalRestrictions("Th - Tu", 0, 0, 119, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  // nth weekday ranges on both ends
  ASSERT_EQ(get_time_range("Oct Su[-1]-Mar Su[4] Su 09:00-16:00").size(), 1);
  TryConditionalRestrictions("Oct Su[-1]-Mar Su[4] Su 09:00-16:00", 0, 1, 1, {10, 1, 5, 9, 0},
                             {3, 1, 4, 16, 0});
}

// conditions that don't fit TimeDomain or are not about time must be rejected in one piece,
// without exceptions and without half parsed leftovers
TEST(TimeParsing, UnsupportedConditionsRejected) {
  // sun events can't be resolved into fixed hours at parse time
  EXPECT_TRUE(get_time_range("sunset-sunrise").empty());
  EXPECT_TRUE(get_time_range("Nov-Feb 08:00-dusk").empty());
  // seasons can't be resolved into a month range without knowing the hemisphere
  EXPECT_TRUE(get_time_range("summer").empty());
  EXPECT_TRUE(get_time_range("winter").empty());
  // vehicle and road state conditions are not about time at all
  EXPECT_TRUE(get_time_range("wet").empty());
  // an unparsable rule is dropped alone, the rest of the value still works
  const std::string broken = "qwerty; Sa 08:00-12:00; !?";
  ASSERT_EQ(get_time_range(broken).size(), 1);
  TryConditionalRestrictions(broken, 0, 0, 64, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
}

// TimeDomain repeats a range every year, so a single year says nothing it can store
TEST(TimeParsing, SingleYearIsDropped) {
  ASSERT_EQ(get_time_range("2025 Feb 15-Apr 04").size(), 1);
  TryConditionalRestrictions("2025 Feb 15-Apr 04", 0, 0, 0, {2, 15, 0, 0, 0}, {4, 4, 0, 0, 0});
  ASSERT_EQ(get_time_range("2023 Jan-Nov").size(), 1);
  TryConditionalRestrictions("2023 Jan-Nov", 0, 0, 0, {1, 0, 0, 0, 0}, {11, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range("2025 Dec 25").size(), 1);
  TryConditionalRestrictions("2025 Dec 25", 0, 0, 0, {12, 25, 0, 0, 0}, {12, 25, 0, 0, 0});
}

// a year on both ends bounds the range to a finite interval instead, which TimeDomain would turn
// into a yearly one, leaving a multi year closure open for the months in between
TEST(TimeParsing, BoundedYearRangesRejected) {
  EXPECT_TRUE(get_time_range("2023 Jul 10-2023 Sep 04").empty());
  EXPECT_TRUE(get_time_range("(2026 May 10–2027 Dec 31)").empty());
  EXPECT_TRUE(get_time_range("(2020 Nov 17 - 2024 Jul 31)").empty());
  // the year is also written after the date
  EXPECT_TRUE(get_time_range("(Jan 05 2024-Jul 01 2024)").empty());
  // a year on its own carries no range to keep
  EXPECT_TRUE(get_time_range("2023-2025").empty());
}

// a qualifier the tag adds on top of the time can't be stored, only the time part is kept. The
// restriction then covers more than the tag says, which is what the regexes did as well
TEST(TimeParsing, QualifiersAreSkipped) {
  TryConditionalRestrictions("weight>3.5 AND Mo-Fr 08:00-18:00", 0, 0, 0b0111110, {0, 0, 0, 8, 0},
                             {0, 0, 0, 18, 0});
  TryConditionalRestrictions("Mo-Fr 08:00-18:00 AND weight>3.5", 0, 0, 0b0111110, {0, 0, 0, 8, 0},
                             {0, 0, 0, 18, 0});
  TryConditionalRestrictions("(hazmat:E AND Mo-Su 05:00-23:00)", 0, 0, 0b1111111, {0, 0, 0, 5, 0},
                             {0, 0, 0, 23, 0});
  TryConditionalRestrictions("(Apr 14-Oct 31 AND stationary_noise>95)", 0, 0, 0, {4, 14, 0, 0, 0},
                             {10, 31, 0, 0, 0});
  TryConditionalRestrictions("(learner AND Mo-Sa 07:30-19:00)", 0, 0, 0b1111110, {0, 0, 0, 7, 30},
                             {0, 0, 0, 19, 0});
  const std::string delivery = "(delivery AND 08:00-12:00,14:00-18:00)";
  ASSERT_EQ(get_time_range(delivery).size(), 2);
  TryConditionalRestrictions(delivery, 0, 0, 0, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
  TryConditionalRestrictions(delivery, 1, 0, 0, {0, 0, 0, 14, 0}, {0, 0, 0, 18, 0});
  ASSERT_EQ(get_time_range("(school_days 07:00-17:00)").size(), 1);
  TryConditionalRestrictions("(school_days 07:00-17:00)", 0, 0, 0, {0, 0, 0, 7, 0}, {0, 0, 0, 17, 0});
  // a list of them, the commas in between belong to the list and not to the rule
  TryConditionalRestrictions("(motorcar,moped Mo-Sa 07:30-08:30)", 0, 0, 0b1111110, {0, 0, 0, 7, 30},
                             {0, 0, 0, 8, 30});
  // a qualifier on its own carries no time to keep
  EXPECT_TRUE(get_time_range("weight>7.5").empty());
  EXPECT_TRUE(get_time_range("fuel=diesel AND emissions<euro_6").empty());
}

// a holiday can't be resolved into dates, so it is ignored wherever it sits in a rule
TEST(TimeParsing, HolidayBesideOtherSelectors) {
  TryConditionalRestrictions("(Sa,Su PH)", 0, 0, 0b1000001, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions("Su PH", 0, 0, 1, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions("(Jun-Aug: SH)", 0, 0, 0, {6, 0, 0, 0, 0}, {8, 0, 0, 0, 0});
  const std::string condition = "(Mar-Sep PH 14:00-19:00; Oct-Nov PH 14:00-17:00)";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0, {3, 0, 0, 14, 0}, {9, 0, 0, 19, 0});
  TryConditionalRestrictions(condition, 1, 0, 0, {10, 0, 0, 14, 0}, {11, 0, 0, 17, 0});
  // a rule that is nothing but holidays still has no dates to store
  EXPECT_TRUE(get_time_range("PH 00:00-24:00").empty());
}

// times on a 12 hour clock
TEST(TimeParsing, AmPmTimes) {
  TryConditionalRestrictions("Mo-Fr 04:00pm-7:00pm", 0, 0, 0b0111110, {0, 0, 0, 16, 0},
                             {0, 0, 0, 19, 0});
  TryConditionalRestrictions("Mon-Fri 06:30am-07:30am", 0, 0, 0b0111110, {0, 0, 0, 6, 30},
                             {0, 0, 0, 7, 30});
  // midnight and noon
  TryConditionalRestrictions("12:00am-12:00pm", 0, 0, 0, {0, 0, 0, 0, 0}, {0, 0, 0, 12, 0});
  // a suffix that cannot apply to the hour is dropped, the hour stands as written
  TryConditionalRestrictions("05:00AM-22:00PM", 0, 0, 0, {0, 0, 0, 5, 0}, {0, 0, 0, 22, 0});
}

// the day is written in front of its month too
TEST(TimeParsing, DayBeforeMonth) {
  TryConditionalRestrictions("(26 November)", 0, 0, 0, {11, 26, 0, 0, 0}, {11, 26, 0, 0, 0});
  TryConditionalRestrictions("(26 November AND 15:00-17:30)", 0, 0, 0, {11, 26, 0, 15, 0},
                             {11, 26, 0, 17, 30});
  TryConditionalRestrictions("1 Jan-3 Jan", 0, 0, 0, {1, 1, 0, 0, 0}, {1, 3, 0, 0, 0});
}

// a list of days mixes single days and ranges, and belongs to one rule
TEST(TimeParsing, WeekdayLists) {
  TryConditionalRestrictions("Mo-Tu,Th-Fr 07:15-08:45", 0, 0, 0b0110110, {0, 0, 0, 7, 15},
                             {0, 0, 0, 8, 45});
  TryConditionalRestrictions("Mo,Th-Fr 08:00-12:00", 0, 0, 0b0110010, {0, 0, 0, 8, 0},
                             {0, 0, 0, 12, 0});
  TryConditionalRestrictions("Mo-We,Fr,Sa 00:00-09:00", 0, 0, 0b1101110, {0, 0, 0, 0, 0},
                             {0, 0, 0, 9, 0});
  ASSERT_EQ(get_time_range("Mo-Fr,Su 07:00-12:00").size(), 1);
  TryConditionalRestrictions("Mo-Fr,Su 07:00-12:00", 0, 0, 0b0111111, {0, 0, 0, 7, 0},
                             {0, 0, 0, 12, 0});
}

// a list of dates shares the weekdays and times of its rule, one restriction per date
TEST(TimeParsing, DateLists) {
  const std::string dates = "Jan 01,Apr 19,Dec 25 09:00-17:00";
  ASSERT_EQ(get_time_range(dates).size(), 3);
  TryConditionalRestrictions(dates, 0, 0, 0, {1, 1, 0, 9, 0}, {1, 1, 0, 17, 0});
  TryConditionalRestrictions(dates, 1, 0, 0, {4, 19, 0, 9, 0}, {4, 19, 0, 17, 0});
  TryConditionalRestrictions(dates, 2, 0, 0, {12, 25, 0, 9, 0}, {12, 25, 0, 17, 0});

  const std::string ranges = "Jan 07-Jul 14,Sep 01-Dec 19 Sa,Su 10:00-20:00";
  ASSERT_EQ(get_time_range(ranges).size(), 2);
  TryConditionalRestrictions(ranges, 0, 0, 0b1000001, {1, 7, 0, 10, 0}, {7, 14, 0, 20, 0});
  TryConditionalRestrictions(ranges, 1, 0, 0b1000001, {9, 1, 0, 10, 0}, {12, 19, 0, 20, 0});

  // several dates and several times give a restriction for each combination, dates outer
  const std::string both = "Jan 01,Dec 25 09:00-12:00,14:00-17:00";
  ASSERT_EQ(get_time_range(both).size(), 4);
  TryConditionalRestrictions(both, 0, 0, 0, {1, 1, 0, 9, 0}, {1, 1, 0, 12, 0});
  TryConditionalRestrictions(both, 1, 0, 0, {1, 1, 0, 14, 0}, {1, 1, 0, 17, 0});
  TryConditionalRestrictions(both, 2, 0, 0, {12, 25, 0, 9, 0}, {12, 25, 0, 12, 0});
  TryConditionalRestrictions(both, 3, 0, 0, {12, 25, 0, 14, 0}, {12, 25, 0, 17, 0});
}

// weekdays are also written in front of the dates
TEST(TimeParsing, WeekdaysBeforeDates) {
  ASSERT_EQ(get_time_range("Su,Mo Jul 16-Sep 04").size(), 1);
  TryConditionalRestrictions("Su,Mo Jul 16-Sep 04", 0, 0, 0b0000011, {7, 16, 0, 0, 0},
                             {9, 4, 0, 0, 0});
}

// the selector is also written behind the times, which is still one rule
TEST(TimeParsing, SelectorAfterTimes) {
  ASSERT_EQ(get_time_range("08:00-20:00 Mo-Fr").size(), 1);
  TryConditionalRestrictions("08:00-20:00 Mo-Fr", 0, 0, 0b0111110, {0, 0, 0, 8, 0}, {0, 0, 0, 20, 0});
  ASSERT_EQ(get_time_range("(07:00-16:30 AND Mo-Fr)").size(), 1);
  TryConditionalRestrictions("(07:00-16:30 AND Mo-Fr)", 0, 0, 0b0111110, {0, 0, 0, 7, 0},
                             {0, 0, 0, 16, 30});
  ASSERT_EQ(get_time_range("06:00-22:00 Jun 15-Sep 15").size(), 1);
  TryConditionalRestrictions("06:00-22:00 Jun 15-Sep 15", 0, 0, 0, {6, 15, 0, 6, 0},
                             {9, 15, 0, 22, 0});
  // every time range of the rule takes the weekdays
  const std::string two = "07:00-10:00, 16:00-20:00 Mo-Fr";
  ASSERT_EQ(get_time_range(two).size(), 2);
  TryConditionalRestrictions(two, 0, 0, 0b0111110, {0, 0, 0, 7, 0}, {0, 0, 0, 10, 0});
  TryConditionalRestrictions(two, 1, 0, 0b0111110, {0, 0, 0, 16, 0}, {0, 0, 0, 20, 0});
  // a selector that brings its own times is a rule of its own instead
  const std::string apart = "Mo-Fr 18:00-11:00 AND Sa 00:00-10:00";
  ASSERT_EQ(get_time_range(apart).size(), 2);
  TryConditionalRestrictions(apart, 0, 0, 0b0111110, {0, 0, 0, 18, 0}, {0, 0, 0, 11, 0});
  TryConditionalRestrictions(apart, 1, 0, 0b1000000, {0, 0, 0, 0, 0}, {0, 0, 0, 10, 0});
  // and so is one held apart by a comma, where an absent time means the whole day
  const std::string comma = "Mo-Sa 06:00-20:00, Su";
  ASSERT_EQ(get_time_range(comma).size(), 2);
  TryConditionalRestrictions(comma, 0, 0, 0b1111110, {0, 0, 0, 6, 0}, {0, 0, 0, 20, 0});
  TryConditionalRestrictions(comma, 1, 0, 0b0000001, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
}

// a comma also shows up between the selectors of a single rule
TEST(TimeParsing, CommaBetweenSelectors) {
  const std::string condition = "Oct-Mar, 07:00-19:00; Apr-Sep, 07:00-21:30";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0, {10, 0, 0, 7, 0}, {3, 0, 0, 19, 0});
  TryConditionalRestrictions(condition, 1, 0, 0, {4, 0, 0, 7, 0}, {9, 0, 0, 21, 30});
  ASSERT_EQ(get_time_range("Mo-Sa, 18:30-10:30").size(), 1);
  TryConditionalRestrictions("Mo-Sa, 18:30-10:30", 0, 0, 0b1111110, {0, 0, 0, 18, 30},
                             {0, 0, 0, 10, 30});
  ASSERT_EQ(get_time_range("May 01-May 31, Mo-Fr").size(), 1);
  TryConditionalRestrictions("May 01-May 31, Mo-Fr", 0, 0, 0b0111110, {5, 1, 0, 0, 0},
                             {5, 31, 0, 0, 0});
  // the dates belong to the weekdays in front of them, not to a rule of their own
  ASSERT_EQ(get_time_range("(Su, Jul-Aug)").size(), 1);
  TryConditionalRestrictions("(Su, Jul-Aug)", 0, 0, 1, {7, 0, 0, 0, 0}, {8, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range("(Su, PH, Jan 02-03 09:00-17:00)").size(), 1);
  TryConditionalRestrictions("(Su, PH, Jan 02-03 09:00-17:00)", 0, 0, 1, {1, 2, 0, 9, 0},
                             {1, 3, 0, 17, 0});

  // a holiday sits in the middle of a list of days without ending it, so every day named keeps
  // both time ranges rather than the ones before it becoming a rule of their own
  const std::string mixed = "(Sa, Su, PH, Mo-Fr 00:00-07:00, 19:00-24:00)";
  ASSERT_EQ(get_time_range(mixed).size(), 2);
  TryConditionalRestrictions(mixed, 0, 0, 0b1111111, {0, 0, 0, 0, 0}, {0, 0, 0, 7, 0});
  TryConditionalRestrictions(mixed, 1, 0, 0b1111111, {0, 0, 0, 19, 0}, {0, 0, 0, 0, 0});
  // the same list without the holiday in it, to keep the two in step
  ASSERT_EQ(get_time_range("Sa, Su, Mo-Fr 08:00-12:00").size(), 1);
  TryConditionalRestrictions("Sa, Su, Mo-Fr 08:00-12:00", 0, 0, 0b1111111, {0, 0, 0, 8, 0},
                             {0, 0, 0, 12, 0});
}

// time ranges are usually comma separated, but a bare space between them happens too
TEST(TimeParsing, SpaceSeparatedTimes) {
  const std::string condition = "Su 08:00-12:30 14:30-19:00";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 1, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 30});
  TryConditionalRestrictions(condition, 1, 0, 1, {0, 0, 0, 14, 30}, {0, 0, 0, 19, 0});
}

// a `||` fallback rule is parsed as a rule of its own, the quoted comment is skipped
TEST(TimeParsing, FallbackRule) {
  ASSERT_EQ(get_time_range("Oct 01-Apr 30||\"when barrier is locked closed\"").size(), 1);
  TryConditionalRestrictions("Oct 01-Apr 30||\"when barrier is locked closed\"", 0, 0, 0,
                             {10, 1, 0, 0, 0}, {4, 30, 0, 0, 0});
}

// a day of the month and the nth weekday of it share one field, a rule asking for both is
// dropped instead of storing a day where a weekday is expected
TEST(TimeParsing, DayAndNthWeekdayConflict) {
  EXPECT_TRUE(get_time_range("May 15 Su[1] 09:00-11:00").empty());
  EXPECT_TRUE(get_time_range("May 16-20 Su[1]").empty());
}

// 24/7 next to a date range adds nothing, the range already says every day of it
TEST(TimeParsing, AlwaysActiveBesideDates) {
  ASSERT_EQ(get_time_range("Jan-Mar 24/7").size(), 1);
  TryConditionalRestrictions("Jan-Mar 24/7", 0, 0, 0, {1, 0, 0, 0, 0}, {3, 0, 0, 0, 0});
}

// an nth weekday selector we can't read is dropped whole. Its pieces must not be mistaken for
// selectors of their own, which would leave an all day restriction behind
TEST(TimeParsing, UnreadableNthWeekday) {
  ASSERT_EQ(get_time_range("Su[1,-1] 12:00-17:00").size(), 1);
  TryConditionalRestrictions("Su[1,-1] 12:00-17:00", 0, 0, 1, {0, 0, 0, 12, 0}, {0, 0, 0, 17, 0});
}

// a malformed time voids its whole rule rather than leaving the ranges before it in place: half
// a restriction is not the one the tag describes
TEST(TimeParsing, MalformedTimeVoidsTheRule) {
  EXPECT_TRUE(get_time_range("Mo-Fr 08:00-12:00, 13:00").empty());
  EXPECT_TRUE(get_time_range("Mo-Fr 08:00-12:00, 14:00-15").empty());
  // the rules around it still stand
  const std::string condition = "Mo 08:00-12:00; Tu 09:00; We 10:00-14:00";
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 2, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
  TryConditionalRestrictions(condition, 1, 0, 8, {0, 0, 0, 10, 0}, {0, 0, 0, 14, 0});
}

// 00:00-24:00 is the whole day and every field of the domain lands on zero, which the callers
// store as a restriction that always holds
TEST(TimeParsing, WholeDayIsAnEmptyDomain) {
  TryConditionalRestrictions("00:00-24:00", {0});
  TryConditionalRestrictions("(00:00-24:00 AND weight<3.5)", {0});
}

// The canonical examples from https://wiki.openstreetmap.org/wiki/Key:opening_hours
TEST(TimeParsing, WikiExamples) {
  std::string condition = "Mo-Fr 08:00-12:00,13:00-17:30"; // closed over lunch
  ASSERT_EQ(get_time_range(condition).size(), 2);
  TryConditionalRestrictions(condition, 0, 0, 0b0111110, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
  TryConditionalRestrictions(condition, 1, 0, 0b0111110, {0, 0, 0, 13, 0}, {0, 0, 0, 17, 30});

  ASSERT_EQ(get_time_range("Mo,We 08:00-12:00").size(), 1);
  TryConditionalRestrictions("Mo,We 08:00-12:00", 0, 0, 0b0001010, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});

  // the Saturday morning, then the same value with holidays named either way round. A holiday
  // resolves to no date, so it neither adds a restriction nor takes the other rules away
  condition = "Mo-Fr 08:00-12:00,13:00-17:30; Sa 08:00-12:00";
  ASSERT_EQ(get_time_range(condition).size(), 3);
  TryConditionalRestrictions(condition, 2, 0, 0b1000000, {0, 0, 0, 8, 0}, {0, 0, 0, 12, 0});
  ASSERT_EQ(get_time_range(condition + "; PH off").size(), 3);
  ASSERT_EQ(get_time_range(condition + "; PH 09:00-12:00").size(), 3);

  ASSERT_EQ(get_time_range("Mo-Fr 08:30-20:00").size(), 1);
  ASSERT_EQ(get_time_range("Sa-Su 00:00-24:00").size(), 1);
  TryConditionalRestrictions("Sa-Su 00:00-24:00", 0, 0, 0b1000001, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
  ASSERT_EQ(get_time_range(
                "Mo 10:00-12:00,12:30-15:00; Tu-Fr 08:00-12:00,12:30-15:00; Sa 08:00-12:00")
                .size(),
            5);

  ASSERT_EQ(get_time_range("Su 10:00+").size(), 1);
  TryConditionalRestrictions("Su 10:00+", 0, 0, 1, {0, 0, 0, 10, 0}, {0, 0, 0, 0, 0});

  ASSERT_EQ(get_time_range("Apr-Oct: Fr-Su 10:00-18:00").size(), 1);
  TryConditionalRestrictions("Apr-Oct: Fr-Su 10:00-18:00", 0, 0, 0b1100001, {4, 0, 0, 10, 0},
                             {10, 0, 0, 18, 0});
  ASSERT_EQ(get_time_range("Dec 11-Dec 17: Su 10:00-17:00").size(), 1);
  TryConditionalRestrictions("Dec 11-Dec 17: Su 10:00-17:00", 0, 0, 1, {12, 11, 0, 10, 0},
                             {12, 17, 0, 17, 0});

  // a range over midnight is written as extra rules separated by commas
  condition = "Su-Tu 11:00-01:00, We-Th 11:00-03:00, Fr 11:00-06:00, Sa 11:00-07:00";
  ASSERT_EQ(get_time_range(condition).size(), 4);
  TryConditionalRestrictions(condition, 0, 0, 0b0000111, {0, 0, 0, 11, 0}, {0, 0, 0, 1, 0});
  TryConditionalRestrictions(condition, 3, 0, 0b1000000, {0, 0, 0, 11, 0}, {0, 0, 0, 7, 0});

  // the fallback rule is a rule of its own, and the quoted text in it is a comment
  ASSERT_EQ(get_time_range("Mo-Sa 08:00-13:00,14:00-17:00 || \"by appointment\"").size(), 2);

  // an `off` rule takes time away from the rules before it, which a TimeDomain cannot express.
  // Dropping it leaves the wider restriction in place, so Tuesday stays covered here
  ASSERT_EQ(get_time_range("Mo-Sa 10:00-20:00; Tu off").size(), 1);
  TryConditionalRestrictions("Mo-Sa 10:00-20:00; Tu off", 0, 0, 0b1111110, {0, 0, 0, 10, 0},
                             {0, 0, 0, 20, 0});
  // spelled out as a narrower rule instead, both of them are kept
  ASSERT_EQ(get_time_range("Mo-Sa 10:00-20:00; Tu 10:00-14:00").size(), 2);
  // the same for a date that is excluded, only the positive rules survive
  ASSERT_EQ(get_time_range("Mo-Su 08:00-18:00; Apr 10-15 off; Jun 08:00-14:00; Aug off; Dec 25 off")
                .size(),
            2);
}

// hours past midnight say the range continues into the next day
TEST(TimeParsing, ExtendedHours) {
  ASSERT_EQ(get_time_range("Mo 20:00-26:00").size(), 1);
  TryConditionalRestrictions("Mo 20:00-26:00", 0, 0, 2, {0, 0, 0, 20, 0}, {0, 0, 0, 2, 0});
  // the same opening hours written the two other ways the wiki lists
  TryConditionalRestrictions("Mo 20:00-02:00", 0, 0, 2, {0, 0, 0, 20, 0}, {0, 0, 0, 2, 0});
  const std::string split = "Mo 20:00-24:00, Tu 00:00-02:00";
  ASSERT_EQ(get_time_range(split).size(), 2);
  TryConditionalRestrictions(split, 0, 0, 2, {0, 0, 0, 20, 0}, {0, 0, 0, 0, 0});
  TryConditionalRestrictions(split, 1, 0, 4, {0, 0, 0, 0, 0}, {0, 0, 0, 2, 0});
  // a range longer than a day has nothing to repeat daily
  EXPECT_TRUE(get_time_range("Mo-Su 05:00-46:00").empty());
  EXPECT_TRUE(get_time_range("Mo-Su 00:00-48:00").empty());
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
