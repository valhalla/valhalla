#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>

#include <boost/algorithm/string/split.hpp>

#include "baldr/datetime.h"
#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "mjolnir/timeparsing.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

std::vector<std::string> GetTagTokens(const std::string& tag_value, const char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(
      tokens, tag_value, [delim](const char c) { return c == delim; },
      boost::algorithm::token_compress_on);
  return tokens;
}

void TryConditionalRestrictions(const std::string& condition,
                                const std::vector<uint64_t>& expected_values) {

  std::vector<uint64_t> results = get_time_range(condition);

  for (uint32_t x = 0; x < results.size(); x++) {
    TimeDomain res = TimeDomain(results.at(x));

    /* used for creating new tests.
    std::cout << condition << " type " << res.type() << " dow " << res.dow() << " begin month " <<
    res.begin_month() << " begin day " << res.begin_day() << " begin week " <<
    res.begin_week() << " begin hrs " << res.begin_hrs() << " begin mins " <<
    res.begin_mins() << " end month " << res.end_month() << " end day " <<
    res.end_day() << " end week " << res.end_week() << " end hrs " <<
    res.end_hrs() << " end mins " << res.end_mins() << std::endl;*/

    EXPECT_EQ(res.td_value(), expected_values.at(x))
        << "Time domain " + condition +
               " test failed.  Expected: " + std::to_string(expected_values.at(x)) +
               " but received " + std::to_string(res.td_value());
  }
}

void TryConditionalRestrictions(const std::string& condition,
                                const uint32_t index,
                                const uint32_t type,
                                const uint32_t dow,
                                const uint32_t begin_month,
                                const uint32_t begin_day,
                                const uint32_t begin_week,
                                const uint32_t begin_hrs,
                                const uint32_t begin_mins,
                                const uint32_t end_month,
                                const uint32_t end_day,
                                const uint32_t end_week,
                                const uint32_t end_hrs,
                                const uint32_t end_mins) {

  std::vector<uint64_t> results = get_time_range(condition);

  TimeDomain res = TimeDomain(results.at(index));

  EXPECT_EQ(res.type(), type);
  EXPECT_EQ(res.dow(), dow);
  EXPECT_EQ(res.begin_month(), begin_month);
  EXPECT_EQ(res.begin_day_dow(), begin_day);
  EXPECT_EQ(res.begin_week(), begin_week);
  EXPECT_EQ(res.begin_hrs(), begin_hrs);
  EXPECT_EQ(res.begin_mins(), begin_mins);
  EXPECT_EQ(res.end_month(), end_month);
  EXPECT_EQ(res.end_day_dow(), end_day);
  EXPECT_EQ(res.end_week(), end_week);
  EXPECT_EQ(res.end_hrs(), end_hrs);
  EXPECT_EQ(res.end_mins(), end_mins);

  if (::testing::Test::HasFailure()) {
    std::cerr << "Time domain: " << condition << std::endl;
  }
}

} // namespace

TEST(TimeParsing, TestConditionalRestrictions) {

  std::string str = "Mo-Fr 06:00-11:00,17:00-19:00;Sa 03:30-19:00";
  std::vector<std::string> conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // Mo-Fr 06:00-11:00,17:00-19:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(23622321788);
      expected_values.push_back(40802193788);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), expected_values);

      TryConditionalRestrictions(conditions.at(x), 0, 0, 62, 0, 0, 0, 6, 0, 0, 0, 0, 11, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 62, 0, 0, 0, 17, 0, 0, 0, 0, 19, 0);

    } else if (x == 1) { // Sa 03:30-19:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(40802435968);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 0, 0, 0, 3, 30, 0, 0, 0, 19, 0);
    }
  }

  str = "Mo,We,Th,Fr 12:00-18:00; Sa-Su 12:00-17:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // Mo,We,Th,Fr 12:00-18:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(38654708852);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 58, 0, 0, 0, 12, 0, 0, 0, 0, 18, 0);

    } else if (x == 1) { // Sa-Su 12:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(36507225218);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 65, 0, 0, 0, 12, 0, 0, 0, 0, 17, 0);
    }
  }

  str = "July 23-Aug 21 Sa 14:00-20:00;JUL 23-jUl 28 Fr,PH 10:00-20:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {

    if (x == 0) { // July 23-Aug 21 Sa 14:00-20:00;
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1512971146104448);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 7, 23, 0, 14, 0, 8, 21, 0, 20, 0);

    } else if (x == 1) { // JUL 23-jUl 28 Fr,PH 10:00-20:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2001154308835904);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 32, 7, 23, 0, 10, 0, 7, 28, 0, 20, 0);
    }
  }

  str = "Apr-Sep Mo-Fr 09:00-13:00,14:00-18:00; Apr-Sep Sa 10:00-13:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Apr-Sep Mo-Fr 09:00-13:00,14:00-18:00;

      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337986940);
      expected_values.push_back(39621075406460);
      TryConditionalRestrictions(conditions.at(x), expected_values);

    } else if (x == 1) { // Apr-Sep Sa 10:00-13:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337987200);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 4, 0, 0, 10, 0, 9, 0, 0, 13, 0);
    }
  }

  str = "Apr-Sep: Monday-Fr 09:00-13:00,14:00-18:00; ApRil-Sept: Sa 10:00-13:00";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Apr-Sep: Monday-Fr 09:00-13:00,14:00-18:00;

      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337986940);
      expected_values.push_back(39621075406460);
      TryConditionalRestrictions(conditions.at(x), expected_values);

    } else if (x == 1) { // ApRil-Sept: Sa 10:00-13:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(39610337987200);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 64, 4, 0, 0, 10, 0, 9, 0, 0, 13, 0);
    }
  }

  str = "06:00-11:00,17:00-19:45";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // 06:00-11:00,17:00-19:45

      std::vector<uint64_t> expected_values;
      expected_values.push_back(23622321664);
      expected_values.push_back(3133178646784);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 11, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 0, 0, 0, 0, 17, 0, 0, 0, 0, 19, 45);
    }
  }

  str = " Feb 16-Oct 15 09:00-18:30; Oct 16-Nov 15: 09:00-17:30; Nov 16-Feb 15: 09:00-16:30";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Feb 16-Oct 15 09:00-18:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1101612002052352);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    } else if (x == 1) { // Oct 16-Nov 15: 09:00-17:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1106007905274112);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 10, 16, 0, 9, 0, 11, 15, 0, 17, 30);

    } else if (x == 2) { // Nov 16-Feb 15: 09:00-16:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1066423339714816);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 11, 16, 0, 9, 0, 02, 15, 0, 16, 30);
    }
  }

  str = "th 07:00-08:30; th-friday 06:00-09:30; May 15 09:00-11:30; May 07:00-08:30; May 16-31 "
        "11:00-13:30";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // th 07:00-08:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2078764173088);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 16, 0, 0, 0, 7, 0, 0, 0, 0, 8, 30);
    } else if (x == 1) { // th-friday 06:00-09:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2080911656544);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 48, 0, 0, 0, 6, 0, 0, 0, 0, 9, 30);
    } else if (x == 2) { // May 15 09:00-11:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(1079606730295552);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 15, 0, 9, 0, 5, 15, 0, 11, 30);
    } else if (x == 3) { // May 07:00-08:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(24068999350016);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 0, 0, 7, 0, 5, 0, 0, 8, 30);
    } else if (x == 4) { // May 16-31 11:00-13:30
      std::vector<uint64_t> expected_values;
      expected_values.push_back(2205510940494592);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 5, 16, 0, 11, 0, 5, 31, 0, 13, 30);
    }
  }

  str = "(Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50;Sep-Jun We 08:15-08:45,11:55-12:35)";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(29497840232556);
      expected_values.push_back(29856470044524);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 54, 9, 0, 0, 8, 15, 6, 0, 0, 8, 45);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 54, 9, 0, 0, 15, 20, 6, 0, 0, 15, 50);
    } else if (x == 1) { // Sep-Jun We 08:15-08:45,11:55-12:35
      std::vector<uint64_t> expected_values;
      expected_values.push_back(29497840232464);
      expected_values.push_back(28819235728144);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 8, 9, 0, 0, 8, 15, 6, 0, 0, 8, 45);
      TryConditionalRestrictions(conditions.at(x), 1, 0, 8, 9, 0, 0, 11, 55, 6, 0, 0, 12, 35);
    }
  }

  str = "Oct Su[-1]-Mar th[4] (Su 09:00-16:00; PH 09:00-16:00);Mar Su[-1]-Oct Su[-1] (Su "
        "09:00-18:00; PH 09:00-18:00)";
  conditions = GetTagTokens(str, ';');

  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Oct Su[-1]-Mar th[4] (Su 09:00-16:00; PH 09:00-16:00)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(9372272830712067);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 10, 1, 5, 9, 0, 3, 5, 4, 16, 0);
    } else if (x == 1) { // PH 09:00-16:00  Holidays are tossed for now
      std::vector<uint64_t> expected_values;
      expected_values.push_back(0);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    } else if (x == 2) { // Mar Su[-1]-Oct Su[-1] (Su 09:00-18:00; PH 09:00-18:00)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11373388284561667);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 3, 1, 5, 9, 0, 10, 1, 5, 18, 0);
    } else if (x == 3) { // PH 09:00-18:00  Holidays are tossed for now
      std::vector<uint64_t> expected_values;
      expected_values.push_back(0);
      TryConditionalRestrictions(conditions.at(x), expected_values);
    }
  }

  str = "Dec Fr[-1]-Jan Sa[3] Su,Sat 09:00-16:00, 15:00-17:00; Dec Su[-1] Su-Sa 15:00-17:00";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sep-Jun Mo,Tu,Th,Fr 08:15-08:45,15:20-15:50)
      std::vector<uint64_t> expected_values;
      expected_values.push_back(7252414455351683);
      expected_values.push_back(7252416602836867);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 65, 12, 6, 5, 9, 0, 1, 7, 3, 16, 0);
      TryConditionalRestrictions(conditions.at(x), 1, 1, 65, 12, 6, 5, 15, 0, 1, 7, 3, 17, 0);
    } else if (x == 1) { // Dec Su[-1] Su-Sa 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642943);

      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    }
  }

  str =
      "Sun 09:00-16:00; Su[1]; Dec; Dec Su[-1] 15:00-17:00; Dec Su[-1] Th 15:00-17:00;"
      "Dec Su[-1]; Dec Su[-1]-Mar 3 Sat;Mar 3-Dec Su[-1] Sat;Dec Su[-1]-Mar 3 Sat 15:00-17:00;"
      "Mar 3-Dec Su[-1] Sat 15:00-17:00; Mar 3-Dec Su[-1] Sat,PH 15:00-17:00; Mar 3-Dec Su[-1] PH,Sat 15:00-17:00";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Sun 09:00-16:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(34359740674);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 1, 0, 0, 0, 9, 0, 0, 0, 0, 16, 0);
    } else if (x == 1) { // Su[1]
      std::vector<uint64_t> expected_values;
      expected_values.push_back(268435459);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
    } else if (x == 2) { // Dec
      std::vector<uint64_t> expected_values;
      expected_values.push_back(52776564424704);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 0, 0, 12, 0, 0, 0, 0, 12, 0, 0, 0, 0);
    } else if (x == 3) { // Dec Su[-1] 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642943);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    } else if (x == 4) { // Dec Su[-1] Th 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311813490642721);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 16, 12, 1, 5, 15, 0, 12, 0, 5, 17, 0);
    } else if (x == 5) { // Dec Su[-1]
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11311776983417087);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 127, 12, 1, 5, 0, 0, 12, 0, 5, 0, 0);
    } else if (x == 6) { // Dec Su[-1]-Mar 3 Sat
      std::vector<uint64_t> expected_values;
      expected_values.push_back(224301728923777);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 12, 1, 5, 0, 0, 3, 3, 0, 0, 0);
    } else if (x == 7) { // Mar 3-Dec Su[-1] Sat
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382144397475969);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 0, 0, 12, 1, 5, 0, 0);
    } else if (x == 8) { // Dec Su[-1]-Mar 3 Sat 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(224338236149633);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 12, 1, 5, 15, 0, 3, 3, 0, 17, 0);
    } else if (x == 9) { // Mar 3-Dec Su[-1] Sat 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382180904701825);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 15, 0, 12, 1, 5, 17, 0);
    } else if (x == 10) { // Mar 3-Dec Su[-1] Sat,PH 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382180904701825);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 15, 0, 12, 1, 5, 17, 0);
    } else if (x == 11) { // Mar 3-Dec Su[-1] PH,Sat 15:00-17:00
      std::vector<uint64_t> expected_values;
      expected_values.push_back(11382180904701825);
      TryConditionalRestrictions(conditions.at(x), expected_values);
      TryConditionalRestrictions(conditions.at(x), 0, 1, 64, 3, 3, 0, 15, 0, 12, 1, 5, 17, 0);
    }
  }

  str = "Mon;Wed;Fr;Friday-Friday";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) {
      TryConditionalRestrictions(conditions.at(x), {4});
      TryConditionalRestrictions(conditions.at(x), 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    } else if (x == 1) {
      TryConditionalRestrictions(conditions.at(x), {16});
      TryConditionalRestrictions(conditions.at(x), 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    } else if (x == 2 || x == 3) {
      TryConditionalRestrictions(conditions.at(x), {64});
      TryConditionalRestrictions(conditions.at(x), 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  }

  str = "-Friday"; // Invalid input, no start day is provided.
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    TryConditionalRestrictions(conditions.at(x), {});
  }

  str = "monday-friday 7:00-9:30,13:00-15:00";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    TryConditionalRestrictions(conditions.at(x), {2080911656828, 32212258172});
    TryConditionalRestrictions(conditions.at(x), 0, 0, 62, 0, 0, 0, 7, 0, 0, 0, 0, 9, 30);
    TryConditionalRestrictions(conditions.at(x), 1, 0, 62, 0, 0, 0, 13, 0, 0, 0, 0, 15, 0);
  }

  // includes end of year
  str = "Jan 04-Jan 01 Mo-Sa;Jan 04-Jan 01 22:00-24:00;Jan 04-Jan 01";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Jan 04-Jan 01 Mo-Sa
      TryConditionalRestrictions(conditions.at(x), {74766824767740});
    } else if (x == 1) { // Jan 04-Jan 01 22:00-24:00
      TryConditionalRestrictions(conditions.at(x), {74766824773120});
    } else if (x == 2) { // Jan 04-Jan 01
      TryConditionalRestrictions(conditions.at(x), {74766824767488});
    }
  }

  // ranges without time
  str = "Mon-Friday;Mo,Wed;March-May;March 18-April 30";
  conditions = GetTagTokens(str, ';');
  for (uint32_t x = 0; x < conditions.size(); x++) {
    if (x == 0) { // Mon-Friday
      TryConditionalRestrictions(conditions.at(x), {124});
    } else if (x == 1) { // Mo,Wed
      TryConditionalRestrictions(conditions.at(x), {20});
    } else if (x == 2) { // March-May
      TryConditionalRestrictions(conditions.at(x), {21990234128384});
    } else if (x == 3) { // March 18-April 30
      TryConditionalRestrictions(conditions.at(x), {2128654663942144});
    }
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
