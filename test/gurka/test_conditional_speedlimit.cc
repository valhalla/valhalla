#include "baldr/datetime.h"
#include "baldr/timedomain.h"
#include "gurka.h"
#include "test.h"

#include <vector>

using namespace valhalla;

TimeDomain from_proto(const TripLeg_TimeDomain& proto) {
  TimeDomain td;
  td.set_type(proto.day_dow_type());
  td.set_dow(proto.dow_mask());
  td.set_begin_hrs(proto.begin_hrs());
  td.set_begin_mins(proto.begin_mins());
  td.set_begin_month(proto.begin_month());
  td.set_begin_day_dow(proto.begin_day_dow());
  td.set_begin_week(proto.begin_week());
  td.set_end_hrs(proto.end_hrs());
  td.set_end_mins(proto.end_mins());
  td.set_end_month(proto.end_month());
  td.set_end_day_dow(proto.end_day_dow());
  td.set_end_week(proto.end_week());
  return td;
}

namespace valhalla {
bool operator==(const TripLeg_TimeDomain& proto, const TimeDomain& td) {
  return td.td_value() == from_proto(proto).td_value();
}

std::ostream& operator<<(std::ostream& os, TripLeg_TimeDomain const& proto) {
  return os << from_proto(proto).to_string();
}

namespace baldr {
std::ostream& operator<<(std::ostream& os, TimeDomain const& td) {
  return os << td.to_string();
}
} // namespace baldr
} // namespace valhalla

class ConditionalSpeedlimit : public ::testing::Test {
protected:
  static gurka::map map;

  static void SetUpTestSuite() {
    constexpr double grid_size_meters = 100;

    const std::string ascii_map = R"(
               M
              /|\
             / 3 2
            /  |  \
    A------B-------C---1--D
    E------F-------G------H
            \  |  /
             \ | /
              \|/
               N
               |
               4
               |
               O--5--P-6-R
                         |
                         7
                         |
                         Q
    )";

    const gurka::ways ways = {
        {"DCBA",
         {
             {"ref", "A20"},
             {"highway", "motorway"},
             {"oneway", "yes"},
             {"maxspeed", "100"},
             {"maxspeed:conditional", "120 @ (19:00-06:00)"},
         }},
        {"EFGH",
         {
             {"ref", "A20"},
             {"highway", "motorway"},
             {"oneway", "yes"},
             {"maxspeed", "100"},
             {"maxspeed:conditional", "120 @ (19:00-06:00)"},
         }},

        {"MB", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"maxspeed", "100"}}},
        {"CM", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"maxspeed", "100"}}},
        {"NG", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"maxspeed", "100"}}},
        {"FN", {{"highway", "motorway_link"}, {"oneway", "yes"}, {"maxspeed", "100"}}},

        {"MNO",
         {
             {"highway", "secondary"},
             {"maxspeed", "80"},
             {"maxspeed:conditional", "70 @ (Jun-Aug Sa,Su 08:00-20:00)"},
         }},

        {"OP", {{"highway", "tertiary"}, {"maxspeed", "50"}}},
        {"PR",
         {
             {"highway", "tertiary"},
             {"maxspeed", "50"},
             {"maxspeed:conditional", "30 @ (Mo-Fr 07:00-16:00; Sa 09:00-16:00; Su 09:00-16:00)"},
         }},
        {"RQ",
         {
             {"highway", "tertiary"},
             {"maxspeed", "50"},
             {"maxspeed:conditional", "20 @ (Dec Fr[-1]-Jan Mo[2])"},
         }},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, grid_size_meters);
    map = gurka::buildtiles(layout, ways, {}, {},
                            VALHALLA_BUILD_DIR "test/data/conditional_speedlimits",
                            {{"mjolnir.timezone", {VALHALLA_BUILD_DIR "test/data/tz.sqlite"}}});
  }
};

gurka::map ConditionalSpeedlimit::map = {};

/*************************************************************/

TEST_F(ConditionalSpeedlimit, RouteApiProto) {
  const auto result = gurka::do_action(valhalla::Options::route, map, {"D", "Q"}, "auto",
                                       {{"/date_time/value", "2024-08-15T21:00"}});
  ASSERT_EQ(result.trip().routes_size(), 1);
  ASSERT_EQ(result.trip().routes(0).legs_size(), 1);
  const auto leg = result.trip().routes(0).legs(0);
  ASSERT_EQ(leg.node_size(), 8);

  gurka::assert::raw::expect_path(result, {"A20/DCBA", "CM", "MNO", "MNO", "OP", "PR", "RQ"});
  // Regular speed limit are not affected, even if date was passed
  EXPECT_EQ(leg.node(0).edge().speed_limit(), 100); // DC
  EXPECT_EQ(leg.node(1).edge().speed_limit(), 100); // CM
  EXPECT_EQ(leg.node(2).edge().speed_limit(), 80);  // MN
  EXPECT_EQ(leg.node(3).edge().speed_limit(), 80);  // NO
  EXPECT_EQ(leg.node(4).edge().speed_limit(), 50);  // OP
  EXPECT_EQ(leg.node(5).edge().speed_limit(), 50);  // PR
  EXPECT_EQ(leg.node(6).edge().speed_limit(), 50);  // RQ
  EXPECT_EQ(leg.node(7).edge().speed_limit(), 0);   // the end

  // Now check conditional speed limits
  // 120 @ (19:00-06:00)
  ASSERT_EQ(leg.node(0).edge().conditional_speed_limits_size(), 1); // one condition on DC
  EXPECT_EQ(leg.node(0).edge().conditional_speed_limits(0).speed_limit(), 120);
  {
    baldr::TimeDomain condition;
    condition.set_begin_hrs(19);
    condition.set_end_hrs(6);
    EXPECT_EQ(leg.node(0).edge().conditional_speed_limits(0).condition(), condition);
  }

  EXPECT_EQ(leg.node(1).edge().conditional_speed_limits_size(), 0);

  // 90 @ (Jun-Aug Sa,Su 08:00-20:00)
  {
    baldr::TimeDomain condition;
    condition.set_begin_month(6);
    condition.set_end_month(8);
    condition.set_dow(0b1000001); // Week starts from kSunday
    condition.set_begin_hrs(8);
    condition.set_end_hrs(20);

    ASSERT_EQ(leg.node(2).edge().conditional_speed_limits_size(), 1);
    EXPECT_EQ(leg.node(2).edge().conditional_speed_limits(0).speed_limit(), 70);
    EXPECT_EQ(leg.node(2).edge().conditional_speed_limits(0).condition(), condition);

    ASSERT_EQ(leg.node(3).edge().conditional_speed_limits_size(), 1);
    EXPECT_EQ(leg.node(3).edge().conditional_speed_limits(0).speed_limit(), 70);
    EXPECT_EQ(leg.node(3).edge().conditional_speed_limits(0).condition(), condition);
  }

  EXPECT_EQ(leg.node(4).edge().conditional_speed_limits_size(), 0);

  // "30 @ (Mo-Fr 07:00-16:00; Sa 09:00-16:00; Su 09:00-16:00)"}
  EXPECT_EQ(leg.node(5).edge().conditional_speed_limits_size(), 3); // three time ranges on PR
  std::vector<TripLeg_TimeDomain> condtitions;
  for (const auto& conditional_speed_limit : leg.node(5).edge().conditional_speed_limits()) {
    EXPECT_EQ(conditional_speed_limit.speed_limit(), 30);
    condtitions.emplace_back(conditional_speed_limit.condition());
  }
  // Conditions are stored unsorted, so let's sort them for comparison
  std::sort(condtitions.begin(), condtitions.end(), [](const auto& lha, const auto& rha) {
    if (lha.dow_mask() < rha.dow_mask()) {
      return true;
    } else if (lha.dow_mask() > rha.dow_mask()) {
      return false;
    } else if (lha.begin_hrs() < rha.begin_hrs()) {
      return true;
    } else if (lha.begin_hrs() > rha.begin_hrs()) {
      return false;
    } else if (lha.end_hrs() < rha.end_hrs()) {
      return true;
    } else {
      return false;
    }
  });

  {
    // Su 09:00-16:00
    baldr::TimeDomain condition;
    condition.set_dow(0b0000001); // Week starts from kSunday
    condition.set_begin_hrs(9);
    condition.set_end_hrs(16);
    EXPECT_EQ(condtitions[0], condition);
  }
  {
    // Mo-Fr 07:00-16:00
    baldr::TimeDomain condition;
    condition.set_dow(0b0111110); // Week starts from kSunday
    condition.set_begin_hrs(7);
    condition.set_end_hrs(16);
    EXPECT_EQ(condtitions[1], condition);
  }
  {
    // Sa 09:00-16:00
    baldr::TimeDomain condition;
    condition.set_dow(0b1000000); // Week starts from kSunday
    condition.set_begin_hrs(9);
    condition.set_end_hrs(16);
    EXPECT_EQ(condtitions[2], condition);
  }

  ASSERT_EQ(leg.node(6).edge().conditional_speed_limits_size(), 1);
  EXPECT_EQ(leg.node(6).edge().conditional_speed_limits(0).speed_limit(), 20);
  {
    // Dec Fr[-1]-Jan Mo[2]
    baldr::TimeDomain condition;
    condition.set_type(kNthDow);
    condition.set_dow(0b1111111);   // Magic mask that means every day, don't ask why it is not 0
    condition.set_begin_month(12);  // Dec
    condition.set_begin_day_dow(6); // Fr
    condition.set_begin_week(5);    // [-1]
    condition.set_end_month(1);     // Jan
    condition.set_end_day_dow(2);   // Mo
    condition.set_end_week(2);      // [2]
    EXPECT_EQ(leg.node(6).edge().conditional_speed_limits(0).condition(), condition);
  }

  EXPECT_EQ(leg.node(7).edge().conditional_speed_limits_size(), 0);
}

TEST_F(ConditionalSpeedlimit, LocateApiJson) {
  auto reader = test::make_clean_graphreader(map.config.get_child("mjolnir"));
  std::string json;
  const auto result =
      gurka::do_action(valhalla::Options::locate, map, {"1", "2", "3", "4", "5", "6", "7"}, "auto",
                       {}, reader, &json);

  rapidjson::Document root;
  root.Parse(json);
  ASSERT_FALSE(root.HasParseError()) << json;
  ASSERT_EQ(root.GetArray().Size(), 7);

  const auto& edge_info_0 = *rapidjson::Pointer("/0/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_0["speed_limit"], 100);
  ASSERT_TRUE(edge_info_0.HasMember("conditional_speed_limits"));
  for (const auto& [k, v] : edge_info_0["conditional_speed_limits"].GetObject()) {
    EXPECT_STREQ(k.GetString(), "19:00-06:00");
    EXPECT_EQ(v, 120);
  }

  const auto& edge_info_1 = *rapidjson::Pointer("/1/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_1["speed_limit"], 100);
  EXPECT_FALSE(edge_info_1.HasMember("conditional_speed_limits"));

  const auto& edge_info_2 = *rapidjson::Pointer("/2/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_2["speed_limit"], 80);
  ASSERT_TRUE(edge_info_2.HasMember("conditional_speed_limits"));
  for (const auto& [k, v] : edge_info_2["conditional_speed_limits"].GetObject()) {
    EXPECT_STREQ(k.GetString(), "Jun-Aug Su,Sa 08:00-20:00");
    EXPECT_EQ(v, 70);
  }

  const auto& edge_info_3 = *rapidjson::Pointer("/3/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_3["speed_limit"], 80);
  ASSERT_TRUE(edge_info_3.HasMember("conditional_speed_limits"));
  for (const auto& [k, v] : edge_info_3["conditional_speed_limits"].GetObject()) {
    EXPECT_STREQ(k.GetString(), "Jun-Aug Su,Sa 08:00-20:00");
    EXPECT_EQ(v, 70);
  }

  const auto& edge_info_4 = *rapidjson::Pointer("/4/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_4["speed_limit"], 50);
  EXPECT_FALSE(edge_info_4.HasMember("conditional_speed_limits"));

  const auto& edge_info_5 = *rapidjson::Pointer("/5/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_5["speed_limit"], 50);
  ASSERT_TRUE(edge_info_5.HasMember("conditional_speed_limits"));

  // Conditions might be in different order, let's sort them and then check
  std::vector<std::string> conditions_str;
  for (const auto& [name, value] : edge_info_5["conditional_speed_limits"].GetObject()) {
    conditions_str.push_back(name.GetString());
    EXPECT_EQ(value, 30);
  }
  std::sort(conditions_str.begin(), conditions_str.end());
  ASSERT_EQ(conditions_str.size(), 3);
  EXPECT_EQ(conditions_str[0], "Mo-Fr 07:00-16:00");
  EXPECT_EQ(conditions_str[1], "Sa 09:00-16:00");
  EXPECT_EQ(conditions_str[2], "Su 09:00-16:00");

  const auto& edge_info_6 = *rapidjson::Pointer("/6/edges/0/edge_info").Get(root);
  EXPECT_EQ(edge_info_6["speed_limit"], 50);
  ASSERT_TRUE(edge_info_6.HasMember("conditional_speed_limits"));
  for (const auto& [k, v] : edge_info_6["conditional_speed_limits"].GetObject()) {
    EXPECT_STREQ(k.GetString(), "Dec Fr[-1]-Jan Mo[2]");
    EXPECT_EQ(v, 20);
  }
}
