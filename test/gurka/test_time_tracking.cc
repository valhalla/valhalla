#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "gurka.h"
#include "loki/search.h"
#include "proto/api.pb.h"
#include "sif/costfactory.h"
#include "thor/pathalgorithm.h"

#include <gtest/gtest.h>

using namespace valhalla;
namespace dt = valhalla::baldr::DateTime;

TEST(TimeTracking, make) {

  // build a very simple graph
  const std::string ascii_map = R"(A----B)";
  const gurka::ways ways = {{"AB", {{"highway", "trunk"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_time_tracking_make");

  // need to access the tiles
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // get some loki results
  sif::CostFactory<sif::DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  Options options;
  options.set_costing(Costing::none_);
  auto costing = factory.Create(options);
  auto found = loki::Search({baldr::Location(map.nodes.begin()->second)}, reader, costing);
  auto* location = options.add_locations();
  baldr::PathLocation::toPBF(found.begin()->second, location, reader);

  // no time
  auto ti = thor::TimeInfo::make(*location, reader);
  ASSERT_EQ(ti, thor::TimeInfo{});
  ASSERT_FALSE(location->has_date_time());

  // current time (technically we could fail if the minute changes between the next 3 lines)
  location->set_date_time("current");
  ti = thor::TimeInfo::make(*location, reader);
  auto now_str = dt::iso_date_time(dt::get_tz_db().from_index(1));
  auto lt = dt::seconds_since_epoch(now_str, dt::get_tz_db().from_index(1));
  std::tm t = dt::iso_to_tm(now_str);
  std::mktime(&t);
  auto sec = t.tm_wday * valhalla::midgard::kSecondsPerDay +
             t.tm_hour * valhalla::midgard::kSecondsPerHour + t.tm_sec;
  ASSERT_EQ(ti, (thor::TimeInfo{true, 1, lt, sec, 0}));
  ASSERT_EQ(location->date_time(), now_str);

  // not current time but the same date time just set as a string
  now_str = dt::iso_date_time(dt::get_tz_db().from_index(1));
  location->set_date_time(now_str);
  ti = thor::TimeInfo::make(*location, reader);
  lt = dt::seconds_since_epoch(now_str, dt::get_tz_db().from_index(1));
  t = dt::iso_to_tm(now_str);
  std::mktime(&t);
  sec = t.tm_wday * valhalla::midgard::kSecondsPerDay +
        t.tm_hour * valhalla::midgard::kSecondsPerHour + t.tm_sec;
  ASSERT_EQ(ti, (thor::TimeInfo{true, 1, lt, sec, 0}));
  ASSERT_EQ(location->date_time(), now_str);

  // offset the time from now a bit
  now_str = dt::iso_date_time(dt::get_tz_db().from_index(1));
  auto minutes = std::atoi(now_str.substr(now_str.size() - 2, 2).c_str());
  int offset = 7;
  if (minutes + offset > 60)
    offset = -offset;
  now_str = now_str.substr(0, now_str.size() - 2) + std::to_string(minutes + offset);
  location->set_date_time(now_str);
  ti = thor::TimeInfo::make(*location, reader);
  lt = dt::seconds_since_epoch(now_str, dt::get_tz_db().from_index(1));
  t = dt::iso_to_tm(now_str);
  std::mktime(&t);
  sec = t.tm_wday * valhalla::midgard::kSecondsPerDay +
        t.tm_hour * valhalla::midgard::kSecondsPerHour + t.tm_sec;
  ASSERT_EQ(ti, (thor::TimeInfo{true, 1, lt, sec, offset * 60}));
  ASSERT_EQ(location->date_time(), now_str);

  // messed up date time
  location->set_date_time("4000BC");
  ti = thor::TimeInfo::make(*location, reader);
  ASSERT_EQ(ti, thor::TimeInfo{});
  ASSERT_EQ(location->date_time(), "4000BC");
}

TEST(TimeTracking, increment) {
  // invalid should stay that way
  auto ti = thor::TimeInfo{false} + thor::TimeInfo::Offset{10, 1};
  ASSERT_EQ(ti, (thor::TimeInfo{false}));

  // change in timezone should result in some offset (LA to NY)
  ti = thor::TimeInfo{true, 94, 123456789} + thor::TimeInfo::Offset{10, 110};
  ASSERT_EQ(ti, (thor::TimeInfo{true, 110, 123456789 + 10 + 60 * 60 * 3, 10 + 60 * 60 * 3, 10}));

  // wrap around second of week
  ti = thor::TimeInfo{true, 1, 2, midgard::kSecondsPerWeek - 5} + thor::TimeInfo::Offset{10, 1};
  ASSERT_EQ(ti, (thor::TimeInfo{true, 1, 12, 5, 10}));
}

TEST(TimeTracking, decrement) {
  // invalid should stay that way
  auto ti = thor::TimeInfo{false} - thor::TimeInfo::Offset{10, 1};
  ASSERT_EQ(ti, (thor::TimeInfo{false}));

  // change in timezone should result in some offset (NY to LA)
  ti = thor::TimeInfo{true, 110, 123456789} - thor::TimeInfo::Offset{10, 94};
  ASSERT_EQ(ti, (thor::TimeInfo{true, 94, 123456789 - 10 - 60 * 60 * 3,
                                midgard::kSecondsPerWeek - 10 - 60 * 60 * 3, -10}));

  // wrap around second of week
  ti = thor::TimeInfo{true, 1, 22, 5} - thor::TimeInfo::Offset{10, 1};
  ASSERT_EQ(ti, (thor::TimeInfo{true, 1, 12, midgard::kSecondsPerWeek - 5, -10}));
}