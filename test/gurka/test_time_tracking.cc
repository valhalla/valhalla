#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/time_info.h"
#include "boost/format.hpp"
#include "gurka.h"
#include "loki/search.h"
#include "proto/api.pb.h"
#include "sif/costfactory.h"
#include "tyr/actor.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace valhalla;
namespace dt = valhalla::baldr::DateTime;

TEST(TimeTracking, make) {
  // build a very simple graph
  const std::string ascii_map = R"(A----B)";
  const gurka::ways ways = {{"AB", {{"highway", "trunk"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_time_tracking_make",
                               {{"mjolnir.timezone", "/path/to/timezone.sqlite"}});

  // need to access the tiles
  baldr::GraphReader reader(map.config.get_child("mjolnir"));

  // this is what the default should be, with constrained second of day
  baldr::TimeInfo basic_ti{false, 0, 0, baldr::kInvalidSecondsOfWeek};

  // once without tz cache and once with
  for (auto* cache : std::vector<baldr::DateTime::tz_sys_info_cache_t*>{
           nullptr,
           new baldr::DateTime::tz_sys_info_cache_t,
       }) {
    // get some loki results
    auto costing = sif::CostFactory().Create(Costing::none_);
    auto found = loki::Search({baldr::Location(map.nodes.begin()->second)}, reader, costing);
    Location location;
    baldr::PathLocation::toPBF(found.begin()->second, &location, reader);

    // no time
    auto ti = baldr::TimeInfo::make(location, reader, cache);
    ASSERT_EQ(ti, basic_ti);
    ASSERT_TRUE(location.date_time().empty());

    // bad timezone defaults to UTC
    location.set_date_time("2020-04-01T12:34");
    ti = baldr::TimeInfo::make(location, reader, cache, 7777);
    // zero out the part we dont care to test
    ti.local_time = 0;
    ti.second_of_week = 0;
    ti.seconds_from_now = 0;
    ti.negative_seconds_from_now = 0;
    ASSERT_EQ(ti, (baldr::TimeInfo{1, 291}));
    ASSERT_EQ(location.date_time(), "2020-04-01T12:34");

    // current time (technically we could fail if the minute changes between the next 3 lines)
    location.set_date_time("current");
    ti = baldr::TimeInfo::make(location, reader, cache);
    const auto* tz = dt::get_tz_db().from_index(291);
    auto now_str = dt::iso_date_time(tz);
    auto lt = dt::seconds_since_epoch(now_str, tz);
    auto sec = dt::second_of_week(lt, tz);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 291, lt, sec, 0}));
    ASSERT_EQ(location.date_time(), now_str);

    // not current time but the same date time just set as a string
    now_str = dt::iso_date_time(tz);
    location.set_date_time(now_str);
    ti = baldr::TimeInfo::make(location, reader, cache);
    lt = dt::seconds_since_epoch(now_str, dt::get_tz_db().from_index(1));
    sec = dt::second_of_week(lt, tz);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 291, lt, sec, 0}));
    ASSERT_EQ(location.date_time(), now_str);

    // offset the time from now a bit
    now_str = dt::iso_date_time(tz);
    int minutes = std::atoi(now_str.substr(now_str.size() - 2, 2).c_str());
    int offset = minutes > 52 ? -7 : 7;
    now_str = now_str.substr(0, now_str.size() - 2) + (minutes + offset < 10 ? "0" : "") +
              std::to_string(minutes + offset);
    location.set_date_time(now_str);
    ti = baldr::TimeInfo::make(location, reader, cache);
    lt = dt::seconds_since_epoch(now_str, tz);
    sec = dt::second_of_week(lt, tz);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 291, lt, sec, static_cast<uint64_t>(std::abs(offset * 60)),
                                   offset < 0}));
    ASSERT_EQ(location.date_time(), now_str);

    // messed up date time
    location.set_date_time("4000BC");
    ti = baldr::TimeInfo::make(location, reader, cache);
    ASSERT_EQ(ti, basic_ti);
    ASSERT_EQ(location.date_time(), "4000BC");

    // user specified date time
    location.set_date_time("2020-03-31T11:16");
    ti = baldr::TimeInfo::make(location, reader, cache, dt::get_tz_db().to_index("America/New_York"));
    // zero out the part we dont care to test
    ti.seconds_from_now = 0;
    ti.negative_seconds_from_now = 0;
    ASSERT_EQ(ti, (baldr::TimeInfo{1, 110, 1585667787, 213387}));
    ASSERT_EQ(location.date_time(), "2020-03-31T11:16");
  }
}

TEST(TimeTracking, forward) {
  // once without tz cache and once with
  for (auto* cache : std::vector<baldr::DateTime::tz_sys_info_cache_t*>{
           nullptr,
           new baldr::DateTime::tz_sys_info_cache_t,
       }) {
    // invalid should stay that way
    auto ti = baldr::TimeInfo{false, 0, 0, 0, 0, 0, cache}.forward(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{false}));

    // change in timezone should result in some offset (LA to NY)
    ti = baldr::TimeInfo{true, 94, 123456789, 0, 0, 0, cache}.forward(10, 110);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 110, 123456789 + 10 + 60 * 60 * 3, 10 + 60 * 60 * 3, 10}));

    // change in timezone should result in some offset (NY to LA) wrap around backwards
    ti = baldr::TimeInfo{true, 110, 123456789, 0, 0, 0, cache}.forward(10, 94);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 94, 123456789 + 10 - 60 * 60 * 3,
                                   midgard::kSecondsPerWeek + 10 - 60 * 60 * 3, 10}));

    // wrap around second of week
    ti = baldr::TimeInfo{true, 1, 2, midgard::kSecondsPerWeek - 5, 0, 0, cache}.forward(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 12, 5, 10}));

    // cross now time
    ti = baldr::TimeInfo{true, 1, 2, 0, 5, 1, cache}.forward(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 12, 10, 5}));

    // dont cross now time
    ti = baldr::TimeInfo{true, 1, 2, 0, 5, 1, cache}.forward(2, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 4, 2, 3, 1}));
  }
}

TEST(TimeTracking, reverse) {
  // once without tz cache and once with
  for (auto* cache : std::vector<baldr::DateTime::tz_sys_info_cache_t*>{
           nullptr,
           new baldr::DateTime::tz_sys_info_cache_t,
       }) {
    // invalid should stay that way
    auto ti = baldr::TimeInfo{false, 0, 0, 0, 0, 0, cache}.reverse(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{false}));

    // change in timezone should result in some offset (NY to LA)
    ti = baldr::TimeInfo{true, 110, 123456789, 0, 0, 0, cache}.reverse(10, 94);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 94, 123456789 - 10 - 60 * 60 * 3,
                                   midgard::kSecondsPerWeek - 10 - 60 * 60 * 3, 10, 1}));

    // change in timezone should result in some offset (LA to NY)
    ti = baldr::TimeInfo{true, 94, 123456789, midgard::kSecondsPerWeek - 1, 0, 0, cache}.reverse(10,
                                                                                                 110);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 110, 123456789 - 10 + 60 * 60 * 3, -1 - 10 + 60 * 60 * 3, 10,
                                   1}));

    // wrap around second of week
    ti = baldr::TimeInfo{true, 1, 22, 5, 0, 0, cache}.reverse(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 12, midgard::kSecondsPerWeek - 5, 10, 1}));

    // cross now time
    ti = baldr::TimeInfo{true, 1, 22, midgard::kSecondsPerWeek - 1, 5, 0, cache}.reverse(10, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 12, midgard::kSecondsPerWeek - 11, 5, 1}));

    // dont cross now time
    ti = baldr::TimeInfo{true, 1, 22, midgard::kSecondsPerWeek - 1, 5, 0, cache}.reverse(2, 1);
    ASSERT_EQ(ti, (baldr::TimeInfo{true, 1, 20, midgard::kSecondsPerWeek - 3, 3}));
  }
}

TEST(TimeTracking, routes) {
  // build a very simple graph
  const std::string ascii_map = R"(A----B----C----D---I
                                                  |   |
                                                  1   |
                                                  |   |
                                   H----G----F----E---J)";
  const gurka::ways ways = {
      {"AB", {{"highway", "residential"}}}, {"BC", {{"highway", "residential"}}},
      {"CD", {{"highway", "residential"}}}, {"DE", {{"highway", "residential"}}},
      {"EF", {{"highway", "primary"}}},     {"FG", {{"highway", "primary"}}},
      {"GH", {{"highway", "primary"}}},     {"DI", {{"highway", "residential"}}},
      {"IJ", {{"highway", "primary"}}},     {"JE", {{"highway", "primary"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100, {5.1079374, 52.0887174});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_time_tracking_make",
                               {{"mjlonir.timezone", "test/data/tz.sqlite"}});

  const std::vector<double> expected = {0,       31.5771, 63.1543, 94.7314, 120.052, 0,
                                        91.6466, 105.024, 144.325, 159.079, 173.815, 188.551};

  auto test_result = [&expected](const std::vector<double>& actual_times, const Api& request) {
    ASSERT_THAT(actual_times, testing::Pointwise(testing::DoubleNear(0.001), expected));
    // first location has a waiting_time, second not since it was "through"
    EXPECT_EQ(request.options().locations(1).waiting_secs(), 600.f);
    EXPECT_EQ(request.options().locations(2).waiting_secs(), 0.f);
    EXPECT_EQ(request.info().warnings(0).code(), 203);
  };

  std::vector<double> times;
  auto result = gurka::do_action(Options::route, map, {"A", "I", "1", "H"}, "auto",
                                 {{"/date_time/type", "0"},
                                  {"/locations/1/waiting", "600"},
                                  {"/locations/2/type", "through"},
                                  {"/locations/2/waiting", "600"}});
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }
  test_result(times, result);

  // route between them with a depart_at
  result = gurka::do_action(Options::route, map, {"A", "I", "1", "H"}, "auto",
                            {{"/date_time/type", "1"},
                             {"/date_time/value", "1982-12-08T17:17"},
                             {"/locations/1/waiting", "600"},
                             {"/locations/2/type", "through"},
                             {"/locations/2/waiting", "600"}});
  times.clear();
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }
  test_result(times, result);

  // route between them with a arrive_by
  result = gurka::do_action(Options::route, map, {"A", "I", "1", "H"}, "auto",
                            {{"/date_time/type", "2"},
                             {"/date_time/value", "1982-12-08T17:17"},
                             {"/locations/1/waiting", "600"},
                             {"/locations/2/type", "through"},
                             {"/locations/2/waiting", "600"}});
  times.clear();
  for (const auto& route : result.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }
  test_result(times, result);
}

TEST(TimeTracking, dst) {
  // BST and GMT ambiguity where we gain an hour due to DST
  std::string date_time = "2020-10-25T01:57";
  int tz_idx = dt::get_tz_db().to_index("Europe/London");
  EXPECT_NO_THROW(baldr::TimeInfo::make(date_time, tz_idx))
      << " could not apply timezone to local time";
}
