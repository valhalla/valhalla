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
  baldr::TimeInfo basic_ti{false, 0, 0, baldr::kConstrainedFlowSecondOfDay};

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
    ASSERT_FALSE(location.has_date_time());

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
  const std::string ascii_map = R"(A----B----C----D
                                                  |
                                                  |
                                                  |
                                   H----G----F----E)";
  const gurka::ways ways = {
      {"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}},
      {"CD", {{"highway", "motorway"}}}, {"DE", {{"highway", "motorway_link"}}},
      {"EF", {{"highway", "primary"}}},  {"FG", {{"highway", "primary"}}},
      {"GH", {{"highway", "primary"}}},
  };
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_time_tracking_make",
                               {{"mjlonir.timezone", "/path/to/timezone.sqlite"}});

  // pick out a start and end ll by finding the appropriate edges in the graph
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto start = map.nodes["A"];
  auto end = map.nodes["H"];

  // route between them with a current time
  auto req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":0},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  valhalla::Api api;
  tyr::actor_t actor(map.config, reader);
  actor.route(req.str(), nullptr, &api);

  // check the timings
  std::vector<double> times;
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }
  const std::vector<double> expected = {0,        17.1429,  34.2857,  51.4286,
                                        193.9319, 245.4273, 269.4273, 293.4273};
  ASSERT_THAT(times, testing::Pointwise(testing::DoubleNear(0.0001), expected));

  // route between them with a depart_at
  req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":1,"value":"1982-12-08T17:17"},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  actor.route(req.str(), nullptr, &api);

  // check the timings
  times.clear();
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }
  ASSERT_THAT(times, testing::Pointwise(testing::DoubleNear(0.0001), expected));

  // route between them with a arrive_by
  req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":2,"value":"1982-12-08T17:17"},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  actor.route(req.str(), nullptr, &api);

  // check the timings
  times.clear();
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.cost().elapsed_cost().seconds());
      }
    }
  }

  ASSERT_THAT(times, testing::Pointwise(testing::DoubleNear(0.0001), expected));
}

TEST(TimeTracking, dst) {
  // BST and GMT ambiguity where we gain an hour due to DST
  std::string date_time = "2020-10-25T01:57";
  int tz_idx = dt::get_tz_db().to_index("Europe/London");
  EXPECT_NO_THROW(baldr::TimeInfo::make(date_time, tz_idx))
      << " could not apply timezone to local time";
}
