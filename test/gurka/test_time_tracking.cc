#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "boost/format.hpp"
#include "gurka.h"
#include "loki/search.h"
#include "proto/api.pb.h"
#include "sif/costfactory.h"
#include "thor/pathalgorithm.h"
#include "tyr/actor.h"

#include <gtest/gtest.h>

using namespace valhalla;
namespace dt = valhalla::baldr::DateTime;

TEST(TimeTracking, make) {

  // build a very simple graph
  const std::string ascii_map = R"(A----B)";
  const gurka::ways ways = {{"AB", {{"highway", "trunk"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_time_tracking_make",
                               {{"mjlonir.timezone", "/path/to/timezone.sqlite"}});

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

  // bad timezone defaults to UTC
  location->set_date_time("2020-04-01T12:34");
  ti = thor::TimeInfo::make(*location, reader, 7777);
  // zero out the part we dont care to test
  ti.local_time = 0;
  ti.second_of_week = 0;
  ti.seconds_from_now = 0;
  ASSERT_EQ(ti, (thor::TimeInfo{1, 291}));
  ASSERT_EQ(location->date_time(), "2020-04-01T12:34");

  // current time (technically we could fail if the minute changes between the next 3 lines)
  location->set_date_time("current");
  ti = thor::TimeInfo::make(*location, reader);
  const auto* tz = dt::get_tz_db().from_index(291);
  auto now_str = dt::iso_date_time(tz);
  auto lt = dt::seconds_since_epoch(now_str, tz);
  auto sec = dt::second_of_week(lt, tz);
  ASSERT_EQ(ti, (thor::TimeInfo{true, 291, lt, sec, 0}));
  ASSERT_EQ(location->date_time(), now_str);

  // not current time but the same date time just set as a string
  now_str = dt::iso_date_time(tz);
  location->set_date_time(now_str);
  ti = thor::TimeInfo::make(*location, reader);
  lt = dt::seconds_since_epoch(now_str, dt::get_tz_db().from_index(1));
  sec = dt::second_of_week(lt, tz);
  ASSERT_EQ(ti, (thor::TimeInfo{true, 291, lt, sec, 0}));
  ASSERT_EQ(location->date_time(), now_str);

  // offset the time from now a bit
  now_str = dt::iso_date_time(tz);
  auto minutes = std::atoi(now_str.substr(now_str.size() - 2, 2).c_str());
  int offset = 7;
  if (minutes + offset > 60)
    offset = -offset;
  now_str = now_str.substr(0, now_str.size() - 2) + std::to_string(minutes + offset);
  location->set_date_time(now_str);
  ti = thor::TimeInfo::make(*location, reader);
  lt = dt::seconds_since_epoch(now_str, tz);
  sec = dt::second_of_week(lt, tz);
  ASSERT_EQ(ti, (thor::TimeInfo{true, 291, lt, sec, offset * 60}));
  ASSERT_EQ(location->date_time(), now_str);

  // messed up date time
  location->set_date_time("4000BC");
  ti = thor::TimeInfo::make(*location, reader);
  ASSERT_EQ(ti, thor::TimeInfo{});
  ASSERT_EQ(location->date_time(), "4000BC");

  // user specified date time
  location->set_date_time("2020-03-31T11:16");
  ti = thor::TimeInfo::make(*location, reader, 110);
  // zero out the part we dont care to test
  ti.seconds_from_now = 0;
  ASSERT_EQ(ti, (thor::TimeInfo{1, 110, 1585667787, 213387}));
  ASSERT_EQ(location->date_time(), "2020-03-31T11:16");
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
  auto opp_start_edge = gurka::findEdge(reader, map.nodes, "AB", "B");
  const baldr::GraphTile* tile = nullptr;
  const auto* node = reader.nodeinfo(std::get<3>(opp_start_edge)->endnode(), tile);
  auto start = node->latlng(tile->header()->base_ll());
  auto end_edge = gurka::findEdge(reader, map.nodes, "GH", "H");
  node = reader.nodeinfo(std::get<1>(end_edge)->endnode(), tile);
  auto end = node->latlng(tile->header()->base_ll());

  // route between them with a current time
  auto req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":0},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  valhalla::Api api;
  tyr::actor_t actor(map.config, reader);
  actor.route(
      req.str(), []() -> void {}, &api);

  // check the timings
  std::vector<double> times;
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.elapsed_time());
      }
    }
  }
  std::vector<double> expected = {0, 17.1429, 34.2857, 176.789, 220.289, 244.289, 268.289};
  ASSERT_EQ(times.size(), expected.size());
  ASSERT_TRUE(std::equal(times.begin(), times.end(), expected.begin(), expected.end(),
                         [](double a, double b) { return std::abs(a - b) < .0001; }));

  // route between them with a depart_at
  req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":1,"value":"1982-12-08T17:17"},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  actor.route(
      req.str(), []() -> void {}, &api);

  // check the timings
  times.clear();
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.elapsed_time());
      }
    }
  }
  expected = {0, 17.1429, 34.2857, 176.789, 220.289, 244.289, 268.289};
  ASSERT_EQ(times.size(), expected.size());
  ASSERT_TRUE(std::equal(times.begin(), times.end(), expected.begin(), expected.end(),
                         [](double a, double b) { return std::abs(a - b) < .0001; }));

  // route between them with a arrive_by
  req =
      boost::format(
          R"({"costing":"auto","date_time":{"type":2,"value":"1982-12-08T17:17"},"locations":[{"lon":%1%,"lat":%2%},{"lon":%3%,"lat":%4%}]})") %
      start.first % start.second % end.first % end.second;
  actor.route(
      req.str(), []() -> void {}, &api);

  // check the timings
  times.clear();
  for (const auto& route : api.trip().routes()) {
    for (const auto& leg : route.legs()) {
      for (const auto& node : leg.node()) {
        times.push_back(node.elapsed_time());
      }
    }
  }
  // TODO: why does arrive by have an extra node in here...
  expected = {0, 0, 17.1429, 34.2857, 176.789, 220.289, 244.289, 268.289};
  ASSERT_EQ(times.size(), expected.size());
  ASSERT_TRUE(std::equal(times.begin(), times.end(), expected.begin(), expected.end(),
                         [](double a, double b) { return std::abs(a - b) < .0001; }));
}