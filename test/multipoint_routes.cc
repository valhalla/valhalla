#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "midgard/util.h"
#include "odin/worker.h"
#include "sif/autocost.h"
#include "thor/worker.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::odin;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::tyr;

namespace {

const auto conf = test::make_config("test/data/utrecht_tiles");

struct route_tester {
  route_tester()
      : reader(new GraphReader(conf.get_child("mjolnir"))), loki_worker(conf, reader),
        thor_worker(conf, reader), odin_worker(conf) {
  }
  Api test(const std::string& request_json) {
    Api request;
    ParseApi(request_json, Options::route, request);
    loki_worker.route(request);
    thor_worker.route(request);
    odin_worker.narrate(request);
    loki_worker.cleanup();
    thor_worker.cleanup();
    odin_worker.cleanup();
    return request;
  }
  std::shared_ptr<GraphReader> reader;
  loki_worker_t loki_worker;
  thor_worker_t thor_worker;
  odin_worker_t odin_worker;
};

float mid_break_distance;
float mid_through_distance;

void check_dates(bool time_dependent,
                 const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                 baldr::GraphReader& reader) {
  // non-time dependent should get no dates, time dependent should all have dates that increase
  uint64_t epoch = 0;
  for (const auto& l : locations) {
    if (!l.date_time().empty()) {
      EXPECT_TRUE(time_dependent) << "Routes without time dependency should have not dates attached";
    } else {
      EXPECT_FALSE(time_dependent) << "Routes with time dependency should have dates attached";
    }

    if (!l.date_time().empty()) {
      // should be localtime, ie no timezone
      EXPECT_TRUE(l.date_time().find('+', l.date_time().find('T')) == std::string::npos &&
                  l.date_time().find('-', l.date_time().find('T')) == std::string::npos);
      // get the timezone
      baldr::GraphId edge_id(l.correlation().edges().begin()->graph_id());
      auto tile = reader.GetGraphTile(edge_id);
      const auto* edge = tile->directededge(edge_id);
      tile = reader.GetGraphTile(edge->endnode(), tile);
      const auto* node = tile->node(edge->endnode());
      // get the epoch time
      uint64_t ltime =
          DateTime::seconds_since_epoch(l.date_time(),
                                        DateTime::get_tz_db().from_index(node->timezone()));
      // check it
      EXPECT_GE(ltime, epoch) << "Date time should be increasing with each route leg";

      epoch = ltime;
    }
  }
}

void test_mid_break(const std::string& date_time) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.09015,"lon":5.06362},{"lat":52.09041,"lon":5.06337,"type":"break"},{"lat":52.09015,"lon":5.06362}],"costing":"auto"})";

  request.pop_back();
  request += date_time;
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  // Should have two legs with two sets of directions
  EXPECT_EQ(legs.size(), 2);
  EXPECT_EQ(directions.size(), 2);

  std::vector<std::string> names;
  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name())
        name += n.value() + " ";
      if (!name.empty())
        name.pop_back();
      names.push_back(name);

      // Should not encounter any u-turns
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnRight);
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnLeft);
    }
  }

  EXPECT_EQ(names, (std::vector<std::string>{"Korianderstraat", "Maanzaadstraat", "",
                                             "Maanzaadstraat", "Korianderstraat", ""}))
      << "Should be a destination at the midpoint and reverse the route for the second leg";

  check_dates(date_time.find(R"("type":)") != std::string::npos, response.options().locations(),
              *tester.reader);

  mid_break_distance =
      directions.begin()->summary().length() + directions.rbegin()->summary().length();
}

void test_mid_through(const std::string& date_time) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.09015,"lon":5.06362},{"lat":52.09041,"lon":5.06337,"type":"through"},{"lat":52.09015,"lon":5.06362, "heading": 0}],"costing":"auto"})";

  request.pop_back();
  request += date_time;
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  // Should have 1 leg with 1 set of directions
  EXPECT_EQ(legs.size(), 1);
  EXPECT_EQ(directions.size(), 1);

  std::vector<std::string> names;
  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name())
        name += n.value() + " ";
      if (!name.empty())
        name.pop_back();
      names.push_back(name);

      // Should not encounter any u-turns
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnRight);
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnLeft);
    }
  }

  EXPECT_EQ(names, (std::vector<std::string>{"Korianderstraat", "Maanzaadstraat", "Pimpernelstraat",
                                             "Selderiestraat", "Korianderstraat", ""}))
      << "Should continue through the midpoint and around the block";

  check_dates(date_time.find(R"("type":)") != std::string::npos, response.options().locations(),
              *tester.reader);

  mid_through_distance = directions.begin()->summary().length();
}

void test_mid_via(const std::string& date_time) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.09015,"lon":5.06362},{"lat":52.09041,"lon":5.06337,"type":"via"},{"lat":52.09015,"lon":5.06362}],"costing":"auto"})";

  request.pop_back();
  request += date_time;
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  // Should have 1 leg with 1 set of directions
  EXPECT_EQ(legs.size(), 1);
  EXPECT_EQ(directions.size(), 1);

  std::vector<std::string> names;
  unsigned int uturns = 0;
  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name())
        name += n.value() + " ";
      if (!name.empty())
        name.pop_back();
      names.push_back(name);
      uturns += m.type() == DirectionsLeg_Maneuver_Type_kUturnRight ||
                m.type() == DirectionsLeg_Maneuver_Type_kUturnLeft;
    }
  }

  EXPECT_EQ(uturns, 1) << "Should be exactly 1 u-turn but there are: " + std::to_string(uturns);

  float mid_via_distance = directions.begin()->summary().length();
  EXPECT_NEAR(mid_via_distance, mid_break_distance, 0.001f)
      << "The only difference in path between mid break and mid via is arrive/depart guidance";

  EXPECT_EQ(names, (std::vector<std::string>{"Korianderstraat", "Maanzaadstraat", "Maanzaadstraat",
                                             "Korianderstraat", ""}))
      << "Should be a uturn at the mid point";

  check_dates(date_time.find(R"("type":)") != std::string::npos, response.options().locations(),
              *tester.reader);
}

void test_mid_break_through(const std::string& date_time) {
  route_tester tester;
  std::string request =
      R"({"locations":[{"lat":52.09015,"lon":5.06362},{"lat":52.09041,"lon":5.06337,"type":"break_through"},{"lat":52.09015,"lon":5.06362,"heading":0}],"costing":"auto"})";

  request.pop_back();
  request += date_time;
  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  // Should have two legs with two sets of directions
  EXPECT_EQ(legs.size(), 2);
  EXPECT_EQ(directions.size(), 2);

  std::vector<std::string> names;
  for (const auto& d : directions) {
    for (const auto& m : d.maneuver()) {
      std::string name;
      for (const auto& n : m.street_name())
        name += n.value() + " ";
      if (!name.empty())
        name.pop_back();
      names.push_back(name);

      // Should not encounter any u-turns
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnRight);
      EXPECT_NE(m.type(), DirectionsLeg_Maneuver_Type_kUturnLeft);
    }
  }

  float mid_break_through_distance =
      directions.begin()->summary().length() + directions.rbegin()->summary().length();

  EXPECT_NEAR(mid_break_through_distance, mid_through_distance, 0.001f)
      << "The only difference in path between mid through and mid break through is arrive/depart guidance";

  EXPECT_EQ(names,
            (std::vector<std::string>{"Korianderstraat", "Maanzaadstraat", "", "Maanzaadstraat",
                                      "Pimpernelstraat", "Selderiestraat", "Korianderstraat", ""}))
      << "Should be a destination at the midpoint and continue around the block";

  check_dates(date_time.find(R"("type":)") != std::string::npos, response.options().locations(),
              *tester.reader);
}

TEST(MultiPointRoutesBreak, test_mid_break_no_time) {
  test_mid_break("}");
}

TEST(MultiPointRoutesBreak, test_mid_break_depart_at) {
  test_mid_break(R"(,"date_time":{"type":1,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesBreak, test_mid_break_arrive_by) {
  test_mid_break(R"(,"date_time":{"type":2,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesThrough, test_mid_through_no_time) {
  test_mid_through("}");
}

TEST(MultiPointRoutesThrough, test_mid_through_depart_at) {
  test_mid_through(R"(,"date_time":{"type":1,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesThrough, test_mid_through_arrive_by) {
  test_mid_through(R"(,"date_time":{"type":2,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesVia, test_mid_via_no_time) {
  test_mid_via("}");
}

TEST(MultiPointRoutesVia, test_mid_via_depart_at) {
  test_mid_via(R"(,"date_time":{"type":1,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesVia, test_mid_via_arrive_by) {
  test_mid_via(R"(,"date_time":{"type":2,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesBreakThrough, test_mid_break_through_no_time) {
  test_mid_break_through("}");
}

TEST(MultiPointRoutesBreakThrough, test_mid_break_through_depart_at) {
  test_mid_break_through(R"(,"date_time":{"type":1,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesBreakThrough, test_mid_break_through_arrive_by) {
  test_mid_break_through(R"(,"date_time":{"type":2,"value":"2016-07-03T08:06"}})");
}

TEST(MultiPointRoutesBreakThrough, test_mid_break_through_elapsed) {
  // The second leg of this route triggers a behavior in bidirectional a* where the forward path is 3
  // edges and the reverse path is 1 edge. That 1 reverse path edge is the same edge as the last edge
  // in the forward path. This is a special case in BidirectionalAstar::FormPath. Because the forward
  // expansion only cares about trimming the first edge from the origin and the reverse expansion only
  // cares about trimming the last edge up to the destination but we've gotten the destination edge
  // from the forward expansion, which means that last edge is untrimmed. so we hit some special logic
  // that says replace its cost and time with the trimmed edge from the reverse expansion but keep the
  // turn cost onto it which only the forward expansion got
  route_tester tester;
  std::string request =
      R"({"locations":[
            {"lat": 52.1191920, "lon":5.1026068, "type": "break_through", "node_snap_tolerance": 0},
            {"lat": 52.1189946, "lon": 5.1029042, "type": "break_through", "node_snap_tolerance": 0},
            {"lat": 52.1192212, "lon": 5.1024556, "type": "break_through", "node_snap_tolerance": 0}], "costing":"auto"})";

  auto response = tester.test(request);
  const auto& legs = response.trip().routes(0).legs();
  const auto& directions = response.directions().routes(0).legs();

  // Should have two legs with two sets of directions
  EXPECT_EQ(legs.size(), 2);
  EXPECT_EQ(directions.size(), 2);
  // So when we make path infos in the routing algorithms we always attach the turncost from edge A to
  // B to the beginning of edge B and it is included in edge Bs elapsed time (which is the time at the
  // end of B)
  // When we build trips/routes/legs we actually put the turncost at the end of A instead and it is
  // not included in the elapsed time (which is the time at the end of A)
  // So if the end of our route is at the end of B and we subtract the A's ending turncost from it the
  // time we get should never be less than the elapsed time at the end of A because edge B should,
  // even without turncost, have taken some non zero time to traverse
  auto last_node = legs.rbegin()->node().rbegin();
  auto previous_node = std::next(last_node);
  EXPECT_LT(previous_node->cost().elapsed_cost().seconds(),
            last_node->cost().elapsed_cost().seconds() -
                previous_node->cost().transition_cost().seconds());
  EXPECT_NEAR(legs.rbegin()->node().rbegin()->cost().elapsed_cost().seconds(), 11.2, .2);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
