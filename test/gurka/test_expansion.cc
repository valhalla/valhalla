#include "gurka.h"
#include <boost/algorithm/string/join.hpp>
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

class ExpansionTest : public ::testing::TestWithParam<std::vector<std::string>> {
protected:
  static gurka::map expansion_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    // 5 birectional edges, 1 oneway = 11 directed edges
    const std::string ascii_map = R"(
            B  F--G
            |  |  |
         E--A--C--H
            |
            D
    )";

    const gurka::ways ways = {
        {"DAB", {{"highway", "primary"}}},
        {"EACH", {{"highway", "primary"}}},
        {"CFG", {{"highway", "primary"}, {"oneway", "yes"}}},
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    expansion_map = gurka::buildtiles(layout, ways, {}, {}, "test/data/expansion");
  }

  // common method can't deal with arrays of strings
  std::string build_local_req(rapidjson::Document& doc,
                              rapidjson::MemoryPoolAllocator<>& allocator,
                              const std::vector<std::string>& waypoints,
                              const std::string& costing,
                              const std::string& action,
                              bool skip_opps,
                              const std::vector<std::string> props) {
    std::vector<midgard::PointLL> lls;
    for (const auto& p : waypoints) {
      lls.push_back(expansion_map.nodes.at(p));
    }

    rapidjson::Value locations(rapidjson::kArrayType);
    for (const auto& ll : lls) {
      rapidjson::Value p(rapidjson::kObjectType);
      p.AddMember("lon", ll.lng(), allocator);
      p.AddMember("lat", ll.lat(), allocator);
      locations.PushBack(p, allocator);
    }

    if (props.size()) {
      rapidjson::Value exp_props(rapidjson::kArrayType);
      for (const auto& prop : props) {
        rapidjson::Value p(prop.c_str(), allocator);
        exp_props.PushBack(p, allocator);
      }
      doc.AddMember("expansion_props", exp_props, allocator);
    }

    if (action == "isochrone") {
      rapidjson::Value contours(rapidjson::kArrayType);
      rapidjson::Value cont(rapidjson::kObjectType);
      cont.AddMember("time", 1.0, allocator);
      contours.PushBack(cont, allocator);
      doc.AddMember("contours", contours, allocator);
    }

    doc.AddMember("locations", locations, allocator);
    doc.AddMember("costing", costing, allocator);
    doc.AddMember("action", action, allocator);
    if (skip_opps) {
      doc.AddMember("skip_opposites", rapidjson::Value().SetBool(skip_opps), allocator);
    }

    rapidjson::StringBuffer sb;
    rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
    doc.Accept(writer);
    auto s = sb.GetString();
    return s;
  }

  void check_result(const std::string& action,
                    const std::vector<std::string>& waypoints,
                    bool skip_opps,
                    unsigned exp_feats) {
    rapidjson::Document doc;
    doc.SetObject();
    const auto req =
        build_local_req(doc, doc.GetAllocator(), waypoints, "auto", action, skip_opps, GetParam());

    std::string res;
    auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr, &res);

    // get the MultiLineString feature
    rapidjson::Document res_doc;
    res_doc.Parse(res.c_str());
    auto feat = res_doc["features"][0].GetObject();

    ASSERT_EQ(feat["geometry"]["type"].GetString(), std::string("MultiLineString"));

    auto coords_size = feat["geometry"]["coordinates"].GetArray().Size();
    ASSERT_EQ(coords_size, exp_feats);
    for (const auto& prop : GetParam()) {
      ASSERT_EQ(feat["properties"][prop].GetArray().Size(), coords_size);
    }
  }
};

gurka::map ExpansionTest::expansion_map = {};

TEST_P(ExpansionTest, Isochrone) {
  // test Dijkstra expansion
  // 11 because there's a one-way
  check_result("isochrone", {"A"}, false, 11);
}

TEST_P(ExpansionTest, IsochroneNoOpposites) {
  // test Dijkstra expansion and skip collecting more expensive opposite edges
  check_result("isochrone", {"A"}, true, 6);
}

TEST_P(ExpansionTest, Routing) {
  // test AStar expansion
  check_result("route", {"E", "H"}, false, 23);
}

TEST_P(ExpansionTest, RoutingNoOpposites) {
  // test AStar expansion and no properties in the output
  check_result("route", {"E", "H"}, true, 16);
}

TEST_F(ExpansionTest, UnsupportedAction) {
  rapidjson::Document doc;
  doc.SetObject();
  const auto req = build_local_req(doc, doc.GetAllocator(), {"A"}, "auto", "locate", false, {});

  try {
    auto api = gurka::do_action(Options::expansion, expansion_map, req, nullptr);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 144); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, UnsupportedPropType) {

  rapidjson::Document doc;
  doc.SetObject();
  const auto req =
      build_local_req(doc, doc.GetAllocator(), {"A"}, "auto", "isochrone", false, {"valhalla"});

  try {
    auto api = gurka::do_action(Options::expansion, expansion_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 168); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

TEST_F(ExpansionTest, NoAction) {
  const auto& center_node = expansion_map.nodes["A"];
  const std::string req = R"({"locations":[{"lat":)" + std::to_string(center_node.lat()) +
                          R"(,"lon":)" + std::to_string(center_node.lng()) +
                          R"(}],"costing":"auto","contours":[{"distance":0.05}]})";

  try {
    auto api = gurka::do_action(Options::expansion, expansion_map, req);
  } catch (const valhalla_exception_t& err) { EXPECT_EQ(err.code, 115); } catch (...) {
    FAIL() << "Expected valhalla_exception_t.";
  };
}

INSTANTIATE_TEST_SUITE_P(ExpandPropsTest,
                         ExpansionTest,
                         ::testing::Values(std::vector<std::string>{"statuses"},
                                           std::vector<std::string>{"distances", "durations"},
                                           std::vector<std::string>{"edge_ids", "costs"},
                                           std::vector<std::string>{}));
