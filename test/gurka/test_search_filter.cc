#include "gurka.h"
#include <boost/format.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

TEST(Standalone, HeadingFilter) {

  const std::string ascii_map = R"(
    B----2----C
    |         |
    |1        |
    A---------D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "primary"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}},
                            {"AD", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_1");

  rapidjson::Document doc;
  doc.SetObject();
  auto& allocator = doc.GetAllocator();
  rapidjson::Value locations(rapidjson::kArrayType);

  // first location
  rapidjson::Value l0(rapidjson::kObjectType);
  l0.AddMember("lon", map.nodes.at("1").lng(), allocator);
  l0.AddMember("lat", map.nodes.at("1").lat(), allocator);
  l0.AddMember("heading", "180", allocator);
  l0.AddMember("heading_tolerance", "45", allocator);
  locations.PushBack(l0, allocator);

  // second location
  rapidjson::Value l1(rapidjson::kObjectType);
  l1.AddMember("lon", map.nodes.at("2").lng(), allocator);
  l1.AddMember("lat", map.nodes.at("2").lat(), allocator);
  locations.PushBack(l1, allocator);

  doc.AddMember("locations", locations, allocator);
  doc.AddMember("costing", "auto", allocator);

  rapidjson::StringBuffer sb;
  rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
  doc.Accept(writer);

  auto result = gurka::route(map, sb.GetString());

  // should take the long way around starting southbound
  gurka::assert::osrm::expect_route(result, {"AB", "AD", "CD", "BC"});
}

TEST(Standalone, RoadClassFilter) {

  const std::string ascii_map = R"(
    B--2--C
    |     |
    |     |
    |     |
    A 1   D
         )";

  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}},
                            {"BC", {{"highway", "primary"}}},
                            {"CD", {{"highway", "primary"}}}};

  const double gridsize = 100;
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/search_filter_2");

  // Should snap origin to AB as it's closest
  const std::string& request_1 =
      (boost::format(R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_1 = gurka::route(map, request_1);
  gurka::assert::osrm::expect_route(result_1, {"AB", "BC"});

  // Should snap origin to CD as the search_filter disallows motorways
  const std::string& request_2 =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s,"search_filter":{"max_road_class":"primary"}},{"lat":%s,"lon":%s}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_2 = gurka::route(map, request_2);
  gurka::assert::osrm::expect_route(result_2, {"CD", "BC"});

  // Should snap destination to AB as the search_filter disallows primary
  const std::string& request_3 =
      (boost::format(
           R"({"locations":[{"lat":%s,"lon":%s},{"lat":%s,"lon":%s,"search_filter":{"min_road_class":"motorway"}}],"costing":"auto"})") %
       std::to_string(map.nodes.at("1").lat()) % std::to_string(map.nodes.at("1").lng()) %
       std::to_string(map.nodes.at("2").lat()) % std::to_string(map.nodes.at("2").lng()))
          .str();
  auto result_3 = gurka::route(map, request_3);
  gurka::assert::osrm::expect_route(result_3, {"AB"});
}
