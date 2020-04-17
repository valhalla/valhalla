#include "baldr/graphreader.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "thor/triplegbuilder.h"

#include "gurka.h"
#include <gtest/gtest.h>

using namespace valhalla;

valhalla::Location
fake_location(const baldr::GraphId& edge_id, const midgard::PointLL& ll, float along) {
  valhalla::Location loc;
  loc.mutable_ll()->set_lng(ll.first);
  loc.mutable_ll()->set_lat(ll.second);
  auto* path_edge = loc.add_path_edges();
  path_edge->set_graph_id(edge_id);
  path_edge->mutable_ll()->set_lng(ll.first);
  path_edge->mutable_ll()->set_lat(ll.second);
  path_edge->set_percent_along(along);
  path_edge->set_begin_node(false);
  path_edge->set_end_node(false);
  return loc;
}

TEST(TimeTracking, routes) {

  // build a very simple graph
  const std::string ascii_map = R"(AB----C)";
  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 90.857, {52.0981145, 5.1309431});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_trimming_make",
                               {{"mjlonir.timezone", "/path/to/timezone.sqlite"}});

  // grab start and end edges that we want to use
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId start_id, end_id;
  const baldr::DirectedEdge *start_edge, *end_edge;
  std::tie(start_id, start_edge, std::ignore, std::ignore) =
      gurka::findEdge(reader, map.nodes, "AB", "B");
  std::tie(end_id, end_edge, std::ignore, std::ignore) =
      gurka::findEdge(reader, map.nodes, "BC", "C");

  // get a point on the shape that is super close to the end such that trimming would fail
  // if we use imprecise length measurement on the edge
  const auto* tile = reader.GetGraphTile(start_id);
  auto start_shape = tile->edgeinfo(start_edge->edgeinfo_offset()).shape();
  if (!start_edge->forward())
    std::reverse(start_shape.begin(), start_shape.end());
  auto end_shape = tile->edgeinfo(end_edge->edgeinfo_offset()).shape();
  if (!end_edge->forward())
    std::reverse(end_shape.begin(), end_shape.end());
  auto start_length = midgard::length<decltype(start_shape)>(start_shape);
  auto end_length = midgard::length<decltype(end_shape)>(end_shape);
  EXPECT_LT(start_length, start_edge->length());
  auto start = start_shape.back();
  start.first -= .000005f;
  EXPECT_LT(start.Distance(start_shape.back()), 0.5f);
  auto offset = start_shape.front().Distance(start) / start_length;
  EXPECT_LT(offset, 1.f);
  EXPECT_GT(offset * start_edge->length(), start_length);
  auto end = end_shape.back();

  // fake a costing
  valhalla::Options options;
  const rapidjson::Document doc;
  sif::ParseAutoCostOptions(doc, "/costing_options/auto", options.add_costing_options());
  auto costing = sif::CreateAutoCost(valhalla::Costing::auto_, options);
  sif::cost_ptr_t costings[int(sif::TravelMode::kMaxTravelMode)];
  costings[static_cast<int>(costing->travel_mode())] = costing;

  // fake up a route
  std::vector<thor::PathInfo> path{{costing->travel_mode(), .001, start_id, 0, .001, false},
                                   {costing->travel_mode(), 45, end_id, 0, 45, false}};
  valhalla::Location origin = fake_location(start_id, start, offset);
  valhalla::Location dest = fake_location(end_id, end, 1);

  // build a leg with a tiny portion of the start edge and all of the end edge
  thor::AttributesController c;
  valhalla::TripLeg leg;
  thor::TripLegBuilder::Build(c, reader, costings, path.cbegin(), path.cend(), origin, dest, {}, leg);
  auto leg_shape = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
  auto leg_length = midgard::length(leg_shape);

  // since the start point is at the very end of the start edge, the shape should pretty much be just
  // the second edge
  EXPECT_NEAR(end_length, leg_length, 1.f);
}