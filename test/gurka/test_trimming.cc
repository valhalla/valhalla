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
  auto* path_edge = loc.mutable_correlation()->add_edges();
  path_edge->set_graph_id(edge_id);
  path_edge->mutable_ll()->set_lng(ll.first);
  path_edge->mutable_ll()->set_lat(ll.second);
  path_edge->set_percent_along(along);
  path_edge->set_begin_node(false);
  path_edge->set_end_node(false);
  return loc;
}

TEST(Trimming, routes) {

  // build a very simple graph
  const std::string ascii_map = R"(AB----C)";
  const gurka::ways ways = {{"AB", {{"highway", "motorway"}}}, {"BC", {{"highway", "motorway"}}}};
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 90.857, {52.0981145, 5.1309431});
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_trimming_make",
                               {{"mjlonir.timezone", "/path/to/timezone.sqlite"}});

  // grab start and end edges that we want to use for a route
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  baldr::GraphId start_id, end_id;
  const baldr::DirectedEdge *start_edge, *end_edge;
  std::tie(start_id, start_edge, std::ignore, std::ignore) =
      gurka::findEdge(reader, map.nodes, "AB", "B");
  std::tie(end_id, end_edge, std::ignore, std::ignore) =
      gurka::findEdge(reader, map.nodes, "BC", "C");

  // we start by getting the shape for the two edges we want a route on
  auto tile = reader.GetGraphTile(start_id);
  auto start_shape = tile->edgeinfo(start_edge).shape();
  if (!start_edge->forward())
    std::reverse(start_shape.begin(), start_shape.end());
  auto end_shape = tile->edgeinfo(end_edge).shape();
  if (!end_edge->forward())
    std::reverse(end_shape.begin(), end_shape.end());
  auto start_length = midgard::length<decltype(start_shape)>(start_shape);
  auto end_length = midgard::length<decltype(end_shape)>(end_shape);

  // we expect that the actual length is shorter than the rounded off length;
  // but, due to the possible precision loss while encoding-decoding edge shape we can't
  // completely rely on this fact. So, we just check that actual length is close to the edge length
  EXPECT_NEAR(start_length, start_edge->length(), 1.);
  // we expect that the distance to the starting point of our route is within the
  // margin of length rounding error to the end of the edge
  auto start = start_shape.back();
  auto end = end_shape.back();
  start.first -= .0000005;
  EXPECT_LT(start.Distance(start_shape.back()), 0.5);
  // we expect that the percentage along the edge with respect to the actual length is less than 1
  // meaning its not all the way at the end of the edge
  auto offset = start_shape.front().Distance(start) / start_length;
  EXPECT_LT(offset, 1.);
  // we expect that multiplying the percent along by the rounded length will result in a number that
  // is larger than the actual length
  // TODO: with the fixes in precision to PointLL this test no longer exposes the issue, needs new
  // test EXPECT_GT(offset * start_edge->length(), start_length);

  // fake a costing
  const rapidjson::Document doc;
  valhalla::Options options;
  options.set_costing_type(Costing::auto_);
  sif::ParseCosting(doc, "/costing_options", options);
  sif::TravelMode mode;
  sif::CostFactory factory;
  auto mode_costings = factory.CreateModeCosting(options, mode);
  auto costing = mode_costings[static_cast<size_t>(mode)];

  // fake up a route
  std::vector<thor::PathInfo>
      path{{costing->travel_mode(), {.001, .001}, start_id, 0, baldr::kInvalidRestriction},
           {costing->travel_mode(), {45, 45}, end_id, 0, baldr::kInvalidRestriction}};
  valhalla::Location origin = fake_location(start_id, start, offset);
  valhalla::Location dest = fake_location(end_id, end, 1);

  // build a leg with a tiny portion of the start edge and all of the end edge. in previous versions
  // of the code the shape trimming was not robust to the discrepancy between rounded off edge length
  // and actual length of edge shape. this would lead to the trimmer not trimming anything at the
  // beginning or end of the edge and you getting the whole shape when what you wanted was just a tiny
  // sliver of the end of the edge
  baldr::AttributesController c;
  valhalla::TripLeg leg;
  thor::TripLegBuilder::Build({}, c, reader, mode_costings, path.cbegin(), path.cend(), origin, dest,
                              leg, {});
  auto leg_shape = midgard::decode<std::vector<midgard::PointLL>>(leg.shape());
  auto leg_length = midgard::length(leg_shape);

  // since the start point is at the very end of the start edge, the shape should pretty much be just
  // the second edge
  EXPECT_NEAR(end_length, leg_length, 1.);
}
