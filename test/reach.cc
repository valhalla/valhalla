#include "gurka/gurka.h"
#include "test.h"

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "loki/reach.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "sif/costfactory.h"
#include "sif/dynamiccost.h"

#include <algorithm>
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
namespace vs = valhalla::sif;

namespace {

const auto conf = test::make_config("test/data/utrecht_tiles");

GraphId begin_node(GraphReader& reader, const DirectedEdge* edge) {
  // grab the node
  auto tile = reader.GetGraphTile(edge->endnode());
  const auto* node = tile->node(edge->endnode());
  // grab the opp edges end node
  const auto* opp_edge = tile->directededge(node->edge_index() + edge->opp_index());
  return opp_edge->endnode();
}

TEST(Reach, check_all_reach) {
  // get tile access
  GraphReader reader(conf.get_child("mjolnir"));

  auto costing = vs::CostFactory{}.Create(Costing::auto_);
  Reach reach_finder;

  // look at all the edges
  for (auto tile_id : reader.GetTileSet()) {
    auto tile = reader.GetGraphTile(tile_id);
    // loop over edges
    for (GraphId edge_id = tile->header()->graphid();
         edge_id.id() < tile->header()->directededgecount(); ++edge_id) {
      // use the simple method to find the reach for the edge in both directions
      const auto* edge = tile->directededge(edge_id);
      auto reach = reach_finder(edge, edge_id, 50, reader, costing, kInbound | kOutbound);

      // shape is nice to have
      auto shape = tile->edgeinfo(edge).shape();
      if (!edge->forward())
        std::reverse(shape.begin(), shape.end());
      auto shape_str = midgard::encode(shape);

      // begin and end nodes are nice to check
      auto node_id = begin_node(reader, edge);
      auto t = reader.GetGraphTile(node_id);
      const auto* begin = t->node(node_id);
      t = reader.GetGraphTile(edge->endnode());
      const auto* end = t->node(edge->endnode());

      // if you have non zero reach but you dont have access, something is wrong
      EXPECT_FALSE((reach.outbound > 0 || reach.inbound > 0) &&
                   !(edge->forwardaccess() & kAutoAccess) && !(edge->reverseaccess() & kAutoAccess))
          << "This edge should have 0 reach as its not accessable: " + std::to_string(edge_id.value) +
                 " " + shape_str;

      // if inbound is 0 and outbound is not then it must be an edge leaving a dead end
      // meaning a begin node that is not accessable
      EXPECT_FALSE(reach.inbound == 0 && reach.outbound > 0 && costing->Allowed(begin))
          << "Only outbound reach should mean an edge that leaves a dead end: " +
                 std::to_string(edge_id.value) + " " + shape_str;

      // if outbound is 0 and inbound is not then it must be an edge entering a dead end
      // meaning an end node that is not accessable
      EXPECT_FALSE(reach.inbound > 0 && reach.outbound == 0 && costing->Allowed(end))
          << "Only inbound reach should mean an edge that enters a dead end: " +
                 std::to_string(edge_id.value) + " " + shape_str;
    }
  }
}

TEST(Reach, transition_misscount) {
  const std::string ascii_map = R"(
      b--c--d
      |  |  |
      |  |  |
      a--f--e
      |  |  |
      |  |  |
      g--h--i
    )";

  const gurka::ways ways = {
      {"abcdefaghie", {{"highway", "residential"}}},
      {"cfh", {{"highway", "tertiary"}}},
  };

  // build the graph
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/wrong_reach");

  // get an auto costing
  sif::CostFactory factory;
  auto costing = factory.Create(Costing::auto_);

  // find the problem edge
  baldr::GraphReader reader(map.config.get_child("mjolnir"));
  auto edge = gurka::findEdgeByNodes(reader, map.nodes, "a", "f");

  // check its reach
  loki::Reach reach_checker;
  auto reach = reach_checker(std::get<1>(edge), std::get<0>(edge), 50, reader, costing);

  // all edges should have the same in/outbound reach
  EXPECT_EQ(reach.inbound, reach.outbound);

  // they all should be 7. there are only 5 obvious graph nodes above but we insert 2 more
  // at d and g because the path the way makes loops back on itself
  EXPECT_EQ(reach.inbound, 7);
  EXPECT_EQ(reach.outbound, 7);
}

} // namespace

int main(int argc, char* argv[]) {
  logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
