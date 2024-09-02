#include "loki/search.h"

#include <cstdint>
#include <filesystem>

#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/pathlocation.h"
#include "baldr/tilehierarchy.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "sif/nocost.h"

#include "test.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
namespace vs = valhalla::sif;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

/* this is what it looks like
    b
    |\
  1 | \ 0
    |  \
  2 |   \ 7
    |    \
    a-3-8-d
    |    /
  4 |   / 9
    |  /
  5 | / 6
    |/
    c
*/
const std::string tile_dir = "test/search_tiles";
GraphId tile_id = TileHierarchy::GetGraphId({.125, .125}, 2);
PointLL base_ll = TileHierarchy::get_tiling(tile_id.level()).Base(tile_id.tileid());
std::pair<GraphId, PointLL> b({tile_id.tileid(), tile_id.level(), 0}, {.01, .2});
std::pair<GraphId, PointLL> a({tile_id.tileid(), tile_id.level(), 1}, {.01, .1});
std::pair<GraphId, PointLL> c({tile_id.tileid(), tile_id.level(), 2}, {.01, .01});
std::pair<GraphId, PointLL> d({tile_id.tileid(), tile_id.level(), 3}, {.2, .1});

void clean_tiles() {
  if (std::filesystem::is_directory(tile_dir)) {
    std::filesystem::remove_all(tile_dir);
  }
}

void make_tile() {
  using namespace valhalla::mjolnir;
  using namespace valhalla::baldr;

  // make sure that all the old tiles are gone before trying to make new ones.
  clean_tiles();

  // basic tile information
  GraphTileBuilder tile(tile_dir, tile_id, false);
  uint32_t edge_index = 0;

  auto add_node = [&](const std::pair<GraphId, PointLL>& v, const uint32_t edge_count) {
    NodeInfo node_builder;
    node_builder.set_latlng(base_ll, v.second);
    // node_builder.set_road_class(RoadClass::kSecondary);
    node_builder.set_access(kAllAccess);
    node_builder.set_edge_count(edge_count);
    node_builder.set_edge_index(edge_index);
    node_builder.set_timezone(1);
    edge_index += edge_count;
    tile.nodes().emplace_back(node_builder);
  };

  auto add_edge = [&](const std::pair<GraphId, PointLL>& u, const std::pair<GraphId, PointLL>& v,
                      const uint32_t localedgeidx, const uint32_t opp_local_idx, const bool forward) {
    DirectedEdgeBuilder edge_builder({}, v.first, forward, u.second.Distance(v.second) + .5, 1, 1,
                                     Use::kRoad, RoadClass::kMotorway, localedgeidx, false, false,
                                     false, false, 0, 0, false);
    edge_builder.set_opp_index(opp_local_idx); // How is this different from opp_local_idx
    edge_builder.set_opp_local_idx(opp_local_idx);
    edge_builder.set_localedgeidx(localedgeidx);
    edge_builder.set_forwardaccess(kAllAccess);
    edge_builder.set_reverseaccess(kAllAccess);
    edge_builder.set_free_flow_speed(100);
    edge_builder.set_constrained_flow_speed(10);
    std::vector<PointLL> shape = {u.second, u.second.PointAlongSegment(v.second), v.second};
    if (!forward) {
      std::reverse(shape.begin(), shape.end());
    }
    bool added;
    // make more complex edge geom so that there are 3 segments, affine combination doesnt properly
    // handle arcs but who cares
    uint32_t edge_info_offset =
        tile.AddEdgeInfo(localedgeidx, u.first, v.first, 123, // way_id
                         0, 0,
                         120, // speed limit in kph
                         shape, {std::to_string(localedgeidx)}, {}, {}, 0, added);
    // assert(added);
    edge_builder.set_edgeinfo_offset(edge_info_offset);
    tile.directededges().emplace_back(edge_builder);
  };

  /* this is what it looks like
      b 0
      |\
    1 | \ 0
      |  \
    2 |   \ 7
      |    \
     1a-3-8-d 3
      |    /
    4 |   / 9
      |  /
    5 | / 6
      |/
      c 2

   NOTE: edge ids are in the order the edges are added, so b->d is 0, b->a is 1, a->b is 2 and so
   */

  // B
  {
    add_edge(b, d, 0, 0, true);
    add_edge(b, a, 1, 0, true); // 1
    add_node(b, 2);
  }

  // A
  {
    add_edge(a, b, 0, 1, false); // 2
    add_edge(a, d, 1, 1, true);  // 3
    add_edge(a, c, 2, 0, false); // 4
    add_node(a, 3);
  }

  // C
  {
    add_edge(c, a, 0, 2, true);  // 5
    add_edge(c, d, 1, 2, false); // 6
    add_node(c, 2);
  }

  // D
  {
    add_edge(d, b, 0, 0, false); // 7
    add_edge(d, a, 1, 1, false); // 8
    add_edge(d, c, 2, 1, true);  // 9
    add_node(d, 3);
  }

  // write the tile
  tile.StoreTileData();

  // write the bin data
  GraphTileBuilder::tweeners_t tweeners;
  auto reloaded = GraphTile::Create(tile_dir, tile_id);
  auto bins = GraphTileBuilder::BinEdges(reloaded, tweeners);
  GraphTileBuilder::AddBins(tile_dir, reloaded, bins);

  {
    // Verify tiles
    boost::property_tree::ptree conf;
    conf.put("tile_dir", tile_dir);
    valhalla::baldr::GraphReader reader(conf);
    auto tile = reader.GetGraphTile(tile_id);
    ASSERT_EQ(tile->header()->directededgecount(), 10);
  }
}

std::shared_ptr<vs::DynamicCost> create_costing() {
  return vs::CreateNoCost({});
}

void search(valhalla::baldr::Location location,
            bool expected_node,
            const valhalla::midgard::PointLL& expected_point,
            const std::vector<PathLocation::PathEdge>& expected_edges,
            bool exact = false) {
  // make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);
  valhalla::baldr::GraphReader reader(conf);

  // send it to pbf and back just in case something is wrong with that conversion
  valhalla::Location pbf;
  PathLocation::toPBF(location, &pbf, reader);
  location = PathLocation::fromPBF(pbf);
  location.street_side_max_distance_ = 5000;

  const auto costing = create_costing();
  const auto results = Search({location}, reader, costing);
  const auto& p = results.at(location);

  ASSERT_EQ((p.edges.front().begin_node() || p.edges.front().end_node()), expected_node)
      << p.edges.front().begin_node() << ":" << p.edges.front().end_node()
      << (expected_node ? " Should've snapped to node" : " Shouldn't've snapped to node");

  ASSERT_TRUE(p.edges.size()) << "Didn't find any node/edges";
  ASSERT_TRUE(p.edges.front().projected.ApproximatelyEqual(expected_point)) << "Found wrong point";

  valhalla::baldr::PathLocation answer(location);
  for (const auto& expected_edge : expected_edges) {
    answer.edges.emplace_back(
        PathLocation::PathEdge{expected_edge.id, expected_edge.percent_along, expected_point,
                               expected_point.Distance(location.latlng_), expected_edge.sos});
  }
  // note that this just checks that p has the edges that answer has
  // p can have more edges than answer has and that wont fail this check!
  ASSERT_TRUE(answer.shares_edges(p)) << "Did not find expected edges";
  // if you want to enforce that the result didnt have more then expected
  if (exact) {
    ASSERT_EQ(answer.edges.size(), p.edges.size()) << "Got more edges than expected";
  }
}

void search(valhalla::baldr::Location location, size_t result_count, int reachability) {
  // make the config file
  boost::property_tree::ptree conf;
  conf.put("tile_dir", tile_dir);
  valhalla::baldr::GraphReader reader(conf);

  // send it to pbf and back just in case something is wrong with that conversion
  valhalla::Location pbf;
  PathLocation::toPBF(location, &pbf, reader);
  location = PathLocation::fromPBF(pbf);
  location.street_side_max_distance_ = 5000;

  const auto costing = create_costing();

  const auto results = Search({location}, reader, costing);
  if (results.empty() && result_count == 0)
    return;

  const auto& path = results.at(location);

  ASSERT_EQ(path.edges.size(), result_count) << "Wrong number of edges";
  for (const auto& edge : path.edges) {
    ASSERT_GE(edge.outbound_reach, reachability);
    ASSERT_GE(edge.inbound_reach, reachability);
  }
}

TEST(Search, test_edge_search) {
  auto t = a.first.tileid();
  auto l = a.first.level();
  using S = PathLocation::SideOfStreet;
  using PE = PathLocation::PathEdge;
  using ST = Location::StopType;
  using PS = Location::PreferredSide;

  // snap to node searches
  search({a.second}, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}});
  search({d.second}, true, d.second,
         {PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE},
          PE{{t, l, 9}, 0, d.second, 0, S::NONE}});
  // snap to end of edge degenerates to node searches
  search({{d.second.first + .049, d.second.second}}, true, d.second,
         {
             PE{{t, l, 7}, 0, d.second, 0, S::NONE}, PE{{t, l, 8}, 0, d.second, 0, S::NONE},
             PE{{t, l, 9}, 0, d.second, 0, S::NONE}, // leaving edges
             PE{{t, l, 0}, 1, d.second, 0, S::NONE}, PE{{t, l, 3}, 1, d.second, 0, S::NONE},
             PE{{t, l, 6}, 1, d.second, 0, S::NONE} // arriving edges
         });

  // regression test for #2023, displace the point beyond search_cutoff but within node_snap_tolerance
  Location near_node{a.second};
  near_node.latlng_.second += 0.00001; // 1.11 meters
  near_node.search_cutoff_ = 1.0;
  near_node.node_snap_tolerance_ = 2.0;
  search(near_node, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}, PE{{t, l, 1}, 1, a.second, 0, S::NONE},
          PE{{t, l, 8}, 1, a.second, 0, S::NONE}, PE{{t, l, 5}, 1, a.second, 0, S::NONE}},
         true);

  // snap to node as through location should be all edges
  Location x{a.second, Location::StopType::THROUGH};
  search(x, true, a.second,
         {PE{{t, l, 2}, 0, a.second, 0, S::NONE}, PE{{t, l, 3}, 0, a.second, 0, S::NONE},
          PE{{t, l, 4}, 0, a.second, 0, S::NONE}, PE{{t, l, 1}, 1, a.second, 0, S::NONE},
          PE{{t, l, 8}, 1, a.second, 0, S::NONE}, PE{{t, l, 5}, 1, a.second, 0, S::NONE}},
         true);
  // with a heading should just be a single outgoing
  x.heading_ = 180;
  search(x, true, a.second, {PE{{t, l, 4}, 0, a.second, 0, S::NONE}}, true);

  // mid point search
  auto answer = a.second.PointAlongSegment(d.second);
  search({a.second.PointAlongSegment(d.second)}, false, a.second.PointAlongSegment(d.second),
         {PE{{t, l, 3}, .5f, answer, 0, S::NONE}, PE{{t, l, 8}, .5f, answer, 0, S::NONE}});

  // set a point 40% along the edge runs with the shape direction
  answer = a.second.PointAlongSegment(d.second, .4f);
  auto ratio = a.second.Distance(answer) / a.second.Distance(d.second);
  x = {answer};
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // with heading
  x.heading_ = 90;
  search(x, false, answer, {PE{{t, l, 3}, ratio, answer, 0, S::NONE}});
  x.heading_ = 0;
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});
  x.heading_ = 269;
  search(x, false, answer, {PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // check for side of street by offsetting the test point from the line orthogonally
  auto ortho = (d.second - a.second).GetPerpendicular(true).Normalize() * .01;
  PointLL test{answer.first + ortho.x(), answer.second + ortho.y()};
  search({test}, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT}});

  // try getting side of street by setting the display ll instead
  x = {answer};
  x.display_latlng_ = test;
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT}});

  // try display ll on the other side
  x.display_latlng_ = PointLL{answer.first - ortho.x(), answer.second - ortho.y()};
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::LEFT}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::RIGHT}});

  // try nuking display ll by setting it farther away than street_side_max_distance
  x.display_latlng_ = PointLL{answer.first - 5 * ortho.x(), answer.second - 5 * ortho.y()};
  search(x, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // check that the side of street tolerance works
  Location sst_huge(test);
  sst_huge.street_side_tolerance_ = 2000;
  search(sst_huge, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::NONE}, PE{{t, l, 8}, 1.f - ratio, answer, 0, S::NONE}});

  // we only want opposite driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::OPPOSITE}, false, answer,
         {PE{{t, l, 3}, ratio, answer, 0, S::RIGHT}}, true);

  // we only want same driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::SAME}, false, answer,
         {PE{{t, l, 8}, 1.f - ratio, answer, 0, S::LEFT}}, true);

  // set a point 40% along the edge that runs in reverse of the shape
  answer = b.second.PointAlongSegment(d.second, .4f);
  ratio = b.second.Distance(answer) / b.second.Distance(d.second);
  search({answer}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::NONE}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::NONE}});

  // check for side of street by offsetting the test point from the line orthogonally
  ortho = (d.second - b.second).GetPerpendicular(false).Normalize() * .01;
  test.Set(answer.first + ortho.x(), answer.second + ortho.y());
  search({test}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::LEFT}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::RIGHT}});

  // check that the side of street tolerance works
  sst_huge.latlng_ = test;
  search(sst_huge, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::NONE}, PE{{t, l, 7}, 1.f - ratio, answer, 0, S::NONE}});

  // we only want opposite driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::OPPOSITE}, false, answer,
         {PE{{t, l, 7}, 1.f - ratio, answer, 0, S::RIGHT}}, true);

  // we only want same driving side, tiles are left hand driving
  search({test, ST::BREAK, 0, 0, 0, PS::SAME}, false, answer,
         {PE{{t, l, 0}, ratio, answer, 0, S::LEFT}}, true);

  // TODO: add more tests
}

TEST(Search, test_reachability_radius_zero_everything) {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);

  LOGLN_WARN("zero everything should be a single closest result");
  search({ob, Location::StopType::BREAK, 0, 0, 0}, 2, 0);
}

TEST(Search, test_reachability_radius_high) {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);
  unsigned int longest = ob.Distance(d.second);

  LOGLN_WARN("set radius high to get them all");
  search({b.second, Location::StopType::BREAK, 0, 0, longest + 100}, 10, 0);
}

TEST(Search, test_reachability_radius_mid) {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);
  unsigned int shortest = ob.Distance(a.second);

  LOGLN_WARN("set radius mid to get just some");
  search({b.second, Location::StopType::BREAK, 0, 0, shortest - 100}, 4, 0);
}

TEST(Search, test_reachability_radius_reachability_high) {
  PointLL ob(b.second.first - .001f, b.second.second - .001f);

  LOGLN_WARN("set reachability high to see it gets all edges reachable");
  search({ob, Location::StopType::BREAK, 10, 10, 0}, 2, 4);
}

TEST(Search, test_reachability_radius_off_by_one) {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);

  LOGLN_WARN("set reachability right on to see we arent off by one");
  search({ob, Location::StopType::BREAK, 4, 4, 0}, 2, 4);
}

TEST(Search, test_reachability_radius_give_up_early) {
  PointLL ob(b.second.first - .001f, b.second.second - .01f);

  LOGLN_WARN("set reachability lower to see we give up early");
  search({ob, Location::StopType::BREAK, 3, 3, 0}, 2, 3);
}

TEST(Search, test_search_cutoff) {
  // test default cutoff of 35km showing no results
  search({{-77, -77}}, 0, 0);

  // test limits of cut off
  auto t = c.second;
  t.first -= 1;
  t.second -= 1;
  auto dist = t.Distance(c.second);

  // set the cut off just too small and get nothing
  Location x(t);
  x.search_cutoff_ = dist - 1;
  search(x, 0, 0);

  // set the cut off just big enough and get one node snap result with 4 edges at it
  x.search_cutoff_ = dist + 1;
  search(x, 4, 0);

  // lets try an edge snap
  t = c.second;
  t.first -= .00005;
  t.second += .00005;
  dist = t.Distance({0.01, 0.01005});

  // just out of reach
  x.latlng_ = t;
  x.search_cutoff_ = dist - 1;
  search(x, 0, 0);

  // just within reach
  x.search_cutoff_ = dist + 1;
  search(x, 2, 0);
}

} // namespace

// Setup and tearown will be called only once for the entire suite121
class SearchTestSuiteEnv : public ::testing::Environment {
public:
  void SetUp() override {
    make_tile();
  }
  void TearDown() override {
    // todo: think whether we want to clean tile dir after the test
  }
};

int main(int argc, char* argv[]) {
  testing::AddGlobalTestEnvironment(new SearchTestSuiteEnv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
