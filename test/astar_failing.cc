#include "midgard/logging.h"
#include "test.h"
#include <cstdint>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "filesystem.h"
#include "loki/search.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/vector2.h"
#include "mjolnir/graphbuilder.h"
#include "mjolnir/graphenhancer.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/graphvalidator.h"
#include "mjolnir/pbfgraphparser.h"
#include "mjolnir/util.h"
#include "odin/directionsbuilder.h"
#include "odin/worker.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/triplegbuilder.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "tyr/serializers.h"
#include "worker.h"

#include "gurka.h"

#include "proto/directions.pb.h"
#include "proto/options.pb.h"
#include "proto/trip.pb.h"

#include <boost/algorithm/string/join.hpp>
#include <boost/property_tree/ptree.hpp>

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

namespace bpt = boost::property_tree;

using namespace valhalla;
namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vk = valhalla::loki;
namespace vj = valhalla::mjolnir;
namespace vo = valhalla::odin;
namespace vr = valhalla::tyr;

#include "mjolnir/directededgebuilder.h"
#include "mjolnir/graphtilebuilder.h"

namespace {

void create_costing_options(Options& options, Costing::Type costing, bool hierarchy_limits = true) {
  const rapidjson::Document doc;

  options.set_costing_type(costing);
  sif::ParseCosting(doc, "/costing_options", options);
}
// Convert locations to format needed by PathAlgorithm
std::vector<valhalla::Location> ToPBFLocations(const std::vector<vb::Location>& locations,
                                               vb::GraphReader& graphreader,
                                               const std::shared_ptr<vs::DynamicCost>& costing) {
  const auto projections = loki::Search(locations, graphreader, costing);
  std::vector<valhalla::Location> result;
  for (const auto& loc : locations) {
    valhalla::Location pbfLoc;
    const auto& correlated = projections.at(loc);
    PathLocation::toPBF(correlated, &pbfLoc, graphreader);
    result.emplace_back(std::move(pbfLoc));
  }
  return result;
}

TEST(BiDiAstar, test_recost_path) {
  const std::string ascii_map = R"(
           X-----------Y
          /             \
    1----A               E---2
          \             /
           B--C--------D
           |  |        |
           3  4        5
  )";
  const gurka::ways ways = {
      // make ABC to be a shortcut
      {"ABC", {{"highway", "motorway"}, {"maxspeed", "80"}}},
      // make CDE to be a shortcut
      {"CDE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"1A", {{"highway", "secondary"}}},
      {"B3", {{"highway", "secondary"}}},
      {"C4", {{"highway", "secondary"}}},
      {"D5", {{"highway", "secondary"}}},
      // set speeds less than on ABCDE path to force the algorithm
      // to go through ABCDE nodes instead of AXY
      {"AX", {{"highway", "motorway"}, {"maxspeed", "70"}}},
      {"XY", {{"highway", "trunk"}, {"maxspeed", "70"}}},
      {"YE", {{"highway", "primary"}, {"maxspeed", "80"}}},
      {"E2", {{"highway", "secondary"}}},
  };

  auto nodes = gurka::detail::map_to_coordinates(ascii_map, 500);

  const std::string test_dir = VALHALLA_BUILD_DIR "test/data/astar_shortcuts_recosting";
  const auto map = gurka::buildtiles(nodes, ways, {}, {}, test_dir);

  vb::GraphReader graphreader(map.config.get_child("mjolnir"));

  // before continue check that ABC is actually a shortcut
  const auto ABC = gurka::findEdgeByNodes(graphreader, nodes, "A", "C");
  ASSERT_TRUE(std::get<1>(ABC)->is_shortcut()) << "Expected ABC to be a shortcut";
  // before continue check that CDE is actually a shortcut
  const auto CDE = gurka::findEdgeByNodes(graphreader, nodes, "C", "E");
  ASSERT_TRUE(std::get<1>(CDE)->is_shortcut()) << "Expected CDE to be a shortcut";

  auto const set_constrained_speed = [&graphreader, &test_dir](const std::vector<GraphId>& edge_ids) {
    for (const auto& edgeid : edge_ids) {
      GraphId tileid(edgeid.tileid(), edgeid.level(), 0);
      auto tile = graphreader.GetGraphTile(tileid);
      vj::GraphTileBuilder tile_builder(test_dir, tileid, false);
      std::vector<DirectedEdge> edges;
      for (uint32_t j = 0; j < tile->header()->directededgecount(); ++j) {
        DirectedEdge& edge = tile_builder.directededge(j);
        // update only superseded edges
        if (edgeid.id() == j)
          edge.set_constrained_flow_speed(10);
        edges.emplace_back(std::move(edge));
      }
      tile_builder.UpdatePredictedSpeeds(edges);
    }
  };
  // set constrained speed for all superseded edges;
  // this speed will be used for them in costing model
  set_constrained_speed(graphreader.RecoverShortcut(std::get<0>(ABC)));
  set_constrained_speed(graphreader.RecoverShortcut(std::get<0>(CDE)));
  // reset cache to see updated speeds
  graphreader.Clear();

  Options options;
  create_costing_options(options, Costing::auto_);
  vs::TravelMode travel_mode = vs::TravelMode::kDrive;

  const auto mode_costing = vs::CostFactory().CreateModeCosting(options, travel_mode);

  std::vector<vb::Location> locations;
  // set origin location
  locations.push_back({nodes["1"]});
  // set destination location
  locations.push_back({nodes["2"]});
  auto pbf_locations = ToPBFLocations(locations, graphreader, mode_costing[int(travel_mode)]);
  auto config = test::make_config("");
  vt::BidirectionalAStar astar;

  const auto path =
      astar.GetBestPath(pbf_locations[0], pbf_locations[1], graphreader, mode_costing, travel_mode)
          .front();

  // check that final path doesn't contain shortcuts
  for (const auto& info : path) {
    const auto* edge = graphreader.directededge(info.edgeid);
    ASSERT_FALSE(edge->is_shortcut()) << "Final path shouldn't contain shortcuts";
  }
  // check that final path contains right number of edges
  ASSERT_EQ(path.size(), 6) << "Final path has wrong number of edges";

  // calculate edge duration based on length and speed
  const auto get_edge_duration = [&graphreader, &mode_costing,
                                  travel_mode](const vb::GraphId& edgeid,
                                               const vb::DirectedEdge* edge) {
    auto tile = graphreader.GetGraphTile(edgeid);
    const float speed_meters_per_sec =
        (1000.f / 3600.f) * tile->GetSpeed(edge, mode_costing[int(travel_mode)]->flow_mask());
    return edge->length() / speed_meters_per_sec;
  };

  // Check that final path really was recosted. To do that we compare actual
  // edges durations with durations in the final path.
  const std::vector<std::string> superseded_nodes = {"A", "B", "C", "D", "E"};
  for (size_t i = 0; (i + 1) < superseded_nodes.size(); ++i) {
    const auto edge =
        gurka::findEdgeByNodes(graphreader, nodes, superseded_nodes[i], superseded_nodes[i + 1]);
    ASSERT_EQ(path[i + 1].edgeid, std::get<0>(edge)) << "Not expected edge in the path";
    EXPECT_NEAR((path[i + 1].elapsed_cost - path[i].elapsed_cost - path[i + 1].transition_cost).secs,
                get_edge_duration(std::get<0>(edge), std::get<1>(edge)), 0.1f);
  }
}
} // anonymous namespace

int main(int argc, char* argv[]) {
  // logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
