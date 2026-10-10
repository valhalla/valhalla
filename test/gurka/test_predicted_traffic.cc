#include "baldr/predictedspeeds.h"
#include "gurka.h"
#include "mjolnir/add_predicted_speeds.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>

using namespace valhalla;
using namespace valhalla::baldr;

namespace {

const std::string ascii_map = R"(A----B----C----D----E----F----G----H)";
const gurka::ways ways = {
    {"AB", {{"highway", "primary"}}}, {"BC", {{"highway", "primary"}}},
    {"CD", {{"highway", "primary"}}}, {"DE", {{"highway", "primary"}}},
    {"EF", {{"highway", "primary"}}}, {"FG", {{"highway", "primary"}}},
    {"GH", {{"highway", "primary"}}},
};

std::string encoded_speeds(float kph) {
  std::vector<float> buckets(kBucketsPerWeek, kph);
  const auto coefficients = compress_speed_buckets(buckets.data());
  return encode_compressed_speeds(coefficients.data());
}

std::string edge_str(const GraphId& edge_id) {
  return std::to_string(edge_id.level()) + "/" + std::to_string(edge_id.tileid()) + "/" +
         std::to_string(edge_id.id());
}

// writes each tile's rows to <traffic_dir>/<tile suffix>.csv
void write_csvs(const std::filesystem::path& traffic_dir,
                const std::map<GraphId, std::string>& rows_per_tile) {
  std::filesystem::remove_all(traffic_dir);
  for (const auto& [tile_id, rows] : rows_per_tile) {
    auto csv = traffic_dir / GraphTile::FileSuffix(tile_id);
    csv.replace_extension(".csv");
    std::filesystem::create_directories(csv.parent_path());
    std::ofstream(csv) << rows;
  }
}

} // namespace

TEST(PredictedTraffic, MalformedRowsAreSkipped) {
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_predicted_traffic_malformed");
  const auto tile_dir = map.config.get<std::string>("mjolnir.tile_dir");

  std::map<std::string, GraphId> edges;
  {
    GraphReader reader(map.config.get_child("mjolnir"));
    for (const auto& [name, _] : ways) {
      edges[name] =
          std::get<0>(gurka::findEdgeByNodes(reader, layout, name.substr(0, 1), name.substr(1, 1)));
      ASSERT_TRUE(edges[name].is_valid()) << name;
      const auto* de = reader.GetGraphTile(edges[name])->directededge(edges[name]);
      ASSERT_EQ(de->free_flow_speed(), 0) << name;
      ASSERT_FALSE(de->has_predicted_speed()) << name;
    }
  }

  const auto valid = encoded_speeds(50);
  std::map<GraphId, std::string> rows;
  auto add_row = [&](const std::string& way, const std::string& rest) {
    rows[edges[way].tile_base()] += edge_str(edges[way]) + "," + rest + "\n";
  };
  // well-formed row: everything is applied
  add_row("AB", "45,35," + valid);
  // unparsable speeds or predicted speeds drop the whole row, also fields parsed before the error
  add_row("BC", "fast,35," + valid);
  add_row("CD", "45,slow," + valid);
  add_row("DE", "45,35,not-base64!");
  // only the first row for an edge counts
  add_row("EF", "45,35,");
  add_row("EF", "20,10," + valid);
  // columns after the predicted speeds are ignored
  add_row("FG", "45,35," + valid + ",extra,columns");
  // outliers are kept, the user is responsible for them
  add_row("GH", "45,35," + encoded_speeds(200));
  // rows whose edge id can't be parsed are skipped without affecting the others
  rows[edges["AB"].tile_base()] += "not/an/edge,45,35,\n1/2,45,35,\n";
  write_csvs("test/data/gurka_predicted_traffic_malformed_csv", rows);

  mjolnir::ProcessTrafficTiles(tile_dir, "test/data/gurka_predicted_traffic_malformed_csv", false,
                               map.config);

  GraphReader reader(map.config.get_child("mjolnir"));
  auto check = [&](const std::string& way, uint32_t ff, uint32_t cf, bool predicted) {
    const auto* de = reader.GetGraphTile(edges[way])->directededge(edges[way]);
    EXPECT_EQ(de->free_flow_speed(), ff) << way;
    EXPECT_EQ(de->constrained_flow_speed(), cf) << way;
    EXPECT_EQ(de->has_predicted_speed(), predicted) << way;
  };
  check("AB", 45, 35, true);
  check("BC", 0, 0, false);
  check("CD", 0, 0, false);
  check("DE", 0, 0, false);
  check("EF", 45, 35, false);
  check("FG", 45, 35, true);
  check("GH", 45, 35, true);

  const auto tile = reader.GetGraphTile(edges["GH"]);
  const auto* gh = tile->directededge(edges["GH"]);
  EXPECT_NEAR(tile->GetSpeed(gh, kPredictedFlowMask, 0), 200, 2);
  const auto* ab = reader.GetGraphTile(edges["AB"])->directededge(edges["AB"]);
  EXPECT_NEAR(reader.GetGraphTile(edges["AB"])->GetSpeed(ab, kPredictedFlowMask, 0), 50, 2);
}

TEST(PredictedTraffic, StrayFilesAndMissingTilesAreIgnored) {
  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/gurka_predicted_traffic_stray");
  const auto tile_dir = map.config.get<std::string>("mjolnir.tile_dir");

  GraphId ab;
  {
    GraphReader reader(map.config.get_child("mjolnir"));
    ab = std::get<0>(gurka::findEdgeByNodes(reader, layout, "A", "B"));
    ASSERT_TRUE(ab.is_valid());
  }

  // a tile far away from the map, which isn't in the tileset
  const GraphId missing_tile(0, 2, 0);
  ASSERT_FALSE(
      std::filesystem::exists(std::filesystem::path(tile_dir) / GraphTile::FileSuffix(missing_tile)));

  const std::filesystem::path traffic_dir = "test/data/gurka_predicted_traffic_stray_csv";
  write_csvs(traffic_dir, {{ab.tile_base(), edge_str(ab) + ",45,35,\n"},
                           {missing_tile, edge_str(missing_tile) + ",45,35,\n"}});
  // files whose path isn't a tile id are skipped
  std::ofstream(traffic_dir / "README.txt") << "2/0/0,45,35,\n";

  mjolnir::ProcessTrafficTiles(tile_dir, traffic_dir, false, map.config);

  GraphReader reader(map.config.get_child("mjolnir"));
  const auto* de = reader.GetGraphTile(ab)->directededge(ab);
  EXPECT_EQ(de->free_flow_speed(), 45);
  EXPECT_EQ(de->constrained_flow_speed(), 35);
  EXPECT_FALSE(
      std::filesystem::exists(std::filesystem::path(tile_dir) / GraphTile::FileSuffix(missing_tile)));
}
