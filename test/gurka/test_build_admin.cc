#include <gtest/gtest.h>

#include "gurka.h"

#include "baldr/admin.h"
#include "mjolnir/adminbuilder.h"
#include "mjolnir/admin.h"
#include "mjolnir/pbfgraphparser.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;


class AdminTest : public ::testing::Test
{
protected:
  static gurka::map admin_map;

  static void SetUpTestSuite() {
    constexpr double gridsize = 10;

    const std::string ascii_map = R"(
          A-------B-------C
          |       |       |
          |       |       |
          |       |       |
          F-------E-------D
    )";

    // To define an administrative boundary, the nodes must for a closed polygon.
    const gurka::ways ways =
    {
      {
        "ABCDEFA",
        {
          {
            "highway",
            "primary"
          },
        }
      },
      {
        "ABEFA",
        {
          {
            "highway",
            "primary"
          },
        }
      },
      {
        "BCDEB",
        {
          {
            "highway",
            "primary"
          },
        }
      }
    };

    const gurka::relations relations =
    {
      {
        {
          {
            { gurka::way_member, "ABEFA", "outer" }
          }
        },
       {
         { "type", "boundary" },
         { "boundary", "administrative" },
         { "admin_level", "4" },
       }
      },
      {
        {
          {
            { gurka::way_member, "BCDEB", "outer" }
          }
        },
        {
          { "type", "boundary" },
          { "boundary", "administrative" },
          { "admin_level", "4" },
        },
      },
      {
        {
          {
            { gurka::way_member, "ABCDEFA", "outer" }
          }
        },
        {
          { "type", "boundary" },
          { "boundary", "administrative" },
          { "admin_level", "2" },
        }
      }
    };

    const auto layout = gurka::detail::map_to_coordinates(ascii_map, gridsize);
    admin_map = gurka::buildtiles(layout, ways, {}, relations, "test/data/admin");
  }
};

gurka::map AdminTest::admin_map;


TEST_F(AdminTest, test) {
  // create a config file
  const std::string config_file = "test/data/admin/config";
  std::ofstream file;
  file.open(config_file, std::ios_base::trunc);
  ASSERT_TRUE(file.is_open());
  file << "{ \
    \"mjolnir\": { \
    \"concurrency\": 1, \
    \"id_table_size\": 1000, \
    \"tile_dir\": \"test/data/admin\", \
    \"admin\": \"test/data/admin.sqlite\", \
    \"timezone\": \"test/data/not_needed.sqlite\" \
    } \
  }";
  file.close();

  boost::property_tree::ptree pt;
  rapidjson::read_json(config_file, pt);

  // AdminTest::SetUpTestSuite() creates our mock map.pbf here.
  std::vector<std::string> input_files = {"test/data/admin/map.pbf"};
  BuildAdminFromPBF(pt.get_child("mjolnir"), input_files);

  // reverse engineer the tile_id from the nodes that make up
  // our mock map.pbf. Its probably overkill to check every node
  // for its tile-id if I know they are all in the same tile-id...
  std::unordered_set<GraphId> tile_ids;
  for (const auto& node : admin_map.nodes) {
    midgard::PointLL latlon(node.second);
    GraphId tile_id = TileHierarchy::GetGraphId(latlon, 0);
    tile_ids.insert(tile_id);
  }
  ASSERT_EQ(tile_ids.size(), 1);

  GraphId tile_id(*tile_ids.begin());

  GraphReader graph_reader(pt.get_child("mjolnir"));
  auto t = GraphTile::Create(graph_reader.tile_dir(), tile_id);
  ASSERT_TRUE(t);

  sqlite3* db_handle = NULL;
  std::string dbname = pt.get<std::string>("mjolnir.admin");
  uint32_t ret = sqlite3_open_v2(dbname.c_str(), &db_handle, SQLITE_OPEN_READONLY, NULL);
  EXPECT_EQ(ret, SQLITE_OK);
  std::unordered_map<uint32_t, bool> drive_on_right;
  std::unordered_map<uint32_t, bool> allow_intersection_names;
  AABB2<PointLL> world_box(-180.0f, -90.0f, 180.f, 90.f);
  GraphTileBuilder tilebuilder(graph_reader.tile_dir(), tile_id, true);
  std::unordered_multimap<uint32_t, multi_polygon_type> polys;
  polys = GetAdminInfo(db_handle, drive_on_right, allow_intersection_names, world_box, tilebuilder);
  sqlite3_close(db_handle);

  // Very loose test constraints...
  ASSERT_EQ(polys.size(), 3);
  ASSERT_EQ(drive_on_right.size(), 1);
  ASSERT_EQ(allow_intersection_names.size(), 1);
}
