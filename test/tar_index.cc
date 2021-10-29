#include "test.h"

#include "baldr/graphreader.h"
#include <boost/property_tree/ptree.hpp>

namespace vb = valhalla::baldr;

class TestGraphReader : vb::GraphReader {
public:
  using vb::GraphReader::GetGraphTile;
  using vb::GraphReader::GraphReader;
  using vb::GraphReader::tile_extract_;
};

auto config_tar = test::make_config("test/data/utrecht_tiles",
                                    {{"mjolnir.tile_extract", "test/data/utrecht_tiles/tiles.tar"}});
auto config_dir = test::make_config("test/data/utrecht_tiles");

TEST(TarIndexer, TestTrafficTar) {
  // read the tile headers from tar & dir tiles and memcmp them
  TestGraphReader reader_tar(config_tar.get_child("mjolnir"));
  GraphReader reader_dir(config_dir.get_child("mjolnir"));

  for (const auto& tile_id : reader_dir.GetTileSet()) {
    auto dir_tile = reader_dir.GetGraphTile(tile_id);
    auto tar_tile = reader_tar.GetGraphTile(tile_id);

    ASSERT_EQ(memcmp(reinterpret_cast<const char*>(dir_tile->header()),
                     reinterpret_cast<const char*>(tar_tile->header()), sizeof(vb::GraphTileHeader)),
              0);
  }
}

TEST(TarIndexer, CheckScanTar) {
  config_tar.add("mjolnir.data_processing.scan_tar", true);
  TestGraphReader reader_tar(config_tar.get_child("mjolnir"));

  ASSERT_NE(reader_tar.tile_extract_->checksum, 0);
}
