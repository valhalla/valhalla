#include "test.h"

#include "baldr/graphreader.h"
#include <boost/property_tree/ptree.hpp>

namespace vb = valhalla::baldr;

class TestGraphReader : vb::GraphReader {
public:
  using vb::GraphReader::GraphReader;
  using vb::GraphReader::tile_extract_;
};

auto config = test::make_config("test/data/utrecht_tiles");

TEST(TarIndexer, TestTrafficTar) {
  TestGraphReader reader(config.get_child("mjolnir"));

  ASSERT_TRUE(reader.tile_extract_->archive->success_index);
  ASSERT_EQ(reader.tile_extract_->tiles.size(), 3);
  ASSERT_EQ(reader.tile_extract_->traffic_tiles.size(), 3);
}
