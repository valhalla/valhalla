#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "test.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace {

boost::property_tree::ptree get_conf(const std::string& tile_dir) {
  std::stringstream ss;
  ss << R"({"mjolnir":{"tile_dir":")" + tile_dir + R"(", "concurrency": 1}})";
  boost::property_tree::ptree conf;
  rapidjson::read_json(ss, conf);
  return conf;
}

struct bb_tester {
  GraphReader reader;
  AABB2<PointLL> bb;
  bb_tester(const std::string& tile_dir) : reader(get_conf(tile_dir).get_child("mjolnir")) {
    // to get the bb of the whole data set we can just look at nodes of all tiles
    bb = AABB2<PointLL>{PointLL{}, PointLL{}};
    for (const auto& id : reader.GetTileSet()) {
      const auto* t = reader.GetGraphTile(id);
      for (const auto& node : t->GetNodes()) {
        auto node_ll = node.latlng(t->header()->base_ll());
        if (!bb.minpt().IsValid())
          bb = AABB2<PointLL>{node_ll, node_ll};
        else
          bb.Expand(node_ll);
      }
    }
  }
  AABB2<PointLL> operator()(const AABB2<PointLL>& b) {
    return reader.GetMinimumBoundingBox(b);
  }
};

bool ApproxEqual(const AABB2<PointLL>& a, const AABB2<PointLL>& b) {
  return a.minpt().ApproximatelyEqual(b.minpt(), 0.000001f) &&
         a.maxpt().ApproximatelyEqual(b.maxpt(), 0.000001f);
}

TEST(MinBB, utrecht_bb) {
  bb_tester t("test/data/utrecht_tiles");

  EXPECT_TRUE(t.bb.minpt().IsValid());
  EXPECT_TRUE(t.bb.maxpt().IsValid());

  EXPECT_PRED2(ApproxEqual, t(t.bb), t.bb)
      << "Expanding the bbox from the largest bbox shouldn't change the bbox";

  AABB2<PointLL> sbb(t.bb.minpt() + Vector2(0.0001f, 0.0001f),
                     t.bb.maxpt() - Vector2(0.0001f, 0.0001f));

  EXPECT_PRED2(ApproxEqual, t(sbb), t.bb)
      << "Expanding a slightly smaller bbox shouldn't change the bbox";

  AABB2<PointLL> lbb(t.bb.minpt() - Vector2(0.0001f, 0.0001f),
                     t.bb.maxpt() + Vector2(0.0001f, 0.0001f));

  EXPECT_PRED2(ApproxEqual, t(lbb), t.bb)
      << "Expanding a slightly larger bbox shouldn't change the bbox";
}

TEST(MinBB, null_bb) {
  bb_tester t("");

  // no dataset should have an invalid bounding box
  EXPECT_FALSE(t.bb.minpt().IsValid());
  EXPECT_FALSE(t.bb.maxpt().IsValid());

  auto bb = t(t.bb);
  EXPECT_EQ(bb, t.bb) << "reader should also give back invalid bounding box";

  bb = t(AABB2<PointLL>{});
  EXPECT_EQ(bb, t.bb) << "reader should still give back invalid bounding box";
}

} // namespace

int main(int argc, char* argv[]) {
  valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
