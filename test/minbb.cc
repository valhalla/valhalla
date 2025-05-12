#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/vector2.h"
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
    // to get the bb of the whole data set
    bb = AABB2<PointLL>{PointLL{}, PointLL{}};
    for (const auto& id : reader.GetTileSet()) {
      auto t = reader.GetGraphTile(id);
      for (const auto& node : t->GetNodes()) {
        const auto* edge = t->directededge(node.edge_index());
        const std::vector<PointLL> shape = t->edgeinfo(edge).shape();
        for (const auto& p : shape) {
          if (bb.maxpt().IsValid()) {
            bb.Expand(p);
          } else {
            bb = {p, p};
          }
        }
      }
    }
  }
  AABB2<PointLL> operator()(const AABB2<PointLL>& b) {
    return reader.GetMinimumBoundingBox(b);
  }
};

bool ApproxEqual(const AABB2<PointLL>& a, const AABB2<PointLL>& b) {
  return a.minpt().ApproximatelyEqual(b.minpt(), 0.000001) &&
         a.maxpt().ApproximatelyEqual(b.maxpt(), 0.000001);
}

TEST(MinBB, utrecht_bb) {
  bb_tester t("test/data/utrecht_tiles");

  EXPECT_TRUE(t.bb.minpt().IsValid());
  EXPECT_TRUE(t.bb.maxpt().IsValid());

  auto f = t(t.bb);
  EXPECT_PRED2(ApproxEqual, f, t.bb)
      << "Expanding the bbox from the largest bbox shouldn't change the bbox";

  AABB2<PointLL> sbb(t.bb.minpt() + Vector2d(0.0001, 0.0001),
                     t.bb.maxpt() - Vector2d(0.0001, 0.0001));

  EXPECT_PRED2(ApproxEqual, t(sbb), t.bb)
      << "Expanding a slightly smaller bbox shouldn't change the bbox";

  AABB2<PointLL> lbb(t.bb.minpt() - Vector2d(0.0001, 0.0001),
                     t.bb.maxpt() + Vector2d(0.0001, 0.0001));

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
