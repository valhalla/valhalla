#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

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

// an approximate inequality measure
bool operator!=(const AABB2<PointLL>& a, const AABB2<PointLL>& b) {
  return !(a.minpt().ApproximatelyEqual(b.minpt(), 0.000001f) &&
           a.maxpt().ApproximatelyEqual(b.maxpt(), 0.000001f));
}

void utrecht_bb() {
  bb_tester t("test/data/utrecht_tiles");
  if (!t.bb.minpt().IsValid() || !t.bb.maxpt().IsValid())
    throw std::logic_error("actual data should have a valid bounding box");

  if (t(t.bb) != t.bb)
    throw std::logic_error("Expanding the bbox from the largest bbox shouldn't change the bbox");

  AABB2<PointLL> sbb(t.bb.minpt() + Vector2(0.0001f, 0.0001f),
                     t.bb.maxpt() - Vector2(0.0001f, 0.0001f));
  if (t(sbb) != t.bb)
    throw std::logic_error("Expanding a slightly smaller bbox shouldn't change the bbox");

  AABB2<PointLL> lbb(t.bb.minpt() - Vector2(0.0001f, 0.0001f),
                     t.bb.maxpt() + Vector2(0.0001f, 0.0001f));
  if (t(lbb) != t.bb)
    throw std::logic_error("Expanding a slightly larger bbox shouldn't change the bbox");
}

void null_bb() {
  bb_tester t("");
  if (t.bb.minpt().IsValid() || t.bb.maxpt().IsValid())
    throw std::logic_error("no dataset should have an invalid bounding box");

  auto bb = t(t.bb);
  if (bb != t.bb)
    throw std::logic_error("reader should also give back invalid bounding box");

  bb = t(AABB2<PointLL>{});
  if (bb != t.bb)
    throw std::logic_error("reader should still give back invalid bounding box");
}

} // namespace

int main(int argc, char* argv[]) {
  test::suite suite("Minimum Bounding Box");

  valhalla::midgard::logging::Configure({{"type", ""}});

  suite.test(TEST_CASE(utrecht_bb));

  suite.test(TEST_CASE(null_bb));

  return suite.tear_down();
}
