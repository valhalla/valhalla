#include "test.h"

#include <iostream>
#include <string>
#include <vector>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "midgard/encoded.h"
#include "midgard/util.h"
#include "src/baldr/shortcut_recovery.h"
#include <boost/property_tree/ptree.hpp>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace std {
std::string to_string(const midgard::PointLL& p) {
  return "[" + to_string(p.first) + "," + to_string(p.second) + "]";
}
} // namespace std

const auto conf = test::make_config("test/data/utrecht_tiles");

// expose the constructor
struct testable_recovery : public shortcut_recovery_t {
  testable_recovery(GraphReader* reader) : shortcut_recovery_t(reader) {
  }
};

void recover(bool cache) {
  GraphReader graphreader(conf.get_child("mjolnir"));
  testable_recovery recovery{cache ? &graphreader : nullptr};
  size_t total = 0;
  size_t bad = 0;

  // for each tile set per level
  for (const auto& level : TileHierarchy::levels()) {
    // we dont get shortcuts on level 2 and up
    if (level.level > 1)
      continue;

    // for each tile in the tile set
    auto tileset = graphreader.GetTileSet(level.level);
    for (const auto tileid : tileset) {
      printf("bad: %zu, total: %zu\n", bad, total);
      if (graphreader.OverCommitted())
        graphreader.Trim();

      // for each edge in the tile
      auto tile = graphreader.GetGraphTile(tileid);
      for (size_t j = 0; j < tile->header()->directededgecount(); ++j) {
        // skip it if its not a shortcut or the shortcut is one we will never traverse
        const auto* edge = tile->directededge(j);
        if (!edge->is_shortcut() ||
            (!(edge->forwardaccess() & kAutoAccess) && !(edge->reverseaccess() & kAutoAccess)))
          continue;

        // we'll have the shape to compare to
        auto shortcut_shape = tile->edgeinfo(edge).shape();
        if (!edge->forward())
          std::reverse(shortcut_shape.begin(), shortcut_shape.end());

        // make a graph id out of the shortcut to send to recover
        auto shortcutid = tileid;
        shortcutid.set_id(j);
        auto edgeids = recovery.get(shortcutid, graphreader);

        // if it gave us back the shortcut we failed
        if (edgeids.front() == shortcutid) {
          //          FAIL() << "We couldnt recover the shortcut\nShortcut was: " +
          //              midgard::encode(shortcut_shape);
          ++bad;
          ++total;
          continue;
        }

        // accumulate the shape along the edges that we recovered
        std::vector<PointLL> recovered_shape;
        for (auto edgeid : edgeids) {
          auto tile = graphreader.GetGraphTile(edgeid);
          const auto* de = tile->directededge(edgeid);
          auto de_shape = tile->edgeinfo(de).shape();
          if (!de->forward()) {
            std::reverse(de_shape.begin(), de_shape.end());
          }
          recovered_shape.insert(recovered_shape.end(),
                                 (de_shape.begin() + (recovered_shape.size() == 0 ? 0 : 1)),
                                 de_shape.end());
        }

        // check the number of coords match
        if (shortcut_shape.size() != recovered_shape.size()) {
          //          FAIL() << "shape lengths do not match: " + std::to_string(shortcut_shape.size())
          //          +
          //              " != " + std::to_string(recovered_shape.size()) +
          //              "\nShortcut was: " + midgard::encode(shortcut_shape) +
          //              "\nRecovered was: " + midgard::encode(recovered_shape);
          ++bad;
          ++total;
          continue;
        }

        // check if the shape matches approximatly
        for (size_t k = 0; k < shortcut_shape.size(); ++k) {
          if (!shortcut_shape[k].ApproximatelyEqual(recovered_shape[k])) {
            //            FAIL() << "edge shape points are not equal: " +
            //            std::to_string(shortcut_shape[k]) +
            //                " != " + std::to_string(recovered_shape[k]) +
            //                "\nShortcut was: " + midgard::encode(shortcut_shape) +
            //                "\nRecovered was: " + midgard::encode(recovered_shape);
            ++bad;
            break;
          }
        }
        ++total;
      }
    }
  }
  printf("bad: %zu, total: %zu\n", bad, total);
  EXPECT_LE(double(bad) / double(total), .001) << "More than 0.1% is too much";
}

TEST(RecoverShortcut, test_recover_shortcut_edges_no_cache) {
  recover(false);
}

TEST(RecoverShortcut, test_recover_shortcut_edges_cache) {
  recover(true);
}

int main(int argc, char* argv[]) {
  // valhalla::midgard::logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
