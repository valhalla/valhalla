#include "test.h"

#include "baldr/graphtile.h"
#include "config.h"
#include "thor/edgestatus.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

void TryGet(const EdgeStatus& edgestatus, const GraphId& edgeid, const EdgeSet expected) {
  EdgeStatusInfo r = edgestatus.Get(edgeid);
  if (r.set() != expected)
    throw runtime_error("EdgeStatus get test failed");
}

struct test_tile : public GraphTile {
  using GraphTile::header_;
};

void TestStatus() {
  EdgeStatus edgestatus;

  // Dummy tile header
  GraphTileHeader header;
  header.set_directededgecount(200000);
  test_tile tt;
  tt.header_ = &header;

  const GraphTile* tile = &tt;

  // Add some edges
  edgestatus.Set(GraphId(555, 1, 100100), EdgeSet::kPermanent, 1, tile);
  edgestatus.Set(GraphId(555, 2, 100100), EdgeSet::kPermanent, 2, tile);
  edgestatus.Set(GraphId(555, 3, 100100), EdgeSet::kPermanent, 3, tile);
  edgestatus.Set(GraphId(555, 1, 55555), EdgeSet::kTemporary, 4, tile);
  edgestatus.Set(GraphId(555, 2, 55555), EdgeSet::kTemporary, 5, tile);
  edgestatus.Set(GraphId(555, 3, 55555), EdgeSet::kTemporary, 6, tile);
  edgestatus.Set(GraphId(555, 1, 1), EdgeSet::kPermanent, 7, tile);
  edgestatus.Set(GraphId(555, 2, 1), EdgeSet::kPermanent, 8, tile);
  edgestatus.Set(GraphId(555, 3, 1), EdgeSet::kPermanent, 9, tile);

  // Test various get
  TryGet(edgestatus, GraphId(555, 1, 100100), EdgeSet::kPermanent);
  TryGet(edgestatus, GraphId(555, 2, 100100), EdgeSet::kPermanent);
  TryGet(edgestatus, GraphId(555, 3, 100100), EdgeSet::kPermanent);
  TryGet(edgestatus, GraphId(555, 1, 55555), EdgeSet::kTemporary);
  TryGet(edgestatus, GraphId(555, 2, 55555), EdgeSet::kTemporary);
  TryGet(edgestatus, GraphId(555, 3, 55555), EdgeSet::kTemporary);
  TryGet(edgestatus, GraphId(555, 1, 1), EdgeSet::kPermanent);
  TryGet(edgestatus, GraphId(555, 2, 1), EdgeSet::kPermanent);
  TryGet(edgestatus, GraphId(555, 3, 1), EdgeSet::kPermanent);

  // Clear and make sure all status are kUnreached
  edgestatus.clear();
  TryGet(edgestatus, GraphId(555, 1, 100100), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 2, 100100), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 3, 100100), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 1, 55555), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 2, 55555), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 3, 55555), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 1, 1), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 2, 1), EdgeSet::kUnreached);
  TryGet(edgestatus, GraphId(555, 3, 1), EdgeSet::kUnreached);
}

} // namespace

int main() {
  test::suite suite("edgestatus");

  // Test setting status, getting status, and clearing
  suite.test(TEST_CASE(TestStatus));

  return suite.tear_down();
}
