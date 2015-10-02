#include "test.h"

#include "config.h"
#include "thor/edgestatus.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

void TryGet(const EdgeStatus& edgestatus, const GraphId& edgeid,
               const EdgeSet expected) {
  EdgeStatusInfo r = edgestatus.Get(edgeid);
  if (r.set() != expected)
    throw runtime_error("EdgeStatus get test failed");
}

void TestStatus() {
  EdgeStatus edgestatus;

  // Add some edges
  edgestatus.Set(GraphId(555, 1, 100100), EdgeSet::kPermanent, 1);
  edgestatus.Set(GraphId(555, 2, 100100), EdgeSet::kPermanent, 2);
  edgestatus.Set(GraphId(555, 3, 100100), EdgeSet::kPermanent, 3);
  edgestatus.Set(GraphId(555, 1, 55555), EdgeSet::kTemporary, 4);
  edgestatus.Set(GraphId(555, 2, 55555), EdgeSet::kTemporary, 5);
  edgestatus.Set(GraphId(555, 3, 55555), EdgeSet::kTemporary, 6);
  edgestatus.Set(GraphId(555, 1, 1), EdgeSet::kPermanent, 7);
  edgestatus.Set(GraphId(555, 2, 1), EdgeSet::kPermanent, 8);
  edgestatus.Set(GraphId(555, 3, 1), EdgeSet::kPermanent, 9);

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
  edgestatus.Init();
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

}

int main() {
  test::suite suite("edgestatus");

  // Test setting status, getting status, and clearing
  suite.test(TEST_CASE(TestStatus));

  return suite.tear_down();
}
