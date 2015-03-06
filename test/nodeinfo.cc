#include "test.h"

#include "baldr/nodeinfo.h"
#include <valhalla/midgard/util.h>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

// Expected size is 32 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kNodeInfoExpectedSize = 32;

namespace {

  class test_node : public NodeInfo {
   public:
    test_node(float a, float b) {
      latlng_ = {a, b};
    }
   protected:
    using NodeInfo::latlng_;
  };

  void test_sizeof() {
    if (sizeof(NodeInfo) != kNodeInfoExpectedSize)
      throw std::runtime_error("NodeInfo size should be " +
                std::to_string(kNodeInfoExpectedSize) + " bytes");
  }
  void test_ll() {
    NodeInfo n;
    if(n.latlng().first != 0 || n.latlng().second != 0)
      throw std::runtime_error("NodeInfo ll should be 0,0");
    test_node t(3, -2);
    if(t.latlng().first != 3 || t.latlng().second != -2)
      throw std::runtime_error("NodeInfo ll should be 3,-2");
    if(!valhalla::midgard::equal<float>(static_cast<Point2>(t.latlng()).DistanceSquared({8,8}), 125))
      throw std::runtime_error("Distance squared is wrong");
    if(!static_cast<Point2>(t.latlng()).MidPoint({8,8}).ApproximatelyEqual({5.5f, 3.f}))
      throw std::runtime_error("Mid point is wrong");
    if(!valhalla::midgard::equal<float>(Point2(8,8).DistanceSquared(static_cast<Point2>(t.latlng())), 125))
      throw std::runtime_error("Distance squared is wrong");
    if(!Point2(8,8).MidPoint(static_cast<Point2>(t.latlng())).ApproximatelyEqual({5.5f, 3.f}))
      throw std::runtime_error("Mid point is wrong");
  }

}

int main(void)
{
  test::suite suite("location");

  suite.test(TEST_CASE(test_sizeof));
  suite.test(TEST_CASE(test_ll));
  //TODO: many more

  return suite.tear_down();
}
