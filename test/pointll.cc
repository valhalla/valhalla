#include "valhalla/midgard/pointll.h"

#include "test.h"

using namespace std;
using namespace valhalla::midgard;

namespace {
  void test_invalid() {
    PointLL ll;
    if(ll.IsValid())
      throw std::logic_error("PointLL default initialization should not be valid");
    ll.Set(0,0);
    if(!ll.IsValid())
      throw std::logic_error("0,0 is a valid coordinate");
    ll.Invalidate();
    if(ll.IsValid())
      throw std::logic_error("Invalidation produced valid coordinates");
  }

  void test_constructor() {
    PointLL ll{1,2};
    if(ll.y() !=1 || ll.x() != 2)
      throw std::runtime_error("PointLL object should be set");
  }
}

int main(void)
{
  test::suite suite("pointll");

  suite.test(TEST_CASE(test_invalid));
  suite.test(TEST_CASE(test_constructor));
  //TODO: many more!

  return suite.tear_down();
}
