#include "test.h"

#include "include/config.h"
#include "include/geo/pointll.h"

using namespace std;
using namespace valhalla::geo;

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
}

int main(void)
{
  test::suite suite("pointll");

  suite.test(TEST_CASE(test_invalid));
  //TODO: many more!

  return suite.tear_down();
}
