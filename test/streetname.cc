#include "test.h"
#include "valhalla/odin/streetname.h"

#include <vector>
#include <algorithm>

using namespace std;
using namespace valhalla::odin;

namespace {

void TryCtor(const std::string& text) {
  StreetName street_name(text);

  if (text != street_name.value())
    throw std::runtime_error("Incorrect street name text");

}

void TestCtor() {
  // Street name
  TryCtor("Main Street");

  // Ref
  TryCtor("PA 743");

  // Ref with post modifier
  TryCtor("US 220 Business");

  // Ref with directional
  TryCtor("I 81 South");

}

}

int main() {
  test::suite suite("streetname");

  // Constructor
  suite.test(TEST_CASE(TestCtor));

  return suite.tear_down();
}
