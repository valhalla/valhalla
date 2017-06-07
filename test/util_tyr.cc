#include <string>
#include "route.pb.h"

#include "midgard/logging.h"
#include "tyr/util.h"
#include "test.h"

using namespace valhalla;
using namespace tyr;

namespace {

void jsonToProtoRouteTest () {
  std::string jsonTest = R"({"trip":{"language":"en-US"}})";
  valhalla::Route r;
  std::cout << std::endl;
  jsonRouteToProtoRoute(jsonTest, r);
}

}

int main() {
  test::suite suite("util_tyr");

  suite.test(TEST_CASE(jsonToProtoRouteTest));
}
