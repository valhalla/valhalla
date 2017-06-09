#include <string>
#include "route.pb.h"

#include "midgard/logging.h"
#include "tyr/util.h"
#include "test.h"

using namespace valhalla;
using namespace tyr;

namespace {

const std::string testStrings[] = {
  R"({
    "trip":{
      "units":"km",
      "locations":[
         {"lat":2.3,"lon":3.4,"type":"break","heading":320,"name":"Location 1","street":"random dr.","city":"Lancaster",
          "state":"PA","postal_code":"17603","country":"USA","date_time":"05/03/2017","side_of_street":"left",
          "orignial_index":0},
         {"lat":2.7,"lon":3.7,"postal_code":23545,"original_index":1,"non Field":1734,"name":"trip"}],
      "language":"en-US",
      "id":"abc"
    }
  })"
};

void jsonToProtoRouteTest () {
  std::string jsonTest = testStrings[0];
  valhalla::Route r;
  std::cout << std::endl;
  jsonToProtoRoute(jsonTest, r);
}

}

int main() {
  test::suite suite("util_tyr");

  suite.test(TEST_CASE(jsonToProtoRouteTest));
}
