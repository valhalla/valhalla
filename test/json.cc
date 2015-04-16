#include "test.h"
#include "tyr/json.h"

namespace {

void TestJsonSerialize() {

  using namespace std;
  using namespace valhalla::tyr;
  auto json = json::map
  ({
    {"hint_data", json::map
      ({
        {"locations", json::array
          ({
            string("_____38_SADaFQQAKwEAABEAAAAAAAAAdgAAAFfLwga4tW0C4P6W-wAARAA"),
            string("fzhIAP____8wFAQA1AAAAC8BAAAAAAAAAAAAAP____9Uu20CGAiX-wAAAAA")
          })
        },
        {"checksum", static_cast<uint64_t>(2875622111)}
      })
    },
    {"route_name", json::array({string("West 26th Street"), string("Madison Avenue")})},
    {"via_indices", json::array({uint64_t(0), uint64_t(9)})},
    {"found_alternative", bool(false)},
    {"route_summary", json::map
      ({
        {"end_point", string("West 29th Street")},
        {"start_point", string("West 26th Street")},
        {"total_time", uint64_t(145)},
        {"total_distance", uint64_t(878)}
      })
    },
    {"via_points", json::array
      ({
        json::array({(long double)(40.744377), (long double)(-73.990433)}),
        json::array({(long double)(40.745811), (long double)(-73.988075)})
      })
    },
    {"route_instructions", json::array
      ({
        json::array({ string("10"), string("West 26th Street"), uint64_t(216), uint64_t(0), uint64_t(52), string("215m"), string("SE"), uint64_t(118) }),
        json::array({ string("1"), string("East 26th Street"), uint64_t(153), uint64_t(2), uint64_t(29), string("153m"), string("SE"), uint64_t(120) }),
        json::array({ string("7"), string("Madison Avenue"), uint64_t(237), uint64_t(3), uint64_t(25), string("236m"), string("NE"), uint64_t(29) }),
        json::array({ string("7"), string("East 29th Street"), uint64_t(155), uint64_t(6), uint64_t(29), string("154m"), string("NW"), uint64_t(299) }),
        json::array({ string("1"), string("West 29th Street"), uint64_t(118), uint64_t(7), uint64_t(21), string("117m"), string("NW"), uint64_t(299) }),
        json::array({ string("15"), string(""), uint64_t(0), uint64_t(8), uint64_t(0), string("0m"), string("N"), uint64_t(0) })
      })
    },
    {"route_geometry", string("ozyulA~p_clCfc@ywApTar@li@ybBqe@c[ue@e[ue@i[ci@dcB}^rkA")},
    {"status_message", string("Found route between points")},
    {"status", uint64_t(0)},
    {"escaped_string", string("\"\t\r\n\\")}
  });

  string answer = "{\"escaped_string\":\"\\\"\\t\\r\\n\\\\\",\"hint_data\":{\"checksum\":2875622111,\"locations\":[\"_____38_SADaFQQAKwEAABEAAAAAAAAAdgAAAFfLwga4tW0C4P6W-wAARAA\",\"fzhIAP____8wFAQA1AAAAC8BAAAAAAAAAAAAAP____9Uu20CGAiX-wAAAAA\"]},\"route_name\":[\"West 26th Street\",\"Madison Avenue\"],\"found_alternative\":false,\"route_summary\":{\"total_distance\":878,\"total_time\":145,\"start_point\":\"West 26th Street\",\"end_point\":\"West 29th Street\"},\"via_points\":[[40.744377,-73.990433],[40.745811,-73.988075]],\"route_instructions\":[[\"10\",\"West 26th Street\",216,0,52,\"215m\",\"SE\",118],[\"1\",\"East 26th Street\",153,2,29,\"153m\",\"SE\",120],[\"7\",\"Madison Avenue\",237,3,25,\"236m\",\"NE\",29],[\"7\",\"East 29th Street\",155,6,29,\"154m\",\"NW\",299],[\"1\",\"West 29th Street\",118,7,21,\"117m\",\"NW\",299],[\"15\",\"\",0,8,0,\"0m\",\"N\",0]],\"route_geometry\":\"ozyulA~p_clCfc@ywApTar@li@ybBqe@c[ue@e[ue@i[ci@dcB}^rkA\",\"status_message\":\"Found route between points\",\"via_indices\":[0,9],\"status\":0}";
  ostringstream result;
  result << *json;

  if(answer != result.str())
    throw std::runtime_error("Expected: " + answer + " But got: " + result.str());
}

}

int main() {
  test::suite suite("json");

  suite.test(TEST_CASE(TestJsonSerialize));

  return suite.tear_down();
}
