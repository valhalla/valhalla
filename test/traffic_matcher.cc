#include "test.h"

#include <stdexcept>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphid.h"
#include "meili/traffic_segment_matcher.h"

using namespace valhalla;

namespace {

  class testable_matcher : public meili::TrafficSegmentMatcher {
   public:
    using meili::TrafficSegmentMatcher::TrafficSegmentMatcher;

    //this way we call the standard functions that someone would from the outside
    //but we save the state so we can see what is going on from the tests
    std::vector<meili::MatchedTrafficSegment> form_segments(const std::vector<meili::Measurement>& m,
      const std::vector<valhalla::meili::MatchResult>& r) override {
      measurements = m;
      match_results = r;
      segments = meili::TrafficSegmentMatcher::form_segments(m, r);
    }

    std::vector<meili::Measurement> measurements;
    std::vector<valhalla::meili::MatchResult> match_results;
    std::vector<meili::MatchedTrafficSegment> segments;
  };

  void test_matcher() {
    //fake config
    std::stringstream conf_json; conf_json << R"({
      "mjolnir":{"tile_dir":"test/traffic_matcher_tiles"},
      "meili":{"mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,
                          "interpolation_distance":10,"max_route_distance_factor":3,"max_search_radius":100,
                          "route":true,"search_radius":50,"sigma_z":4.07,"turn_penalty_factor":0}}
    })";
    boost::property_tree::ptree conf;
    boost::property_tree::read_json(conf_json, conf);

    //find me a find, catch me a catch
    testable_matcher matcher(conf);

    //some edges should have no matches and most will have no segments
    auto json = matcher.match(R"({"trace":[{"lon":-76.556168,"lat":40.368839,"time":0},
                                           {"lon":-76.554987,"lat":40.369231,"time":1},
                                           {"lon":-76.553689,"lat":40.369051,"time":2},
                                           {"lon":-76.549612,"lat":40.369231,"time":3},
                                           {"lon":-76.549612,"lat":40.368324,"time":4},
                                           {"lon":-76.548507,"lat":40.366158,"time":5} ]})");

  }

}

int main() {
  test::suite suite("traffic matcher");

  suite.test(TEST_CASE(test_matcher));

  return suite.tear_down();
}
