#include "test.h"

#include <stdexcept>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/graphreader.h"
#include "baldr/graphid.h"
#include "meili/traffic_segment_matcher.h"

using namespace valhalla;

namespace {

  //here we hijack a couple of methods and save off some state while we're at it
  //this way the standard calling pattern used from the outside is the same as in the test
  //but we now have the internal state so we can see what is going on at more detail
  class testable_matcher : public meili::TrafficSegmentMatcher {
   public:
    using meili::TrafficSegmentMatcher::TrafficSegmentMatcher;

    std::list<std::list<meili::interpolation_t> > interpolate_matches(const std::vector<meili::MatchResult>& r,
      const std::shared_ptr<meili::MapMatcher>& m) const override {
      matches = r;
      matcher = m;
      interpolations = meili::TrafficSegmentMatcher::interpolate_matches(r, m);
      return interpolations;
    }

    std::list<meili::traffic_segment_t> form_segments(const std::list<std::list<meili::interpolation_t> >& i,
      baldr::GraphReader& r) const override {
      segments = meili::TrafficSegmentMatcher::form_segments(i, r);
      return segments;
    }

    mutable std::vector<valhalla::meili::MatchResult> matches;
    mutable std::shared_ptr<meili::MapMatcher> matcher;
    mutable std::list<std::list<meili::interpolation_t> > interpolations;
    mutable std::list<meili::traffic_segment_t> segments;
  };

  //TODO: build the test tiles in the test, need to move traffic association into library to do that
  //currently all the logic is in the application

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
