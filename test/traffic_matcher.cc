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

    std::list<std::vector<meili::interpolation_t> > interpolate_matches(const std::vector<meili::MatchResult>& r,
      const std::shared_ptr<meili::MapMatcher>& m) const override {
      matches = r;
      matcher = m;
      interpolations = meili::TrafficSegmentMatcher::interpolate_matches(r, m);
      return interpolations;
    }

    std::vector<meili::traffic_segment_t> form_segments(const std::list<std::vector<meili::interpolation_t> >& i,
      baldr::GraphReader& r) const override {
      segments = meili::TrafficSegmentMatcher::form_segments(i, r);
      return segments;
    }

    mutable std::vector<valhalla::meili::MatchResult> matches;
    mutable std::shared_ptr<meili::MapMatcher> matcher;
    mutable std::list<std::vector<meili::interpolation_t> > interpolations;
    mutable std::vector<meili::traffic_segment_t> segments;
  };

  //TODO: build the test tiles in the test, need to move traffic association into library to do that
  //currently all the logic is in the application

  using ots_t = meili::traffic_segment_t;
  using ots_matches_t = std::vector<ots_t>;
  using sid_t = baldr::GraphId;
  std::vector<std::pair<std::string, ots_matches_t> > test_cases {
    //partial, partial
    std::make_pair(R"({"trace":[{"lon":-76.376045,"lat":40.539207,"time":0},{"lon":-76.357056,"lat":40.541309,"time":1}]})",
      ots_matches_t{ots_t{sid_t(772127161),-1,0,-1,0,-1}, ots_t{sid_t(805681593),-1,0,-1,1,-1}}),
    //partial, full, partial
    std::make_pair(R"({"trace":[{"lon":-76.376045,"lat":40.539207,"time":0},{"lon":-76.351089,"lat":40.541504,"time":1}]})",
      ots_matches_t{ots_t{sid_t(772127161),-1,0,-1,0,-1}, ots_t{sid_t(805681593),-1,0,-1,0,1000}, ots_t{sid_t(839236025),-1,0,-1,1,-1}}),
  };

  void test_matcher() {
    //fake config
    std::stringstream conf_json; conf_json << R"({
      "mjolnir":{"tile_dir":"test/traffic_matcher_tiles"},
      "meili":{"mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":10000,"geometry":false,"gps_accuracy":5.0,
                          "interpolation_distance":10,"max_route_distance_factor":3,"max_search_radius":100,
                          "route":true,"search_radius":50,"sigma_z":4.07,"turn_penalty_factor":0}}
    })";
    boost::property_tree::ptree conf;
    boost::property_tree::read_json(conf_json, conf);

    //find me a find, catch me a catch
    testable_matcher matcher(conf);

    //some edges should have no matches and most will have no segments
    for(const auto& test_case : test_cases) {
      auto json = matcher.match(test_case.first);
      if(test_case.second.size() != matcher.segments.size())
        throw std::logic_error("wrong number of segments matched");
    }

  }

}

int main() {
  test::suite suite("traffic matcher");

  suite.test(TEST_CASE(test_matcher));

  return suite.tear_down();
}
