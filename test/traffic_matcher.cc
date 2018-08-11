#include "test.h"

#include <stdexcept>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "meili/traffic_segment_matcher.h"

#if !defined(VALHALLA_SOURCE_DIR)
#define VALHALLA_SOURCE_DIR
#endif

using namespace valhalla;

// NOTE: this test relies on pregenerated data. If you change the data format
// you need to update the tiles. The data itself is checked in:
// test/traffic_matcher_tiles/map.osm.gz
// simply gunzip it, create the tiles with it using osmconvert and valhalla_build_tiles
// then individually gzip the tiles using some combination of find xargs and gzip

namespace {

// here we hijack a couple of methods and save off some state while we're at it
// this way the standard calling pattern used from the outside is the same as in the test
// but we now have the internal state so we can see what is going on at more detail
class testable_matcher : public meili::TrafficSegmentMatcher {
public:
  using meili::TrafficSegmentMatcher::TrafficSegmentMatcher;

  std::list<std::vector<meili::interpolation_t>>
  interpolate_matches(const std::vector<meili::MatchResult>& r,
                      std::vector<meili::EdgeSegment>& e,
                      const std::shared_ptr<meili::MapMatcher>& m) const override {
    matches = r;
    matcher = m;
    interpolations = meili::TrafficSegmentMatcher::interpolate_matches(r, e, m);
    return interpolations;
  }

  std::vector<meili::traffic_segment_t>
  form_segments(const std::list<std::vector<meili::interpolation_t>>& i,
                baldr::GraphReader& r) const override {
    segments = meili::TrafficSegmentMatcher::form_segments(i, r);
    return segments;
  }

  mutable std::vector<valhalla::meili::MatchResult> matches;
  mutable std::shared_ptr<meili::MapMatcher> matcher;
  mutable std::list<std::vector<meili::interpolation_t>> interpolations;
  mutable std::vector<meili::traffic_segment_t> segments;
};

// TODO: build the test tiles in the test, need to move traffic association into library to do that
// currently all the logic is in the application

using ots_t = meili::traffic_segment_t; // id,start time,start idx,end time,end idx,length
using ots_matches_t = std::vector<ots_t>;
using sid_t = baldr::GraphId;
std::vector<std::pair<std::string, ots_matches_t>> test_cases{
    // partial, partial with duplicate point
    std::make_pair(
        R"({"trace":[{"lon":-76.376045,"lat":40.539207,"time":0},{"lon":-76.357056,"lat":40.541309,"time":99},{"lon":-76.357056,"lat":40.541309,"time":100}],"trace_options":{"breakage_distance":10000}})",
        ots_matches_t{ots_t{sid_t(0), -1, 0, 50.f, 0, -1}, ots_t{sid_t(0), 50.f, 0, -1, 2, -1}}),
    // partial, partial
    std::make_pair(
        R"({"trace":[{"lon":-76.376045,"lat":40.539207,"time":0},{"lon":-76.357056,"lat":40.541309,"time":100}],"trace_options":{"breakage_distance":10000}})",
        ots_matches_t{ots_t{sid_t(0), -1, 0, 50.f, 0, -1}, ots_t{sid_t(0), 50.f, 0, -1, 1, -1}}),
    // partial, full, partial
    std::make_pair(
        R"({"trace":[{"lon":-76.376045,"lat":40.539207,"time":0},{"lon":-76.351089,"lat":40.541504,"time":300}],"trace_options":{"breakage_distance":10000}})",
        ots_matches_t{ots_t{sid_t(0), -1, 0, 110.f, 0, -1}, ots_t{sid_t(0), 110.f, 0, 250.f, 0, 1000},
                      ots_t{sid_t(0), 250.f, 0, -1, 1, -1}}),
    // partial, full, full, full
    std::make_pair(
        R"({"trace":[{"lon":-76.38126,"lat":40.55602,"time":0},{"lon":-76.35784,"lat":40.56786,"time":600}],"trace_options":{"breakage_distance":10000}})",
        ots_matches_t{ots_t{sid_t(0), -1, 0, 60.f, 0, -1}, ots_t{sid_t(0), 60.f, 0, 110.f, 0, 200},
                      ots_t{sid_t(0), 110.f, 0, 350.f, 0, 1000},
                      ots_t{sid_t(0), 350.f, 0, 600.f, 1, 1000}}),
    // full, full, partial
    std::make_pair(
        R"({"trace":[{"lon":-76.35784,"lat":40.56786,"time":0},{"lon":-76.38126,"lat":40.55602,"time":600}],"trace_options":{"breakage_distance":10000}})",
        ots_matches_t{ots_t{sid_t(0), 0.f, 0, 250.f, 0, 1000},
                      ots_t{sid_t(0), 250.f, 0, 490.f, 0, 1000},
                      ots_t{sid_t(0), 490.f, 0, -1, 0, -1}}),

    // TODO: add test where its all full segments
    // TODO: add test where you are on at the start of a segment, you get off on a small road in
    // between,
    // but come back on again before the segment ends, this segment should be seen twice in output
    // as partials
    // TODO: add test where you are consecutively in the same spot at different times, ie you aren't
    // moving
    // TODO: add test where there is discontinuity in matches so it has to do two sets of matches
    // TODO: add test where intermediate trace points dont get matches, this causes their times to
    // not be used
    // for interpolation but we can still get valid segments on the edges for the entire trace
    // TODO: add a test where you enter a segment, leave it and come back onto it where it starts,
    // via loop,
    // then finish it and you should see partial, then full and the full should not count the length
    // of the partial in it
};

void test_matcher() {
  // fake config
  std::stringstream conf_json;
  conf_json << R"({
      "mjolnir":{"tile_dir":"test/traffic_matcher_tiles"},
      "meili":{"customizable": ["breakage_distance"],
               "mode":"auto","grid":{"cache_size":100240,"size":500},
               "default":{"beta":3,"breakage_distance":2000,"geometry":false,"gps_accuracy":5.0,"interpolation_distance":10,
               "max_route_distance_factor":5,"max_route_time_factor":5,"max_search_radius":100,"route":true,
               "search_radius":50,"sigma_z":4.07,"turn_penalty_factor":200}}
    })";
  boost::property_tree::ptree conf;
  rapidjson::read_json(conf_json, conf);
  conf.get_child("mjolnir").put("tile_dir", VALHALLA_SOURCE_DIR "test/traffic_matcher_tiles");

  // find me a find, catch me a catch
  testable_matcher matcher(conf);

  // some edges should have no matches and most will have no segments
  for (const auto& test_case : test_cases) {
    auto json = matcher.match(test_case.first);
    std::stringstream json_ss;
    json_ss << json;
    boost::property_tree::ptree answer;
    rapidjson::read_json(json_ss, answer);

    const auto& a_segs = test_case.second;
    auto& b_segs = matcher.segments;

    // TODO: for portions of routes with no ots coverage add matching fake segments
    // for now we just remove the fake ones from the result
    auto seg_itr =
        std::remove_if(b_segs.begin(), b_segs.end(), [](const meili::traffic_segment_t& ots) {
          return !ots.segment_id.Is_Valid();
        });
    b_segs.erase(seg_itr, b_segs.end());

    if (a_segs.size() != b_segs.size())
      throw std::logic_error("wrong number of segments matched");
    for (size_t i = 0; i < a_segs.size(); ++i) {
      const auto& a = a_segs[i];
      const auto& b = b_segs[i];
      if (a.begin_shape_index != b.begin_shape_index)
        throw std::logic_error("begin_shape_index mismatch");
      if (a.end_shape_index != b.end_shape_index)
        throw std::logic_error("end_shape_index mismatch");
      if (std::fabs(a.start_time - b.start_time) > 10)
        throw std::logic_error("start time is out of tolerance");
      if (std::fabs(a.end_time - b.end_time) > 10)
        throw std::logic_error("end time is out of tolerance");
      if (std::fabs(a.length - b.length) > 50)
        throw std::logic_error("length is out of tolerance");
    }
  }
}

} // namespace

int main() {
  test::suite suite("traffic matcher");

  suite.test(TEST_CASE(test_matcher));

  return suite.tear_down();
}
