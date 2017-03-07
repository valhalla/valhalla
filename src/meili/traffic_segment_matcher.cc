#include <vector>
#include <string>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/logging.h"
#include "midgard/pointll.h"

#include "meili/traffic_segment_matcher.h"

using namespace valhalla::baldr;

namespace {

float GetEdgeDist(const valhalla::meili::MatchResult& res,
                  std::shared_ptr<valhalla::meili::MapMatcher> matcher) {
  if (res.HasState()) {
    bool found = false;
    const auto& state = matcher->mapmatching().state(res.stateid());
    PathLocation loc = state.candidate();
    for (const auto& e : loc.edges) {
     if (e.id == res.edgeid()) {
       return e.dist;
     }
    }
  }
  return 1.0f;
}

};

namespace valhalla {
namespace meili {

TrafficSegmentMatcher::TrafficSegmentMatcher(const boost::property_tree::ptree& config):
  matcher_factory(config), reader(matcher_factory.graphreader()) {
}

std::string TrafficSegmentMatcher::match(const std::string& json) {
  // Create a matcher
  std::shared_ptr<valhalla::meili::MapMatcher> matcher;
  float default_accuracy, default_search_radius;
  try {
    matcher.reset(matcher_factory.Create("auto")); //TODO: get the mode from the request
    default_accuracy = matcher->config().get<float>("gps_accuracy");
    default_search_radius = matcher->config().get<float>("search_radius");
  }
  catch (...) { throw std::runtime_error("Couldn't create traffic matcher using configuration."); }

  // Populate a measurement measurements to pass to the map matcher
  auto measurements = parse_measurements(json, default_accuracy, default_search_radius);
  if(measurements.empty())
    return R"({"segments":[]})";

  // Create the vector of matched path results
  auto match_results = matcher->OfflineMatch(measurements);

  // Get the segments along the measurments
  auto traffic_segments = form_segments(measurements, match_results);

  //give back json
  return serialize(traffic_segments);
}

std::vector<MatchedTrafficSegment> TrafficSegmentMatcher::form_segments(const std::vector<meili::Measurement>& measurements,
  const std::vector<valhalla::meili::MatchResult>& match_results) {
  if (measurements.size() != match_results.size())
    throw std::runtime_error("Sequence size not equal to match result size");

  //TODO:

  std::vector<MatchedTrafficSegment> traffic_segments;
  return traffic_segments;
}

std::vector<meili::Measurement> TrafficSegmentMatcher::parse_measurements(const std::string& json,
  float default_accuracy, float default_search_radius) {
  //try to parse json
  boost::property_tree::ptree request;
  try { std::stringstream stream(json); boost::property_tree::read_json(stream, request); }
  catch (...) { throw std::runtime_error("Couln't parse json input"); }

  //check for required parameters
  auto trace_pts = request.get_child_optional("trace");
  if (!trace_pts)
    throw std::runtime_error("Missing required json array 'trace'");

  // Populate a measurement sequence to pass to the map matcher
  std::vector<valhalla::meili::Measurement> measurements;
  try {
    for (const auto& pt : *trace_pts) {
      float lat = pt.second.get<float>("lat");
      float lon = pt.second.get<float>("lon");
      uint32_t epoch_time = pt.second.get<uint32_t>("time");
      float accuracy = pt.second.get<float>("accuracy", default_accuracy);
      measurements.emplace_back(PointLL{lon, lat}, accuracy, default_search_radius, epoch_time);
    }
  }
  catch (...) { throw std::runtime_error("Missing parameters, trace points require lat, lon and time"); }
  return measurements;
}

std::string TrafficSegmentMatcher::serialize(const std::vector<MatchedTrafficSegment>& traffic_segments) {
  auto segments = json::array({});
  for (const auto& seg : traffic_segments) {
    segments->emplace_back(json::map
      ({
        {"partial_start", seg.partial_start},
        {"partial_end", seg.partial_end},
        {"segment_id", seg.segment_id.value},
        {"start_time", json::fp_t{seg.start_time, 1}},
        {"end_time", json::fp_t{seg.end_time, 1}},
        {"length", static_cast<uint64_t>(seg.length)},
        {"begin_shape_index", static_cast<uint64_t>(seg.begin_shape_index)},
        {"end_shape_index", static_cast<uint64_t>(seg.end_shape_index)},
      })
    );
  }
  std::stringstream ss;
  ss << *json::map({{"segments",segments}});
  return ss.str();
}

}
}
