#include <algorithm>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "meili/traffic_segment_matcher.h"

using namespace valhalla::baldr;

namespace {

}

namespace valhalla {
namespace meili {

TrafficSegmentMatcher::TrafficSegmentMatcher(const boost::property_tree::ptree& config): matcher_factory(config) {
}

std::string TrafficSegmentMatcher::match(const std::string& json) {
  // Create a matcher
  std::shared_ptr<MapMatcher> matcher;
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
  if (match_results.size() != measurements.size())
    throw std::runtime_error("Sequence size not equal to match result size");

  // Get the edges associated with the entire connected trace path
  // NOTE: it may not be entirely connected so we much check for that
  auto trace_edges = form_edges(match_results, matcher);

  // Get the segments along the measurments
  auto traffic_segments = form_segments(trace_edges, match_results);

  //give back json
  return serialize(traffic_segments);
}

std::vector<trace_edge_t> TrafficSegmentMatcher::form_edges(std::vector<MatchResult>& match_results,
  const std::shared_ptr<meili::MapMatcher>& matcher) const {
  //NOTE: at this point if the input didnt match to an entirely connected path it will throw and we lose the whole thing
  //get all of the edges along the path from the state info
  auto segments = ConstructRoute(matcher->mapmatching(), match_results.begin(), match_results.end());
  //merging the edges that are the same into the first ones record
  auto segment_itr = segments.begin();
  while(segment_itr != segments.end()) {
    auto initial_edge = segment_itr;
    ++segment_itr;
    while(segment_itr != segments.end() && segment_itr->edgeid == initial_edge->edgeid) {
      initial_edge->target = segment_itr->target;
      ++segment_itr;
    }
  }

  //delete duplicates that we copied to the initial edge
  auto remove_pos = std::remove_if(segments.begin(), segments.end(), [&segments](const EdgeSegment& s){
    return &segments.back() != &s && s.edgeid == (&s + 1)->edgeid;
  });
  segments.erase(remove_pos, segments.end());

  //NOTE: an alternate version of ConstructRoute would make stuff above and this next part moot
  //rip through the edges to get their lengths figured out
  std::vector<trace_edge_t> edges;
  float total_length = -segments.front().source *
    matcher->graphreader().GetGraphTile(segments.front().edgeid)->directededge(segments.front().edgeid)->length();
  for(const auto& segment : segments) {
    float length = matcher->graphreader().GetGraphTile(segment.edgeid)->directededge(segment.edgeid)->length();
    edges.push_back({segment.edgeid, total_length, total_length + length});
    total_length += length;
  }

  //TODO: add match results for the ends of each edge, then back fill the time information

  return edges;
}

std::vector<MatchedTrafficSegment> TrafficSegmentMatcher::form_segments(const std::vector<trace_edge_t>& trace_edges,
  const std::vector<MatchResult>& match_results) const {
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
  std::vector<Measurement> measurements;
  try {
    for (const auto& pt : *trace_pts) {
      auto lat = pt.second.get<float>("lat");
      auto lon = pt.second.get<float>("lon");
      auto epoch_time = pt.second.get<float>("time"); //surely this wont make it to 2038-01-19 03:14:08
      auto accuracy = pt.second.get<float>("accuracy", default_accuracy);
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
