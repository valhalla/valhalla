#include <vector>
#include <string>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>

#include "meili/traffic_segment_matcher.h"

namespace {

float GetEdgeDist(const valhalla::meili::MatchResult& res,
                  std::shared_ptr<valhalla::meili::MapMatcher> matcher) {
  if (res.HasState()) {
    bool found = false;
    const auto& state = matcher->mapmatching().state(res.stateid());
    valhalla::baldr::PathLocation loc = state.candidate();
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

TrafficSegmentMatcher::TrafficSegmentMatcher(const boost::property_tree::ptree& config)
    : reader(config.get_child("mjolnir")),
      matcher_factory(config) {
}

std::string  TrafficSegmentMatcher::match(const std::string& json) {
  // From ptree from JSON string
  boost::property_tree::ptree request;
  try {
    std::stringstream stream(json);
    boost::property_tree::read_json(stream, request);
  } catch (...) {
    LOG_ERROR("Error parsing JSON= " + json);
    return "{\"foo\":\"bar\"}";
  }

  // Form trace positions
  std::vector<PointLL> trace;
  std::vector<uint32_t> times;
  auto trace_pt = request.get_child_optional("trace");

  if (trace_pt) {
    for (const auto& lls : *trace_pt) {
      float lat = lls.second.get<float>("lat");
      float lon = lls.second.get<float>("lon");
      trace.emplace_back(lon, lat);
      times.push_back(lls.second.get<int>("time"));
    }
  } else {
    LOG_ERROR("Could not form trace from input JSON= " + json);
    return "{\"foo\":\"bar\"}";
  }

  // Need to add a ptree to set the mode to use within matching
  // TODO - do we need to overrides to defaults?
  // TODO - later could input a mode in the request?
  boost::property_tree::ptree trace_config;
  trace_config.put<std::string>("mode", "auto");

  // Call Meili for map matching to get a collection of pathLocation Edges
  // Create a matcher
  std::shared_ptr<valhalla::meili::MapMatcher> matcher;
  try {
    matcher.reset(matcher_factory.Create(trace_config));
  } catch (const std::invalid_argument& ex) {
    // TODO - what to return?
    return "{\"foo\":\"bar\"}";
  }

  // Populate a measurement sequence to pass to the map matcher
  std::vector<valhalla::meili::Measurement> sequence;
  for (const auto& coord : trace) {
    sequence.emplace_back(coord,
                          matcher->config().get<float>("gps_accuracy"),
                          matcher->config().get<float>("search_radius"));
  }

  // Create the vector of matched path results
  std::vector<valhalla::meili::MatchResult> results;
  if (sequence.size() > 0) {
    results = matcher->OfflineMatch(sequence);
  }

  if (sequence.size() != results.size()) {
    LOG_ERROR("Sequence size not equal to match reslt size");
    return "{\"foo\":\"bar\"}";
  }

  // TODO - more robust list of edges. Handle cases where multiple
  // edges lie on the path between GPS locations (sequence) and
  // handle cases where discontinuities arise

  // Form a list of edges for each result - match times to each.
  size_t idx = 0;
  std::vector<EdgeOnTrace> trace_edges;
  for (auto res : results) {
    // Make sure edge is valid
    if (res.edgeid().Is_Valid()) {
      trace_edges.emplace_back(res.edgeid(), GetEdgeDist(res, matcher), times[idx]);
    }
    idx++;
  }

  // Iterate through the edges and form a list of unique edges
  valhalla::baldr::GraphId prior_edge;
  std::vector<UniqueEdgeOnTrace> edges;
  for (auto edge : trace_edges) {
    if (!prior_edge.Is_Valid()) {
      edges.emplace_back(edge.edge_id, edge.dist, edge.dist, edge.secs, edge.secs);
    } else  if (edge.edge_id == prior_edge) {
      // Update the time at the end
      edges.back().end_pct = edge.dist;
      edges.back().secs2   = edge.secs;
    } else {
      // TODO - verify that the edges are connected.
      // what if these edges are not connected?

      // Update time and distance at the end of the prior edge
      edges.back().end_pct = 1.0f;
      edges.back().secs2   = edge.secs ;

      // New edge
      edges.emplace_back(edge.edge_id, 0.0f, edge.dist, edge.secs, edge.secs);
    }
    prior_edge = edge.edge_id;
  }

  valhalla::baldr::GraphId prior_segment;
  std::vector<MatchedTrafficSegments> traffic_segment;
  for (const auto& edge : edges) {
    // Get the directed edge Id and tile
    valhalla::baldr::GraphId edge_id = edge.edge_id;
    const valhalla::baldr::GraphTile* tile = matcher->graphreader().GetGraphTile(edge_id);
    const valhalla::baldr::DirectedEdge* directededge = tile->directededge(edge_id);
    float length = directededge->length() * (edge.end_pct - edge.start_pct);

    // Get the traffic segment(s) associated to this edge
    float begin_time, end_time;
    auto segments = tile->GetTrafficSegments(edge_id);
    if (segments.size() > 0) {
      // TODO - support chunks (more than 1 segment per edge)
      for (const auto& seg : segments) {
        float p1 = seg.first.begin_percent();
        float p2 = seg.first.end_percent();
        valhalla::baldr::GraphId segment_id = seg.first.segment_id();
        float weight = seg.second;
        if (segment_id == prior_segment) {
          // Update end time, end_pct, and length of the current segment
          traffic_segment.back().end_time = edge.secs2;
          traffic_segment.back().length += length;
          if (seg.first.ends_segment() && edge.end_pct == 1.0f) {
            traffic_segment.back().partial_end = false;
          }
        } else if (segment_id.value == 0) {
          // No segment associated to this edge...
          ; // LOG_INFO("Traffic segment ID == 0");
        } else {
          bool starts = (seg.first.starts_segment() && edge.start_pct == 0.0f);
          bool ends =   (seg.first.ends_segment() && edge.end_pct == 1.0f);
          traffic_segment.emplace_back(!starts, !ends, segment_id,
                      edge.secs1, edge.secs2, length);
          prior_segment = segment_id;
        }
      }
    } else {
      // No traffic segment associated to this edge
      if (directededge->classification() <= valhalla::baldr::RoadClass::kTertiary) {
        LOG_ERROR("No traffic associated to this edge");
      }
    }
    prior_edge = edge_id;
  }

  // Serialize and return as a string
  boost::property_tree::ptree result;
  boost::property_tree::ptree segments;
  for (const auto& seg : traffic_segment) {
    segments.push_back(std::make_pair("", seg.ToPtree()));
  }
  result.put_child("segments", segments);
  std::stringstream ss;
  boost::property_tree::write_json(ss, result);
  return ss.str();
}

}
}
