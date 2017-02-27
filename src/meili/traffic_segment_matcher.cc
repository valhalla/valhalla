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
  auto trace_pts = request.get_child_optional("trace");

  if (trace_pts) {
    for (const auto& pt : *trace_pts) {
      float lat = pt.second.get<float>("lat");
      float lon = pt.second.get<float>("lon");
      trace.emplace_back(lon, lat);
      times.push_back(pt.second.get<int>("time"));
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
    LOG_ERROR("Sequence size not equal to match result size");
    return "{\"foo\":\"bar\"}";
  }

  // TODO - more robust list of edges. Handle cases where multiple
  // edges lie on the path between GPS locations (sequence) and
  // handle cases where discontinuities arise

  // Form a list of edges for each result - match times to each.
  size_t idx = 0;
  std::vector<EdgeOnTrace> trace_edges;
  for (const auto& res : results) {
    // Make sure edge is valid
    if (res.edgeid().Is_Valid()) {
      trace_edges.emplace_back(EdgeOnTrace{res.edgeid(), GetEdgeDist(res, matcher), static_cast<float>(times[idx])});
    }
    idx++;
  }

  // Iterate through the edges and form a list of unique edges
  GraphId prior_edge;
  std::vector<UniqueEdgeOnTrace> edges;
  for (auto& edge : trace_edges) {
    if (!prior_edge.Is_Valid()) {
      edges.emplace_back(UniqueEdgeOnTrace{edge.edge_id, edge.dist, edge.dist, edge.secs, edge.secs, &edge - &trace_edges.front(), &edge - &trace_edges.front()});
    } else  if (edge.edge_id == prior_edge) {
      // Update the time at the end
      edges.back().end_pct = edge.dist;
      edges.back().end_secs = edge.secs;
      edges.back().end_edge_index = &edge - &trace_edges.front();
    } else {
      // TODO - verify that the edges are connected.
      // what if these edges are not connected?

      // Update time and distance at the end of the prior edge
      edges.back().end_pct = 1.0f;
      edges.back().end_secs = edge.secs ;

      // New edge
      edges.emplace_back(UniqueEdgeOnTrace{edge.edge_id, 0.0f, edge.dist, edge.secs, edge.secs, &edge - &trace_edges.front(), &edge - &trace_edges.front()});
    }
    prior_edge = edge.edge_id;
  }

  //TODO: when we are generating segments we need to be careful about the times
  //used. we should use the lengths with the percentages along to do a linear
  //combination to properly set the begin and end times of a given segment. this also
  //means that the time is now floating point (or only second resolution)

  GraphId prior_segment;
  std::vector<MatchedTrafficSegment> traffic_segment;
  for (const auto& edge : edges) {
    // Get the directed edge Id and tile
    GraphId edge_id = edge.edge_id;
    const GraphTile* tile = matcher->graphreader().GetGraphTile(edge_id);
    const DirectedEdge* directededge = tile->directededge(edge_id);

    // Compute the length of the trace along this edge
    float length = directededge->length() * (edge.end_pct - edge.start_pct);

    // Get the traffic segment(s) associated to this edge
    float begin_time, end_time;
    auto segments = tile->GetTrafficSegments(edge_id.id());
    if (segments.size() == 1) {
      // Edge is associated to a single traffic segment
      TrafficSegment seg = segments.front();
      if (!seg.segment_id_.Is_Valid()) {
        // No segment associated to this edge...
        LOG_DEBUG("Traffic segment ID is invalid");
      } else if (seg.segment_id_ == prior_segment) {
        // A continuation of a segment
        // Update end time, end_pct, and length of the current segment
        traffic_segment.back().end_time = edge.end_secs;
        traffic_segment.back().length += length;
        traffic_segment.back().end_shape_index = edge.end_edge_index;
        if (seg.ends_segment_ && edge.end_pct == 1.0f) {
          traffic_segment.back().partial_end = false;
        }
      } else {
        // A new segment
        bool starts = (seg.starts_segment_ && edge.start_pct == 0.0f);
        bool ends =   (seg.ends_segment_ && edge.end_pct == 1.0f);
        traffic_segment.emplace_back(MatchedTrafficSegment{!starts, !ends, seg.segment_id_,
                              edge.start_secs, edge.end_secs, static_cast<uint32_t>(length), edge.begin_edge_index, edge.end_edge_index});
        prior_segment = seg.segment_id_;
      }
    } else if (segments.size() > 1) {
      // Edge is associated to more than 1 segment (chunks)
      for (const auto& seg : segments) {
        // Skip this segment chunk if outside the range of the edge traversed
        // by this part of the trace
        if (!seg.segment_id_.Is_Valid() ||
            seg.end_percent_ < edge.start_pct ||
            seg.begin_percent_ > edge.end_pct) {
          continue;
        }

        // Compute percentage of the edge that this segment touches
        // TODO - Intersect this with the percentages of the edge?
        float seg_length = length;
       // float pct = seg.end_percent_ - seg.begin_percent_;

        // TODO - need to adjust the times and lengths applied to each segment

        if (seg.segment_id_ == prior_segment) {
          // TODO: Update end time, end_pct, and length of the current segment
          traffic_segment.back().end_time = edge.end_secs;
          traffic_segment.back().length += seg_length;
          for (size_t i = edge.begin_edge_index; i <= edge.end_edge_index; ++i) {
            if(trace_edges[i].dist < seg.end_percent_)
              traffic_segment.back().end_shape_index = i;
          }
          if (seg.ends_segment_) {
            // Mark this segment as ended
            traffic_segment.back().partial_end = false;
          }
        } else {
          // Compute start time, end time, and length along this segment
          bool starts = seg.starts_segment_;
          bool ends =   seg.ends_segment_;
          float start_secs = edge.start_secs;  // TODO!l
          float end_secs = edge.end_secs;    // TODO!!
          traffic_segment.emplace_back(MatchedTrafficSegment{!starts, !ends, seg.segment_id_,
                                start_secs, end_secs, static_cast<uint32_t>(seg_length),
                                edge.begin_edge_index, edge.end_edge_index});
          for (size_t i = edge.begin_edge_index; i <= edge.end_edge_index; ++i) {
            if(trace_edges[i].dist < seg.begin_percent_)
              traffic_segment.back().begin_shape_index = i;
            if(trace_edges[i].dist < seg.end_percent_)
              traffic_segment.back().end_shape_index = i;
          }
          prior_segment = seg.segment_id_;
        }
      }
    } else {
      // No traffic segment associated to this edge
      if (directededge->classification() <= RoadClass::kTertiary) {
        LOG_ERROR("No traffic associated to this edge");
      }
    }
    prior_edge = edge_id;
  }

  // Serialize and return as a string
  auto segments = json::array({});
  for (const auto& seg : traffic_segment) {
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
