#include <algorithm>
#include <cstdio>

#include "meili/traffic_segment_matcher.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "worker.h"

namespace {

void clean_edges(std::vector<valhalla::meili::EdgeSegment>& edges) {
  // merging the edges that are the same into the final edges record
  for (auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
    auto prev = std::prev(edge_itr);
    if (edge_itr != edges.begin() && edge_itr->edgeid == prev->edgeid) {
      edge_itr->source = prev->source;
    }
  }

  // delete duplicates that we copied to the final edge
  auto edge_itr =
      std::remove_if(edges.begin(), edges.end(), [&edges](const valhalla::meili::EdgeSegment& s) {
        return &edges.back() != &s && s.edgeid == (&s + 1)->edgeid;
      });
  edges.erase(edge_itr, edges.end());
}

// is this edge considered an internal type, an edge that can be ignored for the purposes of ots's
bool is_internal(const valhalla::baldr::DirectedEdge* edge) {
  return edge->roundabout() || edge->internal() || edge->use() == valhalla::baldr::Use::kTurnChannel;
}
bool is_turn_channel(const valhalla::baldr::DirectedEdge* edge) {
  return edge->use() == valhalla::baldr::Use::kTurnChannel;
}

struct merged_traffic_segment_t {
  valhalla::baldr::TrafficSegment segment;
  valhalla::baldr::GraphId begin_edge;
  valhalla::baldr::GraphId end_edge;
  bool internal;
  bool turn_channel;
  const valhalla::baldr::TrafficSegment* operator->() const {
    return &segment;
  }
  valhalla::baldr::TrafficSegment* operator->() {
    return &segment;
  }
  std::vector<uint64_t> way_ids;
};
std::vector<merged_traffic_segment_t>
merge_segments(const std::vector<valhalla::meili::interpolation_t>& markers,
               valhalla::baldr::GraphReader& reader) {
  std::vector<merged_traffic_segment_t> merged;
  const valhalla::baldr::GraphTile* tile = nullptr;
  valhalla::baldr::GraphId edge;
  for (const auto& marker : markers) {
    // skip if its a repeat or we cant get the tile
    if (marker.edge == edge || !reader.GetGraphTile(marker.edge, tile)) {
      continue;
    }
    // get segments for this edge
    edge = marker.edge;
    const auto* directed_edge = tile->directededge(edge);
    // if there were no segments we'll start an invalid one to serve
    // as a placeholder for the section of the path that has no ots's
    auto segments = tile->GetTrafficSegments(edge);
    if (segments.empty()) {
      segments = {valhalla::baldr::TrafficSegment{{}, 0, 1, true, true}};
    };
    // the way id for this edge
    auto way_id = tile->edgeinfo(directed_edge->edgeinfo_offset()).wayid();
    // merge them into single entries per segment id
    for (const auto& segment : segments) {
      // continue one
      if (!merged.empty() && merged.back()->segment_id_ == segment.segment_id_) {
        merged.back().end_edge = edge;
        merged.back()->end_percent_ = segment.end_percent_;
        merged.back()->ends_segment_ = segment.ends_segment_;
        merged.back().internal = merged.back().internal && is_internal(directed_edge);
        merged.back().turn_channel = merged.back().turn_channel && is_turn_channel(directed_edge);
        if (merged.back().way_ids.size() == 0 || merged.back().way_ids.back() != way_id) {
          merged.back().way_ids.push_back(way_id);
        }
      } // new one
      else {
        merged.emplace_back(merged_traffic_segment_t{segment,
                                                     edge,
                                                     edge,
                                                     is_internal(directed_edge),
                                                     is_turn_channel(directed_edge),
                                                     {way_id}});
      }
    }
  }
  return merged;
}

/*
void print(const valhalla::meili::interpolation_t& i) {
  printf("%zu\t%.2f\t%.2f\t%.2f\n", i.edge.value, i.total_distance, i.epoch_time, i.edge_distance);
}
void print(const merged_traffic_segment_t& m) {
  printf("%zu:\t%zu->%zu\t%d->%d\t%.2f->%.2f\n",
    m.segment.segment_id_.value, m.begin_edge.value, m.end_edge.value,
    m.segment.starts_segment_, m.segment.ends_segment_,
    m.segment.begin_percent_, m.segment.end_percent_);
}
void print(const valhalla::meili::traffic_segment_t& t) {
  printf("%zu\t%.2f kph\t%d\t%.2f->%.2f\t%zu->%zu\n", t.segment_id.value, t.length/(t.end_time -
t.start_time)*3.6, t.length, t.start_time, t.end_time, t.begin_shape_index, t.end_shape_index);
}*/
} // namespace

namespace valhalla {
namespace meili {

// Threshold speed below which we assume a queue occurs (meters/sec)
constexpr float kQueueSpeedThreshold = 2.0f; // approx 4.3 MPH

TrafficSegmentMatcher::TrafficSegmentMatcher(const boost::property_tree::ptree& config)
    : matcher_factory(config),
      customizable(midgard::ToSet<boost::property_tree::ptree, std::unordered_set<std::string>>(
          config.get_child("meili.customizable"))) {
}

std::string TrafficSegmentMatcher::match(const std::string& json) {
  // Try to parse json
  Api request;
  request.parse(json, valhalla::Options::trace_route);

  // Create a matcher
  std::shared_ptr<MapMatcher> matcher;
  float default_accuracy, default_search_radius;
  try {
    matcher.reset(matcher_factory.Create(request.options.costing(), request.options));
    default_accuracy = matcher->config().get<float>("gps_accuracy");
    default_search_radius = matcher->config().get<float>("search_radius");
  } catch (...) { throw std::runtime_error("Couldn't create traffic matcher using configuration."); }

  // Populate a measurement measurements to pass to the map matcher
  auto measurements = parse_measurements(request.options, default_accuracy, default_search_radius);
  if (measurements.empty()) {
    return R"({"segments":[]})";
  }

  // Create the matched path results
  auto topk_matches = matcher->OfflineMatch(measurements);
  const auto& match_results = topk_matches.front().results;
  auto& edge_segments = topk_matches.front().segments;
  if (match_results.size() != measurements.size()) {
    throw std::runtime_error("Sequence size not equal to match result size.");
  }

  // Get the edges associated with the entire connected trace path
  auto interpolations = interpolate_matches(match_results, edge_segments, matcher);

  // Get the segments along the measurements
  auto traffic_segments = form_segments(interpolations, matcher->graphreader());

  // Check if we are overcommitted on either cache and and clear if needed
  matcher_factory.ClearFullCache();

  // give back json
  return serialize(traffic_segments);
}

std::list<std::vector<interpolation_t>>
TrafficSegmentMatcher::interpolate_matches(const std::vector<MatchResult>& matches,
                                           std::vector<EdgeSegment>& edges,
                                           const std::shared_ptr<meili::MapMatcher>& matcher) const {

  // get all of the edges along the path from the state info
  clean_edges(edges);

  // TODO: backtracking could have happened. maybe it really happened but maybe there were
  // positional inaccuracies. for now we should detect when there are backtracks and give up
  // otherwise the the timing reported here might be suspect

  // find each set of continuous edges
  std::list<std::vector<interpolation_t>> interpolations;
  size_t idx = 0;
  for (auto begin_edge = edges.cbegin(), end_edge = edges.cbegin() + 1; begin_edge != edges.cend();
       begin_edge = end_edge, end_edge += 1) {
    // find the end of the this block
    while (end_edge != edges.cend()) {
      if (!matcher->graphreader().AreEdgesConnectedForward(std::prev(end_edge)->edgeid,
                                                           end_edge->edgeid)) {
        break;
      }
      ++end_edge;
    }

    // go through each edge and each match keeping the distance each point is along the entire trace
    std::vector<interpolation_t> interpolated;
    size_t last_idx = idx;
    for (auto segment = begin_edge; segment != end_edge; ++segment) {
      float edge_length = matcher->graphreader()
                              .GetGraphTile(segment->edgeid)
                              ->directededge(segment->edgeid)
                              ->length();
      float total_length = segment == begin_edge ? -edges.front().source * edge_length
                                                 : interpolated.back().total_distance;
      // get the distance and match result for the begin node of the edge
      interpolated.emplace_back(interpolation_t{segment->edgeid, total_length, 0.f, last_idx, -1});
      // add distances for all the match points that happened on this edge
      for (; idx < matches.size(); ++idx) {
        // skip unroutable ones, we dont know what edge they were on
        if (!matches[idx].edgeid.Is_Valid()) {
          continue;
          // if its a valid one that doesnt match we move on
        } else if (matches[idx].edgeid != segment->edgeid) {
          break;
        }
        // it was the right thing we were looking for
        interpolated.emplace_back(
            interpolation_t{segment->edgeid, matches[idx].distance_along * edge_length + total_length,
                            matches[idx].distance_along, idx, matches[idx].epoch_time});
        last_idx = idx;
      }
      // add the end node of the edge
      interpolated.emplace_back(
          interpolation_t{segment->edgeid, edge_length + total_length, 1.f, last_idx, -1});
    }

    // finally backfill the time information for those points that dont have it
    auto backfill = interpolated.begin();
    while (backfill != interpolated.end()) {
      // this one is done already
      if (backfill->epoch_time != -1) {
        ++backfill;
        continue;
      }
      // find the range that have values (or the ends of the range
      auto left = backfill != interpolated.begin() ? std::prev(backfill) : backfill;
      auto right = std::next(backfill);
      for (; right != interpolated.end() && right->epoch_time == -1; ++right) {
        ;
      }
      // backfill between left and right
      while (backfill != right) {
        // if both indices are valid we interpolate
        if (left != interpolated.begin() && right != interpolated.end()) {
          double time_diff = right->epoch_time - left->epoch_time;
          float distance_diff = right->total_distance - left->total_distance;
          float distance_ratio =
              distance_diff > 0 ? (backfill->total_distance - left->total_distance) / distance_diff
                                : 0.f;
          backfill->epoch_time = left->epoch_time + distance_ratio * time_diff;
        } // if left index is valid we carry it forward if we can
        else if (left != interpolated.begin() && backfill->total_distance == left->total_distance) {
          backfill->epoch_time = left->epoch_time;
          backfill->original_index = left->original_index;
        }
        // right index is valid we carry it forward if we can
        else if (right != interpolated.end() && backfill->total_distance == right->total_distance) {
          backfill->epoch_time = right->epoch_time;
          backfill->original_index = right->original_index;
        }
        // next backfill
        ++backfill;
      }
    }

    // keep this set of interpolations
    interpolations.emplace_back(std::move(interpolated));
  }

  // give back the distances and updated match results
  return interpolations;
}

// Compute queue length. Determine where (and if) speed drops below the
// threshold along a segment.
int TrafficSegmentMatcher::compute_queue_length(std::vector<interpolation_t>::const_iterator left,
                                                std::vector<interpolation_t>::const_iterator right,
                                                const float threshold) const {
  // Iterate through pairs of interpolations until speed drops below threshold
  for (auto i1 = left, i2 = left + 1; i2 <= right; ++i1, ++i2) {
    // Skip any duplicate interpolations (happens at true graph nodes)
    float d = i2->total_distance - i1->total_distance;
    if (d == 0) {
      continue;
    }

    // Compute speed. If it falls below threshold return the remaining length
    // (from the midpoint between the 2 interpolations)
    if (d / (i2->epoch_time - i1->epoch_time) < threshold) {
      return static_cast<int>(right->total_distance -
                              (i1->total_distance + i2->total_distance) * 0.5);
    }
  }
  return 0;
}

std::vector<traffic_segment_t>
TrafficSegmentMatcher::form_segments(const std::list<std::vector<interpolation_t>>& interpolations,
                                     baldr::GraphReader& reader) const {
  // loop over each set of interpolations
  std::vector<traffic_segment_t> traffic_segments;
  for (const auto& markers : interpolations) {
    /*printf("\nInterpolations:\n");
    for(const auto& marker : markers)
      print(marker);*/

    // get all the segments for this matched path merging them into single entries
    auto merged_segments = merge_segments(markers, reader);

    /*printf("\nMerged Segments:\n");
    for(const auto& segment : merged_segments)
      print(segment);
    printf("\nReported Segments:\n");*/

    // go over the segments and move the interpolation markers accordingly
    float prior_start_acc_length = 0.0f;
    float prior_end_acc_length = 0.0f;
    auto left = markers.cbegin(), right = markers.cbegin();
    for (const auto& segment : merged_segments) {
      // move the left marker right until its adjacent to the segment begin
      left = std::find_if(left, markers.cend(), [&segment](const interpolation_t& mark) {
        return mark.edge == segment.begin_edge && mark.edge_distance >= segment->begin_percent_;
      });
      if (left->edge_distance > segment->begin_percent_) {
        left = std::prev(left);
      }

      // move the right marker right until its adjacent to the segment end
      right = std::find_if(right, markers.cend(), [&segment](const interpolation_t& mark) {
        return mark.edge == segment.end_edge && mark.edge_distance >= segment->end_percent_;
      });

      // skip any segments composed entirely of transition edges (should only be one edge really)
      // they should have no valid segment id, be marked internal and also have no way ids
      if (!segment->segment_id_.Is_Valid() && segment.internal && segment.way_ids.empty()) {
        continue;
      }

      // interpolate the length and time at the start
      double start_time = -1;
      float start_length = -1;
      auto next = segment->begin_percent_ == left->edge_distance ? left : std::next(left);
      if (segment->starts_segment_ && left->epoch_time != -1 && next->epoch_time != -1) {
        float start_diff = next->edge_distance - left->edge_distance;
        float ratio =
            start_diff > 0.f ? (segment->begin_percent_ - left->edge_distance) / start_diff : 0.f;
        start_length = left->total_distance + (next->total_distance - left->total_distance) * ratio;
        start_time = left->epoch_time + (next->epoch_time - left->epoch_time) * ratio;
      }

      // interpolate the length and time at the end
      double end_time = -1;
      float end_length = -1;
      auto prev = segment->end_percent_ == right->edge_distance ? right : std::prev(right);
      if (segment->ends_segment_ && prev->epoch_time != -1 && right->epoch_time != -1) {
        float end_diff = right->edge_distance - prev->edge_distance;
        float ratio = end_diff > 0.f ? (segment->end_percent_ - prev->edge_distance) / end_diff : 0.f;
        end_length = prev->total_distance + (right->total_distance - prev->total_distance) * ratio;
        end_time = prev->epoch_time + (right->epoch_time - prev->epoch_time) * ratio;
      }

      // figure out the total length of the segment
      int length = start_length != -1 && end_length != -1 ? (end_length - start_length) + .5f : -1;

      // Special cases for turn channels:
      //   if the prior segment is a turn channel set start to end time of
      //       the prior segment. Update length if segment has a valid end
      //   if this segment is a turn channel set the prior segment end time to
      //       the start of this turn channel segment and set the prior
      //       segment length
      size_t idx = &segment - &merged_segments[0];
      if (idx > 0 && merged_segments[idx - 1].turn_channel && start_length == -1) {
        // Set the segment start time to the end time of the turn channel
        if (traffic_segments.size() > 0) {
          start_time = traffic_segments.back().end_time;
        }

        // Update length if this segment has a valid end
        if (segment->ends_segment_ && end_length != -1) {
          length = (end_length - prior_end_acc_length) + traffic_segments.back().length / 2;
        }
      } else if (segment.turn_channel && traffic_segments.size() > 0 &&
                 traffic_segments.back().end_time == -1) {
        traffic_segments.back().end_time = start_time;
        if (length > 0 && traffic_segments.back().start_time > 0.0) {
          traffic_segments.back().length = (start_length - prior_start_acc_length) + length / 2;
        }
      }

      // Store prior start and end accumulated length
      prior_start_acc_length = start_length; // left->total_distance;
      prior_end_acc_length = end_length;     // right->total_distance;

      // compute queue length (skip if this is a partial segment)
      int queue_length = (length == -1) ? 0 : compute_queue_length(left, right, kQueueSpeedThreshold);

      // this is what we know so far
      // NOTE: in both cases we take the left most value for the shape index in an effort to be
      // conservative
      traffic_segments.emplace_back(traffic_segment_t{segment->segment_id_, start_time,
                                                      left->original_index, end_time,
                                                      prev->original_index, length, queue_length,
                                                      segment.internal, segment.way_ids});

      // print(traffic_segments.back());

      // Break out of the loop once we are past where we have interpolations
      // (e.g. for a long edge with many segments).
      if (idx > 0 && right->epoch_time == -1) {
        break;
      }

      // if the right side of this was the end of this edge then at least we need to start from the
      // next edge
      if (segment->end_percent_ == 1.f) {
        ++right;
        left = right;
      }
    }
  }

  return traffic_segments;
}

std::vector<meili::Measurement>
TrafficSegmentMatcher::parse_measurements(const Options& options,
                                          float default_accuracy,
                                          float default_search_radius) {
  // check for required trace points
  if (options.trace().empty()) {
    throw std::runtime_error("Missing required 'trace' points.");
  }

  // Populate a measurement sequence to pass to the map matcher
  std::vector<Measurement> measurements;
  try {
    for (const auto& trace_point : options.trace()) {
      double lat = trace_point.ll().lat();
      double lon = trace_point.ll().lng();
      double epoch_time = trace_point.time();
      double accuracy = default_accuracy;
      if (trace_point.has_accuracy()) {
        accuracy = trace_point.accuracy();
      }
      double search_radius = default_search_radius;
      if (trace_point.has_radius()) {
        search_radius = trace_point.radius();
      }
      measurements.emplace_back(PointLL{lon, lat}, accuracy, search_radius, epoch_time);
    }
  } catch (...) {
    throw std::runtime_error("Missing parameters, trace points require lat, lon and time.");
  }

  // not enough data
  if (measurements.size() < 2) {
    throw std::runtime_error("2 or more trace points are required.");
  }
  // out of order data, should we reorder?
  for (size_t i = 1; i < measurements.size(); ++i) {
    if (measurements[i - 1].epoch_time() > measurements[i].epoch_time()) {
      throw std::runtime_error("Trace points must be in chronological order.");
    }
  }
  // same time different place?
  auto remove_itr =
      std::remove_if(measurements.begin(), measurements.end(), [&measurements](const Measurement& m) {
        return &measurements.back() != &m && m.epoch_time() == (&m + 1)->epoch_time();
      });
  measurements.erase(remove_itr, measurements.end());
  // done with them
  return measurements;
}

std::string TrafficSegmentMatcher::serialize(const std::vector<traffic_segment_t>& traffic_segments) {
  auto segments = baldr::json::array({});
  for (const auto& seg : traffic_segments) {
    auto segment =
        baldr::json::map({{"start_time", baldr::json::fp_t{seg.start_time, 3}},
                          {"end_time", baldr::json::fp_t{seg.end_time, 3}},
                          {"length", static_cast<int64_t>(seg.length)},
                          {"begin_shape_index", static_cast<uint64_t>(seg.begin_shape_index)},
                          {"end_shape_index", static_cast<uint64_t>(seg.end_shape_index)},
                          {"queue_length", static_cast<int64_t>(seg.queue_length)},
                          {"internal", static_cast<bool>(seg.internal)}});
    // some of the segments are just sections of the path with no ots's
    if (seg.segment_id.Is_Valid()) {
      segment->emplace("segment_id", seg.segment_id.value);
    }

    auto way_ids = baldr::json::array({});
    for (auto way_id : seg.way_ids) {
      way_ids->push_back(way_id);
    }
    segment->emplace("way_ids", way_ids);

    segments->emplace_back(segment);
  }
  std::stringstream ss;
  ss << *baldr::json::map({{"segments", segments}});
  return ss.str();
}

} // namespace meili
} // namespace valhalla
