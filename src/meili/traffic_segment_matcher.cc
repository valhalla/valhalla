#include <algorithm>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "meili/traffic_segment_matcher.h"

namespace {

  void clean_segments(std::vector<valhalla::meili::EdgeSegment>& segments) {
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
    segment_itr = std::remove_if(segments.begin(), segments.end(), [&segments](const valhalla::meili::EdgeSegment& s){
      return &segments.back() != &s && s.edgeid == (&s + 1)->edgeid;
    });
    segments.erase(segment_itr, segments.end());
  }

  bool is_connected(const valhalla::baldr::GraphId& a, const valhalla::baldr::GraphId& b, valhalla::baldr::GraphReader& reader) {
    //we have to find this node
    const valhalla::baldr::GraphTile* tile;
    auto node_b = reader.GetOpposingEdge(b, tile)->endnode();
    //from edges that leave this node
    if(tile->id() != a.Tile_Base())
      tile = reader.GetGraphTile(a);
    auto node_a = tile->directededge(a)->endnode();
    if(node_a == node_b)
      return true;
    //check the transition edges
    if(tile->id() != node_a.Tile_Base())
      tile = reader.GetGraphTile(a);
    for(const auto& edge : tile->GetDirectedEdges(node_a)) {
      if((edge.trans_down() || edge.trans_up()) && edge.endnode() == node_b) {
        return true;
      }
    }
  }

  struct merged_traffic_segment_t {
    valhalla::baldr::TrafficSegment segment;
    valhalla::baldr::GraphId begin_edge;
    valhalla::baldr::GraphId end_edge;
    const valhalla::baldr::TrafficSegment* operator->() const { return &segment; }
    valhalla::baldr::TrafficSegment* operator->() { return &segment; }
  };
  std::vector<merged_traffic_segment_t> merge_segments(const std::vector<valhalla::meili::interpolation_t>& markers, valhalla::baldr::GraphReader& reader) {
    std::vector<merged_traffic_segment_t> merged;
    const valhalla::baldr::GraphTile* tile = nullptr;
    valhalla::baldr::GraphId edge;
    for(const auto& marker : markers) {
      //skip if its a repeat or we cant get the tile
      if(marker.edge == edge || !reader.GetGraphTile(marker.edge, tile))
        continue;
      //get segments for this edge
      edge = marker.edge;
      auto segments = tile->GetTrafficSegments(edge);
      for(const auto& segment : segments) {
        //new one
        if(merged.empty() || merged.back()->segment_id_ != segment.segment_id_) {
          merged.emplace_back(merged_traffic_segment_t{segment, edge});
        }//continue one
        else {
          merged.back().end_edge = edge;
          merged.back()->end_percent_ = segment.end_percent_;
          merged.back()->ends_segment_ = segment.ends_segment_;
        }

      }
    }
    return merged;
  }

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
  auto interpolations = interpolate_matches(match_results, matcher);

  // Get the segments along the measurements
  auto traffic_segments = form_segments(interpolations, matcher->graphreader());

  // Check if we are overcommitted on either cache and and clear if needed
  matcher_factory.ClearFullCache();

  //give back json
  return serialize(traffic_segments);
}

std::list<std::vector<interpolation_t> > TrafficSegmentMatcher::interpolate_matches(const std::vector<MatchResult>& matches,
  const std::shared_ptr<meili::MapMatcher>& matcher) const {

  // Get all of the edges along the path from the state info
  auto segments = ConstructRoute(matcher->mapmatching(), matches.begin(), matches.end());
  clean_segments(segments);

  // Find each set of continuous edges
  std::list<std::vector<interpolation_t> > interpolations;
  for(auto begin_segment = segments.cbegin(), end_segment = segments.cbegin() + 1; begin_segment != segments.cend(); begin_segment = end_segment) {
    //find the end of the this block
    while(end_segment != segments.cend()) {
      if(!is_connected(std::prev(end_segment)->edgeid, end_segment->edgeid, matcher->graphreader()))
        break;
      ++end_segment;
    }

    //go through each edge and each match keeping the distance each point is along the entire trace
    std::vector<interpolation_t> interpolated;
    size_t i = 0;
    for(auto segment = begin_segment; segment != end_segment; ++segment) {
      float edge_length = matcher->graphreader().GetGraphTile(segment->edgeid)->directededge(segment->edgeid)->length();
      float total_length = segment == begin_segment ? -segments.front().source * edge_length : interpolated.back().total_distance;
      //get the distance and match result for the begin node of the edge
      interpolated.emplace_back(interpolation_t{matches[i].edgeid, total_length, 0.f, i});
      //add distances for all the match points that happened on this edge
      for(++i; i < matches.size() && matches[i].edgeid == segment->edgeid; ++i) {
        interpolated.emplace_back(interpolation_t{matches[i].edgeid, matches[i].distance_along * edge_length + total_length,
          matches[i].distance_along, i, matches[i].epoch_time});
      }
      //add the end node of the edge
      interpolated.emplace_back(interpolation_t{matches[i].edgeid, edge_length + total_length, 1.f, i - 1});
    }

    //finally backfill the time information for those points that dont have it
    auto backfill = interpolated.begin();
    while(backfill != interpolated.end()) {
      //this one is done already
      if(backfill->epoch_time != 0) {
        ++backfill;
        continue;
      }
      //find the range that have values (or the ends of the range
      auto left = backfill != interpolated.begin() ? std::prev(backfill) : backfill;
      auto right = std::next(backfill);
      for(; right != interpolated.end() && right->epoch_time == 0.f; ++right);
      //backfill between left and right
      while(backfill != right) {
        //if both indices are valid we interpolate
        if(left != interpolated.begin() && right != interpolated.end()) {
          auto time_diff = right->epoch_time - left->epoch_time;
          auto distance_diff = right->total_distance - left->total_distance;
          auto distance_ratio = (backfill->total_distance - left->total_distance) / distance_diff;
          backfill->epoch_time = distance_ratio * time_diff;
        }//if left index is valid we carry it forward if we can
        else if(left != interpolated.begin() && backfill->total_distance == left->total_distance) {
          backfill->epoch_time = left->epoch_time;
          backfill->original_index = left->original_index;
        }
        //right index is valid we carry it forward if we can
        else if(right != interpolated.end()&& backfill->total_distance == right->total_distance) {
          backfill->epoch_time = right->epoch_time;
          backfill->original_index = right->original_index;
        }
        //next backfill
        ++backfill;
      }
    }

    //keep this set of interpolations
    interpolations.emplace_back(std::move(interpolated));
  }

  //give back the distances and updated match results
  return interpolations;
}

std::vector<traffic_segment_t> TrafficSegmentMatcher::form_segments(const std::list<std::vector<interpolation_t> >& interpolations,
  baldr::GraphReader& reader) const {
  //loop over each set of interpolations
  std::vector<traffic_segment_t> traffic_segments;
  for(const auto& markers : interpolations) {

    //get all the segments for this matched path merging them into single entries
    auto merged_segments = merge_segments(markers, reader);

    //go over the segments and move the interpolation markers accordingly
    auto left = markers.cbegin(), right = markers.cbegin();
    for(const auto& segment : merged_segments){
      //move the left marker right until its adjacent to the segment begin
      left = std::find_if(left, markers.cend(), [&segment](const interpolation_t& mark) {
        return mark.edge == segment.begin_edge && mark.edge_distance >= segment->begin_percent_;
      });
      left = std::prev(left);

      //interpolate the length and time at the start
      float start_time = 0, start_length = 0;
      if(segment->starts_segment_) {
        auto next = segment->begin_percent_ == left->edge_distance ? left : std::next(left);
        float start_diff = next->edge_distance - left->edge_distance;
        float start_ratio = start_diff == 0.f ? 0.f : (segment->begin_percent_ - left->edge_distance) / start_diff;
        start_length = start_ratio * left->total_distance + (1.f - start_ratio) * next->total_distance;
        start_time = start_ratio * left->epoch_time + (1.f - start_ratio) * next->epoch_time;
      }

      //move the right marker right until its adjacent to the segment end
      right = std::find_if(right, markers.cend(), [&segment](const interpolation_t& mark) {
        return mark.edge == segment.end_edge && mark.edge_distance >= segment->end_percent_;
      });

      //interpolate the length and time at the end
      float end_time = 0, end_length = 0;
      if(segment->ends_segment_) {
        auto prev = segment->end_percent_ == right->edge_distance ? right : std::prev(right);
        float end_diff = right->edge_distance - prev->edge_distance;
        float end_ratio = end_diff == 0.f ? 0.f : (segment->end_percent_ - prev->edge_distance) / end_diff;
        end_length = end_ratio * prev->total_distance + (1.f - end_ratio) * right->total_distance;
        end_time = end_ratio * prev->epoch_time + (1.f - end_ratio) * right->epoch_time;
      }

      //figure out the total length of the segment
      uint32_t length = (segment->starts_segment_ && segment->ends_segment_ ? end_length - end_time : 0.f) + .5f;

      //this is what we know so far
      traffic_segments.emplace_back(traffic_segment_t{segment->segment_id_, start_time, left->original_index, end_time, right->original_index, length});
    }
  }

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
      auto epoch_time = pt.second.get<float>("time"); //surely this wont last until 2038-01-19T03:14:08Z
      auto accuracy = pt.second.get<float>("accuracy", default_accuracy);
      measurements.emplace_back(PointLL{lon, lat}, accuracy, default_search_radius, epoch_time);
    }
  }
  catch (...) { throw std::runtime_error("Missing parameters, trace points require lat, lon and time"); }
  return measurements;
}

std::string TrafficSegmentMatcher::serialize(const std::vector<traffic_segment_t>& traffic_segments) {
  auto segments = baldr::json::array({});
  for (const auto& seg : traffic_segments) {
    segments->emplace_back(baldr::json::map
      ({
        {"segment_id", seg.segment_id.value},
        {"start_time", baldr::json::fp_t{seg.start_time, 1}},
        {"end_time", baldr::json::fp_t{seg.end_time, 1}},
        {"length", static_cast<uint64_t>(seg.length)},
        {"begin_shape_index", static_cast<uint64_t>(seg.begin_shape_index)},
        {"end_shape_index", static_cast<uint64_t>(seg.end_shape_index)},
      })
    );
  }
  std::stringstream ss;
  ss << *baldr::json::map({{"segments",segments}});
  return ss.str();
}

}
}
