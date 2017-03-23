#include <algorithm>
#include <boost/property_tree/json_parser.hpp>

#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "meili/traffic_segment_matcher.h"

namespace {

  void clean_edges(std::vector<valhalla::meili::EdgeSegment>& edges) {
    //merging the edges that are the same into the final edges record
    for(auto edge_itr = edges.begin(); edge_itr != edges.end(); ++edge_itr) {
      auto prev = std::prev(edge_itr);
      if(edge_itr != edges.begin() && edge_itr->edgeid == prev->edgeid)
        edge_itr->source = prev->source;
    }

    //delete duplicates that we copied to the final edge
    auto edge_itr = std::remove_if(edges.begin(), edges.end(), [&edges](const valhalla::meili::EdgeSegment& s){
      return &edges.back() != &s && s.edgeid == (&s + 1)->edgeid;
    });
    edges.erase(edge_itr, edges.end());
  }

  bool is_connected(const valhalla::baldr::GraphId& a, const valhalla::baldr::GraphId& b, valhalla::baldr::GraphReader& reader) {
    //we have to find this node
    const valhalla::baldr::GraphTile* tile;
    auto node_b = reader.GetOpposingEdge(b, tile)->endnode();
    //from edges that leave this node
    //NOTE: whats the effect if we cant check if its connected
    if(!reader.GetGraphTile(a, tile))
      return false;
    auto node_a = tile->directededge(a)->endnode();
    if(node_a == node_b)
      return true;
    //check the transition edges from the end node
    //NOTE: whats the effect if we cant check if its connected
    if(!reader.GetGraphTile(node_a, tile))
      return false;
    for(const auto& edge : tile->GetDirectedEdges(node_a)) {
      if((edge.trans_down() || edge.trans_up()) && edge.endnode() == node_b) {
        return true;
      }
    }
    //not connected
    return false;
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
    bool continuable = false;
    for(const auto& marker : markers) {
      //skip if its a repeat or we cant get the tile
      if(marker.edge == edge || !reader.GetGraphTile(marker.edge, tile))
        continue;
      //get segments for this edge
      edge = marker.edge;
      auto segments = tile->GetTrafficSegments(edge);
      //merge them into single entries per segment id
      for(const auto& segment : segments) {
        //continue one
        if(continuable && merged.back()->segment_id_ == segment.segment_id_){
          merged.back().end_edge = edge;
          merged.back()->end_percent_ = segment.end_percent_;
          merged.back()->ends_segment_ = segment.ends_segment_;
        }//new one
        else
          merged.emplace_back(merged_traffic_segment_t{segment, edge, edge});
      }
      //if we just handled some segments we could continue them
      continuable = !segments.empty();
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

  //get all of the edges along the path from the state info
  auto edges = ConstructRoute(matcher->mapmatching(), matches.begin(), matches.end());
  clean_edges(edges);

  //TODO: backtracking could have happened. maybe it really happened but maybe there were positional
  //inaccuracies. for now we should detect when there are backtracks and give up otherwise the
  //the timing reported here might be suspect

  //find each set of continuous edges
  std::list<std::vector<interpolation_t> > interpolations;
  for(auto begin_edge = edges.cbegin(), end_edge = edges.cbegin() + 1; begin_edge != edges.cend(); begin_edge = end_edge, end_edge += 1) {
    //find the end of the this block
    while(end_edge != edges.cend()) {
      if(!is_connected(std::prev(end_edge)->edgeid, end_edge->edgeid, matcher->graphreader()))
        break;
      ++end_edge;
    }

    //go through each edge and each match keeping the distance each point is along the entire trace
    std::vector<interpolation_t> interpolated;
    size_t i = 0, last_index = 0;
    for(auto segment = begin_edge; segment != end_edge; ++segment) {
      float edge_length = matcher->graphreader().GetGraphTile(segment->edgeid)->directededge(segment->edgeid)->length();
      float total_length = segment == begin_edge ? -edges.front().source * edge_length : interpolated.back().total_distance;
      //get the distance and match result for the begin node of the edge
      interpolated.emplace_back(interpolation_t{segment->edgeid, total_length, 0.f, last_index, -1});
      //add distances for all the match points that happened on this edge
      for(; i < matches.size(); ++i) {
        //skip unroutable ones, we dont know what edge they were on
        if(!matches[i].edgeid.Is_Valid())
          continue;
        //if its a valid one that doesnt match we move on
        else if(matches[i].edgeid != segment->edgeid)
          break;
        //it was the right thing we were looking for
        interpolated.emplace_back(interpolation_t{segment->edgeid, matches[i].distance_along * edge_length + total_length,
          matches[i].distance_along, i, matches[i].epoch_time});
        last_index = i;
      }
      //add the end node of the edge
      interpolated.emplace_back(interpolation_t{segment->edgeid, edge_length + total_length, 1.f, last_index, -1});
    }

    //finally backfill the time information for those points that dont have it
    auto backfill = interpolated.begin();
    while(backfill != interpolated.end()) {
      //this one is done already
      if(backfill->epoch_time != -1) {
        ++backfill;
        continue;
      }
      //find the range that have values (or the ends of the range
      auto left = backfill != interpolated.begin() ? std::prev(backfill) : backfill;
      auto right = std::next(backfill);
      for(; right != interpolated.end() && right->epoch_time == -1; ++right);
      //backfill between left and right
      while(backfill != right) {
        //if both indices are valid we interpolate
        if(left != interpolated.begin() && right != interpolated.end()) {
          double time_diff = right->epoch_time - left->epoch_time;
          float distance_diff = right->total_distance - left->total_distance;
          float distance_ratio = distance_diff > 0 ? (backfill->total_distance - left->total_distance) / distance_diff : 0.f;
          backfill->epoch_time = left->epoch_time + distance_ratio * time_diff;
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
      if(left->edge_distance > segment->begin_percent_)
        left = std::prev(left);

      //interpolate the length and time at the start
      double start_time = -1;
      float start_length = -1;
      auto next = segment->begin_percent_ == left->edge_distance ? left : std::next(left);
      if(segment->starts_segment_ && left->epoch_time != -1 && next->epoch_time != -1) {
        float start_diff = next->edge_distance - left->edge_distance;
        float coef = start_diff > 0.f ? (segment->begin_percent_ - left->edge_distance) / start_diff : 0.f;
        start_length = (1.f - coef) * left->total_distance + coef * next->total_distance;
        start_time = (1.f - coef) * left->epoch_time + coef * next->epoch_time;
      }

      //move the right marker right until its adjacent to the segment end
      right = std::find_if(right, markers.cend(), [&segment](const interpolation_t& mark) {
        return mark.edge == segment.end_edge && mark.edge_distance >= segment->end_percent_;
      });

      //interpolate the length and time at the end
      double end_time = -1;
      float end_length = -1;
      auto prev = segment->end_percent_ == right->edge_distance ? right : std::prev(right);
      if(segment->ends_segment_ && prev->epoch_time != -1 && right->epoch_time != -1) {
        float end_diff = right->edge_distance - prev->edge_distance;
        float coef = end_diff > 0.f ? (segment->end_percent_ - prev->edge_distance) / end_diff : 0.f;
        end_length = (1.f - coef) * prev->total_distance + coef * right->total_distance;
        end_time = (1.f - coef) * prev->epoch_time + coef * right->epoch_time;
      }

      //if we didnt have a start or end time then we are past where we had interpolations
      if(start_time == -1 && end_time == -1)
        break;

      //figure out the total length of the segment
      int length = start_length != -1 && end_length != -1 ? (end_length - start_length) +.5f : -1;

      //this is what we know so far
      //NOTE: in both cases we take the left most value for the shape index in an effort to be conservative
      traffic_segments.emplace_back(traffic_segment_t{segment->segment_id_, start_time, left->original_index, end_time, prev->original_index, length});

      //if the right side of this was the end of this edge then at least we need to start from the next edge
      if(segment->end_percent_ == 1.f) {
        ++right;
        left = right;
      }
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
      double lat = pt.second.get<double>("lat");
      double lon = pt.second.get<double>("lon");
      double epoch_time = pt.second.get<double>("time");
      double accuracy = pt.second.get<double>("accuracy", default_accuracy);
      measurements.emplace_back(PointLL{lon, lat}, accuracy, default_search_radius, epoch_time);
    }
  }
  catch (...) { throw std::runtime_error("Missing parameters, trace points require lat, lon and time"); }
  if(measurements.size() < 2)
    throw std::runtime_error("2 or more trace points are required");
  for(size_t i = 1; i < measurements.size(); ++i)
    if(measurements[i - 1].epoch_time() > measurements[i].epoch_time())
      throw std::runtime_error("Trace points must be in chronological order");
  return measurements;
}

std::string TrafficSegmentMatcher::serialize(const std::vector<traffic_segment_t>& traffic_segments) {
  auto segments = baldr::json::array({});
  for (const auto& seg : traffic_segments) {
    segments->emplace_back(baldr::json::map
      ({
        {"segment_id", seg.segment_id.value},
        {"start_time", baldr::json::fp_t{seg.start_time, 3}},
        {"end_time", baldr::json::fp_t{seg.end_time, 3}},
        {"length", static_cast<int64_t>(seg.length)},
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
