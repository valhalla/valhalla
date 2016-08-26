#include <vector>

#include "meili/graph_helpers.h"
#include "meili/map_matcher.h"


namespace {

using namespace valhalla;
using namespace valhalla::meili;


inline float
GreatCircleDistanceSquared(const Measurement& left,
                           const Measurement& right)
{ return left.lnglat().DistanceSquared(right.lnglat()); }


// Collect a nodeid set of a path location
std::unordered_set<baldr::GraphId>
collect_nodes(baldr::GraphReader& graphreader, const Candidate& location)
{
  std::unordered_set<baldr::GraphId> results;
  const baldr::GraphTile* tile = nullptr;

  for (const auto& edge : location.edges) {
    if (!edge.id.Is_Valid()) continue;
    if (edge.dist == 0.f) {
      const auto& startnodeid = helpers::edge_startnodeid(graphreader, edge.id, tile);
      if (startnodeid.Is_Valid()) {
        results.insert(startnodeid);
      }
    } else if (edge.dist == 1.f) {
      const auto& endnodeid = helpers::edge_endnodeid(graphreader, edge.id, tile);
      if (endnodeid.Is_Valid()) {
        results.insert(endnodeid);
      }
    }
  }

  return results;
}


MatchResult
guess_source_result(const MapMatching::state_iterator source,
                    const MapMatching::state_iterator target,
                    const Measurement& source_measurement)
{
  if (source.IsValid() && target.IsValid()) {
    baldr::GraphId last_valid_id;
    GraphType last_valid_type = GraphType::kUnknown;
    for (auto label = source->RouteBegin(*target);
         label != source->RouteEnd(); label++) {
      if (label->nodeid.Is_Valid()) {
        last_valid_id = label->nodeid;
        last_valid_type = GraphType::kNode;
      } else if (label->edgeid.Is_Valid()) {
        last_valid_id = label->edgeid;
        last_valid_type = GraphType::kEdge;
      }
    }
    const auto& c = source->candidate();
    //Note: technically a candidate can have correlated to more than one place in the graph
    //but the way its used in meili we only correlated it to one place so .front() is safe
    return {c.edges.front().projected, c.distance(), last_valid_id, last_valid_type, source->id()};
  } else if (source.IsValid()) {
    return {source_measurement.lnglat(), 0.f, baldr::GraphId(), GraphType::kUnknown, source->id()};
  }

  return {source_measurement.lnglat()};
}


MatchResult
guess_target_result(const MapMatching::state_iterator source,
                    const MapMatching::state_iterator target,
                    const Measurement& target_measurement)
{
  if (source.IsValid() && target.IsValid()) {
    auto label = source->RouteBegin(*target);
    baldr::GraphId graphid;
    GraphType graphtype = GraphType::kUnknown;
    if (label != source->RouteEnd()) {
      if (label->nodeid.Is_Valid()) {
        graphid = label->nodeid;
        graphtype = GraphType::kNode;
      } else if (label->edgeid.Is_Valid()) {
        graphid = label->edgeid;
        graphtype = GraphType::kEdge;
      }
    }
    const auto& c = target->candidate();
    //Note: technically a candidate can have correlated to more than one place in the graph
    //but the way its used in meili we only correlated it to one place so .front() is safe
    return {c.edges.front().projected, c.distance(), graphid, graphtype, target->id()};
  } else if (target.IsValid()) {
    return {target_measurement.lnglat(), 0.f, baldr::GraphId(), GraphType::kUnknown, target->id()};
  }

  return {target_measurement.lnglat()};
}


template <typename candidate_iterator_t>
MatchResult
interpolate(baldr::GraphReader& reader,
            const std::unordered_set<baldr::GraphId>& graphset,
            candidate_iterator_t begin,
            candidate_iterator_t end,
            const Measurement& measurement)
{
  auto closest_candidate = end;
  float closest_sq_distance = std::numeric_limits<float>::infinity();
  baldr::GraphId closest_graphid;
  GraphType closest_graphtype = GraphType::kUnknown;

  for (auto candidate = begin; candidate != end; candidate++) {
    if (candidate->sq_distance() < closest_sq_distance) {
      //Note: technically a candidate can have correlated to more than one place in the graph
      //but the way its used in meili we only correlated it to one place so .front() is safe
      if (!candidate->edges.front().begin_node() && !candidate->edges.front().end_node()) {
        for (const auto& edge : candidate->edges) {
          const auto it = graphset.find(edge.id);
          if (it != graphset.end()) {
            closest_candidate = candidate;
            closest_sq_distance = candidate->sq_distance();
            closest_graphid = edge.id;
            closest_graphtype = GraphType::kEdge;
          }
        }
      } else {
        for (const auto nodeid : collect_nodes(reader, *candidate)) {
          const auto it = graphset.find(nodeid);
          if (it != graphset.end()) {
            closest_candidate = candidate;
            closest_sq_distance = candidate->sq_distance();
            closest_graphid = nodeid;
            closest_graphtype = GraphType::kNode;
          }
        }
      }
    }
  }

  if (closest_candidate != end) {
    //Note: technically a candidate can have correlated to more than one place in the graph
    //but the way its used in meili we only correlated it to one place so .front() is safe
    return {closest_candidate->edges.front().projected, closest_candidate->distance(), closest_graphid, closest_graphtype, kInvalidStateId};
  }

  return {measurement.lnglat()};
}


std::unordered_set<baldr::GraphId>
collect_graphset(baldr::GraphReader& reader,
                 const MapMatching::state_iterator source,
                 const MapMatching::state_iterator target)
{
  std::unordered_set<baldr::GraphId> graphset;
  if (source.IsValid() && target.IsValid()) {
    for (auto label = source->RouteBegin(*target);
         label != source->RouteEnd();
         ++label) {
      if (label->edgeid.Is_Valid()) {
        graphset.insert(label->edgeid);
      }
      if (label->nodeid.Is_Valid()) {
        graphset.insert(label->nodeid);
      }
    }
  } else if (source.IsValid()) {
    const auto& location = source->candidate();
    //Note: technically a location can have correlated to more than one place in the graph
    //but the way its used in meili we only correlated it to one place so .front() is safe
    if (!location.edges.front().begin_node() && !location.edges.front().end_node()) {
      for (const auto& edge : location.edges) {
        if (edge.id.Is_Valid()) {
          graphset.insert(edge.id);
        }
      }
    } else {
      for (const auto nodeid : collect_nodes(reader, location)) {
        if (nodeid.Is_Valid()) {
          graphset.insert(nodeid);
        }
      }
    }
  }

  return graphset;
}


std::vector<MatchResult>
OfflineMatch_(MapMatching& mm,
              const CandidateQuery& cq,
              const std::vector<Measurement>& measurements,
              float interpolation_distance)
{
  mm.Clear();

  if (measurements.empty()) {
    return {};
  }

  using mmt_size_t = std::vector<Measurement>::size_type;
  Time time = 0;
  const float sq_interpolation_distance = interpolation_distance * interpolation_distance;
  std::unordered_map<Time, std::vector<mmt_size_t>> proximate_measurements;

  // Load states
  for (mmt_size_t idx = 0,
             last_idx = 0,
              end_idx = measurements.size() - 1;
       idx <= end_idx; idx++) {
    const auto& measurement = measurements[idx];
    auto sq_distance = GreatCircleDistanceSquared(measurements[last_idx], measurement);
    // Always match the first and the last measurement
    if (sq_interpolation_distance <= sq_distance || idx == 0 || idx == end_idx) {
      const auto& candidates = cq.Query(measurement.lnglat(),
                                        measurement.sq_search_radius(),
                                        mm.costing()->GetEdgeFilter());
      time = mm.AppendState(measurement, candidates.begin(), candidates.end());
      last_idx = idx;
    } else {
      proximate_measurements[time].push_back(idx);
    }
  }

  // Search viterbi path
  std::vector<MapMatching::state_iterator> iterpath;
  iterpath.reserve(mm.size());
  for (auto it = mm.SearchPath(time); it != mm.PathEnd(); it++) {
    iterpath.push_back(it);
  }
  std::reverse(iterpath.begin(), iterpath.end());
  if (!(iterpath.size() == mm.size())) {
    std::logic_error("Every measurement should have matched a state");
  }

  // Interpolate proximate measurements and merge their states into
  // the results
  std::vector<MatchResult> results;
  results.reserve(measurements.size());
  results.emplace_back(measurements.front().lnglat());

  for (Time time = 1; time < mm.size(); time++) {
    const auto &source_state = iterpath[time - 1],
               &target_state = iterpath[time];

    if (!results.back().graphid().Is_Valid()) {
      results.pop_back();
      results.push_back(guess_source_result(source_state, target_state, measurements[results.size()]));
    }

    auto it = proximate_measurements.find(time - 1);
    if (it != proximate_measurements.end()) {
      const auto& graphset = collect_graphset(mm.graphreader(), source_state, target_state);
      for (const auto idx : it->second) {
        const auto& candidates = cq.Query(measurements[idx].lnglat(),
                                          measurements[idx].sq_search_radius(),
                                          mm.costing()->GetEdgeFilter());
        results.push_back(interpolate(mm.graphreader(), graphset,
                                      candidates.begin(), candidates.end(),
                                      measurements[idx]));
      }
    }

    results.push_back(guess_target_result(source_state, target_state, measurements[results.size()]));
  }
  if(results.size() != measurements.size())
    throw std::logic_error("The number of matched points does not match the number of input points.");

  return results;
}

}


namespace valhalla {
namespace meili {

MapMatcher::MapMatcher(const boost::property_tree::ptree& config,
                       baldr::GraphReader& graphreader,
                       CandidateQuery& candidatequery,
                       const sif::cost_ptr_t* mode_costing,
                       sif::TravelMode travelmode)
    : config_(config),
      graphreader_(graphreader),
      candidatequery_(candidatequery),
      mode_costing_(mode_costing),
      travelmode_(travelmode),
      mapmatching_(graphreader_, mode_costing_, travelmode_, config_) {}


MapMatcher::~MapMatcher() {}


std::vector<MatchResult>
MapMatcher::OfflineMatch(const std::vector<Measurement>& measurements)
{
  const auto interpolation_distance = config_.get<float>("interpolation_distance");
  return OfflineMatch_(mapmatching_, candidatequery_, measurements, interpolation_distance);
}

}
}
