// -*- mode: c++ -*-
#ifndef MMP_MAP_MATCHING_H_
#define MMP_MAP_MATCHING_H_

#include <algorithm>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/costconstants.h>

#include <mmp/candidate.h>
#include <mmp/candidate_search.h>
#include <mmp/viterbi_search.h>
#include <mmp/routing.h>


namespace mmp {

using namespace valhalla;


class Measurement
{
 public:
  Measurement(const midgard::PointLL& lnglat)
      : lnglat_(lnglat) {}

  const midgard::PointLL& lnglat() const
  { return lnglat_; }

 private:
  midgard::PointLL lnglat_;
};


class State
{
 public:
  State(const StateId id,
        const Time time,
        const Candidate& candidate);

  const StateId id() const
  { return id_; }

  const Time time() const
  { return time_; }

  const Candidate& candidate() const
  { return candidate_; }

  bool routed() const
  { return labelset_ != nullptr; }

  void route(const std::vector<const State*>& states,
             baldr::GraphReader& graphreader,
             float max_route_distance,
             sif::cost_ptr_t costing,
             std::shared_ptr<const sif::EdgeLabel> edgelabel,
             const float turn_cost_table[181]) const;

  const Label* last_label(const State& state) const;

  RoutePathIterator RouteBegin(const State& state) const
  {
    const auto it = label_idx_.find(state.id());
    if (it != label_idx_.end()) {
      return RoutePathIterator(labelset_.get(), it->second);
    }
    return RoutePathIterator(labelset_.get());
  }

  RoutePathIterator RouteEnd() const
  { return RoutePathIterator(labelset_.get()); }

 private:
  const StateId id_;

  const Time time_;

  const Candidate candidate_;

  mutable std::shared_ptr<LabelSet> labelset_;

  mutable std::unordered_map<StateId, uint32_t> label_idx_;
};


class MapMatching: public ViterbiSearch<State>
{
 public:
  MapMatching(baldr::GraphReader& graphreader,
              const sif::cost_ptr_t* mode_costing,
              const sif::TravelMode mode,
              float sigma_z,
              float beta,
              float breakage_distance,
              float max_route_distance_factor,
              float turn_penalty_factor);

  MapMatching(baldr::GraphReader& graphreader,
              const sif::cost_ptr_t* mode_costing,
              const sif::TravelMode mode,
              const boost::property_tree::ptree& config);

  virtual ~MapMatching();

  void Clear();

  baldr::GraphReader& graphreader() const
  { return graphreader_; }

  sif::cost_ptr_t costing() const
  { return mode_costing_[static_cast<size_t>(mode_)]; }

  const std::vector<const State*>&
  states(Time time) const
  { return states_[time]; }

  const Measurement& measurement(Time time) const
  { return measurements_[time]; }

  const Measurement& measurement(const State& state) const
  { return measurements_[state.time()]; }

  std::vector<Measurement>::size_type size() const
  { return measurements_.size(); }

  template <typename candidate_iterator_t>
  Time AppendState(const Measurement& measurement,
                   candidate_iterator_t begin,
                   candidate_iterator_t end);

 protected:
  virtual float MaxRouteDistance(const State& left, const State& right) const;

  float TransitionCost(const State& left, const State& right) const override;

  float EmissionCost(const State& state) const override;

  double CostSofar(double prev_costsofar, float transition_cost, float emission_cost) const override;

 private:

  baldr::GraphReader& graphreader_;

  const sif::cost_ptr_t* mode_costing_;

  const sif::TravelMode mode_;

  std::vector<Measurement> measurements_;

  std::vector<std::vector<const State*>> states_;

  float sigma_z_;
  double inv_double_sq_sigma_z_;  // equals to 1.f / (sigma_z_ * sigma_z_ * 2.f)

  float beta_;
  float inv_beta_;  // equals to 1.f / beta_

  float breakage_distance_;

  float max_route_distance_factor_;

  float turn_penalty_factor_;

  // Cost for each degree in [0, 180]
  float turn_cost_table_[181];
};


enum class GraphType: uint8_t
{ kUnknown = 0, kEdge, kNode };


class MatchResult
{
 public:
  MatchResult(const midgard::PointLL& lnglat,
              float distance,
              const baldr::GraphId graphid,
              GraphType graphtype,
              const State* state = nullptr)
      : lnglat_(lnglat),
        distance_(distance),
        graphid_(graphid),
        graphtype_(graphtype),
        state_(state) {}

  MatchResult(const midgard::PointLL& lnglat)
      : lnglat_(lnglat),
        distance_(0.f),
        graphid_(),
        graphtype_(GraphType::kUnknown),
        state_(nullptr)
  { assert(!graphid_.Is_Valid()); }

  // Coordinate of the matched point
  const midgard::PointLL& lnglat() const
  { return lnglat_; }

  // Distance from measurement to the matched point
  float distance() const
  { return distance_; }

  // Which edge/node this matched point stays
  const baldr::GraphId graphid() const
  { return graphid_; }

  GraphType graphtype() const
  { return graphtype_; }

  // Attach the state pointer for other information (e.g. reconstruct
  // the route path) and debugging
  const State* state() const
  { return state_; }

 private:
  midgard::PointLL lnglat_;

  float distance_;

  baldr::GraphId graphid_;

  GraphType graphtype_;

  const State* state_;
};


struct EdgeSegment
{
  EdgeSegment(baldr::GraphId the_edgeid,
              float the_source = 0.f,
              float the_target = 1.f);

  std::vector<midgard::PointLL> Shape(baldr::GraphReader& graphreader) const;

  bool Adjoined(baldr::GraphReader& graphreader, const EdgeSegment& other) const;

  // TODO make them private
  baldr::GraphId edgeid;

  float source;

  float target;
};


template <typename segment_iterator_t>
std::string
RouteToString(baldr::GraphReader& graphreader,
              segment_iterator_t segment_begin,
              segment_iterator_t segment_end,
              const baldr::GraphTile*& tile)
{
  // The string will look like: [dummy] [source/startnodeid edgeid target/endnodeid] ...
  std::ostringstream route;

  for (auto segment = segment_begin; segment != segment_end; segment++) {
    if (segment->edgeid.Is_Valid()) {
      route << "[";

      if (segment->source == 0.f) {
        route << helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
      } else {
        route << segment->source;
      }

      route << " " << segment->edgeid << " ";

      if (segment->target == 1.f) {
        route << helpers::edge_endnodeid(graphreader, segment->edgeid, tile);
      } else {
        route << segment->target;
      }

      route << "]";
    } else {
      route << "[dummy]";
    }
    route << " ";
  }

  auto route_str = route.str();
  if (!route_str.empty()) {
    route_str.pop_back();
  }
  return route_str;
}


// Validate a route. It check if all edge segments of the route are
// valid and successive, and no loop
template <typename segment_iterator_t>
bool ValidateRoute(baldr::GraphReader& graphreader,
                   segment_iterator_t segment_begin,
                   segment_iterator_t segment_end,
                   const baldr::GraphTile*& tile)
{
  if (segment_begin == segment_end) {
    return true;
  }

  // The first segment must be dummy
  if (!(!segment_begin->edgeid.Is_Valid()
        && segment_begin->source == 0.f
        && segment_begin->target == 0.f)) {
    LOG_ERROR("Found the first segment's edgeid is not dummpy");
    LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
    return false;
  }

  for (auto segment = std::next(segment_begin);  // Skip the first dummy segment
       segment != segment_end; segment++) {
    // The rest of segments must have valid edgeid
    if (!segment->edgeid.Is_Valid()) {
      LOG_ERROR("Found invalid edgeid at segment " + std::to_string(segment - segment_begin));
      LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
      return false;
    }

    // Skip the first non-dummy segment
    const auto prev_segment = std::prev(segment);
    if (prev_segment == segment_begin) {
      continue;
    }

    // Successive segments must be adjacent and no loop absolutely!
    if (prev_segment->edgeid == segment->edgeid) {
      if (prev_segment->target != segment->source) {
        LOG_ERROR("Found disconnected segments at " + std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));

        // A temporary fix here: this exception is due to a few of
        // loops in the graph. The error message below is one example
        // of the fail case: the edge 2/698780/4075 is a loop since it
        // ends and starts at the same node 2/698780/1433:

        // [ERROR] Found disconnected segments at 2
        // [ERROR] [dummy] [0.816102 2/698780/4075 2/698780/1433] [2/698780/1433 2/698780/4075 0.460951]

        // We should remove this block of code when this issue is
        // solved from upstream
        const auto endnodeid = helpers::edge_endnodeid(graphreader, prev_segment->edgeid, tile);
        const auto startnodeid = helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
        if (endnodeid == startnodeid) {
          LOG_ERROR("This is a loop. Let it go");
          return true;
        }
        // End of the fix

        return false;
      }
    } else {
      const auto endnodeid = helpers::edge_endnodeid(graphreader, prev_segment->edgeid, tile),
               startnodeid = helpers::edge_startnodeid(graphreader, segment->edgeid, tile);
      if (!(prev_segment->target == 1.f
            && segment->source == 0.f
            && endnodeid == startnodeid)) {
        LOG_ERROR("Found disconnected segments at " + std::to_string(segment - segment_begin));
        LOG_ERROR(RouteToString(graphreader, segment_begin, segment_end, tile));
        return false;
      }
    }
  }

  return true;
}


template <typename segment_iterator_t>
void MergeRoute(std::vector<EdgeSegment>& route,
                segment_iterator_t segment_begin,
                segment_iterator_t segment_end)
{
  if (segment_begin == segment_end) {
    return;
  }

  for (auto segment = std::next(segment_begin);  // Skip the first dummy segment
       segment != segment_end; segment++) {
    if (!segment->edgeid.Is_Valid()) {
      throw std::runtime_error("Still found an invalid edgeid in route segments");
    }
    if(!route.empty()) {
      auto& last_segment = route.back();
      if (last_segment.edgeid == segment->edgeid) {
        if (last_segment.target != segment->source
            && segment != std::next(segment_begin)) {
          // TODO should throw runtime error. See the temporary fix
          LOG_ERROR("Still found a disconnected route in which segment "
                    + std::to_string(segment - segment_begin) + " ends at "
                    + std::to_string(last_segment.target)
                    + " but the next segment starts at "
                    + std::to_string(segment->source));
        }
        // and here we should extend last_segment.target =
        // segment->target since last_segment.target <=
        // segment->target but see the temporary fix
        last_segment.target = std::max(last_segment.target, segment->target);
      } else {
        route.push_back(*segment);
      }
    } else {
      route.push_back(*segment);
    }
  }
}


template <typename match_iterator_t>
std::vector<EdgeSegment>
ConstructRoute(baldr::GraphReader& graphreader,
               match_iterator_t begin,
               match_iterator_t end)
{
  std::vector<EdgeSegment> route;
  match_iterator_t previous_match = end;
  const baldr::GraphTile* tile = nullptr;

  for (auto match = begin; match != end; match++) {
    if (match->state()) {
      if (previous_match != end) {
        std::vector<EdgeSegment> segments;
        for (auto segment = previous_match->state()->RouteBegin(*match->state()),
                      end = previous_match->state()->RouteEnd();
             segment != end; segment++) {
          segments.emplace_back(segment->edgeid, segment->source, segment->target);
        }
        if (ValidateRoute(graphreader, segments.rbegin(), segments.rend(), tile)) {
          MergeRoute(route, segments.rbegin(), segments.rend());
        } else {
          throw std::runtime_error("Found invalid route");
        }
      }
      previous_match = match;
    }
  }

  return route;
}


inline float local_tile_size(const baldr::GraphReader& graphreader)
{
  const auto& tile_hierarchy = graphreader.GetTileHierarchy();
  const auto& tiles = tile_hierarchy.levels().rbegin()->second.tiles;
  return tiles.TileSize();
}


// A facade that connects everything
class MapMatcher final
{
 public:
  MapMatcher(const boost::property_tree::ptree&,
             baldr::GraphReader&,
             CandidateGridQuery&,
             const sif::cost_ptr_t*,
             sif::TravelMode);

  ~MapMatcher();

  baldr::GraphReader& graphreader()
  { return graphreader_; }

  CandidateQuery& rangequery()
  { return rangequery_; }

  sif::TravelMode travelmode() const
  { return travelmode_; }

  const boost::property_tree::ptree config() const
  { return config_; }

  boost::property_tree::ptree config()
  { return config_; }

  MapMatching& mapmatching()
  { return mapmatching_; }

  std::vector<MatchResult>
  OfflineMatch(const std::vector<Measurement>&);

 private:
  boost::property_tree::ptree config_;

  baldr::GraphReader& graphreader_;

  CandidateGridQuery& rangequery_;

  const sif::cost_ptr_t* mode_costing_;

  sif::TravelMode travelmode_;

  MapMatching mapmatching_;
};


constexpr size_t kModeCostingCount = 8;


class MapMatcherFactory final
{
public:
  MapMatcherFactory(const boost::property_tree::ptree&);

  ~MapMatcherFactory();

  baldr::GraphReader& graphreader()
  { return graphreader_; }

  CandidateQuery& rangequery()
  { return rangequery_; }

  sif::TravelMode NameToTravelMode(const std::string&);

  const std::string& TravelModeToName(sif::TravelMode);

  MapMatcher* Create(sif::TravelMode travelmode)
  { return Create(travelmode, boost::property_tree::ptree()); }

  MapMatcher* Create(const std::string& name)
  { return Create(NameToTravelMode(name), boost::property_tree::ptree()); }

  MapMatcher* Create(const std::string& name,
                     const boost::property_tree::ptree& preferences)
  { return Create(NameToTravelMode(name), preferences); }

  MapMatcher* Create(const boost::property_tree::ptree&);

  MapMatcher* Create(sif::TravelMode, const boost::property_tree::ptree&);

  boost::property_tree::ptree
  MergeConfig(const std::string&, const boost::property_tree::ptree&);

  boost::property_tree::ptree&
  MergeConfig(const std::string&, boost::property_tree::ptree&);

  void ClearCacheIfPossible();

  void ClearCache();

private:
  typedef sif::cost_ptr_t (*factory_function_t)(const boost::property_tree::ptree&);

  boost::property_tree::ptree config_;

  baldr::GraphReader graphreader_;

  sif::cost_ptr_t mode_costing_[kModeCostingCount];

  std::string mode_name_[kModeCostingCount];

  CandidateGridQuery rangequery_;

  float max_grid_cache_size_;

  size_t register_costing(const std::string&, factory_function_t, const boost::property_tree::ptree&);

  sif::cost_ptr_t* init_costings(const boost::property_tree::ptree&);
};


}


#endif // MMP_MAP_MATCHING_H_
