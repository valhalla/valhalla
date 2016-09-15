#include "sif/transitcost.h"

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr uint32_t kUnitSize = 1;

constexpr float kModeWeight             = 1.0f; // Favor this mode?
constexpr float kDefaultTransferCost    = 15.0f;
constexpr float kDefaultTransferPenalty = 300.0f;

// User propensity to use buses. Range of values from 0 (avoid buses) to
// 1 (totally comfortable riding on buses).
constexpr float kDefaultUseBusFactor = 0.3f;

// User propensity to use rail. Range of values from 0 (avoid rail) to
// 1 (totally comfortable riding on rail).
constexpr float kDefaultUseRailFactor = 0.6f;

// User propensity to use/allow transfers. Range of values from 0
// (avoid transfers) to 1 (totally comfortable with transfers).
constexpr float kDefaultUseTransfersFactor = 0.3f;

Cost kImpossibleCost = { 10000000.0f, 10000000.0f };

}

/**
 * Derived class providing dynamic edge costing for transit parts
 * of multi-modal routes.
 */
class TransitCost : public DynamicCost {
 public:
  /**
   * Constructor. Configuration / options for pedestrian costing are provided
   * via a property tree (JSON).
   * @param  pt  Property tree with configuration/options.
   */
  TransitCost(const boost::property_tree::ptree& pt);

  virtual ~TransitCost();

  /**
   * This method overrides the weight for this mode.  The higher the value
   * the more the mode is favored.
   */
  virtual float GetModeWeight();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @param   density  Relative road density.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const uint32_t density) const;

  /**
   * Get the cost to traverse the specified directed edge using a transit
   * departure (schedule based edge traversal). Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   departure Transit departure record.
   * @param   curr_time Current local time (seconds from midnight).
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

  /**
   * Returns the transfer cost between 2 transit stops.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost TransferCost() const;

  /**
   * Returns the default transfer cost between 2 transit lines.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost DefaultTransferCost() const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const;

  /**
   * Override unit size since walking costs are higher range of vales
   */
  virtual uint32_t UnitSize() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by transit route use.
   * This is used to conflate the stops to OSM way ids and we don't want to
   * include ferries.
   * @return Function/functor to be used in filtering out edges
   */
  virtual const EdgeFilter GetEdgeFilter() const {
    // Throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge) {
      if (edge->trans_up() || edge->trans_down() ||
          edge->use() >= Use::kFerry ||
         !(edge->forwardaccess() & kPedestrianAccess))
        return 0.0f;
      else {
        // TODO - use classification/use to alter the factor
        return 1.0f;
      }
    };
  }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   * @return Function/functor to be used in filtering out nodes
   */
  virtual const NodeFilter GetNodeFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::NodeInfo* node){
      return !(node->access() & kPedestrianAccess);
    };
  }

  /**This method adds to the exclude list based on the
   * user inputed exclude and include lists.
   */
  virtual void AddToExcludeList(const baldr::GraphTile*& tile);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile,
                          const baldr::DirectedEdge* edge);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile,
                          const baldr::NodeInfo* node);

 protected:

  // This is the weight for this mode.  The higher the value the more the
  // mode is favored.
  float mode_weight_;

  // A measure of willingness to ride on buses or rail. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable with taking
  // this transportation. These factors determine how much rail or buses are
  // preferred over each other (if at all).
  float use_bus_;
  float use_rail_;
  float bus_factor_;
  float rail_factor_;

  // A measure of willingness to make transfers. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable.
  float use_transfers_;
  float transfer_factor_;

  float transfer_cost_;     // Transfer cost
  float transfer_penalty_;  // Transfer penalty

  struct TileIndexHasher {
    std::size_t operator()(const tile_index_pair& tile_line) const {
      std::size_t seed = 13;
      boost::hash_combine(seed, id_hasher(tile_line.first));
      boost::hash_combine(seed, id_hasher(tile_line.second));
      return seed;
    }
    //function to hash each id
    std::hash<uint32_t> id_hasher;
  };

  // stops exclude list
  std::unordered_set<std::string> stop_exclude_onestops_;

  // stops include list
  std::unordered_set<std::string> stop_include_onestops_;

  // operator exclude list
  std::unordered_set<std::string> oper_exclude_onestops_;

  // operator include list
  std::unordered_set<std::string> oper_include_onestops_;

  // route excluded list
  std::unordered_set<std::string> route_exclude_onestops_;

  // route include list
  std::unordered_set<std::string> route_include_onestops_;

  //our final one exclude list of pairs
  std::unordered_set<tile_index_pair, TileIndexHasher> exclude_;

  //our final one exclude list of pairs
  std::unordered_set<tile_index_pair, TileIndexHasher> exclude_stops_;
};

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
TransitCost::TransitCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kPublicTransit) {

  mode_weight_ = pt.get<float>("mode_weight", kModeWeight);

  // Willingness to use buses. Make sure this is within range [0, 1].
  use_bus_ = pt.get<float>("use_bus", kDefaultUseBusFactor);
  if (use_bus_ < 0.0f || use_bus_ > 1.0f) {
    use_bus_ = kDefaultUseBusFactor;
    LOG_WARN("Outside valid use_bus factor range " +
              std::to_string(use_bus_) + ": using default");
  }

  // Willingness to use rail. Make sure this is within range [0, 1].
  use_rail_ = pt.get<float>("use_rail", kDefaultUseRailFactor);
  if (use_rail_ < 0.0f || use_rail_ > 1.0f) {
    use_rail_ = kDefaultUseRailFactor;
    LOG_WARN("Outside valid use_rail factor range " +
              std::to_string(use_rail_) + ": using default");
  }

  // Willingness to make transfers. Make sure this is within range [0, 1].
  use_transfers_ = pt.get<float>("use_transfers", kDefaultUseTransfersFactor);
  if (use_transfers_ < 0.0f || use_transfers_ > 1.0f) {
    use_transfers_ = kDefaultUseTransfersFactor;
    LOG_WARN("Outside valid use_transfers factor range " +
              std::to_string(use_transfers_) + ": using default");
  }

  // Set the factors. The factors above 0.5 start to reduce the weight
  // for this mode while factors below 0.5 start to increase the weight for
  // this mode.
  bus_factor_ = (use_bus_ >= 0.5f) ?
                 1.0f - (use_bus_ - 0.5f) :
                 1.0f + (0.5f - use_bus_) * 5.0f;

  rail_factor_ = (use_rail_ >= 0.5f) ?
                 1.0f - (use_rail_ - 0.5f) :
                 1.0f + (0.5f - use_rail_) * 5.0f;

  transfer_factor_ = (use_transfers_ >= 0.5f) ?
                     1.0f - (use_transfers_ - 0.5f) :
                     1.0f + (0.5f - use_transfers_) * 5.0f;

  transfer_cost_ = pt.get<float>("transfer_cost", kDefaultTransferCost);
  transfer_penalty_ = pt.get<float>("transfer_penalty", kDefaultTransferPenalty);

  std::string stop_action = pt.get("filters.stops.action", "");
  if (stop_action.size()) {
    for (const auto& kv : pt.get_child("filters.stops.ids")) {
      if (stop_action == "none")
        stop_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (stop_action == "only")
        stop_include_onestops_.emplace(kv.second.get_value<std::string>());
    }
  }

  std::string operator_action = pt.get("filters.operators.action", "");
  if (operator_action.size()) {
    for (const auto& kv : pt.get_child("filters.operators.ids")) {
      if (operator_action == "none")
        oper_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (operator_action == "only")
        oper_include_onestops_.emplace(kv.second.get_value<std::string>());
    }
  }

  std::string routes_action = pt.get("filters.routes.action", "");
  if (routes_action.size()) {
    for (const auto& kv : pt.get_child("filters.routes.ids")) {
      if (routes_action == "none")
        route_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (routes_action == "only")
        route_include_onestops_.emplace(kv.second.get_value<std::string>());
    }
  }

  // Normalize so the favored mode has factor == 1
  if (rail_factor_ < bus_factor_) {
    float ratio = bus_factor_ / rail_factor_;
    rail_factor_ = 1.0f;
    bus_factor_ *= ratio;
  } else {
    float ratio = rail_factor_ / bus_factor_;
    bus_factor_ = 1.0f;
    rail_factor_ *= ratio;
  }
}

// Destructor
TransitCost::~TransitCost() {
}

// This method overrides the weight for this mode.  The higher the value
// the more the mode is favored.
float TransitCost::GetModeWeight() {
  return mode_weight_;
}

// This method adds tile_index_pairs to the exclude list based on the
// operator, stop, and route exclude_onestops and include_onestops lists.
// The exclude_onestops and include_onestops lists are set by the user.
void TransitCost::AddToExcludeList(const baldr::GraphTile*& tile) {

  //do we have stop work to do?
  if (stop_exclude_onestops_.size() || stop_include_onestops_.size()) {
    const std::unordered_map<std::string, tile_index_pair> stop_onestops =
        tile->GetStopOneStops();

    //avoid these operators
    if (stop_onestops.size()) {
      for (const auto& e : stop_exclude_onestops_ ) {
        const auto& one_stop = stop_onestops.find(e);
        if (one_stop != stop_onestops.end())
          exclude_stops_.emplace(one_stop->second);
      }

      //exclude all operators but the ones the users wants to use
      if (stop_include_onestops_.size()) {
        for(auto const& onestop: stop_onestops) {
          if (stop_include_onestops_.find(onestop.first) == stop_include_onestops_.end())
            exclude_stops_.emplace(onestop.second);
        }
      }
    }
  }

  //do we have operator work to do?
  if (oper_exclude_onestops_.size() || oper_include_onestops_.size()) {
    const std::unordered_map<std::string, std::list<tile_index_pair>> oper_onestops =
        tile->GetOperatorOneStops();

    //avoid these operators
    if (oper_onestops.size()) {
      for (const auto& e : oper_exclude_onestops_ ) {
        const auto& one_stop = oper_onestops.find(e);
        if (one_stop != oper_onestops.end()) {
          for (const auto& tls : one_stop->second)
            exclude_.emplace(tls);
        }
      }

      //exclude all operators but the ones the users wants to use
      if (oper_include_onestops_.size()) {
        for(auto const& onestop: oper_onestops) {
          if (oper_include_onestops_.find(onestop.first) == oper_include_onestops_.end()) {
            for (const auto& tls : onestop.second)
              exclude_.emplace(tls);
          }
        }
      }
    }
  }

  //do we have route work to do?
  if (route_exclude_onestops_.size() || route_include_onestops_.size()) {

    const std::unordered_map<std::string, std::list<tile_index_pair>> route_onestops =
        tile->GetRouteOneStops();

    //avoid these routes
    if (route_onestops.size()) {
      for (const auto& e : route_exclude_onestops_ ) {
        const auto& one_stop = route_onestops.find(e);
        if (one_stop != route_onestops.end()) {
          for (const auto& tls : one_stop->second)
            exclude_.emplace(tls);
        }
      }

      //exclude all routes but the ones the users wants to use
      if (route_include_onestops_.size()) {
        for(auto const& onestop: route_onestops) {
          if (route_include_onestops_.find(onestop.first) == route_include_onestops_.end()) {
            for (const auto& tls : onestop.second)
              exclude_.emplace(tls);
          }
        }
      }
    }
  }
}

// This method acts like an allowed function; however, it uses the exclude list to
// determine if we should not route on a line.
bool TransitCost::IsExcluded(const baldr::GraphTile*& tile,
                             const baldr::DirectedEdge* edge) {
  return (exclude_.find(tile_index_pair(tile->id().tileid(),edge->lineid())) != exclude_.end());
}

// This method acts like an allowed function; however, it uses the exclude list to
// determine if we should not route through this node.
bool TransitCost::IsExcluded(const baldr::GraphTile*& tile,
                             const baldr::NodeInfo* node) {
  return (exclude_stops_.find(tile_index_pair(tile->id().tileid(),
                                             node->stop_index())) != exclude_stops_.end());
}

// Check if access is allowed on the specified edge.
bool TransitCost::Allowed(const baldr::DirectedEdge* edge,
                          const EdgeLabel& pred,
                          const baldr::GraphTile*& tile,
                          const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  if (exclude_stops_.size()) {
    // may be in another tile, skip if it is as will will check it later.
    if (edge->endnode().tileid() == tile->id().tileid()) {
      if (exclude_stops_.find(tile_index_pair(tile->id().tileid(),
                                           tile->node(edge->endnode())->stop_index())) != exclude_stops_.end())
        return false;
    }
  }

  if (edge->use() == Use::kBus) {
    return (use_bus_ > 0.0f) ? true : false;
  } else if (edge->use() == Use::kRail) {
    return (use_rail_ > 0.0f) ? true : false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TransitCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // This method should not be called since time based routes do not use
  // bidirectional A*
  return false;
}

// Check if access is allowed at the specified node.
bool TransitCost::Allowed(const baldr::NodeInfo* node) const {
  return true;
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost TransitCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const uint32_t density) const {
  LOG_ERROR("Wrong transit edge cost called");
  return { 0.0f, 0.0f };
}

// Get the cost to traverse the specified directed edge using a transit
// departure (schedule based edge traversal). Cost includes
// the time (seconds) to traverse the edge. Only transit cost models override
// this method.
Cost TransitCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::TransitDeparture* departure,
                           const uint32_t curr_time) const {
  // Separate wait time from time on transit
  float wait_time = departure->departure_time() - curr_time;

  // Cost is modulated by mode-based weight factor
  float weight = 1.0f;
  if (edge->use() == Use::kBus) {
    weight *= bus_factor_;
  } else if (edge->use() == Use::kRail) {
    weight *= rail_factor_;
  }
  return { wait_time + (departure->elapsed_time() * weight),
           wait_time + departure->elapsed_time() };
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TransitCost::TransitionCost(const baldr::DirectedEdge* edge,
                                 const baldr::NodeInfo* node,
                                 const EdgeLabel& pred) const {
  if (pred.mode() == TravelMode::kPedestrian) {
    // Apply any mode-based penalties when boarding transit
    // Do we want any time cost to board?
    if (edge->use() == Use::kBus) {
      return { (0.5f + bus_factor_), 0.0f };
    } else if (edge->use() == Use::kRail) {
      return { (0.5f + rail_factor_), 0.0f };
    }
  }
  return { 0.0f, 0.0f };
}

// Returns the transfer cost between 2 transit stops.
Cost TransitCost::TransferCost() const {
  // Defaults...15 seconds for in station transfer and 1 minute otherwise
  return { (transfer_cost_ +  transfer_penalty_) * transfer_factor_,
            transfer_cost_ * 4.0f};
}

// Returns the default transfer cost between 2 transit lines.
Cost TransitCost::DefaultTransferCost() const {
  return { transfer_cost_ +  transfer_penalty_ , transfer_cost_ };
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float TransitCost::AStarCostFactor() const {
  return 0.0f;
}

//  Override unit size since walking costs are higher range of values
uint32_t TransitCost::UnitSize() const {
  return kUnitSize;
}

cost_ptr_t CreateTransitCost(const boost::property_tree::ptree& config) {
  return std::make_shared<TransitCost>(config);
}

}
}
