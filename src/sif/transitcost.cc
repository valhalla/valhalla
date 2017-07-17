#include "sif/transitcost.h"

#include "baldr/accessrestriction.h"
#include "midgard/constants.h"
#include "midgard/logging.h"

#ifdef INLINE_TEST
#include "test/test.h"
#include <random>
#include <boost/property_tree/json_parser.hpp>
#endif

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr uint32_t kUnitSize = 1;

constexpr float kModeFactor             = 1.0f; // Favor this mode?
constexpr float kDefaultTransferCost    = 15.0f;
constexpr float kDefaultTransferPenalty = 300.0f;

// User propensity to use buses. Range of values from 0 (avoid buses) to
// 1 (totally comfortable riding on buses).
constexpr float kDefaultUseBus = 0.3f;

// User propensity to use rail. Range of values from 0 (avoid rail) to
// 1 (totally comfortable riding on rail).
constexpr float kDefaultUseRail = 0.6f;

// User propensity to use/allow transfers. Range of values from 0
// (avoid transfers) to 1 (totally comfortable with transfers).
constexpr float kDefaultUseTransfers = 0.3f;

Cost kImpossibleCost = { 10000000.0f, 10000000.0f };

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

// Maximum amount of seconds that will be allowed to be passed in to influence paths
// This can't be too high because sometimes a certain kind of path is required to be taken
constexpr float kMaxSeconds = 12.0f * kSecPerHour; // 12 hours

// Valid ranges and defaults
constexpr ranged_default_t<float> kModeFactorRange{kMinFactor, kModeFactor, kMaxFactor};
constexpr ranged_default_t<float> kUseBusRange{0, kDefaultUseBus, 1.0f};
constexpr ranged_default_t<float> kUseRailRange{0, kDefaultUseRail, 1.0f};
constexpr ranged_default_t<float> kUseTransfersRange{0, kDefaultUseTransfers, 1.0f};
constexpr ranged_default_t<float> kTransferCostRange{0, kDefaultTransferCost, kMaxSeconds};
constexpr ranged_default_t<float> kTransferPenaltyRange{0, kDefaultTransferPenalty, kMaxSeconds};

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
   * Get the wheelchair required flag.
   * @return  Returns true if wheelchair is required.
   */
  bool wheelchair() const;

  /**
   * Get the bicycle required flag.
   * @return  Returns true if bicycle is required.
   */
  bool bicycle() const;

  /**
   * This method overrides the factor for this mode.  The higher the value
   * the more the mode is favored.
   */
  virtual float GetModeFactor();

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  uint32_t access_mode() const;

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
   * @param  tile           Tile for the opposing edge (for looking
   *                        up restrictions).
   * @param  opp_edgeid     Opposing edge Id
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& opp_edgeid) const;

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
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge) const;

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
      if (edge->IsTransition() || edge->is_shortcut() ||
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

 public:
  // Are wheelchair or bicycle required
  bool wheelchair_;
  bool bicycle_;

  // This is the factor for this mode.  The higher the value the more the
  // mode is favored.
  float mode_factor_;

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

  mode_factor_ = kModeFactorRange(
    pt.get<float>("mode_factor", kModeFactor)
  );

  wheelchair_ = pt.get<bool>("wheelchair", false);
  bicycle_ = pt.get<bool>("bicycle", false);

  // Willingness to use buses. Make sure this is within range [0, 1]
  // Otherwise it will default
  use_bus_ = kUseBusRange(
    pt.get<float>("use_bus", kDefaultUseBus)
  );

  // Willingness to use rail. Make sure this is within range [0, 1].
  // Otherwise it will default
  use_rail_ = kUseRailRange(
    pt.get<float>("use_rail", kDefaultUseRail)
  );

  // Willingness to make transfers. Make sure this is within range [0, 1].
  // Otherwise it will default
  use_transfers_ = kUseTransfersRange(
    pt.get<float>("use_transfers", kDefaultUseTransfers)
  );

  // Set the factors. The factors above 0.5 start to reduce the weight
  // for this mode while factors below 0.5 start to increase the weight for
  // this mode.
  bus_factor_ = (use_bus_ >= 0.5f) ?
                 1.5f - use_bus_ :
                 5.0f - use_bus_ * 8.0f;

  rail_factor_ = (use_rail_ >= 0.5f) ?
                 1.5f - use_rail_ :
                 5.0f - use_rail_ * 8.0f;

  transfer_factor_ = (use_transfers_ >= 0.5f) ?
                     1.5f - use_transfers_ :
                     5.0f - use_transfers_ * 8.0f;

  transfer_cost_ = kTransferCostRange(
    pt.get<float>("transfer_cost", kDefaultTransferCost)
  );
  transfer_penalty_ = kTransferPenaltyRange(
    pt.get<float>("transfer_penalty", kDefaultTransferPenalty)
  );

  std::string stop_action = pt.get("filters.stops.action", "");
  if (stop_action.size()) {
    for (const auto& kv : pt.get_child("filters.stops.ids")) {
      if (stop_action == "exclude")
        stop_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (stop_action == "include")
        stop_include_onestops_.emplace(kv.second.get_value<std::string>());
    }
  }

  std::string operator_action = pt.get("filters.operators.action", "");
  if (operator_action.size()) {
    for (const auto& kv : pt.get_child("filters.operators.ids")) {
      if (operator_action == "exclude")
        oper_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (operator_action == "include")
        oper_include_onestops_.emplace(kv.second.get_value<std::string>());
    }
  }

  std::string routes_action = pt.get("filters.routes.action", "");
  if (routes_action.size()) {
    for (const auto& kv : pt.get_child("filters.routes.ids")) {
      if (routes_action == "exclude")
        route_exclude_onestops_.emplace(kv.second.get_value<std::string>());
      else if (routes_action == "include")
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

// Get the wheelchair required flag.
bool TransitCost::wheelchair() const {
  return wheelchair_;
}

// Get the bicycle required flag.
bool TransitCost::bicycle() const {
  return bicycle_;
}

// This method overrides the factor for this mode.  The higher the value
// the more the mode is favored.
float TransitCost::GetModeFactor() {
  return mode_factor_;
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

// Get the access mode used by this costing method.
uint32_t TransitCost::access_mode() const {
  return 0;
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
               const baldr::GraphId& opp_edgeid) const {
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
Cost TransitCost::EdgeCost(const baldr::DirectedEdge* edge) const {
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

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

TransitCost* make_transitcost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({")" << property << R"(":)" << testVal << "}";
  boost::property_tree::ptree costing_ptree;
  boost::property_tree::read_json(ss, costing_ptree);
  return new TransitCost(costing_ptree);
}

std::uniform_real_distribution<float>* make_distributor_from_range (const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

void testTransitCostParams() {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::default_random_engine generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TransitCost> ctorTester;

  // mode_factor_
  distributor.reset(make_distributor_from_range(kModeFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("mode_factor", (*distributor)(generator)));
    if (ctorTester->mode_factor_ < kModeFactorRange.min ||
        ctorTester->mode_factor_ > kModeFactorRange.max) {
      throw std::runtime_error ("mode_factor_ is not within it's range");
    }
  }

  // use_bus_
  distributor.reset(make_distributor_from_range(kUseBusRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("use_bus", (*distributor)(generator)));
    if (ctorTester->use_bus_ < kUseBusRange.min ||
        ctorTester->use_bus_ > kUseBusRange.max) {
      throw std::runtime_error ("use_bus_ is not within it's range");
    }
  }

  // use_rail_
  distributor.reset(make_distributor_from_range(kUseRailRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("use_rail", (*distributor)(generator)));
    if (ctorTester->use_rail_ < kUseRailRange.min ||
        ctorTester->use_rail_ > kUseRailRange.max) {
      throw std::runtime_error ("use_rail_ is not within it's range");
    }
  }

  // use_transfers_
  distributor.reset(make_distributor_from_range(kUseTransfersRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("use_transfers", (*distributor)(generator)));
    if (ctorTester->use_transfers_ < kUseTransfersRange.min ||
        ctorTester->use_transfers_ > kUseTransfersRange.max) {
      throw std::runtime_error ("use_transfers_ is not within it's range");
    }
  }

  // transfer_cost_
  distributor.reset(make_distributor_from_range(kTransferCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("transfer_cost", (*distributor)(generator)));
    if (ctorTester->transfer_cost_ < kTransferCostRange.min ||
        ctorTester->transfer_cost_ > kTransferCostRange.max) {
      throw std::runtime_error ("transfer_cost_ is not within it's range");
    }
  }

  // transfer_penalty_
  distributor.reset(make_distributor_from_range(kTransferPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_transitcost_from_json("transfer_penalty", (*distributor)(generator)));
    if (ctorTester->transfer_penalty_ < kTransferPenaltyRange.min ||
        ctorTester->transfer_penalty_ > kTransferPenaltyRange.max) {
      throw std::runtime_error ("transfer_penalty_ is not within it's range");
    }
  }
}
}

int main() {
  test::suite suite("costing");

  suite.test(TEST_CASE(testTransitCostParams));

  return suite.tear_down();
}

#endif

