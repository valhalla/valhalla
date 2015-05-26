#include "sif/transitcost.h"

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr uint32_t kUnitSize = 1;

// Default neither favors nor avoids buses vs rail/subway
constexpr bool  kDefaultAllowBus    = true;
constexpr bool  kDefaultAllowRail   = true;
constexpr float kDefaultBusFactor   = 1.0f;
constexpr float kDefaultBusPenalty  = 0.0f;
constexpr float kDefaultRailFactor  = 1.0f;
constexpr float kDefaultRailPenalty = 0.0f;
constexpr float kDefaultTransferCost = 60.0f;
constexpr float kDefaultTransferPenalty = 120.0f;  // 2 minute default

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
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge  Pointer to a directed edge.
   * @param  pred  Predecessor edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const;

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
   * @param  transfer  Pointer to transit transfer record.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost TransferCost(const baldr::TransitTransfer* transfer) const;

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
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    // Disallow transit edges as start/end of a route leg
    return [](const baldr::DirectedEdge* edge){
      return false;
    };
  }

 protected:
  bool allow_bus_;
  bool allow_rail_;
  float bus_factor_;
  float bus_penalty_;
  float rail_factor_;
  float rail_penalty_;
  float transfer_cost_;     // Transfer cost when no transfer record exists
  float transfer_penalty_;  // Transfer penalty
};

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
TransitCost::TransitCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kPublicTransit) {
  allow_bus_    = pt.get<bool>("allow_bus",  kDefaultAllowBus);
  allow_rail_   = pt.get<bool>("allow_rail", kDefaultAllowRail);
  bus_factor_   = pt.get<float>("bus_factor", kDefaultBusFactor);
  bus_penalty_  = pt.get<float>("bus_penalty", kDefaultBusPenalty);
  rail_factor_  = pt.get<float>("rail_factor", kDefaultRailFactor);
  rail_penalty_ = pt.get<float>("rail_penalty", kDefaultRailPenalty);
  transfer_cost_ = pt.get<float>("transfer_cost", kDefaultTransferCost);
  transfer_penalty_ = pt.get<float>("transfer_penalty", kDefaultTransferPenalty);
}

// Destructor
TransitCost::~TransitCost() {
}

// Check if access is allowed on the specified edge.
bool TransitCost::Allowed(const baldr::DirectedEdge* edge,
                          const EdgeLabel& pred) const {
  if (edge->use() == Use::kBus) {
    return allow_bus_;
  } else if (edge->use() == Use::kRail) {
    return allow_rail_;
  }
  return true;
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
  // wait time + time on transit until arrival at next stop
  float elapsedtime = static_cast<float>(departure->departure_time() -
                        curr_time + departure->elapsed_time());

  // TODO - temporary
  if (elapsedtime < 0.0f) {
    LOG_ERROR("Negative elapsed time!");
  }

  // Cost is modulated by mode-based weight factor
  float weight = 1.0f;
  if (edge->use() == Use::kBus) {
    weight = bus_factor_;
  } else if (edge->use() == Use::kRail) {
    weight = rail_factor_;
  }
  return { elapsedtime * weight, elapsedtime };
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TransitCost::TransitionCost(const baldr::DirectedEdge* edge,
                                 const baldr::NodeInfo* node,
                                 const EdgeLabel& pred) const {
  if (pred.mode() == TravelMode::kPedestrian) {
    // Apply any mode-based penalties when boarding transit
    // Do we want any time cost to board?
    if (edge->use() == Use::kBus) {
      return { bus_penalty_, 0.0f };
    } else if (edge->use() == Use::kRail) {
      return { rail_penalty_, 0.0f };
    }
  }
  return { 0.0f, 0.0f };
}

// Returns the transfer cost between 2 transit stops.
Cost TransitCost::TransferCost(const TransitTransfer* transfer) const {
  if (transfer == nullptr) {
    // No transfer record exists - use defaults
    return { transfer_cost_, transfer_cost_ +  transfer_penalty_ };
  }
LOG_INFO("Transfer found");
LOG_INFO("Transfer type = " + std::to_string(static_cast<uint32_t>(transfer->type())));
  switch (transfer->type()) {
  case TransferType::kRecommended:
    return { 15.0f, 15.0f + transfer_penalty_};
  case TransferType::kTimed:
    return { 15.0f, 15.0f + transfer_penalty_};
  case TransferType::kMinTime:
    return { static_cast<float>(transfer->mintime()),
             static_cast<float>(transfer->mintime()) + transfer_penalty_};
  case TransferType::kNotPossible:
    return kImpossibleCost;
  }
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
