#include "sif/nocost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"
#include "sif/dynamiccost.h"

#ifdef INLINE_TEST
#include "test.h"
#include "worker.h"
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

/**
 * Derived class providing dynamic edge costing for "direct" no-cost routes.
 *
 * Intended for use-cases where we dont care about mode of travel, this costing allows all edges.
 */
class NoCost : public DynamicCost {
public:
  /**
   * Construct costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  NoCost(const Costing& costing) : DynamicCost(costing, TravelMode::kDrive, kAllAccess) {
  }

  virtual ~NoCost() {
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  is_dest        Is a directed edge the destination?
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool,
                       const EdgeLabel&,
                       const graph_tile_ptr&,
                       const baldr::GraphId&,
                       const uint64_t,
                       const uint32_t,
                       uint8_t&) const override {
    return !edge->is_shortcut();
  }

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge*,
                              const EdgeLabel&,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr&,
                              const baldr::GraphId&,
                              const uint64_t,
                              const uint32_t,
                              uint8_t&) const override {
    return !opp_edge->is_shortcut();
  }

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  bool Allowed(const baldr::NodeInfo*) const override {
    return true;
  }

  /**
   * Checks if access is allowed for the provided edge. The access check based on mode
   * of travel and the access modes allowed on the edge.
   * @param   edge  Pointer to edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool IsAccessible(const baldr::DirectedEdge*) const override {
    return true;
  }

  bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
    return false;
  }

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("NoCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   tile      Graph tile.
   * @param   time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr&,
                        const baldr::TimeInfo&,
                        uint8_t&) const override {
    return {static_cast<float>(edge->length()), static_cast<float>(edge->length())};
  }

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge*,
                              const baldr::NodeInfo*,
                              const EdgeLabel&) const override {
    return {};
  }

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn  Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t,
                                     const baldr::NodeInfo*,
                                     const baldr::DirectedEdge*,
                                     const baldr::DirectedEdge*,
                                     const bool,
                                     const InternalTurn) const override {
    return {};
  }

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override {
    return 1.f;
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by automobile.
   */
  bool Allowed(const baldr::DirectedEdge* edge, const graph_tile_ptr&, uint16_t) const override {
    return !(edge->is_shortcut() || edge->IsTransitLine());
  }
};

void ParseNoCostOptions(const rapidjson::Document&, const std::string&, Costing* c) {
  // this is probably not needed but its part of the contract for costing..
  c->set_type(Costing::none_);
  c->set_name(Costing_Enum_Name(c->type()));
}

cost_ptr_t CreateNoCost(const Costing& costing_options) {
  return std::make_shared<NoCost>(costing_options);
}

} // namespace sif
} // namespace valhalla
