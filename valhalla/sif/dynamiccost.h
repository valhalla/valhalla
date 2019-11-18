#ifndef VALHALLA_SIF_DYNAMICCOST_H_
#define VALHALLA_SIF_DYNAMICCOST_H_

#include "baldr/accessrestriction.h"
#include <cstdint>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/double_bucket_queue.h> // For kInvalidLabel
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/timedomain.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>

#include <memory>
#include <third_party/rapidjson/include/rapidjson/document.h>
#include <unordered_map>

namespace valhalla {
namespace sif {

const sif::Cost kNoCost(0.0f, 0.0f);

/**
 * A callable element which returns a value between 0 and 1 indicating how
 * desirable the edge is for use as a location. A value of 0 indicates the
 * edge is not usable (no access for the travel mode used by this costing)
 * while 1 indicates the edge is highly preferred. Values in between can be
 * used to rank edges such that desirable edges that might be slightly
 * farther from the location than a less desirable edge can be chosen.
 */
using EdgeFilter = std::function<float(const baldr::DirectedEdge*)>;

/**
 * A callable element which returns true if a node should be
 * filtered out/ not used and false if the node is usable
 */
using NodeFilter = std::function<bool(const baldr::NodeInfo*)>;

// Default unit size (seconds) for cost sorting.
constexpr uint32_t kDefaultUnitSize = 1;

// Maximum penalty allowed. Cannot be too high because sometimes one cannot avoid a particular
// attribute or condition to complete a route.
constexpr float kMaxPenalty = 12.0f * midgard::kSecPerHour; // 12 hours

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * midgard::kSecPerHour; // 6 hours

/**
 * Base class for dynamic edge costing. This class defines the interface for
 * costing methods and includes a few base methods that define default behavior
 * for cases where a derived class does not need to override the method.
 * Derived classes must implement methods to consider if access is allowed,
 * compute a cost to traverse an edge, and a couple other methods required to
 * setup A* heuristics and sorting methods. Derived classes can also define
 * edge transition (intersection/turn) costing. EdgeCost and TransitionCost
 * methods return a Cost structure includes a cost for Dijkstra/A* as well as
 * the elapsed time (seconds) so that time along the path can be estimated
 * (for transit schedule lookups. timed restrictions, and other time dependent
 * logic).
 */
class DynamicCost {
public:
  /**
   * Constructor.
   * @param  options Request options in a pbf
   * @param  mode Travel mode
   */
  DynamicCost(const Options& options, const TravelMode mode);

  virtual ~DynamicCost();

  /**
   * Does the costing method allow multiple passes (with relaxed
   * hierarchy limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const;

  /**
   * Get the pass number.
   * @return  Returns the pass through the algorithm.
   */
  uint32_t pass() const {
    return pass_;
  }

  /**
   * Set the pass number.
   * @param  pass  Pass number (incremental).
   */
  void set_pass(const uint32_t pass) {
    pass_ = pass;
  }

  /**
   * Returns the maximum transfer distance between stops that you are willing
   * to travel for this mode.  It is the max distance you are willing to
   * travel between transfers.
   * @return  max transfer distance multimodal
   */
  virtual uint32_t GetMaxTransferDistanceMM();

  /**
   * This method overrides the factor for this mode.  The lower the value
   * the more the mode is favored.
   * @return  mode factor
   */
  virtual float GetModeFactor();

  /**
   * This method overrides the max_distance with the max_distance_mm per segment
   * distance. An example is a pure walking route may have a max distance of
   * 10000 meters (10km) but for a multi-modal route a lower limit of 5000
   * meters per segment (e.g. from origin to a transit stop or from the last
   * transit stop to the destination).
   */
  virtual void UseMaxMultiModalDistance();

  /**
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  virtual uint32_t access_mode() const = 0;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       bool& time_restricted) const = 0;

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
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::GraphTile*& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              bool& has_time_restrictions) const = 0;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards are present.
   * @param   node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const = 0;

  inline virtual bool ModeSpecificAllowed(const baldr::AccessRestriction&) const {
    return true;
  };
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
                        const uint32_t curr_time) const = 0;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge    Pointer to a directed edge.
   * @param   tile    Pointer to the tile which contains the directed edge for speed lookup
   * @param   seconds Seconds of week for predicted speed or free and constrained speed lookup
   * @return  Returns the cost and time (seconds).
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphTile* tile,
                        const uint32_t seconds) const = 0;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge    Pointer to a directed edge.
   * @param   tile    Pointer to the tile which contains the directed edge for speed lookup
   * @return  Returns the cost and time (seconds).
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge, const baldr::GraphTile* tile) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param   edge  Directed edge (the to edge)
   * @param   node  Node (intersection) where transition occurs.
   * @param   pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  opp_edge  Pointer to the opposing directed edge - this is the
   *                   "from" or predecessor edge in the transition.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor. This is the "to" edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* opp_edge,
                                     const baldr::DirectedEdge* opp_pred_edge) const;

  /**
   * Test if an edge should be restricted due to a complex restriction.
   * @param  edge  Directed edge.
   * @param  pred        Predecessor information.
   * @param  edge_labels List of edge labels.
   * @param  tile        Graph tile (to read restriction if needed).
   * @param  edgeid      Edge Id for the directed edge.
   * @param  forward     Forward search or reverse search.
   * @param  current_time Current time (seconds since epoch). A value of 0
   *                     indicates the route is not time dependent.
   * @param  tz_index    timezone index for the node
   * @return Returns true it there is a complex restriction onto this edge
   *         that matches the mode and the predecessor list for the current
   *         path matches a complex restriction.
   */
  template <typename edge_labels_container_t>
  bool Restricted(const baldr::DirectedEdge* edge,
                  const EdgeLabel& pred,
                  const edge_labels_container_t& edge_labels,
                  const baldr::GraphTile*& tile,
                  const baldr::GraphId& edgeid,
                  const bool forward,
                  const uint64_t current_time = 0,
                  const uint32_t tz_index = 0) const {
    // Lambda to get the next predecessor EdgeLabel (that is not a transition)
    auto next_predecessor = [&edge_labels](const EdgeLabel* label) {
      // Get the next predecessor - make sure it is valid. Continue to get
      // the next predecessor if the edge is a transition edge.
      const EdgeLabel* next_pred =
          (label->predecessor() == baldr::kInvalidLabel) ? label : &edge_labels[label->predecessor()];
      return next_pred;
    };

    // If forward, check if the edge marks the end of a restriction, else check
    // if the edge marks the start of a complex restriction.
    if ((forward && (edge->end_restriction() & access_mode())) ||
        (!forward && (edge->start_restriction() & access_mode()))) {
      // Get complex restrictions. Return false if no restrictions are found
      auto restrictions = tile->GetRestrictions(forward, edgeid, access_mode());
      if (restrictions.size() == 0) {
        return false;
      }

      // Iterate through the restrictions
      const EdgeLabel* first_pred = &pred;
      for (const auto& cr : restrictions) {
        // Walk the via list, move to the next restriction if the via edge
        // Ids do not match the path for this restriction.
        bool match = true;
        const EdgeLabel* next_pred = first_pred;
        if (cr->via_count() > 0) {
          // The via list starts immediately after the structure
          baldr::GraphId* via = reinterpret_cast<baldr::GraphId*>(cr + 1);
          for (uint32_t i = 0; i < cr->via_count(); i++, via++) {
            if (via->value != next_pred->edgeid().value) {
              match = false;
              break;
            }
            next_pred = next_predecessor(next_pred);
          }
        }

        // Check against the start/end of the complex restriction
        if (match && ((forward && next_pred->edgeid() == cr->from_graphid()) ||
                      (!forward && next_pred->edgeid() == cr->to_graphid()))) {

          if (current_time && cr->has_dt()) {
            if (baldr::DateTime::is_restricted(cr->dt_type(), cr->begin_hrs(), cr->begin_mins(),
                                               cr->end_hrs(), cr->end_mins(), cr->dow(),
                                               cr->begin_week(), cr->begin_month(),
                                               cr->begin_day_dow(), cr->end_week(), cr->end_month(),
                                               cr->end_day_dow(), current_time,
                                               baldr::DateTime::get_tz_db().from_index(tz_index))) {
              return true;
            }
            continue;
          }
          // TODO: If a user runs a non-time dependent route, we need to provide Manuever Notes for
          // the timed restriction.
          else if (!current_time && cr->has_dt())
            return false;
          return true; // Otherwise this is a non-timed restriction and it exists all the time.
        }
      }
    }
    return false;
  }

  /**
   * Test if an edge should be restricted due to a date time access restriction.
   * @param  restriction  date and time info for the restriction
   * @param  current_time Current time (seconds since epoch). A value of 0
   *                      indicates the route is not time dependent.
   * @param  tz_index     timezone index for the node
   */
  static bool
  IsRestricted(const uint64_t restriction, const uint64_t current_time, const uint32_t tz_index) {

    baldr::TimeDomain td(restriction);
    return baldr::DateTime::is_restricted(td.type(), td.begin_hrs(), td.begin_mins(), td.end_hrs(),
                                          td.end_mins(), td.dow(), td.begin_week(), td.begin_month(),
                                          td.begin_day_dow(), td.end_week(), td.end_month(),
                                          td.end_day_dow(), current_time,
                                          baldr::DateTime::get_tz_db().from_index(tz_index));
  }

  inline static bool IsRestricted(const uint64_t restriction,
                                  const uint64_t current_time,
                                  const uint32_t tz_index,
                                  baldr::AccessType access_type) {

    if (access_type == baldr::AccessType::kTimedAllowed) {
      return IsRestricted(restriction, current_time, tz_index);
    } else if (access_type == baldr::AccessType::kTimedDenied) {
      return !IsRestricted(restriction, current_time, tz_index);
    }
    return true;
  }

  inline bool EvaluateRestrictions(uint16_t auto_type,
                                   const baldr::DirectedEdge* edge,
                                   const baldr::GraphTile*& tile,
                                   const baldr::GraphId& edgeid,
                                   const uint64_t current_time,
                                   const uint32_t tz_index,
                                   bool& has_time_restrictions) const {
    if (edge->access_restriction()) {
      const std::vector<baldr::AccessRestriction>& restrictions =
          tile->GetAccessRestrictions(edgeid.id(), auto_type);
      for (const auto& restriction : restrictions) {
        // Compare the time to the time-based restrictions
        baldr::AccessType access_type = restriction.type();
        if (access_type == baldr::AccessType::kTimedAllowed ||
            access_type == baldr::AccessType::kTimedDenied) {
          has_time_restrictions = true;
          if (current_time == 0) {
            // No time supplied so ignore time-based restrictions
            // (but mark the edge  (`has_time_restrictions`)
            continue;
          } else {
            // allowed at this range or allowed all the time
            if (DynamicCost::IsRestricted(restriction.value(), current_time, tz_index, access_type)) {
              // If edge really is restricted at this time, we can exit early.
              // If not, we should keep looking
              return false;
            }
          }
        }
        // In case there are additional restriction checks for a particular  mode,
        // check them now
        if (!ModeSpecificAllowed(restriction)) {
          return false;
        }
      }
    }
    return true;
  }

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
   * @return  Returns the cost factor used in the A* heuristic.
   */
  virtual float AStarCostFactor() const = 0;

  /**
   * Get the general unit size that can be considered as equal for sorting
   * purposes. The A* method uses an approximate bucket sort, and this value
   * is used to size the buckets used for sorting. For example, for time
   * based costs one might compute costs in seconds and consider any time
   * within 2 seconds of each other as being equal (for sorting purposes).
   * @return  Returns the unit size for sorting (must be an integer value).
   * Defaults to 1 (second).
   * @return  Returns unit size.
   */
  virtual uint32_t UnitSize() const;

  /**
   * Sets the flag indicating whether destination only edges are allowed.
   * Bidirectional path algorithms can (usually) disable access.
   */
  virtual void set_allow_destination_only(const bool allow);

  /**
   * Set to allow use of transit connections.
   * @param  allow  Flag indicating whether transit connections are allowed.
   */
  virtual void SetAllowTransitConnections(const bool allow);

  /**
   * Set the current travel mode.
   * @param  mode  Travel mode
   */
  void set_travel_mode(const TravelMode mode);

  /**
   * Get the current travel mode.
   * @return  Returns the current travel mode.
   */
  TravelMode travel_mode() const;

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const;

  /**
   * Get the wheelchair required flag.
   * @return  Returns true if wheelchair is required.
   */
  virtual bool wheelchair() const;

  /**
   * Get the bicycle required flag.
   * @return  Returns true if bicycle is required.
   */
  virtual bool bicycle() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method.
   */
  virtual const EdgeFilter GetEdgeFilter() const = 0;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   */
  virtual const NodeFilter GetNodeFilter() const = 0;

  /**
   * Gets the hierarchy limits.
   * @return  Returns the hierarchy limits.
   */
  std::vector<HierarchyLimits>& GetHierarchyLimits();

  /**
   * Relax hierarchy limits.
   */
  void RelaxHierarchyLimits(const float factor, const float expansion_within_factor);

  /**
   * Checks if we should exclude or not.
   */
  virtual void AddToExcludeList(const baldr::GraphTile*& tile);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile, const baldr::DirectedEdge* edge);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const baldr::GraphTile*& tile, const baldr::NodeInfo* node);

  /**
   * Adds a list of edges (GraphIds) to the user specified avoid list.
   * This can be used by test programs - alternatively a list of avoid
   * edges will be passed in the property tree for the costing options
   * of a specified type.
   * @param  avoid_edges  Set of edge Ids to avoid along with the percent along the edge.
   */
  void AddUserAvoidEdges(const std::vector<AvoidEdge>& avoid_edges);

  /**
   * Check if the edge is in the user-specified avoid list.
   * @param  edgeid  Directed edge Id.
   * @return Returns true if the edge Id is in the user avoid edges set,
   *         false otherwise.
   */
  bool IsUserAvoidEdge(const baldr::GraphId& edgeid) const {
    return (user_avoid_edges_.size() != 0 &&
            user_avoid_edges_.find(edgeid) != user_avoid_edges_.end());
  }

  /**
   * Check if the edge is in the user-specified avoid list and should be avoided when used
   * as an origin. In this case the edge is avoided if the avoid percent along is greater than
   * the PathEdge percent along (avoid location is ahead of the origin alongn the edge).
   * @param  edgeid  Directed edge Id.
   * @param  pct_along Percent along the edge of the PathEdge (location).
   * @return Returns true if the edge Id is in the user avoid edges set,
   *         false otherwise.
   */
  bool AvoidAsOriginEdge(const baldr::GraphId& edgeid, const float percent_along) const {
    auto avoid = user_avoid_edges_.find(edgeid);
    return (avoid != user_avoid_edges_.end() && avoid->second >= percent_along);
  }

  /**
   * Check if the edge is in the user-specified avoid list and should be avoided when used
   * as a destinationn. In this case the edge is avoided if the avoid percent along is less than
   * the PathEdge percent along (avoid location is behind the destination along the edge).
   * @param  edgeid  Directed edge Id.
   * @param  pct_along Percent along the edge of the PathEdge (location).
   * @return Returns true if the edge Id is in the user avoid edges set,
   *         false otherwise.
   */
  bool AvoidAsDestinationEdge(const baldr::GraphId& edgeid, const float percent_along) const {
    auto avoid = user_avoid_edges_.find(edgeid);
    return (avoid != user_avoid_edges_.end() && avoid->second <= percent_along);
  }

  /**
   * Get the flow mask used for accessing traffic flow data from the tile
   * @return the flow mask used
   */
  uint8_t flow_mask() const {
    return flow_mask_;
  }

protected:
  // Algorithm pass
  uint32_t pass_;

  // Flag indicating whether transit connections are allowed.
  bool allow_transit_connections_;

  // Allow entrance onto destination only edges. Bidirectional A* can (usually)
  // disable access onto destination only edges for driving routes. Pedestrian
  // and bicycle generally allow access (with small penalties).
  bool allow_destination_only_;

  // Travel mode
  TravelMode travel_mode_;

  // Hierarchy limits.
  std::vector<HierarchyLimits> hierarchy_limits_;

  // User specified edges to avoid with percent along (for avoiding PathEdges of locations)
  std::unordered_map<baldr::GraphId, float> user_avoid_edges_;

  // Weighting to apply to ferry edges
  float ferry_factor_;

  // Transition costs
  sif::Cost country_crossing_cost_;
  sif::Cost gate_cost_;
  sif::Cost toll_booth_cost_;
  sif::Cost ferry_transition_cost_;

  // Penalties that all costing methods support
  float maneuver_penalty_;         // Penalty (seconds) when inconsistent names
  float alley_penalty_;            // Penalty (seconds) to use a alley
  float destination_only_penalty_; // Penalty (seconds) using private road, driveway, or parking aisle

  // A mask which determines which flow data the costing should use from the tile
  uint8_t flow_mask_;

  /**
   * Get the base transition costs (and ferry factor) from the costing options.
   * @param costing_options Protocol buffer of costing options.
   */
  void get_base_costs(const CostingOptions& costing_options) {
    // Cost only (no time) penalties
    alley_penalty_ = costing_options.alley_penalty();
    destination_only_penalty_ = costing_options.destination_only_penalty();
    maneuver_penalty_ = costing_options.maneuver_penalty();

    // Transition costs (both time and cost)
    toll_booth_cost_ = {costing_options.toll_booth_cost() + costing_options.toll_booth_penalty(),
                        costing_options.toll_booth_cost()};
    country_crossing_cost_ = {costing_options.country_crossing_cost() +
                                  costing_options.country_crossing_penalty(),
                              costing_options.country_crossing_cost()};
    gate_cost_ = {costing_options.gate_cost() + costing_options.gate_penalty(),
                  costing_options.gate_cost()};

    // Set the cost (seconds) to enter a ferry (only apply entering since
    // a route must exit a ferry (except artificial test routes ending on
    // a ferry!). Modify ferry edge weighting based on use_ferry factor.
    float penalty;
    float use_ferry = costing_options.use_ferry();
    if (use_ferry < 0.5f) {
      // Penalty goes from max at use_ferry = 0 to 0 at use_ferry = 0.5
      penalty = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - use_ferry * 2.0f));

      // Cost X10 at use_ferry == 0, slopes downwards towards 1.0 at use_ferry = 0.5
      ferry_factor_ = 10.0f - use_ferry * 18.0f;
    } else {
      // Add a ferry weighting factor to influence cost along ferries to make
      // them more favorable if desired rather than driving. No ferry penalty.
      // Half the cost at use_ferry == 1, progress to 1.0 at use_ferry = 0.5
      penalty = 0.0f;
      ferry_factor_ = 1.5f - use_ferry;
    }
    ferry_transition_cost_ = {costing_options.ferry_cost() + penalty, costing_options.ferry_cost()};

    // Set the speed mask to determine which speed data types are allowed
    flow_mask_ = costing_options.flow_mask();
  }

  /**
   * Base transition cost that all costing methods use. Includes costs for
   * country crossing, boarding a ferry, toll booth, gates, entering destination
   * only, alleys, and maneuver penalties. Each costing method can provide different
   * costs for these transitions (via costing options).
   * @param node Node at the intersection where the edge transition occurs.
   * @param edge Directed edge entering.
   * @param pred Predecessor edge information.
   * @param idx  Index used for name consistency.
   * @return Returns the transition cost (cost, elapsed time).
   */
  virtual sif::Cost base_transition_cost(const baldr::NodeInfo* node,
                                         const baldr::DirectedEdge* edge,
                                         const sif::EdgeLabel& pred,
                                         const uint32_t idx) const {
    // Cases with both time and penalty: country crossing, ferry, gate, toll booth
    sif::Cost c;
    if (node->type() == baldr::NodeType::kBorderControl) {
      c += country_crossing_cost_;
    }
    if (node->type() == baldr::NodeType::kGate) {
      c += gate_cost_;
    }
    if (node->type() == baldr::NodeType::kTollBooth || (edge->toll() && !pred.toll())) {
      c += toll_booth_cost_;
    }
    if (edge->use() == baldr::Use::kFerry && pred.use() != baldr::Use::kFerry) {
      c += ferry_transition_cost_;
    }

    // Additional penalties without any time cost
    if (edge->destonly() && !pred.destonly()) {
      c.cost += destination_only_penalty_;
    }
    if (edge->use() == baldr::Use::kAlley && pred.use() != baldr::Use::kAlley) {
      c.cost += alley_penalty_;
    }
    if (!edge->link() && !edge->name_consistency(idx)) {
      c.cost += maneuver_penalty_;
    }
    return c;
  }

  /**
   * Base transition cost that all costing methods use. Includes costs for
   * country crossing, boarding a ferry, toll booth, gates, entering destination
   * only, alleys, and maneuver penalties. Each costing method can provide different
   * costs for these transitions (via costing options).
   * @param node Node at the intersection where the edge transition occurs.
   * @param edge Directed edge entering.
   * @param pred Predecessor edge.
   * @param idx  Index used for name consistency.
   * @return Returns the transition cost (cost, elapsed time).
   */
  virtual sif::Cost base_transition_cost(const baldr::NodeInfo* node,
                                         const baldr::DirectedEdge* edge,
                                         const baldr::DirectedEdge* pred,
                                         const uint32_t idx) const {
    // Cases with both time and penalty: country crossing, ferry, gate, toll booth
    sif::Cost c;
    if (node->type() == baldr::NodeType::kBorderControl) {
      c += country_crossing_cost_;
    }
    if (node->type() == baldr::NodeType::kGate) {
      c += gate_cost_;
    }
    if (node->type() == baldr::NodeType::kTollBooth || (edge->toll() && !pred->toll())) {
      c += toll_booth_cost_;
    }
    if (edge->use() == baldr::Use::kFerry && pred->use() != baldr::Use::kFerry) {
      c += ferry_transition_cost_;
    }

    // Additional penalties without any time cost
    if (edge->destonly() && !pred->destonly()) {
      c.cost += destination_only_penalty_;
    }
    if (edge->use() == baldr::Use::kAlley && pred->use() != baldr::Use::kAlley) {
      c.cost += alley_penalty_;
    }
    if (!edge->link() && !edge->name_consistency(idx)) {
      c.cost += maneuver_penalty_;
    }
    return c;
  }

  /**
   * Convenience method to get the transition factor.
   * @param has_traffic Does the intersection have traffic information? Currently
   *                    is based on the exiting edge. TODO - determine if we also
   *                    need this info on the predecessor.
   * @param density_factor Density factor.
   */
  float TransitionFactor(const bool has_traffic, const float density) const {
    return has_traffic ? kTrafficTransitionFactor : density;
  }
};

using cost_ptr_t = std::shared_ptr<DynamicCost>;

/**
 * Parses the cost options from json and stores values in pbf.
 * @param object The json request represented as a DOM tree.
 * @param pbf_costing_options A mutable protocol buffer where the parsed json values will be stored.
 */
void ParseCostOptions(const rapidjson::Value& obj, CostingOptions* pbf_costing_options);

} // namespace sif

} // namespace valhalla

#endif // VALHALLA_SIF_DYNAMICCOST_H_
