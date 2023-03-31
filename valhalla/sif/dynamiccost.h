#ifndef VALHALLA_SIF_DYNAMICCOST_H_
#define VALHALLA_SIF_DYNAMICCOST_H_

#include <cstdint>
#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/double_bucket_queue.h> // For kInvalidLabel
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/nodeinfo.h>
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/baldr/time_info.h>
#include <valhalla/baldr/timedomain.h>
#include <valhalla/baldr/transitdeparture.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/hierarchylimits.h>
#include <valhalla/thor/edgestatus.h>

#include <memory>
#include <rapidjson/document.h>
#include <unordered_map>

// macros aren't great but writing these out for every option is an abomination worse than this macro

/**
 * this macro takes a ranged_default_t and uses it to make sure user provided values (in json or in
 * pbf) are in range before setting it on the costing options
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param range            ranged_default_t object which will check any provided values are in range
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the jsonn
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_RANGED_DEFAULT(costing_options, range, json, json_key, option_name)                 \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        range(rapidjson::get<decltype(range.def)>(json, json_key,                                    \
                                                  costing_options->has_##option_name##_case()        \
                                                      ? costing_options->option_name()               \
                                                      : range.def)));                                \
  }

/**
 * same as above, but for costing options without pbf's awful oneof
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param range            ranged_default_t object which will check any provided values are in range
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the jsonn
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_RANGED_DEFAULT_V2(costing_options, range, json, json_key, option_name)              \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        range(rapidjson::get<decltype(range.def)>(json, json_key,                                    \
                                                  costing_options->option_name()                     \
                                                      ? costing_options->option_name()               \
                                                      : range.def)));                                \
  }

/**
 * this macro takes a default value and uses it when no user provided values exist (in json or in pbf)
 * to set the option on the costing options object
 *
 * @param costing_options  pointer to protobuf costing options object
 * @param def              the default value which is used when neither json nor pbf is provided
 * @param json             rapidjson value object which should contain user provided costing options
 * @param json_key         the json key to use to pull a user provided value out of the json
 * @param option_name      the name of the option will be set on the costing options object
 */

#define JSON_PBF_DEFAULT(costing_options, def, json, json_key, option_name)                          \
  {                                                                                                  \
    costing_options->set_##option_name(                                                              \
        rapidjson::get<std::remove_cv<std::remove_reference<decltype(def)>::type>::                  \
                           type>(json, json_key,                                                     \
                                 costing_options->has_##option_name##_case()                         \
                                     ? costing_options->option_name()                                \
                                     : def));                                                        \
  }

using namespace valhalla::midgard;

namespace valhalla {
namespace sif {

const sif::Cost kNoCost(0.0f, 0.0f);

// Default unit size (seconds) for cost sorting.
constexpr uint32_t kDefaultUnitSize = 1;

// Maximum penalty allowed. Cannot be too high because sometimes one cannot avoid a particular
// attribute or condition to complete a route.
constexpr float kMaxPenalty = 12.0f * midgard::kSecPerHour; // 12 hours

// Maximum ferry penalty (when use_ferry == 0 or use_rail_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 6.0f * midgard::kSecPerHour; // 6 hours

// Default uturn costs
constexpr float kTCUnfavorablePencilPointUturn = 15.f;
constexpr float kTCUnfavorableUturn = 600.f;

// Maximum highway avoidance bias (modulates the highway factors based on road class)
constexpr float kMaxHighwayBiasFactor = 8.0f;

/**
 * Mask values used in the allowed function by loki::reach to control how conservative
 * the decision should be. By default allowed methods will not disallow start/end/simple
 * restrictions/shortcuts and closures are determined by the costing configuration
 */
constexpr uint16_t kDisallowNone = 0x0;
constexpr uint16_t kDisallowStartRestriction = 0x1;
constexpr uint16_t kDisallowEndRestriction = 0x2;
constexpr uint16_t kDisallowSimpleRestriction = 0x4;
constexpr uint16_t kDisallowClosure = 0x8;
constexpr uint16_t kDisallowShortcut = 0x10;

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
   * @param  access_mask Access mask
   * @param  penalize_uturns Should we penalize uturns?
   */
  DynamicCost(const Costing& options,
              const TravelMode mode,
              uint32_t access_mask,
              bool penalize_uturns = false);

  virtual ~DynamicCost();

  DynamicCost(const DynamicCost&) = delete;
  DynamicCost& operator=(const DynamicCost&) = delete;

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
   * Get the access mode used by this costing method.
   * @return  Returns access mode.
   */
  virtual uint32_t access_mode() const {
    return access_mask_;
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
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const = 0;

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
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx) const = 0;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards are present.
   * @param   node  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  inline virtual bool Allowed(const baldr::NodeInfo* node) const {
    return ((node->access() & access_mask_) || ignore_access_) &&
           !(exclude_cash_only_tolls_ && node->cash_only_toll());
  }

  /**
   * Used for determine the viability of a candidate edge as well as a conservative reachability
   * The notable difference to the full featured allowed method is this methods lack of info
   * about the currently tracked path (hence why its conservative)
   *
   * This method is to be used by loki::search and loki::reach and the base class implementation is
   * used to do basically accessibility and adherence to the disallow mask
   *
   * @param edge           the edge that should or shouldnt be allowed
   * @param tile           the tile which contains the edge (for traffic lookup)
   * @param disallow_mask  a mask that controls additional properties that should disallow the edge
   * @return true if the edge is allowed to be used (either as a candidate or a reach traversal)
   */
  inline virtual bool Allowed(const baldr::DirectedEdge* edge,
                              const graph_tile_ptr&,
                              uint16_t disallow_mask = kDisallowNone) const {
    auto access_mask = (ignore_access_ ? baldr::kAllAccess : access_mask_);
    bool accessible = (edge->forwardaccess() & access_mask) ||
                      (ignore_oneways_ && (edge->reverseaccess() & access_mask));
    bool assumed_restricted =
        ((disallow_mask & kDisallowStartRestriction) && edge->start_restriction()) ||
        ((disallow_mask & kDisallowEndRestriction) && edge->end_restriction()) ||
        ((disallow_mask & kDisallowSimpleRestriction) && edge->restrictions()) ||
        ((disallow_mask & kDisallowShortcut) && edge->is_shortcut());
    return accessible && !assumed_restricted && (edge->use() != baldr::Use::kConstruction);
  }

  /**
   * Checks if access is allowed for the provided edge. The access check based on mode
   * of travel and the access modes allowed on the edge.
   * @param   edge  Pointer to edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  inline virtual bool IsAccessible(const baldr::DirectedEdge* edge) const {
    // you can go on it if:
    // you have forward access for the mode you care about
    // you dont care about what mode has access so long as its forward
    // you dont care about the direction the mode has access to
    return ((edge->forwardaccess() & access_mask_) ||
            (ignore_access_ && (edge->forwardaccess() & baldr::kAllAccess)) ||
            (ignore_oneways_ && (edge->reverseaccess() & access_mask_))) &&
           (edge->use() != baldr::Use::kConstruction);
  }

  inline virtual bool ModeSpecificAllowed(const baldr::AccessRestriction&) const {
    return true;
  }

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
   * @param  time_info Time info about edge passing, uses second_of_week for historical speed lookup
   * and seconds_from_now for live-traffic smoothing.
   * @param   flow_sources  Which speed sources were used in this speed calculation.
   * @return  Returns the cost and time (seconds).
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const = 0;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge    Pointer to a directed edge.
   * @param   tile    Pointer to the tile which contains the directed edge for speed lookup
   * @return  Returns the cost and time (seconds).
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge, const graph_tile_ptr& tile) const;

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
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn Did we make a uturn on a short internal edge?
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* opp_edge,
                                     const baldr::DirectedEdge* opp_pred_edge,
                                     const bool has_measured_speed = false,
                                     const InternalTurn internal_turn = InternalTurn::kNoTurn) const;

  /**
   * Test if an edge should be restricted due to a complex restriction.
   * @param  edge  Directed edge.
   * @param  pred        Predecessor information.
   * @param  edge_labels List of edge labels.
   * @param  edge_labels List of edge labels in other direction
   *                     (e.g. bidirectional when connecting the two trees).
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
                  const graph_tile_ptr& tile,
                  const baldr::GraphId& edgeid,
                  const bool forward,
                  thor::EdgeStatus* edgestatus = nullptr,
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
    auto reset_edge_status =
        [&edgestatus](const std::vector<baldr::GraphId>& edge_ids_in_complex_restriction) {
          // A complex restriction spans multiple edges, e.g. from A to C via B.
          //
          // At the point of triggering a complex restriction, all edges leading up to C
          // hav already been evaluated. I.e. B is now marked as kPermanent since
          // we didn't know at the time of B's evaluation that A to B would eventually
          // form a restricted path
          //
          // So, in order to still allow new paths involving B, e.g. D to B to C,
          // we need to go back and revert the permanent status of A and B.
          //
          // We mark it as kUnreachedOrReset so that in effect it is no longer visible. (It can't
          // be removed since that invalidates subsequent indices and setting it to temporary
          // means it'll still do a comparison to existing sort cost and fail later).
          //
          // If we do find a complex restriction has triggered, we must walk back
          // and reset the EdgeStatus of previous edges in the restriction that were
          // already marked as kPermanent.
          if (edgestatus != nullptr) {
            auto first = edge_ids_in_complex_restriction.cbegin();
            auto last = edge_ids_in_complex_restriction.cend();

            // Nothing to do if the restriction has no vias
            if (first == last) {
              return;
            }
            // Reset all but the last edge since there is
            // no point in possibly expanding from A a second time and could lead
            // to infinite loops
            --last;
            std::for_each(first, last, [&edgestatus](baldr::GraphId edge_id) {
              edgestatus->Update(edge_id, thor::EdgeSet::kUnreachedOrReset);
            });
          }
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
        if (cr->type() == baldr::RestrictionType::kNoProbable ||
            cr->type() == baldr::RestrictionType::kOnlyProbable) {
          // A complex restriction can not have a 0 probability set.  range is 1 to 100
          // restriction_probability_= 0 means ignore probable restrictions
          if (restriction_probability_ == 0 || restriction_probability_ > cr->probability()) {
            continue;
          }
        }

        // Walk the via list, move to the next restriction if the via edge
        // Ids do not match the path for this restriction.
        bool match = true;
        const EdgeLabel* next_pred = first_pred;
        // Remember the edge_ids in restriction for later reset
        std::vector<baldr::GraphId> edge_ids_in_complex_restriction;
        edge_ids_in_complex_restriction.reserve(10);

        cr->WalkVias([&match, &next_pred, next_predecessor,
                      &edge_ids_in_complex_restriction](const baldr::GraphId* via) {
          if (via->value != next_pred->edgeid().value) {
            // Pred diverged from restriction, exit early
            match = false;
            return baldr::WalkingVia::StopWalking;
          } else {
            edge_ids_in_complex_restriction.push_back(next_pred->edgeid());
            // Move to the next predecessor and keep walking restriction
            next_pred = next_predecessor(next_pred);
            return baldr::WalkingVia::KeepWalking;
          }
        });
        // Don't forget the last one
        edge_ids_in_complex_restriction.push_back(next_pred->edgeid());

        // Check against the start/end of the complex restriction
        if (match && ((forward && next_pred->edgeid() == cr->from_graphid()) ||
                      (!forward && next_pred->edgeid() == cr->to_graphid()))) {

          if (current_time && cr->has_dt()) {
            // TODO Possibly a bug here. Shouldn't both kTimedDenied and kTimedAllowed
            //      be handled here? As is done in IsRestricted
            if (baldr::DateTime::is_conditional_active(cr->dt_type(), cr->begin_hrs(),
                                                       cr->begin_mins(), cr->end_hrs(),
                                                       cr->end_mins(), cr->dow(), cr->begin_week(),
                                                       cr->begin_month(), cr->begin_day_dow(),
                                                       cr->end_week(), cr->end_month(),
                                                       cr->end_day_dow(), current_time,
                                                       baldr::DateTime::get_tz_db().from_index(
                                                           tz_index))) {
              // We triggered a complex restriction, so make sure we reset edge-status' for
              // earlier edges in restriction that were already marked as permanent
              reset_edge_status(edge_ids_in_complex_restriction);
              return true;
            }
            continue;
          }
          // TODO: If a user runs a non-time dependent route, we need to provide Manuever Notes for
          // the timed restriction.
          else if (!current_time && cr->has_dt()) {
            return false;
          } else {
            // We triggered a complex restriction, so make sure we reset edge-status' for
            // earlier edges in restriction that were already marked as permanent
            reset_edge_status(edge_ids_in_complex_restriction);
            return true; // Otherwise this is a non-timed restriction and it exists all the time.
          }
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
  static bool IsConditionalActive(const uint64_t restriction,
                                  const uint64_t current_time,
                                  const uint32_t tz_index) {

    baldr::TimeDomain td(restriction);
    return baldr::DateTime::is_conditional_active(td.type(), td.begin_hrs(), td.begin_mins(),
                                                  td.end_hrs(), td.end_mins(), td.dow(),
                                                  td.begin_week(), td.begin_month(),
                                                  td.begin_day_dow(), td.end_week(), td.end_month(),
                                                  td.end_day_dow(), current_time,
                                                  baldr::DateTime::get_tz_db().from_index(tz_index));
  }

  inline bool EvaluateRestrictions(uint32_t access_mode,
                                   const baldr::DirectedEdge* edge,
                                   const bool is_dest,
                                   const graph_tile_ptr& tile,
                                   const baldr::GraphId& edgeid,
                                   const uint64_t current_time,
                                   const uint32_t tz_index,
                                   uint8_t& restriction_idx) const {
    if (ignore_restrictions_ || !(edge->access_restriction() & access_mode))
      return true;

    const std::vector<baldr::AccessRestriction>& restrictions =
        tile->GetAccessRestrictions(edgeid.id(), access_mode);

    bool time_allowed = false;

    for (size_t i = 0; i < restrictions.size(); ++i) {
      const auto& restriction = restrictions[i];
      // Compare the time to the time-based restrictions
      baldr::AccessType access_type = restriction.type();
      if (access_type == baldr::AccessType::kTimedAllowed ||
          access_type == baldr::AccessType::kTimedDenied ||
          access_type == baldr::AccessType::kDestinationAllowed) {
        // TODO: if(i > baldr::kInvalidRestriction) LOG_ERROR("restriction index overflow");
        restriction_idx = static_cast<uint8_t>(i);

        if (access_type == baldr::AccessType::kTimedAllowed)
          time_allowed = true;

        if (current_time == 0) {
          // No time supplied so ignore time-based restrictions
          // (but mark the edge  (`has_time_restrictions`)
          continue;
        } else {
          // is in range?
          if (IsConditionalActive(restriction.value(), current_time, tz_index)) {
            // If edge really is restricted at this time, we can exit early.
            // If not, we should keep looking

            // We are in range at the time we are allowed at this edge
            if (access_type == baldr::AccessType::kTimedAllowed)
              return true;
            else if (access_type == baldr::AccessType::kDestinationAllowed)
              return allow_conditional_destination_ || is_dest;
            else
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

    // if we have time allowed restrictions then these restrictions are
    // the only time we can route here.  Meaning all other time is restricted.
    // We looped over all the time allowed restrictions and we were never in range.
    return !time_allowed || (current_time == 0);
  }

  /**
   * Returns the turn type from the predecessor edge.
   * Defaults to InternalTurn::kNoTurn. Costing models that wish to penalize
   * short internal turns in the Transition Cost functions must set penalize_uturns to true in the
   * DynamicCost constructor
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  edge  Directed edge (the to edge)
   * @param  opp_pred_edge Optional.  Opposing predecessor Directed edge (only used for the reverse
   * search)
   * @return  Returns the InternalTurn type
   */
  inline InternalTurn TurnType(const uint32_t idx,
                               const baldr::NodeInfo* node,
                               const baldr::DirectedEdge* edge,
                               const baldr::DirectedEdge* opp_pred_edge = nullptr) const {
    if (!penalize_uturns_ || !edge->internal())
      return InternalTurn::kNoTurn;
    baldr::Turn::Type turntype = opp_pred_edge ? opp_pred_edge->turntype(idx) : edge->turntype(idx);
    if (node->drive_on_right()) {
      // did we make a left onto a small internal edge?
      if (edge->length() <= kShortInternalLength &&
          (turntype == baldr::Turn::Type::kSharpLeft || turntype == baldr::Turn::Type::kLeft))
        return InternalTurn::kLeftTurn;
      // did we make a right onto a small internal edge?
    } else if (edge->length() <= kShortInternalLength &&
               (turntype == baldr::Turn::Type::kSharpRight || turntype == baldr::Turn::Type::kRight))
      return InternalTurn::kRightTurn;
    return InternalTurn::kNoTurn;
  }

  /**
   * Adds a penalty to 3 types of uturns.  1) uturn on a short, internal edge 2) uturn at a node
   * 3) pencil point uturn. Note that motor_scooter and motorcycle costing models do not penalize
   * uturns on a short, internal edge; hence, the boolean penalize_internal_uturns.
   * @param  idx          Directed edge local index
   * @param  node         Node (intersection) where transition occurs.
   * @param  edge         Directed edge (the to edge)
   * @param  has_reverse  Did we perform a reverse?
   * @param  has_left     Did we make a left (left or sharp left)
   * @param  has_right    Did we make a right (right or sharp right)
   * @param  penalize_internal_uturns   Do we want to penalize uturns on a short, internal edge
   * @param  internal_turn              Did we make an internal turn on the previous edge.
   * @param  seconds      Time.
   */
  inline void AddUturnPenalty(const uint32_t idx,
                              const baldr::NodeInfo* node,
                              const baldr::DirectedEdge* edge,
                              const bool has_reverse,
                              const bool has_left,
                              const bool has_right,
                              const bool penalize_internal_uturns,
                              const InternalTurn internal_turn,
                              float& seconds) const {

    if (node->drive_on_right()) {
      // Did we make a uturn on a short, internal edge or did we make a uturn at a node.
      if (has_reverse ||
          (penalize_internal_uturns && internal_turn == InternalTurn::kLeftTurn && has_left))
        seconds += kTCUnfavorableUturn;
      // Did we make a pencil point uturn?
      else if (edge->turntype(idx) == baldr::Turn::Type::kSharpLeft && edge->edge_to_right(idx) &&
               !edge->edge_to_left(idx) && edge->named() && edge->name_consistency(idx))
        seconds *= kTCUnfavorablePencilPointUturn;
    } else {
      // Did we make a uturn on a short, internal edge or did we make a uturn at a node.
      if (has_reverse ||
          (penalize_internal_uturns && internal_turn == InternalTurn::kRightTurn && has_right))
        seconds += kTCUnfavorableUturn;
      // Did we make a pencil point uturn?
      else if (edge->turntype(idx) == baldr::Turn::Type::kSharpRight && !edge->edge_to_right(idx) &&
               edge->edge_to_left(idx) && edge->named() && edge->name_consistency(idx))
        seconds *= kTCUnfavorablePencilPointUturn;
    }
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
   * Sets the flag indicating whether edges with valid restriction conditional=destination are
   * allowed.
   */
  void set_allow_conditional_destination(const bool allow);

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
   * Gets the hierarchy limits.
   * @return  Returns the hierarchy limits.
   */
  std::vector<HierarchyLimits>& GetHierarchyLimits();

  /**
   * Relax hierarchy limits using pre-defined algorithm-cased factors.
   */
  void RelaxHierarchyLimits(const bool using_bidirectional);

  /**
   * Checks if we should exclude or not.
   */
  virtual void AddToExcludeList(const graph_tile_ptr& tile);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const graph_tile_ptr& tile, const baldr::DirectedEdge* edge);

  /**
   * Checks if we should exclude or not.
   * @return  Returns true if we should exclude, false if not.
   */
  virtual bool IsExcluded(const graph_tile_ptr& tile, const baldr::NodeInfo* node);

  /**
   * Adds a list of edges (GraphIds) to the user specified avoid list.
   * This can be used by test programs - alternatively a list of avoid
   * edges will be passed in the property tree for the costing options
   * of a specified type.
   * @param  exclude_edges  Set of edge Ids to avoid along with the percent along the edge.
   */
  void AddUserAvoidEdges(const std::vector<AvoidEdge>& exclude_edges);

  /**
   * Check if the edge is in the user-specified avoid list.
   * @param  edgeid  Directed edge Id.
   * @return Returns true if the edge Id is in the user avoid edges set,
   *         false otherwise.
   */
  bool IsUserAvoidEdge(const baldr::GraphId& edgeid) const {
    return (user_exclude_edges_.size() != 0 &&
            user_exclude_edges_.find(edgeid) != user_exclude_edges_.end());
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
    auto avoid = user_exclude_edges_.find(edgeid);
    return (avoid != user_exclude_edges_.end() && avoid->second >= percent_along);
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
    auto avoid = user_exclude_edges_.find(edgeid);
    return (avoid != user_exclude_edges_.end() && avoid->second <= percent_along);
  }

  /**
   * Get the flow mask used for accessing traffic flow data from the tile
   * @return the flow mask used
   */
  uint8_t flow_mask() const {
    return flow_mask_;
  }

  virtual Cost BSSCost() const;

  /*
   * Determine whether an edge is currently closed due to traffic.
   * @param  edgeid         GraphId of the opposing edge.
   * @return  Returns true if the edge is closed due to live traffic constraints, false if not.
   */
  inline virtual bool IsClosed(const baldr::DirectedEdge* edge, const graph_tile_ptr& tile) const {
    return !ignore_closures_ && (flow_mask_ & baldr::kCurrentFlowMask) && tile->IsClosed(edge);
  }

  float SpeedPenalty(const baldr::DirectedEdge* edge,
                     const graph_tile_ptr& tile,
                     const baldr::TimeInfo& time_info,
                     uint8_t flow_sources,
                     float edge_speed) const {
    // TODO: speed_penality hasn't been extensively tested, might alter this in future
    float average_edge_speed = edge_speed;
    // dont use current speed layer for penalties as live speeds might be too low/too high
    // better to use layers with smoothed/constant speeds
    if (top_speed_ != baldr::kMaxAssumedSpeed && (flow_sources & baldr::kCurrentFlowMask)) {
      average_edge_speed =
          tile->GetSpeed(edge, flow_mask_ & (~baldr::kCurrentFlowMask), time_info.second_of_week);
    }
    float speed_penalty =
        (average_edge_speed > top_speed_) ? (average_edge_speed - top_speed_) * 0.05f : 0.0f;

    return speed_penalty;
  }

protected:
  /**
   * Calculate `track` costs based on tracks preference.
   * @param use_tracks value of tracks preference in range [0; 1]
   */
  virtual void set_use_tracks(float use_tracks);

  /**
   * Calculate `living_street` costs based on living streets preference.
   * @param use_living_streets value of living streets preference in range [0; 1]
   */
  virtual void set_use_living_streets(float use_living_streets);

  /**
   * Calculate `lit` costs based on lit preference.
   * @param use_lit value of lit preference in range [0; 1]
   */
  virtual void set_use_lit(float use_lit);

  // Algorithm pass
  uint32_t pass_;

  // Flag indicating whether transit connections are allowed.
  bool allow_transit_connections_;

  // Allow entrance onto destination only edges. Bidirectional A* can (usually)
  // disable access onto destination only edges for driving routes. Pedestrian
  // and bicycle generally allow access (with small penalties).
  bool allow_destination_only_;

  bool allow_conditional_destination_;

  // Travel mode
  TravelMode travel_mode_;

  // Access mask based on travel mode
  uint32_t access_mask_;

  // Hierarchy limits.
  std::vector<HierarchyLimits> hierarchy_limits_;

  // User specified edges to avoid with percent along (for avoiding PathEdges of locations)
  std::unordered_map<baldr::GraphId, float> user_exclude_edges_;

  // Weighting to apply to ferry edges
  float ferry_factor_, rail_ferry_factor_;
  float track_factor_;         // Avoid tracks factor.
  float living_street_factor_; // Avoid living streets factor.
  float service_factor_;       // Avoid service roads factor.
  float closure_factor_;       // Avoid closed edges factor.
  float unlit_factor_;         // Avoid unlit edges factor.

  // Transition costs
  sif::Cost country_crossing_cost_;
  sif::Cost gate_cost_;
  sif::Cost private_access_cost_;
  sif::Cost toll_booth_cost_;
  sif::Cost ferry_transition_cost_;
  sif::Cost bike_share_cost_;
  sif::Cost rail_ferry_transition_cost_;

  // Penalties that all costing methods support
  float maneuver_penalty_;         // Penalty (seconds) when inconsistent names
  float alley_penalty_;            // Penalty (seconds) to use a alley
  float destination_only_penalty_; // Penalty (seconds) using private road, driveway, or parking aisle
  float living_street_penalty_;    // Penalty (seconds) to use a living street
  float track_penalty_;            // Penalty (seconds) to use tracks
  float service_penalty_;          // Penalty (seconds) to use a generic service road

  // A mask which determines which flow data the costing should use from the tile
  uint8_t flow_mask_;

  // percentage of allowing probable restriction a 0 probability means do not utilize them
  uint8_t restriction_probability_{0};

  // Whether or not to do shortest (by length) routes
  // Note: hierarchy pruning means some costings (auto, truck, etc) won't do absolute shortest
  bool shortest_;

  bool ignore_restrictions_{false};
  bool ignore_oneways_{false};
  bool ignore_access_{false};
  bool ignore_closures_{false};
  uint32_t top_speed_;
  uint32_t fixed_speed_;
  // if ignore_closures_ is set to true by the user request, filter_closures_ is forced to false
  bool filter_closures_{true};

  // Should we penalize uturns on short internal edges?
  bool penalize_uturns_;

  bool exclude_unpaved_{false};

  bool exclude_cash_only_tolls_{false};

  // HOT/HOV flags
  bool include_hot_{false};
  bool include_hov2_{false};
  bool include_hov3_{false};

  /**
   * Get the base transition costs (and ferry factor) from the costing options.
   * @param costing_options Protocol buffer of costing options.
   */
  void get_base_costs(const Costing& costing) {
    const auto& costing_options = costing.options();
    // Cost only (no time) penalties
    alley_penalty_ = costing_options.alley_penalty();
    destination_only_penalty_ = costing_options.destination_only_penalty();
    maneuver_penalty_ = costing_options.maneuver_penalty();

    restriction_probability_ = costing_options.restriction_probability();

    // Transition costs (both time and cost)
    toll_booth_cost_ = {costing_options.toll_booth_cost() + costing_options.toll_booth_penalty(),
                        costing_options.toll_booth_cost()};
    country_crossing_cost_ = {costing_options.country_crossing_cost() +
                                  costing_options.country_crossing_penalty(),
                              costing_options.country_crossing_cost()};
    gate_cost_ = {costing_options.gate_cost() + costing_options.gate_penalty(),
                  costing_options.gate_cost()};
    private_access_cost_ = {costing_options.gate_cost() + costing_options.private_access_penalty(),
                            costing_options.gate_cost()};

    bike_share_cost_ = {costing_options.bike_share_cost() + costing_options.bike_share_penalty(),
                        costing_options.bike_share_cost()};

    // Set the cost (seconds) to enter a ferry (only apply entering since
    // a route must exit a ferry (except artificial test routes ending on
    // a ferry!). Modify ferry edge weighting based on use_ferry factor.
    float ferry_penalty, rail_ferry_penalty;
    float use_ferry = costing_options.use_ferry();
    if (use_ferry < 0.5f) {
      // Penalty goes from max at use_ferry = 0 to 0 at use_ferry = 0.5
      ferry_penalty = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - use_ferry * 2.0f));

      // Cost X10 at use_ferry == 0, slopes downwards towards 1.0 at use_ferry = 0.5
      ferry_factor_ = 10.0f - use_ferry * 18.0f;
    } else {
      // Add a ferry weighting factor to influence cost along ferries to make
      // them more favorable if desired rather than driving. No ferry penalty.
      // Half the cost at use_ferry == 1, progress to 1.0 at use_ferry = 0.5
      ferry_penalty = 0.0f;
      ferry_factor_ = 1.5f - use_ferry;
    }
    ferry_transition_cost_ = {costing_options.ferry_cost() + ferry_penalty,
                              costing_options.ferry_cost()};

    // Set the cost (seconds) to enter a rail_ferry (only apply entering since
    // a route must exit a ferry (except artificial test routes ending on
    // a ferry!). Modify ferry edge weighting based on use_ferry factor.
    float use_rail_ferry = costing_options.use_rail_ferry();
    if (use_rail_ferry < 0.5f) {
      // Penalty goes from max at use_rail_ferry = 0 to 0 at use_rail_ferry = 0.5
      rail_ferry_penalty = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - use_rail_ferry * 2.0f));

      // Cost X10 at use_rail_ferry == 0, slopes downwards towards 1.0 at use_rail_ferry = 0.5
      rail_ferry_factor_ = 10.0f - use_rail_ferry * 18.0f;
    } else {
      // Add a ferry weighting factor to influence cost along ferries to make
      // them more favorable if desired rather than driving. No ferry penalty.
      // Half the cost at use_ferry == 1, progress to 1.0 at use_ferry = 0.5
      rail_ferry_penalty = 0.0f;
      rail_ferry_factor_ = 1.5f - use_rail_ferry;
    }
    rail_ferry_transition_cost_ = {costing_options.rail_ferry_cost() + rail_ferry_penalty,
                                   costing_options.rail_ferry_cost()};

    // Calculate cost factor for track roads
    set_use_tracks(costing_options.use_tracks());

    // Get living street factor from costing options.
    set_use_living_streets(costing_options.use_living_streets());

    // Calculate lit factor from costing options.
    set_use_lit(costing_options.use_lit());

    // Penalty and factor to use service roads
    service_penalty_ = costing_options.service_penalty();
    service_factor_ = costing_options.service_factor();
    // Closure factor to use for closed edges
    closure_factor_ = costing_options.closure_factor();

    // Set the speed mask to determine which speed data types are allowed
    flow_mask_ = costing_options.flow_mask();
    // Set the fixed speed a vehicle can go
    fixed_speed_ = costing_options.fixed_speed();
    // Set the top speed a vehicle wants to go
    top_speed_ =
        fixed_speed_ == baldr::kDisableFixedSpeed ? costing_options.top_speed() : fixed_speed_;

    exclude_unpaved_ = costing_options.exclude_unpaved();

    exclude_cash_only_tolls_ = costing_options.exclude_cash_only_tolls();
  }

  /**
   * Base transition cost that all costing methods use. Includes costs for
   * country crossing, boarding a ferry, toll booth, gates, entering destination
   * only, alleys, and maneuver penalties. Each costing method can provide different
   * costs for these transitions (via costing options).
   *
   * The template allows us to treat edgelabels and directed edges the same. The unidirectinal
   * path algorithms dont have access to the directededge from the label but they have the same
   * function names. At the moment we could change edgelabel to keep the edge pointer because
   * we dont clear tiles while the algorithm is running but for embedded use-cases we might one day
   * do that so its best to keep support for labels and edges here
   *
   * @param node Node at the intersection where the edge transition occurs.
   * @param edge Directed edge entering.
   * @param pred Predecessor edge or edgelabel.
   * @param idx  Index used for name consistency.
   * @return Returns the transition cost (cost, elapsed time).
   */
  template <typename predecessor_t>
  sif::Cost base_transition_cost(const baldr::NodeInfo* node,
                                 const baldr::DirectedEdge* edge,
                                 const predecessor_t* pred,
                                 const uint32_t idx) const {
    // Cases with both time and penalty: country crossing, ferry, rail_ferry, gate, toll booth
    sif::Cost c;
    c += country_crossing_cost_ * (node->type() == baldr::NodeType::kBorderControl);
    c += gate_cost_ * (node->type() == baldr::NodeType::kGate) * (!node->tagged_access());
    c += private_access_cost_ *
         (node->type() == baldr::NodeType::kGate || node->type() == baldr::NodeType::kBollard) *
         node->private_access();
    c += bike_share_cost_ * (node->type() == baldr::NodeType::kBikeShare);
    c += toll_booth_cost_ *
         (node->type() == baldr::NodeType::kTollBooth || (edge->toll() && !pred->toll()));
    c += ferry_transition_cost_ *
         (edge->use() == baldr::Use::kFerry && pred->use() != baldr::Use::kFerry);
    c += rail_ferry_transition_cost_ *
         (edge->use() == baldr::Use::kRailFerry && pred->use() != baldr::Use::kRailFerry);

    // Additional penalties without any time cost
    c.cost += destination_only_penalty_ * (edge->destonly() && !pred->destonly());
    c.cost +=
        alley_penalty_ * (edge->use() == baldr::Use::kAlley && pred->use() != baldr::Use::kAlley);
    c.cost += maneuver_penalty_ * (!edge->link() && !edge->name_consistency(idx));
    c.cost += living_street_penalty_ *
              (edge->use() == baldr::Use::kLivingStreet && pred->use() != baldr::Use::kLivingStreet);
    c.cost +=
        track_penalty_ * (edge->use() == baldr::Use::kTrack && pred->use() != baldr::Use::kTrack);

    if (edge->use() == baldr::Use::kServiceRoad && pred->use() != baldr::Use::kServiceRoad) {
      // Do not penalize internal roads that are marked as service.
      if (!edge->internal())
        c.cost += service_penalty_;
    }

    // shortest ignores any penalties in favor of path length
    c.cost *= !shortest_;
    return c;
  }
};

using cost_ptr_t = std::shared_ptr<DynamicCost>;
using mode_costing_t = std::array<cost_ptr_t, static_cast<size_t>(TravelMode::kMaxTravelMode)>;

/*
 * Structure that stores default values for costing options that are common for most costing models.
 * It mostly contains options used in DynamicCost::get_base_costs() method.
 */
struct BaseCostingOptionsConfig {
  BaseCostingOptionsConfig();

  ranged_default_t<float> dest_only_penalty_;
  ranged_default_t<float> maneuver_penalty_;
  ranged_default_t<float> alley_penalty_;
  ranged_default_t<float> gate_cost_;
  ranged_default_t<float> gate_penalty_;
  ranged_default_t<float> private_access_penalty_;
  ranged_default_t<float> country_crossing_cost_;
  ranged_default_t<float> country_crossing_penalty_;

  bool disable_toll_booth_ = false;
  ranged_default_t<float> toll_booth_cost_;
  ranged_default_t<float> toll_booth_penalty_;

  bool disable_ferry_ = false;
  ranged_default_t<float> ferry_cost_;
  ranged_default_t<float> use_ferry_;

  bool disable_rail_ferry_ = false;
  ranged_default_t<float> rail_ferry_cost_;
  ranged_default_t<float> use_rail_ferry_;

  ranged_default_t<float> service_penalty_;
  ranged_default_t<float> service_factor_;

  ranged_default_t<float> height_;
  ranged_default_t<float> width_;

  ranged_default_t<float> use_tracks_;
  ranged_default_t<float> use_living_streets_;
  ranged_default_t<float> use_lit_;

  ranged_default_t<float> closure_factor_;

  bool exclude_unpaved_;

  bool exclude_cash_only_tolls_ = false;

  bool include_hot_ = false;
  bool include_hov2_ = false;
  bool include_hov3_ = false;
};

/**
 * Parses base cost options that are common for most costing models. If you use this function
 * make sure that different default values were overridden in the config. Also explicitly
 * disable options that shouldn't be parsed.
 * @param json The json request represented as a DOM tree.
 * @param co A mutable protocol buffer where the parsed json values will be stored.
 * @param cfg Default values with enable/disable parsing indicators for costing options.
 */
void ParseBaseCostOptions(const rapidjson::Value& json,
                          Costing* co,
                          const BaseCostingOptionsConfig& cfg);

/**
 * Parses all the costing options for all supported costings
 * @param doc                   json document
 * @param costing_options_key   the key in the json document where the options are located
 * @param options               where to store the parsed costing
 */
void ParseCosting(const rapidjson::Document& doc,
                  const std::string& costing_options_key,
                  Options& options);

/**
 * Parses the costing options for the costing specified within the json object. If the
 * json object has no key named "costing" the type of costing cannot be found and an
 * exception is thrown
 * @param doc                   json document
 * @param key                   the key in the json document where the options are located
 * @param costing_options       where to store the parsed options
 * @param costing               specify the costing you want to parse or let it check the json
 */
void ParseCosting(const rapidjson::Document& doc,
                  const std::string& key,
                  Costing* costing,
                  Costing::Type costing_type = static_cast<Costing::Type>(Costing::Type_ARRAYSIZE));

} // namespace sif

} // namespace valhalla

#endif // VALHALLA_SIF_DYNAMICCOST_H_
