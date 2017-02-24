#include <string.h>
#include "sif/edgelabel.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default constructor
EdgeLabel::EdgeLabel() {
  memset(this, 0, sizeof(EdgeLabel));
}

// Constructor with values.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const DirectedEdge* edge, const Cost& cost,
                     const float sortcost, const float dist,
                     const TravelMode mode, const uint32_t path_distance)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_ {},
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(path_distance),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(0),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(0),
      blockid_(0),
      transit_operator_(0),
      transition_cost_({}),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(false),
      deadend_(edge->deadend()) {
}

// Constructor with values - used in bidirectional A*
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& oppedgeid, const DirectedEdge* edge,
                     const Cost& cost, const float sortcost, const float dist,
                     const TravelMode mode, const Cost& tc,
                     bool not_thru_pruning)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_(oppedgeid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(0),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(0),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(0),
      blockid_(0),
      transit_operator_(0),
      transition_cost_(tc),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(not_thru_pruning),
      deadend_(edge->deadend()) {
}

// Constructor with values.  Used for multi-modal path.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
          const DirectedEdge* edge, const Cost& cost,
          const float sortcost, const float dist,
          const TravelMode mode, const uint32_t path_distance,
          const uint32_t tripid, const GraphId& prior_stopid,
          const uint32_t blockid, const uint32_t transit_operator,
          const bool has_transit)
    : predecessor_(predecessor),
      edgeid_(edgeid),
      opp_edgeid_(prior_stopid),
      endnode_(edge->endnode()),
      cost_(cost),
      sortcost_(sortcost),
      distance_(dist),
      path_distance_(path_distance),
      use_(static_cast<uint32_t>(edge->use())),
      opp_index_(edge->opp_index()),
      opp_local_idx_(edge->opp_local_idx()),
      restrictions_(edge->restrictions()),
      shortcut_(edge->shortcut()),
      mode_(static_cast<uint32_t>(mode)),
      dest_only_(edge->destonly()),
      has_transit_(has_transit),
      origin_(0),
      toll_(edge->toll()),
      classification_(static_cast<uint32_t>(edge->classification())),
      tripid_(tripid),
      blockid_(blockid),
      transit_operator_(transit_operator),
      transition_cost_({}),
      on_complex_rest_(edge->part_of_complex_restriction()),
      not_thru_(edge->not_thru()),
      not_thru_pruning_(false),
      deadend_(edge->deadend()) {
}

// Constructor with values - used in time distance matrix (needs the
// accumulated distance as well as opposing edge information). Sets
// sortcost to the true cost.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                const GraphId& oppedgeid, const DirectedEdge* edge,
                const Cost& cost, const TravelMode mode,
                const Cost& tc, const uint32_t path_distance,
                bool not_thru_pruning)
    :  predecessor_(predecessor),
       edgeid_(edgeid),
       opp_edgeid_(oppedgeid),
       endnode_(edge->endnode()),
       cost_(cost),
       sortcost_(cost.cost),
       distance_(0.0f),
       path_distance_(path_distance),
       use_(static_cast<uint32_t>(edge->use())),
       opp_index_(edge->opp_index()),
       opp_local_idx_(edge->opp_local_idx()),
       restrictions_(edge->restrictions()),
       shortcut_(edge->shortcut()),
       mode_(static_cast<uint32_t>(mode)),
       dest_only_(edge->destonly()),
       has_transit_(0),
       origin_(0),
       toll_(edge->toll()),
       classification_(static_cast<uint32_t>(edge->classification())),
       tripid_(0),
       blockid_(0),
       transit_operator_(0),
       transition_cost_(tc),
       on_complex_rest_(edge->part_of_complex_restriction()),
       not_thru_(edge->not_thru()),
       not_thru_pruning_(not_thru_pruning),
       deadend_(edge->deadend()) {
}

// Constructor given a predecessor edge label. This is used for hierarchy
// transitions where the attributes at the predecessor are needed (rather
// than attributes from the directed edge.
EdgeLabel::EdgeLabel(const uint32_t predecessor, const GraphId& edgeid,
                     const GraphId& endnode, const EdgeLabel& pred) {
  *this        = pred;
  predecessor_ = predecessor;
  edgeid_      = edgeid;
  endnode_     = endnode;
  origin_      = 0;

  // Set the use so we know this is a transition edge. For now we only need to
  // know it is a transition edge so we can skip it in complex restrictions.
  use_ = static_cast<uint32_t>(Use::kTransitionUp);
}

// Update predecessor and cost values in the label.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
}

// Update an existing edge label with new predecessor and cost information.
// Update distance as well (used in time distance matrix)
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
                       const float sortcost, const Cost& tc,
                       const uint32_t distance) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  transition_cost_ = tc;
  path_distance_ = distance;
}

// Update an existing edge label with new predecessor and cost information.
// Update transit information: prior stop Id will stay the same but trip Id
// and block Id may change (a new trip at an earlier departure time).
// The mode, edge Id, and end node remain the same.
void EdgeLabel::Update(const uint32_t predecessor, const Cost& cost,
          const float sortcost, const uint32_t path_distance,
          const uint32_t tripid,  const uint32_t blockid) {
  predecessor_ = predecessor;
  cost_ = cost;
  sortcost_ = sortcost;
  path_distance_ = path_distance;
  tripid_ = tripid;
  blockid_ = blockid;
}

// Operator for sorting.
bool EdgeLabel::operator < (const EdgeLabel& other) const {
  return sortcost() < other.sortcost();
}

}
}
