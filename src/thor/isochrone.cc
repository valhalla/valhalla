#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/isochrone.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Method to get an operator Id from a map of operator strings vs. Id.
uint32_t GetOperatorId(const GraphTile* tile, uint32_t routeid,
            std::unordered_map<std::string, uint32_t>& operators) {
  const TransitRoute* transit_route = tile->GetTransitRoute(routeid);

  // Test if the transit operator changed
  if (transit_route && transit_route->op_by_onestop_id_offset()) {
    // Get the operator name and look up in the operators map
    std::string operator_name =
        tile->GetName(transit_route->op_by_onestop_id_offset());
    auto operator_itr = operators.find(operator_name);
    if (operator_itr == operators.end()) {
      // Operator not found - add to the map
      uint32_t id = operators.size() + 1;
      operators[operator_name] = id;
      return id;
    } else {
      return operator_itr->second;
    }
  }
  return 0;
}

constexpr float to_minutes = 1.0/60.0;

}

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
Isochrone::Isochrone()
    : tile_creation_date_(0),
      shape_interval_(50.0f),
      mode_(TravelMode::kDrive),
      adjacencylist_(nullptr),
      edgestatus_(nullptr) {
}

// Destructor
Isochrone::~Isochrone() {
  Clear();
}

// Clear the temporary information generated during path construction.
void Isochrone::Clear() {
  // Clear the edge labels, edge status flags, and adjacency list
  edgelabels_.clear();
  adjacencylist_.reset();
  edgestatus_.reset();
}

// Construct the isotile. Use a grid size based on travel mode.
// Convert time in minutes to a max distance in meters based on an
// estimate of max average speed for the travel mode.
void Isochrone::ConstructIsoTile(const bool multimodal, const unsigned int max_minutes,
                                 std::vector<baldr::PathLocation>& origin_locations) {
  float grid_size, max_distance;
  auto max_seconds = max_minutes * 60;
  if (multimodal) {
    grid_size = 200.0f;
    max_distance = max_seconds * 70.0f * 0.44704f; // TODO
  } else if (mode_ == TravelMode::kPedestrian) {
    grid_size = 200.0f;
    max_distance = max_seconds * 5.0f * 0.44704f;
  } else if (mode_ == TravelMode::kBicycle) {
    grid_size = 200.0f;
    max_distance = max_seconds * 20.0f * 0.44704f;
  } else {
    // A driving mode
    grid_size = 400.0f;
    max_distance = max_seconds * 70.0f * 0.44704f;
  }
  shape_interval_ = grid_size * 0.25f;

  // Form grid for isotiles. Convert grid size to degrees.
  grid_size /= kMetersPerDegreeLat;
  float lat = origin_locations[0].latlng_.lat();
  float dlat = max_distance / kMetersPerDegreeLat;
  float dlon = max_distance / DistanceApproximator::MetersPerLngDegree(lat);
  AABB2<PointLL> bounds(10000.0f, 10000.0f, -10000.0f, -10000.0f);
  for (const auto& loc : origin_locations) {
    PointLL center = loc.latlng_;
    AABB2<PointLL> bbox(PointLL(center.lng() - dlon, center.lat() - dlat),
                        PointLL(center.lng() + dlon, center.lat() + dlat));
    bounds.Expand(bbox);
  }
  isotile_.reset(new GriddedData<PointLL>(bounds, grid_size, max_minutes + 5));
}

// Initialize - create adjacency list, edgestatus support, and reserve
// edgelabels
void Isochrone::Initialize(const uint32_t bucketsize) {
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new AdjacencyList(0.0f, range, bucketsize));
  edgestatus_.reset(new EdgeStatus());
  edgelabels_.reserve(kInitialEdgeLabelCount);
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<PointLL> > Isochrone::Compute(
             std::vector<PathLocation>& origin_locations,
             const unsigned int max_minutes,
             GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize and create the isotile
  auto max_seconds = max_minutes * 60;
  Initialize(costing->UnitSize());
  ConstructIsoTile(false, max_minutes, origin_locations);

  // Set the origin locations
  SetOriginLocations(graphreader, origin_locations, costing);

  // Compute the isotile
  uint32_t n = 0;
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      return isotile_;
    }

    // Copy the EdgeLabel for use in costing and settle the edge.
    EdgeLabel pred = edgelabels_[predindex];
    edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);

    // Get the end node of the prior directed edge. Skip if tile not found
    // (can happen with regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Get the nodeinfo and update the isotile
    const NodeInfo* nodeinfo = tile->node(node);
    UpdateIsoTile(pred, graphreader, nodeinfo->latlng());
    n++;

    // Return after the time interval has been met
    if (pred.cost().secs > max_seconds) {
      LOG_INFO("Exceed time interval: n = " + std::to_string(n));
      return isotile_;
    }

    // Check access at the node
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Expand from end node.
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Do not transition to upper hierarchies. Skip if no access is allowed
      // to this edge (based on the costing method.
      if (directededge->trans_up() ||
          !costing->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Compute the cost to the end of this edge
      Cost newcost = pred.cost() +
			     costing->EdgeCost(directededge, nodeinfo->density()) +
			     costing->TransitionCost(directededge, nodeinfo, pred);

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, newcost.cost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, 0);
    }
  }
  return isotile_;      // Should never get here
}

std::shared_ptr<const GriddedData<PointLL> > Isochrone::ComputeMultiModal(
             std::vector<PathLocation>& origin_locations,
             const unsigned int max_minutes, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // For pedestrian costing - set flag allowing use of transit connections
  // Set pedestrian costing to use max distance. TODO - need for other modes
  const auto& pc = mode_costing[static_cast<uint8_t>(TravelMode::kPedestrian)];
  pc->SetAllowTransitConnections(true);
  pc->UseMaxMultiModalDistance();

  // Set the mode from the origin
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint8_t>(mode)];
  const auto& tc = mode_costing[static_cast<uint8_t>(TravelMode::kPublicTransit)];

  // Get maximum transfer distance (TODO - want to allow unlimited walking once
  // you get off the transit stop...)
  uint32_t max_transfer_distance = 99999.0f; //costing->GetMaxTransferDistanceMM();

  // Initialize and create the isotile
  auto max_seconds = max_minutes * 60;
  Initialize(costing->UnitSize());
  ConstructIsoTile(true, max_minutes, origin_locations);

  // Set the origin locations.
  SetOriginLocations(graphreader, origin_locations, costing);

  // Update start time
  uint32_t start_time, localtime, date, dow, day = 0;
  bool date_before_tile = false;
  if (origin_locations[0].date_time_) {
    // Set route start time (seconds from midnight), date, and day of week
    start_time = DateTime::seconds_from_midnight(*origin_locations[0].date_time_);
    localtime = start_time;
  }

  // Expand using adjacency list until we exceed threshold
  uint32_t n = 0;
  bool date_set = false;
  uint32_t blockid, tripid;
  std::unordered_map<std::string, uint32_t> operators;
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      return isotile_;
    }

    // Copy the EdgeLabel for use in costing and settle the edge.
    EdgeLabel pred = edgelabels_[predindex];
    edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);

    // Get the end node. Skip if tile not found (can happen with
    // regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Get the nodeinfo and update the isotile
    const NodeInfo* nodeinfo = tile->node(node);
    UpdateIsoTile(pred, graphreader, nodeinfo->latlng());
    n++;

    // Return after the time interval has been met
    if (pred.cost().secs > max_seconds) {
      LOG_INFO("Exceed time interval: n = " + std::to_string(n));
      return isotile_;
    }

    // Check access at the node
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Return after the time interval has been met
    if (pred.cost().secs > max_seconds) {
      LOG_INFO("Exceed time interval: n = " + std::to_string(n));
      return isotile_;
    }

    // Set local time. TODO: adjust for time zone.
    uint32_t localtime = start_time + pred.cost().secs;

    // Set a default transfer penalty at a stop (if not same trip Id and block Id)
    Cost transfer_cost = tc->DefaultTransferCost();

    // Get any transfer times and penalties if this is a transit stop (and
    // transit has been taken at some point on the path) and mode is pedestrian
    mode_ = pred.mode();
    bool has_transit = pred.has_transit();
    GraphId prior_stop = pred.prior_stopid();
    uint32_t operator_id = pred.transit_operator();
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {
      // Get the transfer penalty when changing stations
      if (mode_ == TravelMode::kPedestrian && prior_stop.Is_Valid() && has_transit) {
        transfer_cost = tc->TransferCost();
      }

      // Add transfer time to the local time when entering a stop
      // as a pedestrian. This is a small added cost on top of
      // any costs along paths and roads
      if (mode_ == TravelMode::kPedestrian) {
        localtime += transfer_cost.secs;
      }

      // Update prior stop. TODO - parent/child stop info?
      prior_stop = node;

      // we must get the date from level 3 transit tiles and not level 2.  The level 3 date is
      // set when the fetcher grabbed the transit data and created the schedules.
      if (!date_set) {
        date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(*origin_locations[0].date_time_));
        dow  = DateTime::day_of_week_mask(*origin_locations[0].date_time_);
        uint32_t date_created = tile->header()->date_created();
        if (date < date_created)
          date_before_tile = true;
        else
          day = date - date_created;

        date_set = true;
      }
    }

    // TODO: allow mode changes at special nodes
    //      bike share (pedestrian <--> bicycle)
    //      parking (drive <--> pedestrian)
    //      transit stop (pedestrian <--> transit).
    bool mode_change = false;

    // Expand from end node.
    uint32_t shortcuts = 0;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0, n = nodeinfo->edge_count(); i < n;
                i++, directededge++, edgeid++) {
      // Do not transition to upper hierarchies
      if (directededge->trans_up()) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Reset cost and walking distance
      Cost newcost = pred.cost();
      uint32_t walking_distance = pred.path_distance();

      // If this is a transit edge - get the next departure. Do not check
      // if allowed by costing - assume if you get a transit edge you
      // walked to the transit stop
      tripid  = 0;
      blockid = 0;
      if (directededge->IsTransitLine()) {
        // Check if transit costing allows this edge
        if (!tc->Allowed(directededge, pred, tile, edgeid)) {
          continue;
        }

        // Look up the next departure along this edge
        const TransitDeparture* departure = tile->GetNextDeparture(
                    directededge->lineid(), localtime, day, dow, date_before_tile);
        if (departure) {
          // Check if there has been a mode change
          mode_change = (mode_ == TravelMode::kPedestrian);

          // Update trip Id and block Id
          tripid  = departure->tripid();
          blockid = departure->blockid();
          has_transit = true;

          // There is no cost to remain on the same trip or valid blockId
          if ( tripid == pred.tripid() ||
              (blockid != 0 && blockid == pred.blockid())) {
            // This departure is valid without any added cost. Operator Id
            // is the same as the predecessor
            operator_id = pred.transit_operator();
          } else {
            if (pred.tripid() > 0) {
              // tripId > 0 means the prior edge was a transit edge and this
              // is an "in-station" transfer. Add a small transfer time and
              // call GetNextDeparture again if we cannot make the current
              // departure.
              // TODO - is there a better way?
              if (localtime + 30 > departure->departure_time()) {
                  departure = tile->GetNextDeparture(directededge->lineid(),
                                localtime + 30, day, dow, date_before_tile);
                if (!departure)
                  continue;
              }
            }

            // Get the operator Id
            operator_id = GetOperatorId(tile, departure->routeid(), operators);

            // Add transfer penalty and operator change penalty
            newcost.cost += transfer_cost.cost;
            if (pred.transit_operator() > 0 &&
                pred.transit_operator() != operator_id) {
              // TODO - create a configurable operator change penalty
              newcost.cost += 300;
            }
          }

          // Change mode and costing to transit. Add edge cost.
          mode_ = TravelMode::kPublicTransit;
          newcost += tc->EdgeCost(directededge, departure, localtime);
        } else {
          // No matching departures found for this edge
          continue;
        }
      } else {
        // If current mode is public transit we should only connect to
        // transit connection edges or transit edges
        if (mode_ == TravelMode::kPublicTransit) {
          // Disembark from transit and reset walking distance
          mode_ = TravelMode::kPedestrian;
          walking_distance = 0;
          mode_change = true;
        }

        // Regular edge - use the appropriate costing and check if access
        // is allowed. If mode is pedestrian this will validate walking
        // distance has not been exceeded.
        if (!mode_costing[static_cast<uint32_t>(mode_)]->Allowed(
                directededge, pred, tile, edgeid)) {
          continue;
        }

        Cost c = mode_costing[static_cast<uint32_t>(mode_)]->EdgeCost(
            directededge, nodeinfo->density());
        c.cost *= mode_costing[static_cast<uint32_t>(mode_)]->GetModeWeight();
        newcost += c;

        // Add to walking distance
        if (mode_ == TravelMode::kPedestrian) {
          walking_distance += directededge->length();

          // Prevent going from one transit connection directly to another
          // at a transit stop - this is like entering a station and exiting
          // without getting on transit
          if (nodeinfo->type() == NodeType::kMultiUseTransitStop &&
              pred.use()   == Use::kTransitConnection &&
              directededge->use()  == Use::kTransitConnection)
                continue;
        }
      }

      // Add mode change cost or edge transition cost from the costing model
      if (mode_change) {
        // TODO: make mode change cost configurable. No cost for entering
        // a transit line (assume the wait time is the cost)
        ;  //newcost += {10.0f, 10.0f };
      } else {
        newcost += mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(
               directededge, nodeinfo, pred);
      }

      // Prohibit entering the same station as the prior.
      if (directededge->use() == Use::kTransitConnection &&
          directededge->endnode() == pred.prior_stopid()) {
        continue;
      }

      // Test if exceeding maximum transfer walking distance
      if (directededge->use() == Use::kTransitConnection &&
          pred.prior_stopid().Is_Valid() &&
          walking_distance > max_transfer_distance) {
        continue;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change). Update
      // trip Id and block Id.
      if (edgestatus.set() == EdgeSet::kTemporary) {
        uint32_t idx = edgestatus.status.index;
        float dc = edgelabels_[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels_[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels_[idx].Update(predindex, newcost, newsortcost,
                                  walking_distance, tripid, blockid);
          adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // Add edge label, add to the adjacency list and set edge status
      AddToAdjacencyList(edgeid, newcost.cost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, walking_distance,
                    tripid, prior_stop, blockid, operator_id, has_transit);
    }
  }
  return isotile_;      // Should never get here
}

// Update the isotile
void Isochrone::UpdateIsoTile(const EdgeLabel& pred, GraphReader& graphreader,
                              const PointLL& ll) {
  // Skip if the opposing edge has already been settled.
  GraphId opp = graphreader.GetOpposingEdgeId(pred.edgeid());
  EdgeStatusInfo edgestatus = edgestatus_->Get(opp);
  if (edgestatus.set() == EdgeSet::kPermanent) {
      return;
  }

  // Get time at the end node of the predecessor
  float secs1 = pred.cost().secs;

  // Get the time at the end node of the predecessor
  float secs0;
  uint32_t predindex = pred.predecessor();
  if (predindex == kInvalidLabel) {
    //TODO - do we need partial shape from origin location to end of edge?
    secs0 = 0;
  } else {
    secs0 = edgelabels_[predindex].cost().secs;
  }

  // Get the directed edge and its shape. Make sure shape is forward
  // direction and resample it to the shape interval.
  const GraphTile* tile = graphreader.GetGraphTile(pred.edgeid().Tile_Base());
  const DirectedEdge* edge = tile->directededge(pred.edgeid());
  auto shape = tile->edgeinfo(edge->edgeinfo_offset())->shape();
  if (!edge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  auto resampled = resample_spherical_polyline(shape, shape_interval_);

  // Mark grid cells along the shape if time is less than what is
  // already populated. Get intersection of tiles along each segment
  // so this doesn't miss shape that crosses tile corners
  float delta = (shape_interval_ * (secs1 - secs0)) / edge->length();
  float secs = secs0;
  auto itr1 = resampled.begin();
  auto itr2 = itr1 + 1;
  for (auto itr2 = itr1 + 1; itr2 < resampled.end(); itr1++, itr2++) {
    secs += delta;
    auto tiles = isotile_->Intersect(std::list<PointLL>{*itr1, *itr2});
    for (auto t : tiles) {
      isotile_->SetIfLessThan(t.first, secs * to_minutes);
    }
  }
}

// Convenience method to add an edge to the adjacency list and temporarily
// label it.
void Isochrone::AddToAdjacencyList(const GraphId& edgeid,
                                   const float sortcost) {
  uint32_t idx = edgelabels_.size();
  adjacencylist_->Add(idx, sortcost);
  edgestatus_->Set(edgeid, EdgeSet::kTemporary, idx);
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change)
void Isochrone::CheckIfLowerCostPath(const uint32_t idx,
                                     const uint32_t predindex,
                                     const Cost& newcost) {
  float dc = edgelabels_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_[idx].Update(predindex, newcost, newsortcost);
    adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Add edge(s) at each origin to the adjacency list
void Isochrone::SetOriginLocations(GraphReader& graphreader,
                 std::vector<PathLocation>& origin_locations,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (auto& origin : origin_locations) {
    // Set time at the origin lat, lon grid to 0
    isotile_->Set(origin.latlng_, 0);

    // Iterate through edges and add to adjacency list
    const NodeInfo* nodeinfo = nullptr;
    for (const auto& edge : (origin.edges)) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (edge.end_node()) {
        continue;
      }

      // Get the directed edge
      GraphId edgeid = edge.id;
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Set the tile creation date
      tile_creation_date_ = tile->header()->date_created();

      // Get the tile at the end node. Skip if tile not found as we won't be
      // able to expand from this origin edge.
      const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
      if (endtile == nullptr) {
        continue;
      }

      // Get cost
      nodeinfo = endtile->node(directededge->endnode());
      Cost cost = costing->EdgeCost(directededge,
                      graphreader.GetEdgeDensity(edge.id)) * (1.0f - edge.dist);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.dist));
      adjacencylist_->Add(edgelabels_.size(), cost.cost);
      EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost,
              cost.cost, 0.0f, directededge->restrictions(),
              directededge->opp_local_idx(), mode_, d);
      edge_label.set_origin();

      // Set the origin flag
      edgelabels_.push_back(std::move(edge_label));
    }

    // Set the origin timezone
    if (nodeinfo != nullptr && origin.date_time_ &&
      *origin.date_time_ == "current") {
      origin.date_time_ = DateTime::iso_date_time(
          DateTime::get_tz_db().from_index(nodeinfo->timezone()));
    }
  }
}

}
}
