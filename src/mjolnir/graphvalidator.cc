
#include "mjolnir/graphvalidator.h"
#include "valhalla/mjolnir/graphtilebuilder.h"

#include <valhalla/midgard/logging.h>

#include <ostream>
#include <set>
#include <boost/format.hpp>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <queue>
#include <list>
#include <thread>
#include <future>
#include <mutex>
#include <numeric>
#include <tuple>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphreader.h>


using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

class return_stats {
  std::vector<std::vector<uint32_t> > dupcounts;
  std::vector<std::vector<float> > densities;

public:

  return_stats () : dupcounts(3), densities(3) { }

  std::vector<uint32_t> get_dups(int level) { return dupcounts[level]; }

  std::vector<float> get_densities(int level) { return densities[level]; }

  std::vector<std::vector<uint32_t> > get_dups() { return dupcounts; }

  std::vector<std::vector<float> > get_densities() { return densities; }

  void add_density (float density, int level) {
    this->densities[level].push_back(density);
  }

  void add_dup (uint32_t newdup, int level) {
    this->dupcounts[level].push_back(newdup);
  }

  void add_stat (return_stats& stats) {
    uint32_t level = 0;
    for (auto& dupvec : stats.get_dups()) {
      for (auto& dup : dupvec) {
        this->add_dup(dup, level);
      }
      level++;
    }
    level = 0;
    for (auto& densityvec : stats.get_densities()) {
      for (auto& density : densityvec) {
        this->add_density(density, level);
      }
      level++;
    }
  }
};

class validator_stats {
  std::map<int32_t, std::map<RoadClass, float> > tile_maps;
  std::map<std::string, std::map<RoadClass, float> > country_maps;
  std::set<uint32_t> tile_ids;
  std::set<std::string> iso_codes;

public:
  const std::vector<RoadClass> rclasses = {RoadClass::kMotorway, RoadClass::kPrimary,
                                     RoadClass::kResidential, RoadClass::kSecondary,
                                     RoadClass::kServiceOther, RoadClass::kTertiary,
                                     RoadClass::kTrunk, RoadClass::kUnclassified};
	validator_stats () : tile_maps(), country_maps(), iso_codes(), tile_ids() { }

	void add_tile_road (const uint32_t& tile_id, const RoadClass& rclass, float length) {
	  this->tile_ids.insert(tile_id);
	  this->tile_maps[tile_id][rclass] += length;
	}
	void add_country_road (const std::string& ctry_code, const RoadClass& rclass, float length) {
	  this->iso_codes.insert(ctry_code);
	  this->country_maps[ctry_code][rclass] += length;
	}
	const std::set<uint32_t>& get_ids () const { return tile_ids; }

	const std::set<std::string>& get_isos () const { return iso_codes; }

	const std::map<int32_t, std::map<RoadClass, float> >& get_tile_maps () const { return tile_maps; }

	const std::map<std::string, std::map<RoadClass, float> >& get_country_maps () const { return country_maps; }

	void add (const validator_stats& stats) {
	  auto newTileMaps = stats.get_tile_maps();
	  auto newCountryMaps = stats.get_country_maps();
	  auto ids = stats.get_ids();
	  auto isos = stats.get_isos();
	  for (auto& id : ids) {
	    for (auto& rclass : this->rclasses) {
	      this->add_tile_road(id, rclass, newTileMaps[id][rclass]);
	    }
	  }
	  for (auto& iso : isos) {
	    for (auto& rclass : this->rclasses) {
	      this->add_country_road(iso, rclass, newCountryMaps[iso][rclass]);
	    }
	  }
	}
};

// Get the GraphId of the opposing edge.
uint32_t GetOpposingEdgeIndex(const GraphId& startnode, DirectedEdge& edge,
                              GraphReader& graphreader_, uint32_t& dupcount_, std::string& endnodeiso_, std::mutex& lock) {

  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  lock.lock();
  const GraphTile* tile = graphreader_.GetGraphTile(endnode);
  lock.unlock();
  const NodeInfo* nodeinfo = tile->node(endnode.id());

  // Set the end node iso.  Used for country crossings.
  endnodeiso_ = tile->admin(nodeinfo->admin_index())->country_iso();

  // TODO - check if more than 1 edge has matching startnode and
  // distance!

  // Get the directed edges and return when the end node matches
  // the specified node and length matches
  constexpr uint32_t absurd_index = 777777;
  uint32_t opp_index = absurd_index;
  const DirectedEdge* directededge = tile->directededge(
      nodeinfo->edge_index());
  for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
    // End node must match the start node, shortcut (bool) must match
    // and lengths must match
    if (directededge->endnode() == startnode &&
        edge.is_shortcut() == directededge->is_shortcut() &&
        directededge->length() == edge.length()) {
      if (opp_index != absurd_index) {
        dupcount_++;
      }
      opp_index = i;
    }
  }

  if (opp_index == absurd_index) {
    if (edge.use() >= Use::kRail) {
      // Ignore any except for parent / child stop connections and
      // stop-road connections
      // TODO - verify if we need opposing directed edges for transit lines
      if (edge.use() == Use::kTransitConnection)
        LOG_ERROR("No opposing transit connection edge: endstop = " +
                  std::to_string(nodeinfo->stop_id()) + " has " +
                  std::to_string(nodeinfo->edge_count()));
    } else {
      bool sc = edge.shortcut();
      LOG_ERROR((boost::format("No opposing edge at LL=%1%,%2% Length = %3% Startnode %4% EndNode %5% Shortcut %6%")
      % nodeinfo->latlng().lat() % nodeinfo->latlng().lng() % edge.length()
      % startnode % edge.endnode() % sc).str());

      uint32_t n = 0;
      directededge = tile->directededge(nodeinfo->edge_index());
      for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++) {
        if (sc == directededge->is_shortcut() && directededge->is_shortcut()) {
          LOG_WARN((boost::format("    Length = %1% Endnode: %2%")
          % directededge->length() % directededge->endnode()).str());
          n++;
        }
      }
      if (n == 0) {
        if (sc) {
          LOG_WARN("   No Shortcut edges found from end node");
        } else {
          LOG_WARN("   No regular edges found from end node");
        }
      }
    }
    return kMaxEdgesPerNode;
  }
  return opp_index;
}

void validate(const boost::property_tree::ptree& hierarchy_properties,
              std::queue<GraphId>& tilequeue, std::mutex& lock,
              std::promise<std::tuple<return_stats, validator_stats> >& result) {

    // Our local class for gathering the stats
    return_stats threadStat;
    validator_stats vStats;
    // Local Graphreader
    GraphReader graph_reader(hierarchy_properties);
    // Get some things we need throughout
    auto tile_hierarchy = graph_reader.GetTileHierarchy();
    std::vector<Tiles> levels;
    for (auto level : tile_hierarchy.levels()) {
      levels.push_back(level.second.tiles);
    }
    Tiles *tiles;

    // Check for more tiles
    while (true) {
      lock.lock();
      if (tilequeue.empty()) {
        lock.unlock();
        break;
      }
      // Get the next tile Id
      GraphId tile_id = tilequeue.front();
      tilequeue.pop();
      lock.unlock();

      // Point tiles to the set we need for current level
      size_t level = tile_id.level();
      tiles = &levels[level];
      auto tileid = tile_id.tileid();

      uint32_t dupcount = 0;

      // Get the tile
      GraphTileBuilder tilebuilder(tile_hierarchy, tile_id, false);
      const GraphTile signtile(tile_hierarchy, tile_id);

      // Copy existing header. No need to update any counts or offsets.
      GraphTileHeader existinghdr = *(tilebuilder.header());
      const GraphTileHeaderBuilder hdrbuilder =
          static_cast<const GraphTileHeaderBuilder&>(existinghdr);

      // Update nodes and directed edges as needed
      std::vector<NodeInfoBuilder> nodes;
      std::vector<DirectedEdgeBuilder> directededges;

      // Iterate through the nodes and the directed edges
      float roadlength = 0.0f;
      uint32_t nodecount = tilebuilder.header()->nodecount();
      GraphId node = tile_id;
      for (uint32_t i = 0; i < nodecount; i++, node++) {
        NodeInfoBuilder nodeinfo = tilebuilder.node(i);
        const NodeInfo* signnodeinfo = signtile.node(i);

        lock.lock();
        const GraphTile* tile = graph_reader.GetGraphTile(node);
        lock.unlock();
        std::string begin_node_iso = tile->admin(nodeinfo.admin_index())->country_iso();

        // Go through directed edges and update data
        uint32_t idx = signnodeinfo->edge_index();
        for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++, idx++) {
          const DirectedEdge* signdirectededge = signtile.directededge(idx);
          // Validate signs
          if (signdirectededge->exitsign()) {
            if (signtile.GetSigns(idx).size() == 0) {
              LOG_ERROR("Directed edge marked as having signs but none found");
            }
          }

          DirectedEdgeBuilder& directededge = tilebuilder.directededge(
              nodeinfo.edge_index() + j);
          // Road Length and some variables for statistics
          RoadClass rclass;
          float tempLength;
          bool validLength = false;
          if (!directededge.shortcut() && !directededge.trans_up() &&
              !directededge.trans_down()) {
            tempLength = directededge.length();
            roadlength += tempLength;
            validLength = true;
          }
          // Set the opposing edge index and get the country ISO at the
          // end node)
          std::string end_node_iso;
          directededge.set_opp_index(GetOpposingEdgeIndex(node, directededge,
                                                          graph_reader, dupcount, end_node_iso, lock));
          // Mark a country crossing if country ISO codes do not match
          if (!begin_node_iso.empty() && !end_node_iso.empty() &&
              begin_node_iso != end_node_iso)
            directededge.set_ctry_crossing(true);
          directededges.emplace_back(std::move(directededge));

          // Add road lengths to stat class for current country and current tile
          if (validLength) {
            rclass = directededge.classification();
            vStats.add_country_road(begin_node_iso, rclass, tempLength);
            vStats.add_tile_road(tileid, rclass, tempLength);
          }
        }
        // Add the node to the list
        nodes.emplace_back(std::move(nodeinfo));
      }

      // Add density to return class
      float density = (roadlength * 0.0005f) / tiles->Area(tileid);
      threadStat.add_density(density, level);

      // Write the new file
      lock.lock();
      tilebuilder.Update(tile_hierarchy, hdrbuilder, nodes, directededges);
      lock.unlock();

      // Check if we need to clear the tile cache
      lock.lock();
      if (graph_reader.OverCommitted())
        graph_reader.Clear();
      lock.unlock();

      // Add possible duplicates to return class
      threadStat.add_dup(dupcount, level);
    }

    // Live up to our promise
    std::tuple<return_stats, validator_stats> statistics (threadStat, vStats);
    result.set_value(statistics);
  }
}

namespace valhalla {
namespace mjolnir {

  void GraphValidator::Validate(const boost::property_tree::ptree& pt) {

    // Graphreader
    boost::property_tree::ptree hierarchy_properties = (pt.get_child("mjolnir.hierarchy"));
    GraphReader reader(hierarchy_properties);
    // Make sure there are at least 2 levels!
    if (reader.GetTileHierarchy().levels().size() < 2)
      throw std::runtime_error("Bad tile hierarchy - need 2 levels");

    // Setup threads
    std::vector<std::shared_ptr<std::thread> > threads(
        std::max(static_cast<unsigned int>(1),
                 pt.get<unsigned int>("concurrency",std::thread::hardware_concurrency())));

    // and promises
    //TODO change this to a tuple type to get two obj back
    std::list<std::promise<std::tuple<return_stats, validator_stats> > > results;

    // Create a randomized queue of tiles to work from
    const auto tile_hierarchy = reader.GetTileHierarchy();
    std::deque<GraphId> tempqueue;
    for (auto tier : tile_hierarchy.levels()) {
      auto level = tier.second.level;
      auto tiles = tier.second.tiles;
      for (uint32_t id = 0; id < tiles.TileCount(); id++) {
        // If tile exists add it to the queue
        GraphId tile_id(id, level, 0);
        if (GraphReader::DoesTileExist(tile_hierarchy, tile_id)) {
          tempqueue.push_back(tile_id);
        }
      }
    }
    std::random_shuffle(tempqueue.begin(), tempqueue.end());
    std::queue<GraphId> tilequeue(tempqueue);
    LOG_INFO("Done creating queue of tiles: count = " +
             std::to_string(tilequeue.size()));

    // An atomic object we can use to do the synchronization
    std::mutex lock;

    LOG_INFO("GraphValidator - Validating signs and connectivity");
    // Spawn the threads
    for (auto& thread : threads) {
      results.emplace_back();
      thread.reset(new std::thread(validate, std::cref(hierarchy_properties),
                                   std::ref(tilequeue),
                                   std::ref(lock), std::ref(results.back())));
    }
    // Wait for threads to finish
    for (auto& thread : threads) {
      thread->join();
    }
    // Add up total dupcount_ and find densities
    //TODO modify so that it gets the basic stat class not the tuple
    return_stats stats;
    validator_stats roadStats;
    for (auto& result : results) {
      auto data = result.get_future().get();
      auto returnData = std::get<0>(data);
      stats.add_stat(returnData);
      auto statsData = std::get<1>(data);
      roadStats.add(statsData);
    }

    LOG_INFO("Validation of signs and connectivity is done");
    for (uint8_t level = 0; level <= 2; level++) {
      // Print duplicates info for level
      std::vector<uint32_t> dups = stats.get_dups(level);
      uint32_t dupcount = std::accumulate(dups.begin(), dups.end(), 0);
      LOG_WARN((boost::format("Possible duplicates at level: %1% = %2%")
      % std::to_string(level) % dupcount).str());
      // Get the average density and the max density
      float max_density = 0.0f;
      float sum = 0.0f;
      for (auto density : stats.get_densities(level)) {
        if (density > max_density) {
          max_density = density;
        }
        sum += density;
      }
      float average_density = sum / stats.get_densities(level).size();
      LOG_INFO("Average density = " + std::to_string(average_density) +
               " max = " + std::to_string(max_density));
    }
    bool log_stats = false;
    if (log_stats) {
      std::map<RoadClass, std::string> roadClassToString =
        { {RoadClass::kMotorway, "Motorway"}, {RoadClass::kTrunk, "Trunk"}, {RoadClass::kPrimary, "Primary"},
          {RoadClass::kSecondary, "Secondary"}, {RoadClass::kTertiary, "Tertiary"},
          {RoadClass::kUnclassified, "Unclassified"},{RoadClass::kResidential, "Residential"},
          {RoadClass::kServiceOther, "ServiceOther"}
        };
      auto country_maps = roadStats.get_country_maps();
      for (auto country : roadStats.get_isos()) {
        LOG_INFO("Country: " + country);
        for (auto rclass : roadStats.rclasses) {
          std::string roadStr = roadClassToString[rclass];
          LOG_INFO((boost::format("   %1%: %2% Km") % roadStr % country_maps[country][rclass]).str());
        }
      }
      auto tile_maps = roadStats.get_tile_maps();
      for (auto tileid : roadStats.get_ids()) {
        LOG_INFO("Tile: " + std::to_string(tileid));
        for (auto rclass : roadStats.rclasses) {
          std::string roadStr = roadClassToString[rclass];
          LOG_INFO((boost::format("   %1%: %2% Km") % roadStr % tile_maps[tileid][rclass]).str());
        }
      }
    }
  }
}
}
