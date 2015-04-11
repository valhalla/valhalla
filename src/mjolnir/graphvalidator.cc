
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

// Get the GraphId of the opposing edge.
uint32_t GetOpposingEdgeIndex(const GraphId& startnode, DirectedEdge& edge,
    GraphReader& graphreader_, uint32_t& dupcount_, std::string& endnodeiso_) {

  // Get the tile at the end node and get the node info
  GraphId endnode = edge.endnode();
  const GraphTile* tile = graphreader_.GetGraphTile(endnode);
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
    return kMaxEdgesPerNode;
  }
  return opp_index;
}

}

namespace valhalla {
namespace mjolnir {

void GraphValidator::Validate(const boost::property_tree::ptree& pt) {

  // Number of possible duplicates
  uint32_t dupcount_;

  // Graphreader
  GraphReader graphreader_(pt);
  const auto& tile_hierarchy_ = graphreader_.GetTileHierarchy();
  // Make sure there are at least 2 levels!
  if (graphreader_.GetTileHierarchy().levels().size() < 2)
    throw std::runtime_error("Bad tile hierarchy - need 2 levels");

  // Iterate through all levels and all tiles.
  // TODO - concurrency
  LOG_INFO("GraphValidator - validate signs first");

  // Validate signs
  for (auto tile_level :  tile_hierarchy_.levels()) {
    uint32_t level = (uint32_t)tile_level.second.level;
    uint32_t ntiles = tile_level.second.tiles.TileCount();
    for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
      // Get the graph tile. Skip if no tile exists (common case)
      const GraphTile tile(tile_hierarchy_, GraphId(tileid, level, 0));
      if (tile.size() == 0) {
        continue;
      }

      // Iterate through the tile and validate signs
      for (uint32_t i = 0; i < tile.header()->nodecount(); i++) {
        const NodeInfo* nodeinfo = tile.node(i);

        // Go through directed edges and update data
        uint32_t idx = nodeinfo->edge_index();
        for (uint32_t j = 0, n = nodeinfo->edge_count(); j < n; j++, idx++) {
          const DirectedEdge* directededge = tile.directededge(idx);

          // Validate signs
          if (directededge->exitsign()) {
            if (tile.GetSigns(idx).size() == 0) {
              LOG_ERROR("Directed edge marked as having signs but none found");
            }
          }
        }
      }

      // Check if we need to clear the tile cache
      if(graphreader_.OverCommitted())
        graphreader_.Clear();
    }
  }
  LOG_INFO("Validation signs is done. Validate connectivity.");

  for (auto tile_level :  tile_hierarchy_.levels()) {
    dupcount_ = 0;
    uint32_t level = (uint32_t)tile_level.second.level;
    uint32_t ntiles = tile_level.second.tiles.TileCount();
    for (uint32_t tileid = 0; tileid < ntiles; tileid++) {
      // Get the graph tile. Skip if no tile exists (common case)
      GraphTileBuilder tilebuilder(tile_hierarchy_, GraphId(tileid, level, 0));
      if (tilebuilder.size() == 0) {
        continue;
      }

      // Copy existing header. No need to update any counts or offsets.
      GraphTileHeader existinghdr = *(tilebuilder.header());
      const GraphTileHeaderBuilder hdrbuilder =
          static_cast<const GraphTileHeaderBuilder&>(existinghdr);

      // Update nodes and directed edges as needed
      std::vector<NodeInfoBuilder> nodes;
      std::vector<DirectedEdgeBuilder> directededges;

      // Iterate through the nodes and the directed edges
      uint32_t nodecount = tilebuilder.header()->nodecount();
      GraphId node(tileid, level, 0);
      for (uint32_t i = 0; i < nodecount; i++, node++) {

        NodeInfoBuilder nodeinfo = tilebuilder.node(i);

        const GraphTile* tile = graphreader_.GetGraphTile(node);
        std::string begin_node_iso = tile->admin(nodeinfo.admin_index())->country_iso();

    /*    if (nodeinfo.latlng().first == -82.4237543f && nodeinfo.latlng().second == 42.9984804f ) {
          std::cout << "mid good" << std::endl;
          std::cout << begin_node_iso << std::endl;
          std::cout << nodeinfo.admin_index() << std::endl;

        }


        if (nodeinfo.latlng().first == -82.4326549f && nodeinfo.latlng().second == 42.9988345f) {
          std::cout << "left good" << std::endl;
          std::cout << begin_node_iso << std::endl;
          std::cout << nodeinfo.admin_index() << std::endl;

        }


        if (nodeinfo.latlng().first == -82.4135064f && nodeinfo.latlng().second == 42.994748f) {
          std::cout << "right good" << std::endl;
          std::cout << begin_node_iso << std::endl;
          std::cout << nodeinfo.admin_index() << std::endl;

        }

        if (nodeinfo.latlng().first == -82.4134112f && nodeinfo.latlng().second == 42.9948798f) {
           std::cout << "right bad" << std::endl;
           std::cout << begin_node_iso << std::endl;
           std::cout << nodeinfo.admin_index() << std::endl;

         }

         if (nodeinfo.latlng().first == -82.4235694f && nodeinfo.latlng().second == 42.9988248f) {
           std::cout << "mid bad" << std::endl;
           std::cout << begin_node_iso << std::endl;
           std::cout << nodeinfo.admin_index() << std::endl;

         }
         if (nodeinfo.latlng().first == -82.4330586f && nodeinfo.latlng().second == 42.9992422f ) {
           std::cout << "left bad" << std::endl;
           std::cout << begin_node_iso << std::endl;
           std::cout << nodeinfo.admin_index() << std::endl;

         }*/

        // Go through directed edges and update data
        for (uint32_t j = 0, n = nodeinfo.edge_count(); j < n; j++) {
          DirectedEdgeBuilder& directededge = tilebuilder.directededge(
                                  nodeinfo.edge_index() + j);

          std::string end_node_iso_;
          // Set the opposing edge index
          directededge.set_opp_index(GetOpposingEdgeIndex(node, directededge, graphreader_, dupcount_, end_node_iso_));

          // if the country ISO codes do not match then this is a country crossing.
          if (!begin_node_iso.empty() && !end_node_iso_.empty() &&
               begin_node_iso != end_node_iso_) {
            directededge.set_ctry_crossing(true);

            if (begin_node_iso.empty()) {
              std::cout << nodeinfo.latlng().second << "," << nodeinfo.latlng().first << std::endl;
            }

            /*if ((nodeinfo.latlng().first == -82.4237543f && nodeinfo.latlng().second == 42.9984804f ) ||
               (nodeinfo.latlng().first == -82.4326549f && nodeinfo.latlng().second == 42.9988345f) ||
               (nodeinfo.latlng().first == -82.4135064f && nodeinfo.latlng().second == 42.994748f) ||
               (nodeinfo.latlng().first == -82.4134112f && nodeinfo.latlng().second == 42.9948798f) ||
               (nodeinfo.latlng().first == -82.4235694f && nodeinfo.latlng().second == 42.9988248f) ||
               (nodeinfo.latlng().first == -82.4330586f && nodeinfo.latlng().second == 42.9992422f ))
            {
            std::cout << "begin_node_iso " << begin_node_iso << std::endl;
            std::cout << "end_node_iso_ " << end_node_iso_ << std::endl;
            }*/
          }
          //else directededge.set_ctry_crossing(false);



         /* for (const auto name : tile->edgeinfo(directededge.edgeinfo_offset())->GetNames())
                    {

                      if ("Blue Water Bridge" == name) {

                        std::cout << "Greg " << directededge.ctry_crossing() << std::endl;
                        std::cout << "begin_node_iso " << begin_node_iso << std::endl;
                        std::cout << "end_node_iso_ " << end_node_iso_ << std::endl;
                        std::cout << "is shortcut " << directededge.is_shortcut() << std::endl;
                    }
                    }
*/
          directededges.emplace_back(std::move(directededge));

        }

        // Add the node to the list
        nodes.emplace_back(std::move(nodeinfo));
      }

      // Write the new file
      tilebuilder.Update(tile_hierarchy_, hdrbuilder, nodes, directededges);

      // Check if we need to clear the tile cache
      if(graphreader_.OverCommitted())
        graphreader_.Clear();
    }
    LOG_INFO("Validation of connectivity is done");
    LOG_WARN((boost::format("Possible duplicates at level: %1% = %2%")
        % level % dupcount_).str());
  }
}

}
}
