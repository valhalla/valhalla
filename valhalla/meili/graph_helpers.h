// -*- mode: c++ -*-
#ifndef MMP_GRAPH_HELPERS_H_
#define MMP_GRAPH_HELPERS_H_

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphtile.h>


namespace valhalla{

namespace meili {

namespace helpers {

inline const baldr::DirectedEdge*
edge_directededge(baldr::GraphReader& graphreader,
                  const baldr::GraphId& edgeid,
                  // A reference to a pointer to a const tile
                  const baldr::GraphTile*& tile)
{
  if (tile
      && tile->id().tileid() == edgeid.tileid()
      && tile->id().level() == edgeid.level()) {
    return tile->directededge(edgeid);
  } else {
    // Update tile to be edgeid's tile
    tile = graphreader.GetGraphTile(edgeid);
    return tile? tile->directededge(edgeid) : nullptr;
  }
}


inline const baldr::DirectedEdge*
edge_directededge(baldr::GraphReader& graphreader,
                  const baldr::GraphId& edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_directededge(graphreader, edgeid, NO_TILE);
}


inline baldr::GraphId
edge_opp_edgeid(baldr::GraphReader& graphreader,
                const baldr::DirectedEdge* directededge,
                // A reference to a pointer to a const tile
                const baldr::GraphTile*& tile)
{
  if (directededge) {
    // Skip transit line, which has no opposite edge temporally
    if (directededge->IsTransitLine()) {
      return {};
    }
    auto id = directededge->endnode();
    if (!(tile
          && tile->id().tileid() == id.tileid()
          && tile->id().level() == id.level())) {
      // Update tile to be endnode's tile
      tile = graphreader.GetGraphTile(id);
    }
    if (tile) {
      id.fields.id = tile->node(id)->edge_index() + directededge->opp_index();
      return id;
    }
  }
  return {};
}


inline baldr::GraphId
edge_opp_edgeid(baldr::GraphReader& graphreader,
                const baldr::DirectedEdge* directededge)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_opp_edgeid(graphreader, directededge, NO_TILE);
}


inline baldr::GraphId
edge_opp_edgeid(baldr::GraphReader& graphreader,
                const baldr::GraphId& edgeid,
                // A reference to a pointer to a const tile
                const baldr::GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return edge_opp_edgeid(graphreader, directededge, tile);
}


inline baldr::GraphId
edge_opp_edgeid(baldr::GraphReader& graphreader,
                const baldr::GraphId& edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_opp_edgeid(graphreader, edgeid, NO_TILE);
}


inline const baldr::NodeInfo*
edge_nodeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId& nodeid,
              // A reference to a pointer to a const tile
              const baldr::GraphTile*& tile)
{
  if (tile
      && tile->id().tileid() == nodeid.tileid()
      && tile->id().level() == nodeid.level()) {
    return tile->node(nodeid);
  } else {
    // Update tile to be nodeid's tile
    tile = graphreader.GetGraphTile(nodeid);
    return tile? tile->node(nodeid) : nullptr;
  }
}


inline const baldr::NodeInfo*
edge_nodeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId& nodeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_nodeinfo(graphreader, nodeid, NO_TILE);
}


inline baldr::GraphId
edge_endnodeid(baldr::GraphReader& graphreader,
               const baldr::GraphId& edgeid,
               const baldr::GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  if (directededge) {
    return directededge->endnode();
  } else {
    return {};
  }
}


inline baldr::GraphId
edge_endnodeid(baldr::GraphReader& graphreader,
               const baldr::GraphId& edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_endnodeid(graphreader, edgeid, NO_TILE);
}


inline baldr::GraphId
edge_startnodeid(baldr::GraphReader& graphreader,
                 const baldr::GraphId& edgeid,
                 const baldr::GraphTile*& tile)
{
  const auto opp_edgeid = edge_opp_edgeid(graphreader, edgeid, tile);
  if (opp_edgeid.Is_Valid()) {
    const auto directededge = edge_directededge(graphreader, opp_edgeid, tile);
    if (directededge) {
      return directededge->endnode();
    }
  }
  return {};
}


inline baldr::GraphId
edge_startnodeid(baldr::GraphReader& graphreader,
                 const baldr::GraphId& edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_startnodeid(graphreader, edgeid, NO_TILE);
}


inline std::unique_ptr<const baldr::EdgeInfo>
edge_edgeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId& edgeid,
              const baldr::GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  std::unique_ptr<const baldr::EdgeInfo> result;
  if (directededge)
      result.reset(new baldr::EdgeInfo(tile->edgeinfo(directededge->edgeinfo_offset())));
  return result;
}


inline std::unique_ptr<const baldr::EdgeInfo>
edge_edgeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId& edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_edgeinfo(graphreader, edgeid, NO_TILE);
}


}

}

}


#endif // MMP_GRAPH_HELPERS_H_
