// -*- mode: c++ -*-
#ifndef MM_GRAPH_HELPERS_H_
#define MM_GRAPH_HELPERS_H_


#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphtile.h>


namespace mm {
namespace helpers {

using namespace valhalla::baldr;


inline const DirectedEdge*
edge_directededge(GraphReader& graphreader,
                  const GraphId& edgeid,
                  // A reference to a pointer to a const tile
                  const GraphTile*& tile)
{
  if (tile && tile->id().tileid() == edgeid.tileid()) {
    return tile->directededge(edgeid);
  } else {
    tile = graphreader.GetGraphTile(edgeid);
    return tile? tile->directededge(edgeid) : nullptr;
  }
}


inline const DirectedEdge*
edge_directededge(GraphReader& graphreader,
                  const GraphId& edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_directededge(graphreader, edgeid, NO_TILE);
}


inline GraphId
edge_opp_edgeid(GraphReader& graphreader,
                const DirectedEdge* directededge,
                // A reference to a pointer to a const tile
                const GraphTile*& tile)
{
  if (directededge) {
    auto nodeid = directededge->endnode();
    if (!tile || tile->id().tileid() != nodeid.tileid()) {
      tile = graphreader.GetGraphTile(nodeid);
    }
    if (tile) {
      nodeid.fields.id = tile->node(nodeid)->edge_index() + directededge->opp_index();
      return nodeid;
    }
  }
  return {};
}


inline GraphId
edge_opp_edgeid(GraphReader& graphreader,
                const DirectedEdge* directededge)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_opp_edgeid(graphreader, directededge, NO_TILE);
}


inline GraphId
edge_opp_edgeid(GraphReader& graphreader,
                const GraphId& edgeid,
                // A reference to a pointer to a const tile
                const GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return edge_opp_edgeid(graphreader, directededge, tile);
}


inline GraphId
edge_opp_edgeid(GraphReader& graphreader,
                const GraphId& edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_opp_edgeid(graphreader, edgeid, NO_TILE);
}


inline const NodeInfo*
edge_nodeinfo(GraphReader& graphreader,
              const GraphId& nodeid,
              // A reference to a pointer to a const tile
              const GraphTile*& tile)
{
  if (tile && tile->id().tileid() == nodeid.tileid()) {
    return tile->node(nodeid);
  } else {
    tile = graphreader.GetGraphTile(nodeid);
    return tile? tile->node(nodeid) : nullptr;
  }
}


inline const NodeInfo*
edge_nodeinfo(GraphReader& graphreader,
              const GraphId& nodeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_nodeinfo(graphreader, nodeid, NO_TILE);
}


inline GraphId
edge_endnodeid(GraphReader& graphreader,
               const GraphId& edgeid,
               const GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  if (directededge) {
    return directededge->endnode();
  } else {
    return {};
  }
}


inline GraphId
edge_endnodeid(GraphReader& graphreader,
               const GraphId& edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_endnodeid(graphreader, edgeid, NO_TILE);
}


inline GraphId
edge_startnodeid(GraphReader& graphreader,
                 const GraphId& edgeid,
                 const GraphTile*& tile)
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


inline GraphId
edge_startnodeid(GraphReader& graphreader,
                 const GraphId& edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_startnodeid(graphreader, edgeid, NO_TILE);
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(GraphReader& graphreader,
              const GraphId& edgeid,
              const GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return directededge? tile->edgeinfo(directededge->edgeinfo_offset()) : nullptr;
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(GraphReader& graphreader,
              const GraphId& edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_edgeinfo(graphreader, edgeid, NO_TILE);
}

}
}


#endif // MM_GRAPH_HELPERS_H_
