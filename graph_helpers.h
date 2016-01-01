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
                  const GraphId edgeid,
                  // A reference to a pointer to a const tile
                  const GraphTile*& tile)
{
  if (!tile || tile->id() != edgeid.Tile_Base()) {
    tile = graphreader.GetGraphTile(edgeid);
  }
  return tile? tile->directededge(edgeid) : nullptr;
}


inline const DirectedEdge*
edge_directededge(GraphReader& graphreader,
                  const GraphId edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_directededge(graphreader, edgeid, NO_TILE);
}


inline GraphId
edge_endnodeid(GraphReader& graphreader,
               const GraphId edgeid,
               const GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return directededge? directededge->endnode() : GraphId();
}


inline GraphId
edge_endnodeid(GraphReader& graphreader,
               const GraphId edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_endnodeid(graphreader, edgeid, NO_TILE);
}


inline GraphId
edge_startnodeid(GraphReader& graphreader,
                 const GraphId edgeid,
                 const GraphTile*& tile)
{
  const auto directededge = graphreader.GetOpposingEdge(edgeid, tile);
  return directededge? directededge->endnode() : GraphId();
}


inline GraphId
edge_startnodeid(GraphReader& graphreader,
                 const GraphId edgeid)
{
  const auto directededge = graphreader.GetOpposingEdge(edgeid);
  return directededge? directededge->endnode() : GraphId();
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(GraphReader& graphreader,
              const GraphId edgeid,
              const GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return directededge? tile->edgeinfo(directededge->edgeinfo_offset()) : nullptr;
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(GraphReader& graphreader,
              const GraphId edgeid)
{
  const GraphTile* NO_TILE = nullptr;
  return edge_edgeinfo(graphreader, edgeid, NO_TILE);
}

}
}


#endif // MM_GRAPH_HELPERS_H_
