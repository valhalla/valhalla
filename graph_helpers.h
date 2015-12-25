// -*- mode: c++ -*-


#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphtile.h>


namespace mm {
namespace helpers {

using namespace valhalla;


inline const baldr::DirectedEdge*
edge_directededge(GraphReader& graphreader,
                  const baldr::GraphId edgeid,
                  // A reference to a pointer to a const tile
                  const baldr::GraphTile*& tile)
{
  if (!tile || tile->id() != edgeid.Tile_Base()) {
    tile = graphreader.GetGraphTile(edgeid);
  }
  return tile? tile->directededge(edgeid) : nullptr;
}


inline const baldr::DirectedEdge*
edge_directededge(GraphReader& graphreader,
                  const baldr::GraphId edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_directededge(graphreader, edgeid, NO_TILE);
}


inline baldr::GraphId
edge_endnodeid(baldr::GraphReader& graphreader,
               const baldr::GraphId edgeid,
               const baldr::GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return directededge? directededge->endnode() : GraphId();
}


inline baldr::GraphId
edge_endnodeid(baldr::GraphReader& graphreader,
               const baldr::GraphId edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_endnodeid(graphreader, edgeid, NO_TILE);
}


inline baldr::GraphId
edge_startnodeid(baldr::GraphReader& graphreader,
                 const baldr::GraphId edgeid,
                 const baldr::GraphTile*& tile)
{
  const auto directededge = graphreader.GetOpposingEdge(edgeid, tile);
  return directededge? directededge->endnode() : GraphId();
}


inline baldr::GraphId
edge_startnodeid(baldr::GraphReader& graphreader,
                 const baldr::GraphId edgeid)
{
  const auto directededge = graphreader.GetOpposingEdge(edgeid);
  return directededge? directededge->endnode() : GraphId();
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId edgeid,
              const baldr::GraphTile*& tile)
{
  const auto directededge = edge_directededge(graphreader, edgeid, tile);
  return directededge? tile->edgeinfo(directededge->edgeinfo_offset()) : nullptr;
}


inline std::unique_ptr<const EdgeInfo>
edge_edgeinfo(baldr::GraphReader& graphreader,
              const baldr::GraphId edgeid)
{
  const baldr::GraphTile* NO_TILE = nullptr;
  return edge_edgeinfo(graphreader, edgeid, NO_TILE);
}


}
}
