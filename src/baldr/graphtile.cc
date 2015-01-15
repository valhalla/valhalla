#include "baldr/graphtile.h"
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <locale>
#include <iomanip>
#include <cmath>

namespace {
  struct dir_facet : public std::numpunct<char> {
   protected:
    virtual char do_thousands_sep() const {
        return '/';
    }

    virtual std::string do_grouping() const {
        return "\03";
    }
  };
}

namespace valhalla {
namespace baldr {

// Default constructor
GraphTile::GraphTile()
    : size_(0),
      header_(nullptr),
      nodes_(nullptr),
      directededges_(nullptr),
      edgeinfo_(nullptr),
      textlist_(nullptr),
      edgeinfo_size_(0),
      textlist_size_(0),
      id_() {
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const TileHierarchy& hierarchy, const GraphId& graphid)
    : size_(0),
      id_(graphid.Tile_Base()) {

  // Don't bother with invalid ids
  if (!graphid.Is_Valid())
    return;

  // Open to the end of the file so we can immediately get size;
  std::ifstream file(hierarchy.tile_dir() + "/" + FileSuffix(id_, hierarchy),
                     std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to
    // allocate memory
    size_t filesize = file.tellg();
    graphtile_.reset(new char[filesize]);
    file.seekg(0, std::ios::beg);
    file.read(graphtile_.get(), filesize);
    file.close();

    // Set a pointer to the header (first structure in the binary data).
    char* ptr = graphtile_.get();
    header_ = reinterpret_cast<GraphTileHeader*>(ptr);
    ptr += sizeof(GraphTileHeader);

    // Set a pointer to the node list
    nodes_ = reinterpret_cast<NodeInfo*>(ptr);
    ptr += header_->nodecount() * sizeof(NodeInfo);

    // Set a pointer to the directed edge list
    directededges_ = reinterpret_cast<DirectedEdge*>(ptr);
    ptr += header_->directededgecount() * sizeof(DirectedEdge);

    // Start of edge information and its size
    edgeinfo_ = graphtile_.get() + header_->edgeinfo_offset();
    edgeinfo_size_ = header_->textlist_offset() - header_->edgeinfo_offset();

    // Start of text list and its size
    // TODO - need to adjust the size once Exits are added
    textlist_ = graphtile_.get() + header_->textlist_offset();
    textlist_size_ = filesize - header_->textlist_offset();

    // Set the size to indicate success
    size_ = filesize;
  } else {
//    std::cout << "Tile for " << Filename(basedirectory, id_) << " not found" << std::endl;
  }
}

GraphTile::~GraphTile() {
}

std::string GraphTile::FileSuffix(const GraphId& graphid, const TileHierarchy& heirarchy) {
  /*
  if you have a graphid where level == 8 and tileid == 24134109851
  you should get: 8/024/134/109/851.gph
  since the number of levels is likely to be very small this limits
  the total number of objects in any one directory to 1000, which is an
  empirically derived good choice for mechanical harddrives
  this should be fine for s3 (even though it breaks the rule of most
  unique part of filename first) because there will be just so few
  objects in general in practice
  */

  //figure the largest id for this level
  auto level = heirarchy.levels().find(graphid.level());
  if(level == heirarchy.levels().end())
    throw std::runtime_error("Could not compute FileSuffix for non-existent level");
  uint32_t max_id = Tiles::MaxTileId(AABB2(PointLL(-180, -90), PointLL(180, 90)), level->second.tiles.TileSize());

  //figure out how many digits
  //TODO: dont convert it to a string to get the length there are faster ways..
  size_t max_length = std::to_string(max_id).length();
  size_t remainder = max_length % 3;
  if(remainder)
    max_length += 3 - remainder;

  //make a locale to use as a formatter for numbers
  std::locale dir_locale(std::locale("C"), new dir_facet());
  std::ostringstream stream;
  stream.imbue(dir_locale);

  //if it starts with a zero the pow trick doesn't work
  if(graphid.level() == 0) {
    stream << static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
    std::string suffix = stream.str();
    suffix[0] = '0';
    return suffix;
  }
  //it was something else
  stream << graphid.level() * static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
  return stream.str();
}

size_t GraphTile::size() const {
  return size_;
}

GraphId GraphTile::id() const {
  return id_;
}

const GraphTileHeader* GraphTile::header() const {
  return header_;
}

const NodeInfo* GraphTile::node(const GraphId& node) const {
  if (node.id() < header_->nodecount())
    return &nodes_[node.id()];
  throw std::runtime_error("GraphTile NodeInfo id out of bounds");
}

const NodeInfo* GraphTile::node(const size_t idx) const {
  if (idx < header_->nodecount())
    return &nodes_[idx];
  throw std::runtime_error("GraphTile NodeInfo index out of bounds");
}

const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  if (edge.id() < header_->directededgecount())
    return &directededges_[edge.id()];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

const DirectedEdge* GraphTile::directededge(const size_t idx) const {
  if (idx < header_->directededgecount())
    return &directededges_[idx];
  throw std::runtime_error("GraphTile DirectedEdge index out of bounds");
}

const std::shared_ptr<EdgeInfo> GraphTile::edgeinfo(const size_t offset) const {
  return std::make_shared<EdgeInfo>(edgeinfo_ + offset);
}

const DirectedEdge* GraphTile::GetDirectedEdges(const uint32_t node_index,
                                                uint32_t& count,
                                                uint32_t& edge_index) {
  const NodeInfo* nodeinfo = node(node_index);
  count = nodeinfo->edge_count();
  edge_index = nodeinfo->edge_index();
  return directededge(nodeinfo->edge_index());
}

// Convenience method to get the names for an edge.
std::vector<std::string>& GraphTile::GetNames(const uint32_t edgeinfo_offset,
                                              std::vector<std::string>& names) {
  // Get each name
  names.clear();
  uint32_t offset;
  const std::shared_ptr<EdgeInfo> edge = edgeinfo(edgeinfo_offset);
  uint32_t namecount = edge->name_count();
  for (uint32_t i = 0; i < namecount; i++) {
    offset = edge->GetStreetNameOffset(i);
    if (offset < textlist_size_) {
      names.push_back(textlist_ + offset);
    } else {
      std::cout << "ERROR - offset exceeds size of text list" << std::endl;
    }
  }
  return names;
}

}
}
