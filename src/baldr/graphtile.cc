#include "baldr/graphtile.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

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
      id_() {
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const std::string& basedirectory, const GraphId& graphid)
    : size_(0),
      id_(graphid.Tile_Base()) {

  // Don't bother with invalid ids
  if (!graphid.Is_Valid())
    return;

  // Open to the end of the file so we can immediately get size;
  std::ifstream file(Filename(basedirectory, id_),
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

    // Start of edge information and name list
    edgeinfo_ = graphtile_.get() + header_->edgeinfo_offset();
    textlist_ = graphtile_.get() + header_->textlist_offset();

    textsize_ = filesize - header_->textlist_offset();

    // Set the size to indicate success
    size_ = filesize;
  }
}

GraphTile::~GraphTile() {
}

// Gets the filename given the graphId
std::string GraphTile::Filename(const std::string& basedirectory,
                                const GraphId& graphid) {
  return basedirectory + "/" + FileDirectory(graphid) + "/tile"
      + std::to_string(graphid.tileid()) + ".gph";
}

/**
 * Gets the directory to a given tile.
 * @param  graphid  Graph Id to construct file directory.
 * @return  Returns file directory path relative to tile base directory
 */
std::string GraphTile::FileDirectory(const GraphId& graphid) {
  return std::to_string(graphid.level());
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

const EdgeInfo* GraphTile::edgeinfo(const size_t offset) const {
  return &((reinterpret_cast<EdgeInfo*>(edgeinfo_ + offset))->SetPointers());
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
  const EdgeInfo* edge = edgeinfo(edgeinfo_offset);
  uint32_t namecount = edge->name_count();
  for (uint32_t i = 0; i < namecount; i++) {
    offset = edge->GetStreetNameOffset(i);
    std::cout << i << ":Name Offset = " << offset << " textlist size = "
              << textsize_ << std::endl;
    if (offset < textsize_) {
      names.push_back(textlist_ + offset);
    } else {
      std::cout << "ERROR - offset exceeds size of text list" << std::endl;
    }
  }
  return names;
}

}
}
