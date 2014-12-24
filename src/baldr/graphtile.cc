#include "baldr/graphtile.h"

#include <string>
#include <iostream>
#include <fstream>

namespace valhalla{
namespace baldr{

// Default constructor
GraphTile::GraphTile()
    : graphtile_(nullptr),
      size_(0),
      header_(nullptr),
      nodes_(nullptr),
      directededges_(nullptr),
      edgeinfo_(nullptr),
      textlist_(nullptr),
      id_(){
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const std::string& basedirectory, const GraphId& graphid)
  :id_(graphid.tileid(), graphid.level(), 0) {

  // Open to the end of the file so we can immediately get size;
  std::ifstream file(Filename(basedirectory, id_),
          std::ios::in|std::ios::binary|std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to
    // allocate memory
    size_t filesize = file.tellg();
    graphtile_ = new char[filesize];
    file.seekg (0, std::ios::beg);
    file.read(graphtile_, filesize);
    file.close();

    // Set a pointer to the header (first structure in the binary data).
    char* ptr = graphtile_;
    header_ = reinterpret_cast<GraphTileHeader*>(graphtile_);
    ptr += sizeof(GraphTileHeader);

    // Set a pointer to the node list
    nodes_ = reinterpret_cast<NodeInfo*>(ptr);
    ptr += header_->nodecount() * sizeof(NodeInfo);

    // Set a pointer to the directed edge list
    directededges_ = reinterpret_cast<DirectedEdge*>(ptr);
    ptr += header_->directededgecount() * sizeof(DirectedEdge);

    // Start of edge information and name list
    edgeinfo_ = graphtile_ + header_->edgeinfo_offset();
    textlist_ = graphtile_ + header_->textlist_offset();
  }
  else {
    // TODO - error. Distinguish between file not found vs. a file read error?
    size_ = 0;
  }
}

GraphTile::~GraphTile() {
   delete[] graphtile_;
}

// Gets the filename given the graphId
std::string GraphTile::Filename(const std::string& basedirectory,
               const GraphId& graphid) {
  return basedirectory + "/" + FileDirectory(graphid) +
      "/tile" + std::to_string(graphid.tileid()) + ".gph";
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
  if(node.id() < header_->nodecount())
    return &nodes_[node.id()];
  throw std::runtime_error("GraphTile NodeInfo id out of bounds");
}

const NodeInfo* GraphTile::node(const size_t idx) const {
  if(idx < header_->nodecount())
    return &nodes_[idx];
  throw std::runtime_error("GraphTile NodeInfo index out of bounds");
}

const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  if(edge.id() < header_->directededgecount())
    return &directededges_[edge.id()];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

const DirectedEdge* GraphTile::directededge(const size_t idx) const {
  if(idx < header_->directededgecount())
    return &directededges_[idx];
  throw std::runtime_error("GraphTile DirectedEdge index out of bounds");
}

const EdgeInfo* GraphTile::edgeinfo(uint32_t offset) const {
  return reinterpret_cast<const EdgeInfo*>(edgeinfo_ + offset);
}

}
}
