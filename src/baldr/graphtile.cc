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
      namelist_(nullptr) {
}

// Constructor given a filename. Reads the graph data into
// memory.
GraphTile::GraphTile(const std::string& filename) {
  // Open to the end of the file so we can immediately get size;
  std::ifstream file(filename, std::ios::in|std::ios::binary|std::ios::ate);
  if (file.is_open()) {
    // Read binary file into memory. TODO - protect against failure to
    // allocate memory
    int filesize = file.tellg();
    graphtile_ = new char[filesize];
    file.seekg (0, std::ios::beg);
    file.read(graphtile_, filesize);
    file.close();

    // Set a pointer to the header (first structure in the binary data).
    char* ptr = graphtile_;
    header_ = (GraphTileHeader*)graphtile_;
    ptr += sizeof graphtile_;

    // Set a pointer to the node list
    nodes_ = (NodeInfo*)ptr;
    ptr += header_->nodecount() * sizeof(NodeInfo);

    // Set a pointer to the directed edge list
    directededges_ = (DirectedEdge*)ptr;
    ptr += header_->directededgecount() * sizeof(DirectedEdge);

    // Start of edge information and name list
    edgeinfo_ = graphtile_ + header_->edgeinfo_offset();
    namelist_ = graphtile_ + header_->namelist_offset();
  }
  else {
    // TODO - error. Distinguish between file not found vs. a file read error?
    size_ = -1;
  }
}

GraphTile::~GraphTile() {
   delete[] graphtile_;
}

int GraphTile::size() const {
  return size_;
}

const GraphTileHeader* GraphTile::header() const {
  return header_;
}

const NodeInfo* GraphTile::node(const GraphId& node) const {
  // TODO - do we want to validate the tile and level - or just assume
  // the correct tile is called? Validate we are not overflowing the list?
  return nodes_ + node.id();
}

const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  return directededges_ + edge.id();
}

EdgeInfo* GraphTile::edgeinfo() const {
  // TODO - how do we call this - with an offset (from a directed edge?)
  return nullptr;
}

}
}
