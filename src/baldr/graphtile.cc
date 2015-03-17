#include "baldr/graphtile.h"
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>

#include <ctime>
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
  template <class numeric_t>
  size_t digits(numeric_t number) {
    size_t digits = (number < 0 ? 1 : 0);
    while (static_cast<long long int>(number)) {
        number /= 10;
        digits++;
    }
    return digits;
  }
  const std::locale dir_locale(std::locale("C"), new dir_facet());
  const AABB2 world_box(PointLL(-180, -90), PointLL(180, 90));
}

namespace valhalla {
namespace baldr {

// Default constructor
GraphTile::GraphTile()
    : size_(0),
      header_(nullptr),
      nodes_(nullptr),
      directededges_(nullptr),
      signs_(nullptr),
      edgeinfo_(nullptr),
      textlist_(nullptr),
      admininfo_(nullptr),
      edgeinfo_size_(0),
      textlist_size_(0),
      admininfo_size_(0){
}

// Constructor given a filename. Reads the graph data into memory.
GraphTile::GraphTile(const TileHierarchy& hierarchy, const GraphId& graphid)
    : size_(0) {

  // Don't bother with invalid ids
  if (!graphid.Is_Valid())
    return;

  // Open to the end of the file so we can immediately get size;
  std::string file_location = hierarchy.tile_dir() + "/" +
                FileSuffix(graphid.Tile_Base(), hierarchy);
  std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
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

    // Check internal version
    const GraphTileHeader gh;
    if (header_->internal_version() != gh.internal_version()) {
      LOG_ERROR("Version of tile is out of date or not supported!");
      return;
    }

    // Set a pointer to the node list
   nodes_ = reinterpret_cast<NodeInfo*>(ptr);
   ptr += header_->nodecount() * sizeof(NodeInfo);
   // Set a pointer to the directed edge list
   directededges_ = reinterpret_cast<DirectedEdge*>(ptr);
   ptr += header_->directededgecount() * sizeof(DirectedEdge);
   // Set a pointer to the sign list
   signs_ = reinterpret_cast<Sign*>(ptr);
   ptr += header_->signcount() * sizeof(Sign);

   // Start of edge information and its size
   edgeinfo_ = graphtile_.get() + header_->edgeinfo_offset();
   edgeinfo_size_ = header_->admininfo_offset() - header_->edgeinfo_offset();

   admininfo_ = graphtile_.get() + header_->admininfo_offset();
   admininfo_size_ = header_->textlist_offset() - header_->admininfo_offset();

   // Start of text list and its size
   // TODO - need to adjust the size once Exits are added
   textlist_ = graphtile_.get() + header_->textlist_offset();
   textlist_size_ = filesize - header_->textlist_offset();

   // Set the size to indicate success
   size_ = filesize;
  }
  else {
    LOG_DEBUG("Tile " + file_location + " was not found");
  }
}

GraphTile::~GraphTile() {
}

std::string GraphTile::FileSuffix(const GraphId& graphid, const TileHierarchy& hierarchy) {
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
  const auto level = hierarchy.levels().find(graphid.level());
  if(level == hierarchy.levels().end())
    throw std::runtime_error("Could not compute FileSuffix for non-existent level");
  const uint32_t max_id = Tiles::MaxTileId(world_box, level->second.tiles.TileSize());

  //figure out how many digits
  //TODO: dont convert it to a string to get the length there are faster ways..
  size_t max_length = digits<uint32_t>(max_id);
  const size_t remainder = max_length % 3;
  if(remainder)
    max_length += 3 - remainder;

  //make a locale to use as a formatter for numbers
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
  return header_->graphid();
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

// Get the directed edge given a GraphId
const DirectedEdge* GraphTile::directededge(const GraphId& edge) const {
  if (edge.id() < header_->directededgecount())
    return &directededges_[edge.id()];
  throw std::runtime_error("GraphTile DirectedEdge id out of bounds");
}

// Get the directed edge at the specified index.
const DirectedEdge* GraphTile::directededge(const size_t idx) const {
  if (idx < header_->directededgecount())
    return &directededges_[idx];
  throw std::runtime_error("GraphTile DirectedEdge index out of bounds");
}

std::unique_ptr<const EdgeInfo> GraphTile::edgeinfo(const size_t offset) const {
  return std::unique_ptr<EdgeInfo>(new EdgeInfo(edgeinfo_ + offset, textlist_, textlist_size_));
}

std::unique_ptr<const AdminInfo> GraphTile::admininfo(const size_t offset) const {
  return std::unique_ptr<AdminInfo>(new AdminInfo(admininfo_ + offset, textlist_, textlist_size_));
}

// Get the directed edges outbound from the specified node index.
const DirectedEdge* GraphTile::GetDirectedEdges(const uint32_t node_index,
                                                uint32_t& count,
                                                uint32_t& edge_index) const {
  const NodeInfo* nodeinfo = node(node_index);
  count = nodeinfo->edge_count();
  edge_index = nodeinfo->edge_index();
  return directededge(nodeinfo->edge_index());
}

// Convenience method to get the admin names given the offset to the admin info
std::vector<std::string> GraphTile::GetAdminNames(const uint32_t admininfo_offset) const {
  return admininfo(admininfo_offset)->GetNames();
}

// Convenience method to get the names for an edge given the offset to the
// edge info
std::vector<std::string> GraphTile::GetNames(const uint32_t edgeinfo_offset) const {
  return edgeinfo(edgeinfo_offset)->GetNames();
}

// Convenience method to get the signs for an edge given the
// directed edge index.
std::vector<SignInfo> GraphTile::GetSigns(const uint32_t idx) const {
  uint32_t count = header_->signcount();
  std::vector<SignInfo> signs;
  if (count == 0) {
    return signs;
  }

  // Binary search
  int32_t low = 0;
  int32_t high = count-1;
  int32_t mid;
  bool found = false;
  while (low <= high) {
    mid = (low + high) / 2;
    if (signs_[mid].edgeindex() == idx) {
      found = true;
      break;
    }
    if (idx < signs_[mid].edgeindex() ) {
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }

  if (found) {
    // Back up while prior is equal (or at the beginning)
    while (mid > 0 && signs_[mid-1].edgeindex() == idx) {
      mid--;
    }

    // Add signs
    while (signs_[mid].edgeindex() == idx && mid < count) {
      if (signs_[mid].text_offset() < textlist_size_) {
        signs.emplace_back(signs_[mid].type(),
                (textlist_ + signs_[mid].text_offset()));
      } else {
        throw std::runtime_error("GetSigns: offset exceeds size of text list");
      }
      mid++;
    }
  }
  if (signs.size() == 0) {
    LOG_ERROR("No signs found for idx = " + std::to_string(idx));
  }
  return signs;
}

}
}
