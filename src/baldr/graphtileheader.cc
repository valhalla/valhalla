#include "baldr/graphtileheader.h"
#include "baldr/nodeinfo.h"
#include "baldr/directededge.h"
#include "baldr/edgeinfo.h"
#include "config.h"

namespace valhalla{
namespace baldr{

GraphTileHeader::GraphTileHeader()
    : nodecount_(0),
      directededgecount_(0),
      edgeinfo_offset_(0),
      textlist_offset_(0),
      internal_version_(0) {

  NodeInfo n;
  DirectedEdge de;
  GraphId id;

  internal_version_ = n.hash_value() + de.hash_value() + id.hash_value();

  version_ = PACKAGE_VERSION;

}

size_t GraphTileHeader::nodecount() const {
  return nodecount_;
}

size_t GraphTileHeader::directededgecount() const {
  return directededgecount_;
}

size_t GraphTileHeader::edgeinfo_offset() const {
  return edgeinfo_offset_;
}

size_t GraphTileHeader::textlist_offset() const {
  return textlist_offset_;
}

int64_t GraphTileHeader::internal_version() const {
  return internal_version_;
}

std::string GraphTileHeader::version() const {
  return version_;
}

}
}
