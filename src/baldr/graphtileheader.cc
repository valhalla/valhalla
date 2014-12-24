#include "baldr/graphtileheader.h"

namespace valhalla{
namespace baldr{

GraphTileHeader::GraphTileHeader()
    : nodecount_(0),
      directededgecount_(0),
      edgeinfo_offset_(0),
      textlist_offset_(0) {
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

}
}
