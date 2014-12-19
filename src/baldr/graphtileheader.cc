#include "baldr/graphtileheader.h"

namespace valhalla{
namespace baldr{

GraphTileHeader::GraphTileHeader()
    : nodecount_(0),
      directededgecount_(0),
      edgeinfo_offset_(0),
      textlist_offset_(0) {
}

unsigned int GraphTileHeader::nodecount() const {
  return nodecount_;
}

unsigned int GraphTileHeader::directededgecount() const {
  return directededgecount_;
}

unsigned int GraphTileHeader::edgeinfo_offset() const {
  return edgeinfo_offset_;
}

unsigned int GraphTileHeader::textlist_offset() const {
  return textlist_offset_;
}

}
}
