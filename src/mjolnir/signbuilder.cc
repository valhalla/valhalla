#include "../../valhalla/mjolnir/signbuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor with arguments
SignBuilder::SignBuilder(const uint32_t idx,
                                 const Sign::Type& type,
                                 const uint32_t text_offset)
    : Sign(idx, type, text_offset) {
}

// Set the directed edge index.
void SignBuilder::set_edgeindex(const uint32_t idx) {
  data_.edgeindex = idx;
}

}
}
