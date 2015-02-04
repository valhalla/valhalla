#include "mjolnir/exitsignbuilder.h"

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Constructor with arguments
ExitSignBuilder::ExitSignBuilder(const uint32_t idx,
                                 const ExitSign::Type& type,
                                 const uint32_t text_offset)
    : ExitSign(idx, type, text_offset) {
}

}
}
