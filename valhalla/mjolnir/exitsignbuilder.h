#ifndef VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_
#define VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/exitsign.h>


using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Encapsulates the exit sign type and the associated text index.
// Makes base class writable
class ExitSignBuilder : public baldr::ExitSign {
 public:
  // Constructor with both elements
  ExitSignBuilder(const ExitSign::Type& type, uint32_t text_index);

};

}
}

#endif  // VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_
