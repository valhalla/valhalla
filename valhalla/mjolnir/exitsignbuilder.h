#ifndef VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_
#define VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/exitsign.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

// Encapsulates the exit sign type and the associated text index.
// Includes the directed edge index - exit signs are accessed via
// the directed edge to which they apply.
// Makes base class writable
class ExitSignBuilder : public baldr::ExitSign {
 public:
  /**
   * Constructor with arguments.
   * @param  idx  Index of the directed edge to which this sign applies.
   * @param  type Type of exit sign.
   * @param  text_offset  Offset within the text/name list for the sign text.
   */
  ExitSignBuilder(const uint32_t idx, const ExitSign::Type& type,
                  const uint32_t text_offset);

};

}
}

#endif  // VALHALLA_MJOLNIR_EXITSIGNBUILDER_H_
