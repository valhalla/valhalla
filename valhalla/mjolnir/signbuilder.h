#ifndef VALHALLA_MJOLNIR_SIGNBUILDER_H_
#define VALHALLA_MJOLNIR_SIGNBUILDER_H_

#include <valhalla/baldr/sign.h>
#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

/**
 * Generic sign builder. Encapsulates the sign type and the associated
 * text index. Includes the directed edge index - signs are accessed via
 * the directed edge to which they apply. Makes base class writable.
 */
class SignBuilder : public baldr::Sign {
 public:
  /**
   * Constructor with arguments.
   * @param  idx  Index of the directed edge to which this sign applies.
   * @param  type Type of sign.
   * @param  text_offset  Offset within the text/name list for the sign text.
   */
  SignBuilder(const uint32_t idx, const baldr::Sign::Type& type,
                  const uint32_t text_offset);

  /**
   * Set the directed edge index.
   * @param  idx  Directed edge index.
   */
  void set_edgeindex(const uint32_t idx);
};

}
}

#endif  // VALHALLA_MJOLNIR_SIGNBUILDER_H_
