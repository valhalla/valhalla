#ifndef VALHALLA_BALDR_EXITSIGN_H_
#define VALHALLA_BALDR_EXITSIGN_H_

#include <stdint.h>

#include <valhalla/midgard/util.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

// Encapsulates the exit sign type and the associated text index.
// Read only base class.
class ExitSign {
 public:
  enum class Type : uint32_t {
    kNumber,
    kBranch,
    kToward,
    kName
  };

  /**
   * Get the index of the directed edge this exit sign applies to.
   * @return  Returns the directed edge index (within the same tile
   *          as the exit information).
   */
  uint32_t edgeindex() const;

  // Returns the exit sign type
  const ExitSign::Type& type() const;

  // Returns the text index
  const uint32_t text_offset() const;

 protected:
  // Constructor
  ExitSign(const uint32_t idx, const ExitSign::Type& type,
           const uint32_t text_offset);

  uint32_t edgeindex_;
  ExitSign::Type type_;
  uint32_t text_offset_;
};

}
}

#endif  // VALHALLA_BALDR_EXITSIGN_H_
