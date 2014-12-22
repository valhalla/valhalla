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
  enum class Type {
    kNumber,
    KBranch,
    kToward,
    kName
  };

  // Returns the exit sign type
  const ExitSign::Type& type() const;

  // Returns the text index
  const uint32_t text_offset() const;

 protected:
  // Constructor
  ExitSign(const ExitSign::Type& type, uint32_t text_offset);

  ExitSign::Type type_;
  uint32_t text_offset_;
};

}
}

#endif  // VALHALLA_BALDR_EXITSIGN_H_
