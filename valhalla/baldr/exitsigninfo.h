#ifndef VALHALLA_BALDR_EXITSIGNINFO_H_
#define VALHALLA_BALDR_EXITSIGNINFO_H_

#include <valhalla/baldr/exitsign.h>

namespace valhalla {
namespace baldr {

// Encapsulates the exit sign type and the associated text.
class ExitSignInfo {
 public:
  // Constructor
  ExitSignInfo(const ExitSign::Type& type, const std::string& text);

  // Returns the exit sign type
  const ExitSign::Type& type() const;

  // Returns the text index
  const std::string& text() const;

 protected:
  ExitSign::Type type_;
  std::string text_;
};

}
}

#endif  // VALHALLA_BALDR_EXITSIGNINFO_H_
