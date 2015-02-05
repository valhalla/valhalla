#include "baldr/exitsigninfo.h"

namespace valhalla {
namespace baldr {

const ExitSign::Type& ExitSignInfo::type() const {
  return type_;
}

const std::string& ExitSignInfo::text() const {
  return text_;
}

ExitSignInfo::ExitSignInfo(const ExitSign::Type& type, const std::string& text)
    : type_(type),
      text_(text) {
}

}
}
