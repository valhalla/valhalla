#include "odin/util.h"

namespace valhalla {
namespace odin {

// Returns the specified item surrounded with quotes.
std::string GetQuotedString(const std::string& item) {
  std::string str;
  str += "\"";
  str += item;
  str += "\"";
  return str;
}

}
}
