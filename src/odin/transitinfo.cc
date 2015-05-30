#include "odin/transitinfo.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

// TODO: do we need?
std::string TransitInfo::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";
  //str += GetQuotedString(text_);

  str += delim;
  //str += GetQuotedString(std::to_string(consecutive_count_));

  str += " }";

  return str;
}

}
}
