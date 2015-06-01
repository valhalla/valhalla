#include "odin/transitstop.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

TransitStop::TransitStop(std::string n, std::string adt, std::string ddt)
    : name(n),
      arrival_date_time(adt),
      departure_date_time(ddt) {
}

// TODO: do we need?
std::string TransitStop::ToParameterString() const {
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
