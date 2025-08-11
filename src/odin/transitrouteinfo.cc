#include "odin/transitrouteinfo.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

#ifdef LOGGING_LEVEL_TRACE
std::string TransitRouteInfo::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";

  str += GetQuotedString(onestop_id);

  str += delim;
  str += std::to_string(block_id);

  str += delim;
  str += std::to_string(trip_id);

  str += delim;
  str += GetQuotedString(short_name);

  str += delim;
  str += GetQuotedString(long_name);

  str += delim;
  str += GetQuotedString(headsign);

  str += delim;
  str += std::to_string(color);

  str += delim;
  str += std::to_string(text_color);

  str += delim;
  str += GetQuotedString(description);

  str += delim;
  str += GetQuotedString(operator_onestop_id);

  str += delim;
  str += GetQuotedString(operator_name);

  str += delim;
  str += GetQuotedString(operator_url);

  str += " }";

  return str;
}
#endif

} // namespace odin
} // namespace valhalla
