#include "midgard/logging.h"

#include "odin/transitrouteinfo.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

#if 1
template <class T> std::string Get_string(T item) {
  return std::to_string(item);
}

template <> std::string Get_string(std::string item) {
  return "\"" + item + "\"";
}

template <typename T, typename... Args>
std::string Get_string(T item, Args... args) // recursive variadic function
{
  return Get_string(item) + ", " + Get_string(args...);
}
#endif

#ifdef LOGGING_LEVEL_TRACE
std::string TransitRouteInfo::ToParameterString() const {
#if 0
  const std::string delim = ", ";
#endif
  std::string str;
  str += "{ ";

#if 1
  str += Get_string(onestop_id, block_id, trip_id, short_name, long_name, headsign, color, text_color,
                    description, operator_onestop_id, operator_name, operator_url);
#else
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
#endif

  str += " }";

  return str;
}
#endif

} // namespace odin
} // namespace valhalla
