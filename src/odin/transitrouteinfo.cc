#include "midgard/logging.h"

#include "odin/transitrouteinfo.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {
#ifdef LOGGING_LEVEL_TRACE
std::string TransitRouteInfo::ToParameterString() const {
  std::string str;
  str.reserve(256);
  str += "{ ";
  Get_String(str, ", ", onestop_id, block_id, trip_id, short_name, long_name, headsign, color,
             text_color, description, operator_onestop_id, operator_name, operator_url);
  str += " }";
  return str;
}
#endif

} // namespace odin
} // namespace valhalla
