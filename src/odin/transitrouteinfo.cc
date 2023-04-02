#include "midgard/logging.h"

#include "odin/transitrouteinfo.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {
#ifdef LOGGING_LEVEL_TRACE
std::string TransitRouteInfo::ToParameterString() const {
  return "{ " +
         Get_String(", ", onestop_id, block_id, trip_id, short_name, long_name, headsign, color,
                    text_color, description, operator_onestop_id, operator_name, operator_url) +
         " }";
}
#endif

} // namespace odin
} // namespace valhalla
