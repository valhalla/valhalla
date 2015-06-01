#ifndef VALHALLA_ODIN_TRANSIT_INFO_H_
#define VALHALLA_ODIN_TRANSIT_INFO_H_

#include <string>
#include <list>
#include <valhalla/odin/transitstop.h>

namespace valhalla {
namespace odin {

struct TransitInfo {

  // TODO: do we need?
  std::string ToParameterString() const;

  uint32_t block_id;
  uint32_t trip_id;
  std::string short_name;
  std::string long_name;
  std::string headsign;
  std::list<TransitStop> transit_stops;

};

}
}

#endif  // VALHALLA_ODIN_TRANSIT_INFO_H_
