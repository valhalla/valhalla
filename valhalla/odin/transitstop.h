#ifndef VALHALLA_ODIN_TRANSIT_STOP_H_
#define VALHALLA_ODIN_TRANSIT_STOP_H_

#include <string>

namespace valhalla {
namespace odin {

struct TransitStop {

  TransitStop(std::string n, std::string adt, std::string ddt);

  // TODO: do we need?
  std::string ToParameterString() const;

  std::string name;
  std::string arrival_date_time;
  std::string departure_date_time;

};

}
}

#endif  // VALHALLA_ODIN_TRANSIT_STOP_H_
