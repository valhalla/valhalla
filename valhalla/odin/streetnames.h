#ifndef VALHALLA_ODIN_STREETNAMES_H_
#define VALHALLA_ODIN_STREETNAMES_H_

#include <list>

#include "odin/streetname.h"

namespace valhalla{
namespace odin{

class StreetNames {
 public:
  StreetNames();

  const std::list<StreetName>& names() const;

  bool empty() const;

  void push_back(const StreetName& street_name);

  // TODO - add more functionality later

 protected:
  std::list<StreetName> names_;

};

}
}

#endif  // VALHALLA_ODIN_STREETNAMES_H_
