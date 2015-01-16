#ifndef VALHALLA_ODIN_STREETNAMES_H_
#define VALHALLA_ODIN_STREETNAMES_H_

#include <list>

#include "odin/streetname.h"

namespace valhalla{
namespace odin{

class StreetNames : public std::list<StreetName> {
 public:
  StreetNames();

  std::string ToString() const;

  // TODO - add more functionality later

 protected:

};

}
}

#endif  // VALHALLA_ODIN_STREETNAMES_H_
