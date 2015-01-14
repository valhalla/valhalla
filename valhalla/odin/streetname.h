#ifndef VALHALLA_ODIN_STREETNAME_H_
#define VALHALLA_ODIN_STREETNAME_H_

#include <string>

namespace valhalla{
namespace odin{

class StreetName {
 public:
  StreetName(const std::string& name);

  const std::string& name() const;

  // TODO - add more functionality later

 protected:
  std::string name_;

};

}
}

#endif  // VALHALLA_ODIN_STREETNAME_H_
