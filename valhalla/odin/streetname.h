#ifndef VALHALLA_ODIN_STREETNAME_H_
#define VALHALLA_ODIN_STREETNAME_H_

#include <string>

namespace valhalla{
namespace odin{

class StreetName {
 public:
  StreetName(const std::string& value);

  const std::string& value() const;

  bool operator ==(const StreetName& rhs) const;

  // TODO - add more functionality later and comments

 protected:
  std::string value_;

};

}
}

#endif  // VALHALLA_ODIN_STREETNAME_H_
