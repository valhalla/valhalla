#ifndef VALHALLA_ODIN_STREETNAMES_H_
#define VALHALLA_ODIN_STREETNAMES_H_

#include <list>

#include <google/protobuf/repeated_field.h>

#include <valhalla/odin/streetname.h>

namespace valhalla {
namespace odin {

class StreetNames : public std::list<StreetName> {
 public:
  StreetNames();

  StreetNames(const ::google::protobuf::RepeatedPtrField<::std::string>& names);

  std::string ToString() const;

  StreetNames FindCommonStreetNames(StreetNames other) const;

  // TODO - add more functionality later

 protected:

};

}
}

#endif  // VALHALLA_ODIN_STREETNAMES_H_
