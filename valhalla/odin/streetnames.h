#ifndef VALHALLA_ODIN_STREETNAMES_H_
#define VALHALLA_ODIN_STREETNAMES_H_

#include <list>

#include <google/protobuf/repeated_field.h>

#include <valhalla/baldr/streetnames.h>

namespace valhalla {
namespace odin {

class StreetNames : public baldr::StreetNames {
 public:
  StreetNames();

  StreetNames(const ::google::protobuf::RepeatedPtrField<::std::string>& names);

 protected:

};

}
}

#endif  // VALHALLA_ODIN_STREETNAMES_H_
