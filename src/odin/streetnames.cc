#include <iostream>

#include "odin/streetnames.h"

#include <valhalla/baldr/streetnames.h>

namespace valhalla {
namespace odin {

StreetNames::StreetNames()
    : baldr::StreetNames() {
}

StreetNames::StreetNames(
    const ::google::protobuf::RepeatedPtrField<::std::string>& names)
    : baldr::StreetNames() {
  for (const auto& name : names) {
    this->emplace_back(name);
  }

}

}
}
