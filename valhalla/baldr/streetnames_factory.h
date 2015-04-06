#ifndef VALHALLA_BALDR_STREETNAMES_FACTORY_H_
#define VALHALLA_BALDR_STREETNAMES_FACTORY_H_

#include <vector>
#include <string>
#include <memory>

#include <valhalla/baldr/streetnames.h>

namespace valhalla {
namespace baldr {

class StreetNamesFactory {
 public:
  StreetNamesFactory() = delete;

  static std::unique_ptr<StreetNames> Create(
      const std::string& country_code, const std::vector<std::string>& names);

};

}
}

#endif  // VALHALLA_BALDR_STREETNAMES_FACTORY_H_
