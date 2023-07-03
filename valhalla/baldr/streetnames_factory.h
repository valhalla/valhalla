#ifndef VALHALLA_BALDR_STREETNAMES_FACTORY_H_
#define VALHALLA_BALDR_STREETNAMES_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <valhalla/baldr/streetnames.h>
#include <valhalla/proto/common.pb.h>

namespace valhalla {
namespace baldr {

class StreetNamesFactory {
public:
  StreetNamesFactory() = delete;

  static std::unique_ptr<StreetNames> Create(const std::string& country_code,
                                             const std::vector<std::pair<std::string, bool>>& names);

  static std::unique_ptr<StreetNames>
  Create(const std::string& country_code,
         const google::protobuf::RepeatedPtrField<valhalla::StreetName>& names);
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_STREETNAMES_FACTORY_H_
