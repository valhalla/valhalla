#ifndef VALHALLA_BALDR_STREETNAMES_H_
#define VALHALLA_BALDR_STREETNAMES_H_

#include <cstdint>
#include <list>
#include <memory>
#include <vector>

#include <valhalla/baldr/streetname.h>
#include <valhalla/baldr/verbal_text_formatter.h>
#include <valhalla/baldr/verbal_text_formatter_us.h>
#include <valhalla/proto/common.pb.h>

namespace valhalla {
namespace baldr {

class StreetNames : public std::list<std::unique_ptr<StreetName>> {
public:
  StreetNames();

  StreetNames(const std::vector<std::pair<std::string, bool>>& names);

  StreetNames(const google::protobuf::RepeatedPtrField<valhalla::StreetName>& names);

  virtual ~StreetNames();

  std::string ToString(uint32_t max_count = 0,
                       const std::string& delim = "/",
                       const VerbalTextFormatter* verbal_formatter = nullptr) const;
#ifdef LOGGING_LEVEL_TRACE
  std::string ToParameterString() const;
#endif

  virtual std::unique_ptr<StreetNames> clone() const;

  virtual std::unique_ptr<StreetNames>
  FindCommonStreetNames(const StreetNames& other_street_names) const;

  virtual std::unique_ptr<StreetNames>
  FindCommonBaseNames(const StreetNames& other_street_names) const;

  virtual std::unique_ptr<StreetNames> GetRouteNumbers() const;
  virtual std::unique_ptr<StreetNames> GetNonRouteNumbers() const;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_STREETNAMES_H_
