#ifndef VALHALLA_BALDR_STREETNAMES_H_
#define VALHALLA_BALDR_STREETNAMES_H_

#include <cstdint>
#include <list>
#include <vector>
#include <memory>

#include <valhalla/baldr/streetname.h>
#include <valhalla/baldr/verbal_text_formatter.h>
#include <valhalla/baldr/verbal_text_formatter_us.h>

namespace valhalla {
namespace baldr {

class StreetNames : public std::list<std::unique_ptr<StreetName>> {
 public:
  StreetNames();

  StreetNames(const std::vector<std::string>& names);

  virtual ~StreetNames();

  std::string ToString(
      uint32_t max_count = 0, std::string delim = "/",
      const VerbalTextFormatter* verbal_formatter = nullptr) const;

  std::string ToParameterString() const;

  virtual std::unique_ptr<StreetNames> clone() const;

  virtual std::unique_ptr<StreetNames> FindCommonStreetNames(
      const StreetNames& other_street_names) const;

  virtual std::unique_ptr<StreetNames> FindCommonBaseNames(
      const StreetNames& other_street_names) const;

};

}
}

#endif  // VALHALLA_BALDR_STREETNAMES_H_
