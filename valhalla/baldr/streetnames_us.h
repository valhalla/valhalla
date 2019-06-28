#ifndef VALHALLA_BALDR_STREETNAMES_US_H_
#define VALHALLA_BALDR_STREETNAMES_US_H_

#include <list>
#include <memory>

#include <valhalla/baldr/streetname_us.h>
#include <valhalla/baldr/streetnames.h>

namespace valhalla {
namespace baldr {

class StreetNamesUs : public StreetNames {
public:
  StreetNamesUs();

  StreetNamesUs(const std::vector<std::pair<std::string, bool>>& names);

  ~StreetNamesUs();

  std::unique_ptr<StreetNames> clone() const override;

  std::unique_ptr<StreetNames>
  FindCommonStreetNames(const StreetNames& other_street_names) const override;

  std::unique_ptr<StreetNames>
  FindCommonBaseNames(const StreetNames& other_street_names) const override;

  std::unique_ptr<StreetNames> GetRouteNumbers() const override;
  std::unique_ptr<StreetNames> GetNonRouteNumbers() const override;

protected:
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_STREETNAMES_US_H_
