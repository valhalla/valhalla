#ifndef VALHALLA_BALDR_STREETNAME_US_H_
#define VALHALLA_BALDR_STREETNAME_US_H_

#include <valhalla/baldr/streetname.h>

#include <memory>
#include <string>
#include <vector>

namespace valhalla {
namespace baldr {

class StreetNameUs : public StreetName {
public:
  /**
   * Constructor.
   * @param  value  Street name string.
   * @param  is_route_number   boolean indicating if street name is a reference route number.
   */
  StreetNameUs(const std::string& value, const bool is_route_number);

  std::string GetPreDir() const override;

  std::string GetPostDir() const override;

  std::string GetPostCardinalDir() const override;

  std::string GetBaseName() const override;

  bool HasSameBaseName(const StreetName& rhs) const override;

protected:
  static const std::vector<std::string> pre_dirs_;
  static const std::vector<std::string> post_dirs_;
  static const std::vector<std::string> post_cardinal_dirs_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_STREETNAME_US_H_
