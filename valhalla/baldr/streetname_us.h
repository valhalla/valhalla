#ifndef VALHALLA_BALDR_STREETNAME_US_H_
#define VALHALLA_BALDR_STREETNAME_US_H_

#include <valhalla/baldr/streetname.h>

#include <string>
#include <vector>
#include <memory>

namespace valhalla {
namespace baldr {

class StreetNameUs : public StreetName {
 public:
  StreetNameUs(const std::string& value);

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

}
}

#endif  // VALHALLA_BALDR_STREETNAME_US_H_
