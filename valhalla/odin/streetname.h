#ifndef VALHALLA_ODIN_STREETNAME_H_
#define VALHALLA_ODIN_STREETNAME_H_

#include <string>
#include <vector>

namespace valhalla {
namespace odin {

class StreetName {
 public:
  StreetName(const std::string& value);

  const std::string& value() const;

  bool operator ==(const StreetName& rhs) const;

  bool StartsWith(const std::string& prefix) const;

  bool EndsWith(const std::string& suffix) const;

  std::string GetPreDir() const;

  std::string GetPostDir() const;

  std::string GetBaseName() const;

  bool HasSameBaseName(const StreetName& rhs) const;

  // TODO - add more functionality later and comments

 protected:
  std::string value_;

  static const std::vector<std::string> pre_dirs_;
  static const std::vector<std::string> post_dirs_;
};

}
}

#endif  // VALHALLA_ODIN_STREETNAME_H_
