#ifndef VALHALLA_BALDR_STREETNAME_H_
#define VALHALLA_BALDR_STREETNAME_H_

#include <string>
#include <memory>

namespace valhalla {
namespace baldr {

class StreetName {
 public:
  StreetName(const std::string& value);

  virtual ~StreetName();

  const std::string& value() const;

  bool operator ==(const StreetName& rhs) const;

  bool StartsWith(const std::string& prefix) const;

  bool EndsWith(const std::string& suffix) const;

  virtual std::string GetPreDir() const = 0;

  virtual std::string GetPostDir() const = 0;

  virtual std::string GetPostCardinalDir() const = 0;

  virtual std::string GetBaseName() const = 0;

  virtual bool HasSameBaseName(const StreetName& rhs) const = 0;

 protected:
  std::string value_;

};

}
}

#endif  // VALHALLA_BALDR_STREETNAME_H_
