#ifndef VALHALLA_BALDR_STREETNAMES_H_
#define VALHALLA_BALDR_STREETNAMES_H_

#include <list>
#include <vector>
#include <memory>

#include <valhalla/baldr/streetname.h>

namespace valhalla {
namespace baldr {

class StreetNames : public std::list<std::unique_ptr<StreetName>> {
 public:
  StreetNames();

  virtual ~StreetNames();

  std::string ToString() const;

  std::string ToParameterString() const;

  virtual std::unique_ptr<StreetNames> FindCommonStreetNames(
      const StreetNames& other_street_names) const = 0;

  virtual std::unique_ptr<StreetNames> FindCommonBaseNames(
      const StreetNames& other_street_names) const = 0;

 protected:

};

}
}

#endif  // VALHALLA_BALDR_STREETNAMES_H_
