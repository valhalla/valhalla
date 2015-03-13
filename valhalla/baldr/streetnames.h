#ifndef VALHALLA_BALDR_STREETNAMES_H_
#define VALHALLA_BALDR_STREETNAMES_H_

#include <list>

#include <valhalla/baldr/streetname.h>

namespace valhalla {
namespace baldr {

class StreetNames : public std::list<StreetName> {
 public:
  StreetNames();

  StreetNames(const std::vector<std::string>& names);

  virtual ~StreetNames();

  virtual std::string ToString() const;

  virtual std::string ToParameterString() const;

  virtual StreetNames FindCommonStreetNames(const StreetNames& other_street_names) const;

  virtual StreetNames FindCommonBaseNames(const StreetNames& other_street_names) const;

 protected:

};

}
}

#endif  // VALHALLA_BALDR_STREETNAMES_H_
