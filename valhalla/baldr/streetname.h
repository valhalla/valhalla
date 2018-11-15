#ifndef VALHALLA_BALDR_STREETNAME_H_
#define VALHALLA_BALDR_STREETNAME_H_

#include <memory>
#include <string>

namespace valhalla {
namespace baldr {

class StreetName {
public:
  /**
   * Constructor.
   * @param  value  Street name string.
   * @param  is_route_number   boolean indicating if street name is a reference route number.
   */
  StreetName(const std::string& value, const bool is_route_number);

  virtual ~StreetName();

  const std::string& value() const;

  /**
   * Returns true if street name is a reference route number such as: I 81 South or US 322 West.
   * @return true if street name is a reference route number such as: I 81 South or US 322 West.
   */
  bool is_route_number() const;

  bool operator==(const StreetName& rhs) const;

  bool StartsWith(const std::string& prefix) const;

  bool EndsWith(const std::string& suffix) const;

  virtual std::string GetPreDir() const;

  virtual std::string GetPostDir() const;

  virtual std::string GetPostCardinalDir() const;

  virtual std::string GetBaseName() const;

  virtual bool HasSameBaseName(const StreetName& rhs) const;

protected:
  std::string value_;
  bool is_route_number_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_STREETNAME_H_
