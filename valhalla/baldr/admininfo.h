#ifndef VALHALLA_BALDR_ADMININFO_H_
#define VALHALLA_BALDR_ADMININFO_H_

#include <iostream>
#include <valhalla/baldr/admin.h>

namespace valhalla {
namespace baldr {

/**
 * Interface class used to pass information about an administrative area.
 * Encapsulates the country and state text.
 */
class AdminInfo {
public:
  /**
   * Constructor.
   * @param  country_text country text string.
   * @param  state_text   state text string.
   * @param  country_iso  country iso string.
   * @param  state_iso    state iso string.
   */
  AdminInfo(const std::string& country_text,
            const std::string& state_text,
            const std::string& country_iso,
            const std::string& state_iso)
      : country_text_(country_text), state_text_(state_text), country_iso_(country_iso),
        state_iso_(state_iso) {
  }

  /**
   * Returns true if the specified object is equal to this object.
   * @param  rhs  the specified object to compare against this object.
   * @return true if the specified object is equal to this object.
   */
  bool operator==(const AdminInfo& rhs) const {
    return (country_iso_ == rhs.country_iso_ && country_text_ == rhs.country_text_ &&
            state_iso_ == rhs.state_iso_ && state_text_ == rhs.state_text_);
  }

  /**
   * Returns the country text.
   * @return  Returns the country text as a const reference to the text string.
   */
  const std::string& country_text() const {
    return country_text_;
  }

  /**
   * Returns the state text.
   * @return  Returns the state text as a const reference to the text string.
   */
  const std::string& state_text() const {
    return state_text_;
  }

  /**
   * Get the country iso
   * @return  Returns the country iso
   */
  const std::string& country_iso() const {
    return country_iso_;
  }

  /**
   * Get the state iso
   * @return  Returns the state iso
   */
  const std::string& state_iso() const {
    return state_iso_;
  }

  struct AdminInfoHasher {
    std::size_t operator()(const AdminInfo& ai) const {
      return string_hasher(ai.country_iso_ + ai.country_text_ + ai.state_iso_ + ai.state_text_);
    }
    // function to hash string
    std::hash<std::string> string_hasher;
  };

protected:
  std::string country_text_;
  std::string state_text_;
  std::string country_iso_;
  std::string state_iso_;
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_ADMININFO_H_
