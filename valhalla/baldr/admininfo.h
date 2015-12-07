#ifndef VALHALLA_BALDR_ADMININFO_H_
#define VALHALLA_BALDR_ADMININFO_H_

#include <valhalla/baldr/admin.h>
#include <iostream>
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

/**
 * Interface class used to pass information about a admin.
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
  AdminInfo(const std::string& country_text, const std::string& state_text,
            const std::string& country_iso, const std::string& state_iso);

  /**
   * Returns true if the specified object is equal to this object.
   * @param  rhs  the specified object to compare against this object.
   * @return true if the specified object is equal to this object.
   */
  bool operator ==(const AdminInfo& rhs) const;

  /**
   * Returns the country text.
   * @return  Returns the country text as a const reference to the text string.
   */
  const std::string& country_text() const;

  /**
   * Returns the state text.
   * @return  Returns the state text as a const reference to the text string.
   */
  const std::string& state_text() const;

  /**
   * Get the country iso
   * @return  Returns the country iso
   */
  const std::string& country_iso() const;

  /**
   * Get the state iso
   * @return  Returns the state iso
   */
  const std::string& state_iso() const;

  struct AdminInfoHasher {
    std::size_t operator()(const AdminInfo& ai) const {
      std::size_t seed = 13;
      boost::hash_combine(seed, string_hasher(ai.country_iso_));
      boost::hash_combine(seed, string_hasher(ai.country_text_));
      boost::hash_combine(seed, string_hasher(ai.state_iso_));
      boost::hash_combine(seed, string_hasher(ai.state_text_));
      return seed;
    }
    //function to hash string
    std::hash<std::string> string_hasher;
  };

 protected:
  std::string country_text_;
  std::string state_text_;
  std::string country_iso_;
  std::string state_iso_;
};

}
}

#endif  // VALHALLA_BALDR_ADMININFO_H_
