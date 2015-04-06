#ifndef VALHALLA_BALDR_ADMININFO_H_
#define VALHALLA_BALDR_ADMININFO_H_

#include <valhalla/baldr/admin.h>
#include <iostream>

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
   * @param  start_dst    start dst string.
   * @param  end_dst      end dst  string.
   */
  AdminInfo(const std::string& country_text, const std::string& state_text,
            const std::string& country_iso, const std::string& state_iso,
            const std::string& start_dst, const std::string& end_dst);
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

  /**
   * Get the start dst
   * @return  Returns the start dst
   */
  const std::string& start_dst() const;

  /**
   * Get the end dst
   * @return  Returns the end dst
   */
  const std::string& end_dst() const;

 protected:
  std::string country_text_;
  std::string state_text_;

  std::string country_iso_;
  std::string state_iso_;

  std::string start_dst_;
  std::string end_dst_;

};

}
}

#endif  // VALHALLA_BALDR_ADMININFO_H_
