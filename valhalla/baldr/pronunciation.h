#pragma once

#include <boost/optional.hpp>
#include <string>

#include <valhalla/proto/tripcommon.pb.h>

namespace valhalla {
namespace baldr {

class Pronunciation {
public:
  /**
   * Constructor.
   * @param  alphabet  the phonetic alphabet.
   * @param  value  text string with the pronunciation.
   */
  Pronunciation(const valhalla::Pronunciation_Alphabet alphabet, const std::string& value);

  /**
   * Returns the phonetic alphabet.
   * @return the phonetic alphabet.
   */
  valhalla::Pronunciation_Alphabet alphabet() const;

  /**
   * Returns the pronunciation value.
   * @return  Returns the pronunciation value as a const reference to the text string.
   */
  const std::string& value() const;

protected:
  valhalla::Pronunciation_Alphabet alphabet_;
  std::string value_;
};

} // namespace baldr
} // namespace valhalla
