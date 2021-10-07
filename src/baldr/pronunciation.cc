#include "baldr/pronunciation.h"

namespace valhalla {
namespace baldr {

// Constructor
Pronunciation::Pronunciation(const valhalla::Pronunciation_Alphabet alphabet,
                             const std::string& value)
    : alphabet_(alphabet), value_(value) {
}

valhalla::Pronunciation_Alphabet Pronunciation::alphabet() const {
  return alphabet_;
}

const std::string& Pronunciation::value() const {
  return value_;
}

} // namespace baldr
} // namespace valhalla
