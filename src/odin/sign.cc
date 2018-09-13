#include "odin/sign.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

// Constructor
Sign::Sign(const std::string& text) : text_(text), consecutive_count_(0) {
}

const std::string& Sign::text() const {
  return text_;
}

uint32_t Sign::consecutive_count() const {
  return consecutive_count_;
}

void Sign::set_consecutive_count(uint32_t consecutive_count) {
  consecutive_count_ = consecutive_count;
}

#ifdef LOGGING_LEVEL_TRACE
std::string Sign::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";
  str += GetQuotedString(text_);
  str += delim;
  str += GetQuotedString(std::to_string(consecutive_count_));
  str += " }";

  return str;
}
#endif

bool Sign::operator==(const Sign& rhs) const {
  return ((consecutive_count_ == rhs.consecutive_count_) && (text_ == rhs.text_));
}

} // namespace odin
} // namespace valhalla
