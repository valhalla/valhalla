#include "baldr/streetname.h"

#include "odin/sign.h"
#include "odin/util.h"

namespace valhalla {
namespace odin {

// Constructor
Sign::Sign(const std::string& text,
           const bool is_route_number,
           const boost::optional<baldr::Pronunciation>& pronunciation)
    : text_(text), is_route_number_(is_route_number), consecutive_count_(0),
      pronunciation_(pronunciation) {
}

const std::string& Sign::text() const {
  return text_;
}

bool Sign::is_route_number() const {
  return is_route_number_;
}

uint32_t Sign::consecutive_count() const {
  return consecutive_count_;
}

void Sign::set_consecutive_count(uint32_t consecutive_count) {
  consecutive_count_ = consecutive_count;
}

const boost::optional<baldr::Pronunciation>& Sign::pronunciation() const {
  return pronunciation_;
}

#ifdef LOGGING_LEVEL_TRACE
std::string Sign::ToParameterString() const {
  const std::string delim = ", ";
  std::string str;
  str += "std::make_tuple(";
  str += GetQuotedString(text_);
  str += delim;
  str += std::to_string(is_route_number_);
  str += delim;
  str += std::to_string(consecutive_count_);
  str += ")";

  return str;
}
#endif

bool Sign::operator==(const Sign& rhs) const {
  return ((consecutive_count_ == rhs.consecutive_count_) &&
          (is_route_number_ == rhs.is_route_number_) && (text_ == rhs.text_));
}

} // namespace odin
} // namespace valhalla
