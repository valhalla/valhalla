#include "odin/sign.h"

namespace valhalla {
namespace odin {

// Constructor
Sign::Sign(const std::string& text)
    : text_(text),
      consecutive_count_(0) {
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

std::string Sign::ToUnitTestString() const {
  const std::string delim = ", ";
  std::string str;
  str += "{ ";
  str += GetQuotedString(text_);
  str += delim;
  str += GetQuotedString(std::to_string(consecutive_count_));
  str += " }";

  return str;
}

std::string Sign::GetQuotedString(const std::string& item) const {
  std::string str;
  str += "\"";
  str += item;
  str += "\"";
  return str;
}

}
}
