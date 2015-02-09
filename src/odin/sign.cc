#include "odin/sign.h"

namespace valhalla {
namespace odin {

// Constructor
Sign::Sign(const baldr::Sign::Type& type, const std::string& text)
    : baldr::SignInfo(type, text),
      consecutive_count_(0) {
}

uint32_t Sign::consecutive_count() const {
  return consecutive_count_;
}

void Sign::set_consecutive_count(uint32_t consecutive_count) {
  consecutive_count_ = consecutive_count;
}

}
}
