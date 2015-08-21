#include <iostream>
#include <memory>

#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

VerbalTextFormatterUs::VerbalTextFormatterUs(const std::string& country_code,
                                             const std::string& state_code)
    : VerbalTextFormatter(country_code, state_code) {
}

VerbalTextFormatterUs::~VerbalTextFormatterUs() {
}

std::string VerbalTextFormatterUs::Format(const std::string& text) const {
  std::string verbal_text;


  verbal_text = VerbalTextFormatter::Format(text);
  // TODO: US specific cases

  return verbal_text;
}

}
}
