#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "baldr/verbal_text_formatter_factory.h"
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

std::unique_ptr<VerbalTextFormatter> VerbalTextFormatterFactory::Create(
    const std::string& country_code, const std::string& state_code) {
  if (country_code == "US") {
    return midgard::make_unique<VerbalTextFormatterUs>(country_code, state_code);
  }

  return midgard::make_unique<VerbalTextFormatter>(country_code, state_code);
}

}
}
