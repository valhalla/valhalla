#include "baldr/verbal_text_formatter.h"
#include "baldr/verbal_text_formatter_us.h"
#include "baldr/verbal_text_formatter_us_co.h"
#include "baldr/verbal_text_formatter_us_tx.h"
#include "baldr/verbal_text_formatter_factory.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

std::unique_ptr<VerbalTextFormatter> VerbalTextFormatterFactory::Create(
    const std::string& country_code, const std::string& state_code) {
  if (country_code == "US") {
    if (state_code == "TX") {
      return midgard::make_unique<VerbalTextFormatterUsTx>(country_code,
                                                           state_code);
    } else if (state_code == "CO") {
      return midgard::make_unique<VerbalTextFormatterUsCo>(country_code,
                                                           state_code);
    }
    return midgard::make_unique<VerbalTextFormatterUs>(country_code, state_code);
  }

  return midgard::make_unique<VerbalTextFormatter>(country_code, state_code);
}

}
}
