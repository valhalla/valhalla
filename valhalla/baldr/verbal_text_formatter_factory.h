#ifndef VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_
#define VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_

#include <string>
#include <memory>

#include <valhalla/baldr/verbal_text_formatter.h>

namespace valhalla {
namespace baldr {

class VerbalTextFormatterFactory {
 public:
  VerbalTextFormatterFactory() = delete;

  static std::unique_ptr<VerbalTextFormatter> Create(
      const std::string& country_code, const std::string& state_code);

};

}
}

#endif  // VALHALLA_BALDR_VERBAL_TEXT_FORMATTER_FACTORY_H_
