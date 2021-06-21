#include "baldr/turn.h"
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace valhalla {
namespace baldr {

const std::unordered_map<int, std::string>
    turn_type_to_string{{static_cast<int>(Turn::Type::kStraight), "straight"},
                        {static_cast<int>(Turn::Type::kSlightRight), "slight right"},
                        {static_cast<int>(Turn::Type::kRight), "right"},
                        {static_cast<int>(Turn::Type::kSharpRight), "sharp right"},
                        {static_cast<int>(Turn::Type::kReverse), "reverse"},
                        {static_cast<int>(Turn::Type::kSharpLeft), "sharp left"},
                        {static_cast<int>(Turn::Type::kLeft), "left"},
                        {static_cast<int>(Turn::Type::kSlightLeft), "slight left"}};

// Returns the turn type based on the specified turn degree.
Turn::Type Turn::GetType(uint32_t turn_degree) {
  turn_degree %= 360;
  if ((turn_degree > 349) || (turn_degree < 11)) {
    return Turn::Type::kStraight;
  } else if ((turn_degree > 10) && (turn_degree < 45)) {
    return Turn::Type::kSlightRight;
  } else if ((turn_degree > 44) && (turn_degree < 136)) {
    return Turn::Type::kRight;
  } else if ((turn_degree > 135) && (turn_degree < 160)) {
    return Turn::Type::kSharpRight;
  } else if ((turn_degree > 159) && (turn_degree < 201)) {
    return Turn::Type::kReverse;
  } else if ((turn_degree > 200) && (turn_degree < 225)) {
    return Turn::Type::kSharpLeft;
  } else if ((turn_degree > 224) && (turn_degree < 316)) {
    return Turn::Type::kLeft;
  } else if ((turn_degree > 315) && (turn_degree < 350)) {
    return Turn::Type::kSlightLeft;
  }
  throw std::runtime_error("Turn degree out of range");
}

std::string Turn::GetTypeString(Turn::Type turn_type) {
  auto it_type = turn_type_to_string.find(static_cast<int>(turn_type));
  return (it_type != turn_type_to_string.cend()) ? it_type->second : "undefined";
}

} // namespace baldr
} // namespace valhalla
