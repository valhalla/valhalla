#include "baldr/turn.h"

#include <array>
#include <string>
#include <unordered_map>
namespace valhalla {
namespace baldr {

namespace {
constexpr std::array<Turn::Type, 360> make_turn_type_lut() {
  std::array<Turn::Type, 360> t{};

  auto fill = [&](size_t from_angle, size_t to_angle, Turn::Type type) {
    for (auto angle = from_angle; angle <= to_angle; ++angle)
      t[angle] = type;
  };

  fill(0, 10, Turn::Type::kStraight);
  fill(11, 44, Turn::Type::kSlightRight);
  fill(45, 135, Turn::Type::kRight);
  fill(136, 159, Turn::Type::kSharpRight);
  fill(160, 200, Turn::Type::kReverse);
  fill(201, 224, Turn::Type::kSharpLeft);
  fill(225, 315, Turn::Type::kLeft);
  fill(316, 349, Turn::Type::kSlightLeft);
  fill(350, 359, Turn::Type::kStraight);

  return t;
}
constexpr auto kTurnTypeLUT = make_turn_type_lut();
} // namespace

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
  // this function is on the hot path, so we use a lookup table to avoid branch prediction misses
  return kTurnTypeLUT[turn_degree % 360];
}

std::string Turn::GetTypeString(Turn::Type turn_type) {
  auto it_type = turn_type_to_string.find(static_cast<int>(turn_type));
  return (it_type != turn_type_to_string.cend()) ? it_type->second : "undefined";
}

} // namespace baldr
} // namespace valhalla
