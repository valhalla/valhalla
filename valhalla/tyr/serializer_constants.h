#ifndef VALHALLA_TYR_ROUTE_SERIALIZER_CONSTANTS_
#define VALHALLA_TYR_ROUTE_SERIALIZER_CONSTANTS_
#pragma once
#include <string>

namespace valhalla {
namespace tyr {
namespace osrmconstants {

const std::string kModifierStraight = "straight";
const std::string kModifierUturn = "uturn";

const std::string kModifierSlightLeft = "slight left";
const std::string kModifierLeft = "left";
const std::string kModifierSharpLeft = "sharp left";

const std::string kModifierSlightRight = "slight right";
const std::string kModifierRight = "right";
const std::string kModifierSharpRight = "sharp right";

} // namespace osrmconstants
} // namespace tyr
} // namespace valhalla
#endif // VALHALLA_TYR_ROUTE_SERIALIZER_CONSTANTS_
