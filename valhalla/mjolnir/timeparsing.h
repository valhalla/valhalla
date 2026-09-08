#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

namespace valhalla {
namespace mjolnir {

std::vector<uint64_t> get_time_range(std::string_view condition);

} // namespace mjolnir
} // namespace valhalla
