#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

/**
 * get the dow mask from the provided string.  try to handle most inputs
 * @param   dow entered by a user
 * @return dow mask
 */
uint8_t get_dow_mask(const std::string& dow);

/**
 * get the dow from the provided string.  try to handle most inputs
 * @param   dow entered by a user
 * @return DOW
 */
baldr::DOW get_dow(const std::string& dow);

/**
 * Get the month from the input string.Try to handle most inputs.
 * @param   month entered by a user
 * @return MONTH
 */
baldr::MONTH get_month(const std::string& month);

std::vector<uint64_t> get_time_range(const std::string& condition);

} // namespace mjolnir
} // namespace valhalla
