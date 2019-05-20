#ifndef VALHALLA_MJOLNIR_TIMEPARSING_H
#define VALHALLA_MJOLNIR_TIMEPARSING_H

#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <boost/date_time/local_time/local_time_io.hpp>
#include <boost/date_time/local_time/tz_database.hpp>
#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace mjolnir {

/**
 * get the dow mask from user inputed string.  try to handle most inputs
 * @param   dow entered by a user
 * @return dow mask
 */
uint8_t get_dow_mask(const std::string& dow);

/**
 * get the dow from user inputed string.  try to handle most inputs
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
#endif // VALHALLA_MJOLNIR_TIMEPARSING_H
