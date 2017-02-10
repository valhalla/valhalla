#ifndef VALHALLA_MJOLNIR_UTIL_H_
#define VALHALLA_MJOLNIR_UTIL_H_

#include <vector>
#include <string>

namespace valhalla {
namespace mjolnir {


/**
 * Splits a tag into a vector of strings.
 * @param  tag_value  tag to split
 * @param  delim      defaults to ;
 * @return the vector of strings
*/
std::vector<std::string> GetTagTokens(const std::string& tag_value,
                                      char delim = ';');

/**
 * Remove double quotes.
 * @param  s
 * @return string string with no quotes.
*/std::string remove_double_quotes(const std::string& s);

}
}
#endif  // VALHALLA_MJOLNIR_UTIL_H_
