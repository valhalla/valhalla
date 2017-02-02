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

}
}
#endif  // VALHALLA_MJOLNIR_UTIL_H_
