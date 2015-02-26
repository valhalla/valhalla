#ifndef VALHALLA_ODIN_UTIL_H_
#define VALHALLA_ODIN_UTIL_H_

#include <vector>
#include <string>

namespace valhalla {
namespace odin {

/**
 * Returns the specified item surrounded with quotes.
 * @param item  specified text to surround with quotes
 * @return the specified item surrounded with quotes.
 */
std::string GetQuotedString(const std::string& item);

}
}
#endif  // VALHALLA_ODIN_UTIL_H_
