#include "mjolnir/util.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Splits a tag into a vector of strings.  Delim defaults to ;
 */
std::vector<std::string> GetTagTokens(const std::string& tag_value,
                                      char delim) {
  std::vector<std::string> tokens;
  boost::algorithm::split(tokens, tag_value,
                          std::bind1st(std::equal_to<char>(), delim),
                          boost::algorithm::token_compress_on);
  return tokens;
}

}
}
