#include "mjolnir/util.h"
#include <valhalla/baldr/graphtilefsstorage.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace valhalla {
namespace mjolnir {

/**
 * Create the tile storage handler for given configuration.
 */
std::shared_ptr<valhalla::baldr::GraphTileStorage> CreateTileStorage(const boost::property_tree::ptree& pt) {
   return std::make_shared<valhalla::baldr::GraphTileFsStorage>(pt);
}

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

// remove double quotes.
std::string remove_double_quotes(const std::string& s) {
  std::string ret;
  for (auto c : s) {
    if (c != '"') {
      ret += c;
    }
  }
  return ret;
}

}
}
