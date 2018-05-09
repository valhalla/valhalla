#ifndef VALHALLA_BALDR_REUTIL_H_
#define VALHALLA_BALDR_REUTIL_H_

#ifdef USE_STD_REGEX
#include <regex>
namespace valhalla {
namespace baldr {
namespace re {
using std::regex;
using std::regex_replace;
using std::regex_search;
using std::smatch;
using std::sregex_iterator;
namespace regex_constants {
using namespace std::regex_constants;
}
} // namespace re
} // namespace baldr
} // namespace valhalla
#else
#include <boost/regex.hpp>
namespace valhalla {
namespace baldr {
namespace re {
using boost::regex;
using boost::regex_replace;
using boost::regex_search;
using boost::smatch;
using boost::sregex_iterator;
namespace regex_constants {
using namespace boost::regex_constants;
}
} // namespace re
} // namespace baldr
} // namespace valhalla
#endif // USE_STD_REGEX

#endif // VALHALLA_BALDR_REUTIL_H_
