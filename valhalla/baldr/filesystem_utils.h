#ifndef VALHALLA_BALDR_FILESYSTEM_UTILS_H_
#define VALHALLA_BALDR_FILESYSTEM_UTILS_H_

namespace valhalla {
namespace baldr {
namespace filesystem {

#if defined(_WIN32) || defined(__CYGWIN__)
constexpr char path_separator = L'\\';
#else
constexpr char path_separator = L'/';
#endif

}
}
}

#endif /* VALHALLA_BALDR_FILESYSTEM_UTILS_H_ */
