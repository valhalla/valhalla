#ifndef VALHALLA_BALDR_RAPIDJSON_FWD_H_
#define VALHALLA_BALDR_RAPIDJSON_FWD_H_

// rapidjson asserts by default but we dont want to crash running server
// its more useful to throw and catch for our use case
#define RAPIDJSON_ASSERT_THROWS
#undef RAPIDJSON_ASSERT
#define RAPIDJSON_ASSERT(x)                                                                          \
  if (!(x))                                                                                          \
  throw std::logic_error(RAPIDJSON_STRINGIFY(x))
// Because we now throw exceptions, we need to turn off RAPIDJSON_NOEXCEPT
#define RAPIDJSON_HAS_CXX11_NOEXCEPT 0
// Enable std::string overloads
#define RAPIDJSON_HAS_STDSTRING 1

#include <rapidjson/fwd.h>

namespace rapidjson {
class writer_wrapper_t;
}

#endif
