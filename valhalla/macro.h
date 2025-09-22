#pragma once

#include <boost/config.hpp>

#include <exception>

#define CHECK_THROWS(condition, message)                                                             \
  {                                                                                                  \
    if (!(condition))                                                                                \
      throw std::invalid_argument(message);                                                          \
  }

#define VALHALLA_LIKELY(x) BOOST_LIKELY(x)
