#pragma once

#include <exception>

#define CHECK_THROWS(condition, message)                                                             \
  {                                                                                                  \
    if (!(condition))                                                                                \
      throw std::invalid_argument(message);                                                          \
  }
