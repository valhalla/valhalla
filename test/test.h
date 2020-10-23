// -*- mode: c++ -*-

#ifndef TEST_HPP
#define TEST_HPP

#include <fstream>
#include <random>
#include <stdexcept>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace test {

// Return a random number inside [0, 1)
inline float rand01(std::mt19937& gen) {
  std::uniform_real_distribution<float> dis(0, 1);
  return dis(gen);
}

inline std::string load_binary_file(const std::string& filename) {
  std::string bytes;
  std::ifstream input_pbf(filename, std::ios::in | std::ios::binary);
  if (input_pbf.is_open()) {
    input_pbf.seekg(0, std::ios::end);
    bytes.resize(input_pbf.tellg());
    input_pbf.seekg(0, std::ios::beg);
    input_pbf.read(&bytes[0], bytes.size());
    input_pbf.close();
  } else {
    throw std::runtime_error("Failed to read " + filename);
  }
  return bytes;
}

MATCHER_P2(IsBetween,
           a,
           b,
           std::string(negation ? "isn't" : "is") + " between " + ::testing::PrintToString(a) +
               " and " + ::testing::PrintToString(b)) {
  return a <= arg && arg <= b;
}

template <typename pbf_message_t> bool pbf_equals(const pbf_message_t& a, const pbf_message_t& b) {
  return a.SerializeAsString() == b.SerializeAsString();
}

} // namespace test

#endif
