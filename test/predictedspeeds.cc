#include "test.h"

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include <iostream>

#include "baldr/predictedspeeds.h"
#include "midgard/util.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

// base64 decoding
std::string decode64(const std::string& val) {
  using namespace boost::archive::iterators;
  using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
  return std::string(It(std::begin(val)), It(std::end(val)));
}

// Convert big endian bytes to little endian
int16_t to_little_endian(const int16_t val) {
  return (val << 8) | ((val >> 8) & 0x00ff);
}

// Check if the speed is within threshold for the test
constexpr uint32_t kSpeedErrorThreshold = 2;
inline bool within_threshold(const uint32_t v1, const uint32_t v2) {
  return (v2 > v1) ? (v2 - v1) < kSpeedErrorThreshold : (v1 - v2) < kSpeedErrorThreshold;
}

void try_free_flow_speed(const std::string encoded_str,
                         const uint32_t exp_free_flow,
                         const uint32_t exp_constrained_flow) {
  auto decoded_data = decode64(encoded_str);
  const auto raw = reinterpret_cast<const unsigned char*>(decoded_data.data());
  std::size_t index = 0;
  uint32_t t = static_cast<std::uint32_t>(raw[index++] & 0x1f);
  if (t != 0) {
    throw std::runtime_error("type should be 0 but is " + std::to_string(t));
  }

  uint32_t free_flow = static_cast<std::uint32_t>(raw[index++] & 0xff);
  if (free_flow != exp_free_flow) {
    throw std::runtime_error("free flow speed should be " + std::to_string(exp_free_flow) +
                             " but is " + std::to_string(free_flow));
  }
  uint32_t constrained_flow = static_cast<std::uint32_t>(raw[index++] & 0xff);
  if (constrained_flow != exp_constrained_flow) {
    throw std::runtime_error("constrained flow speed should be " +
                             std::to_string(exp_constrained_flow) + " but is " +
                             std::to_string(constrained_flow));
  }
}

void test_free_flow_speed() {
  try_free_flow_speed("AAie", 8, 158);

  // Add additional cases below
  try_free_flow_speed("AACe", 0, 158);
}

void test_decoding() {
  // base64 encoded string
  std::string encoded_speed_string =
      "AQXFAAkABAAhAAz/+//bABn/3wAMABsAEQAF//gAAAAdABQAEv/wABf//gAsAAkAKgAAACj/+gBDAAQAbAALAQQAKv63AAD/mwAM/87/7P/TAAX/2P/1//P//f/sAAn/z//xAA7//P/y//z/8v/x////+wAMABX/+f/6AA4AGQAEABX/9//vAAf/8gAfAAb/9AAFABH//P/0ABQABv/2////4//7//0AE//+//n/5AATAAcAAQAL/+v//P/3ABMAAAAU//L/+v/8AAAAEP/3AAsABQAE/9f/7AABAAwAAQAGABP//QAJ/+4AB//gABUAAf/+AAv/6P/oABP//gAAABX/5f/5AAT//v/5AAgABv/3AB7/6gAdAAL/+P/r//sACwADAAT/9wAE//MACAAK//cACv/4//sABAAA//j//P/7//H/9v/y//wACwAHAAYABv/4AAL/+QAKAB7//wAHABX/8wAQ/+wAFAAL/+7//AAIAAgADf/9AAz/4gAQ//X/9//+//j/9wAEAAz//wADAAc=";

  // Decode the base64 string and cast the data to a raw string of signed bytes
  auto decoded_str = decode64(encoded_speed_string);
  if (decoded_str.size() != 402) {
    throw std::runtime_error("Decoded speed string size should be 402 but is " +
                             std::to_string(decoded_str.size()));
  }
  auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

  // Check that the first value pair == 1
  if (static_cast<std::int8_t>(raw[0]) != 1) {
    throw std::runtime_error("First value should be 1");
  }

  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number (big endian).
  // Convert to little endian.
  int idx = 1;
  int16_t coefficients[200];
  for (uint32_t i = 0; i < 200; ++i, idx += 2) {
    coefficients[i] = to_little_endian(*(reinterpret_cast<const int16_t*>(&raw[idx])));
  }

  // Bucketized speeds - decoded speeds are compared against this set)
  uint16_t speeds[] =
      {36, 36, 36, 36, 36, 36, 36, 36, 36, 37, 37, 37, 38, 38, 39, 40, 40, 41, 41, 42, 42, 42, 42,
       42, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 43, 43, 44, 44, 45, 45, 45, 46, 46,
       45, 45, 45, 44, 43, 43, 42, 41, 40, 40, 39, 39, 38, 38, 37, 37, 37, 36, 36, 35, 34, 34, 33,
       32, 30, 29, 27, 26, 24, 23, 21, 20, 19, 18, 17, 17, 16, 16, 16, 16, 16, 16, 17, 17, 16, 16,
       16, 15, 15, 14, 13, 12, 12, 11, 11, 10, 10, 11, 12, 13, 14, 16, 17, 19, 21, 24, 25, 27, 29,
       30, 31, 32, 33, 33, 33, 33, 33, 32, 32, 32, 33, 33, 34, 35, 36, 38, 39, 41, 42, 44, 45, 46,
       47, 47, 48, 48, 47, 47, 46, 45, 45, 44, 43, 43, 43, 43, 43, 44, 44, 45, 46, 46, 47, 48, 48,
       49, 49, 48, 48, 48, 47, 46, 46, 45, 44, 44, 43, 43, 43, 43, 43, 43, 43, 42, 42, 42, 41, 41,
       40, 39, 38, 37, 35, 34, 33, 31, 30, 29, 28, 27, 26, 26, 25, 25, 25, 25, 25, 25, 25, 25, 25,
       25, 25, 25, 25, 25, 24, 24, 24, 23, 23, 22, 22, 21, 20, 19, 18, 18, 17, 17, 16, 16, 16, 16,
       17, 17, 18, 19, 20, 22, 23, 24, 25, 27, 27, 28, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 31,
       32, 33, 34, 36, 37, 39, 41, 42, 43, 45, 45, 46, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47,
       48, 49, 49, 50, 51, 52, 52, 53, 53, 53, 53, 52, 52, 51, 50, 49, 48, 46, 45, 44, 44, 43, 42,
       41, 41, 40, 40, 39, 38, 38, 37, 36, 35, 34, 34, 33, 33, 32, 32, 32, 32, 33, 33, 34, 34, 35,
       35, 35, 35, 35, 34, 33, 32, 31, 29, 27, 26, 24, 23, 21, 20, 19, 19, 18, 18, 18, 19, 19, 20,
       20, 20, 21, 21, 21, 21, 21, 21, 21, 20, 20, 21, 21, 21, 22, 22, 23, 24, 25, 26, 27, 28, 29,
       29, 29, 30, 30, 29, 29, 29, 29, 29, 30, 30, 31, 32, 33, 35, 37, 39, 41, 43, 44, 46, 47, 48,
       49, 50, 50, 50, 49, 49, 48, 48, 47, 46, 46, 46, 46, 46, 46, 46, 47, 48, 48, 49, 49, 49, 49,
       49, 49, 49, 48, 47, 46, 45, 44, 43, 42, 42, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40,
       39, 39, 38, 37, 36, 35, 34, 32, 31, 29, 27, 26, 24, 23, 21, 20, 19, 19, 18, 18, 18, 18, 18,
       19, 19, 20, 20, 21, 22, 22, 23, 23, 24, 24, 24, 23, 23, 23, 22, 22, 21, 20, 20, 19, 19, 18,
       18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 20, 21, 22, 23, 24, 25, 27, 28, 30, 32, 33, 35,
       37, 39, 40, 41, 42, 43, 44, 44, 44, 44, 43, 42, 42, 41, 40, 39, 39, 38, 38, 38, 38, 39, 40,
       41, 42, 43, 44, 45, 47, 48, 49, 50, 50, 51, 51, 51, 51, 50, 50, 49, 48, 47, 46, 45, 44, 44,
       43, 42, 41, 41, 40, 40, 39, 38, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 27, 26, 26,
       26, 26, 26, 27, 27, 27, 28, 28, 28, 28, 28, 28, 27, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8,
       7,  6,  5,  5,  5,  5,  6,  7,  8,  9,  10, 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 16,
       16, 17, 18, 19, 21, 23, 25, 27, 29, 32, 34, 36, 38, 40, 41, 42, 43, 44, 44, 45, 45, 44, 44,
       44, 43, 43, 43, 42, 42, 41, 41, 41, 40, 40, 39, 38, 38, 37, 37, 36, 36, 36, 36, 36, 37, 38,
       38, 39, 41, 42, 43, 44, 44, 45, 45, 45, 44, 44, 42, 41, 39, 38, 36, 34, 32, 31, 29, 28, 28,
       27, 27, 27, 27, 28, 28, 29, 29, 29, 30, 29, 29, 29, 28, 27, 26, 25, 23, 22, 21, 20, 20, 19,
       19, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 22, 22, 22, 22, 22, 22, 23, 23, 23, 24, 24,
       24, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 24, 24, 24, 25, 25, 26, 27, 28, 29, 30, 32, 33,
       35, 36, 38, 39, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 49, 50, 51, 52, 53, 53, 54, 54, 54,
       54, 54, 54, 53, 52, 50, 49, 47, 46, 44, 43, 42, 40, 40, 39, 39, 39, 39, 40, 40, 41, 42, 43,
       43, 44, 44, 44, 44, 44, 43, 42, 41, 40, 39, 38, 36, 35, 34, 33, 32, 32, 31, 30, 29, 29, 28,
       28, 27, 26, 25, 25, 24, 23, 22, 21, 21, 20, 19, 19, 19, 18, 18, 18, 18, 18, 17, 17, 17, 16,
       16, 15, 15, 14, 14, 14, 13, 13, 13, 13, 14, 14, 15, 16, 17, 18, 19, 21, 22, 23, 24, 26, 27,
       27, 28, 29, 30, 30, 31, 32, 32, 33, 34, 35, 35, 36, 38, 39, 40, 41, 42, 43, 44, 45, 45, 46,
       46, 46, 46, 46, 46, 46, 45, 45, 44, 44, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 45, 45,
       45, 46, 46, 46, 45, 45, 45, 44, 44, 43, 42, 41, 41, 40, 39, 37, 36, 35, 34, 32, 31, 30, 28,
       27, 25, 24, 23, 21, 20, 19, 19, 18, 18, 18, 18, 18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 23,
       22, 22, 21, 20, 19, 18, 18, 17, 16, 16, 16, 16, 17, 17, 18, 19, 20, 22, 23, 24, 25, 26, 26,
       27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 29, 30, 31, 33, 34, 36, 37, 39, 40, 41, 41, 42,
       42, 42, 42, 41, 40, 40, 39, 38, 38, 37, 37, 37, 38, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
       48, 48, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 48, 47, 46, 45,
       44, 42, 41, 40, 38, 37, 36, 35, 34, 33, 33, 33, 32, 32, 32, 32, 32, 31, 31, 30, 30, 29, 28,
       27, 26, 25, 23, 22, 21, 20, 18, 18, 17, 16, 16, 16, 16, 17, 17, 18, 19, 19, 20, 21, 22, 23,
       24, 24, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 28, 28, 29, 30, 30, 31, 32, 32, 33, 33,
       34, 34, 34, 34, 34, 34, 34, 34, 33, 34, 34, 34, 35, 36, 37, 38, 39, 41, 42, 43, 45, 46, 47,
       48, 49, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 51, 51, 51, 50, 50, 49,
       47, 46, 45, 43, 42, 40, 39, 38, 37, 36, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 36, 35,
       34, 33, 32, 31, 29, 28, 27, 27, 26, 26, 26, 26, 26, 27, 27, 27, 28, 28, 28, 27, 27, 26, 25,
       24, 23, 22, 22, 21, 20, 20, 20, 21, 21, 22, 23, 24, 25, 26, 26, 27, 28, 28, 28, 28, 28, 28,
       27, 27, 26, 26, 26, 26, 26, 27, 27, 28, 29, 30, 30, 31, 32, 33, 34, 35, 36, 37, 37, 38, 39,
       40, 41, 42, 43, 44, 45, 46, 46, 47, 47, 47, 46, 46, 45, 44, 43, 42, 40, 39, 39, 38, 38, 38,
       38, 39, 41, 42, 44, 45, 47, 49, 50, 51, 52, 52, 52, 52, 51, 50, 48, 46, 45, 43, 42, 40, 39,
       38, 38, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 37, 37, 36, 36, 35, 34, 34, 33, 32, 32, 31,
       31, 30, 30, 29, 28, 27, 26, 24, 22, 20, 18, 16, 14, 12, 10, 9,  8,  7,  7,  7,  7,  8,  9,
       10, 12, 13, 15, 16, 17, 18, 19, 20, 20, 21, 21, 21, 21, 21, 21, 21, 22, 23, 24, 25, 26, 27,
       29, 30, 32, 33, 35, 36, 38, 39, 40, 42, 43, 44, 45, 46, 46, 47, 48, 49, 49, 49, 49, 49, 49,
       48, 48, 47, 45, 44, 42, 41, 40, 38, 37, 36, 36, 36, 36, 36, 37, 39, 40, 42, 43, 45, 47, 48,
       49, 50, 50, 50, 50, 49, 47, 46, 43, 41, 39, 36, 34, 32, 29, 28, 26, 25, 24, 24, 23, 23, 24,
       24, 25, 25, 26, 26, 26, 26, 26, 26, 25, 24, 23, 22, 21, 19, 18, 17, 16, 15, 14, 13, 13, 13,
       13, 14, 14, 15, 16, 17, 17, 18, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 22,
       23, 24, 25, 26, 27, 28, 29, 30, 31, 31, 32, 32, 32, 33, 33, 33, 33, 33, 33, 34, 34, 35, 36,
       38, 39, 40, 42, 43, 45, 46, 47, 48, 48, 48, 48, 48, 47, 47, 46, 45, 44, 43, 42, 42, 41, 41,
       41, 41, 42, 42, 42, 43, 43, 44, 44, 44, 43, 43, 43, 42, 41, 40, 40, 39, 38, 38, 38, 38, 38,
       38, 38, 39, 39, 39, 39, 39, 39, 38, 37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 18, 17, 17,
       18, 19, 20, 21, 22, 23, 23, 24, 24, 24, 23, 22, 21, 19, 17, 16, 14, 13, 12, 11, 11, 11, 11,
       12, 14, 15, 17, 19, 21, 23, 24, 25, 26, 27, 28, 28, 28, 28, 28, 28, 29, 29, 30, 31, 33, 35,
       36, 38, 40, 42, 43, 44, 45, 46, 46, 45, 45, 44, 42, 41, 40, 39, 39, 38, 39, 39, 40, 42, 44,
       46, 48, 50, 52, 53, 55, 56, 56, 56, 56, 55, 54, 52, 51, 49, 47, 46, 45, 44, 44, 44, 44, 44,
       45, 45, 46, 47, 47, 47, 47, 47, 46, 46, 45, 43, 42, 41, 40, 39, 37, 37, 36, 35, 34, 34, 33,
       33, 32, 31, 30, 29, 27, 26, 25, 23, 22, 20, 19, 19, 18, 18, 18, 18, 19, 19, 20, 21, 22, 22,
       23, 23, 23, 22, 21, 21, 20, 19, 18, 17, 16, 16, 16, 17, 18, 19, 21, 23, 26, 28, 30, 33, 35,
       37, 38, 39, 40, 40, 40, 40, 39, 39, 38, 37, 37, 36, 36, 36, 36, 37, 37, 38, 39, 40, 41, 42,
       43, 44, 44, 45, 45, 45, 45, 45, 45, 44, 44, 44, 43, 43, 43, 42, 42, 42, 42, 43, 43, 43, 44,
       44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 53, 53, 53, 53, 52, 51, 50, 48, 46, 43, 40, 37, 34,
       32, 29, 26, 24, 22, 20, 19, 19, 19, 19, 19, 20, 21, 22, 24, 25, 26, 27, 28, 29, 29, 29, 29,
       28, 28, 27, 26, 25, 24, 22, 21, 20, 18, 17, 16, 14, 13, 12, 11, 10, 10, 9,  9,  9,  10, 10,
       11, 13, 14, 16, 17, 19, 21, 23, 24, 26, 27, 28, 29, 29, 30, 30, 30, 30, 31, 31, 31, 32, 32,
       33, 34, 35, 36, 37, 39, 40, 41, 41, 42, 42, 43, 43, 43, 43, 42, 42, 42, 42, 42, 42, 43, 43,
       44, 45, 46, 47, 47, 48, 49, 49, 49, 49, 49, 49, 48, 47, 47, 46, 45, 44, 44, 44, 43, 43, 43,
       43, 43, 43, 43, 43, 43, 42, 41, 40, 39, 37, 36, 34, 33, 31, 30, 29, 28, 27, 27, 27, 27, 27,
       27, 27, 27, 26, 26, 25, 24, 23, 21, 20, 18, 17, 15, 14, 14, 13, 13, 14, 15, 16, 18, 20, 21,
       23, 25, 27, 28, 29, 30, 30, 29, 29, 28, 27, 25, 24, 23, 22, 22, 21, 21, 22, 23, 24, 25, 26,
       27, 29, 30, 31, 31, 32, 32, 32, 32, 31, 31, 30, 30, 30, 29};

  // Set data pointers within the PredictedSpeeds class (mimic how this might look for a single
  // directed edge)
  uint32_t indexes[] = {0};
  PredictedSpeeds pred_speeds;
  pred_speeds.set_offset(indexes);
  pred_speeds.set_profiles(coefficients);

  // Test against 5 minute bucket values
  std::cout << std::endl;
  for (int i = 0; i < 2016; ++i) {
    uint32_t secs = i * 5 * 60;
    uint32_t s = static_cast<uint32_t>(pred_speeds.speed(0, secs) + 0.5f);
    if (!within_threshold(s, speeds[i])) {
      throw std::runtime_error("Speed outside of range");
    }
  }
}

/**
 * Test to check for negative speeds in an encoded predicted speed string. If we find cases
 * where we see a negative speed we should trace it back to a particular entry in the
 * csv input file and change the encoded speed string here to see if the issue is valid.
 */
void test_negative_speeds() {
  // base64 encoded string
  std::string encoded_speed_string =
      "AQRu//UAEAAC/+4AA//6//gAAwAFAA//9wAHAAH/4AAd/+wACwAH//0AGQAYAA7//wANAAL/9//mAAUACgATAAb/8v/2//8AC//1ABMAAAAGABX/9//0//0AAAAQAAIAAv/6////9gAJAAcACf/zAAQAAwAC//oACf/2//sADQAVABD/+QADAAcACf/2//gABwAHAAAABv/9AAf/+QAM//kAEAAE//r//wAMAAD/9AAN//D/7QAK//EAE//7AAkAAQAF//f/+AAB//z/6f/y//MAAP/6ABL//AATABX//wAFAAMAGv/2AAf//wAI//sACv/5AAb/8gAOAAYADv/5AAMACP////T/7gAH//P/+f/9//n/9f/0//0AAwAP//3/8gAA//8ACv////gAAgAHAAP//QALAAcAFAAA//8ABP/vAAIAEAAM/+3/9QAC//j//v/tABj/+wAA//sAC//6//0ABwAAAAoABgAMAAb/+P/3AAX/9//7//0ADP/sAAwAB//v/+3//wAMABAACgAF//o=";

  // Decode the base64 string and cast the data to a raw string of signed bytes
  auto decoded_str = decode64(encoded_speed_string);
  if (decoded_str.size() != 402) {
    throw std::runtime_error("Decoded speed string size should be 402 but is " +
                             std::to_string(decoded_str.size()));
  }
  auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

  // Check that the first value pair == 1
  if (static_cast<std::int8_t>(raw[0]) != 1) {
    throw std::runtime_error("First value should be 1");
  }

  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number (big endian).
  // Convert to little endian.
  int idx = 1;
  int16_t coefficients[200];
  for (uint32_t i = 0; i < 200; ++i, idx += 2) {
    coefficients[i] = to_little_endian(*(reinterpret_cast<const int16_t*>(&raw[idx])));
  }

  // Set data pointers within the PredictedSpeeds class (mimic how this might look for a single
  // directed edge)
  uint32_t indexes[] = {0};
  PredictedSpeeds pred_speeds;
  pred_speeds.set_offset(indexes);
  pred_speeds.set_profiles(coefficients);

  // Test against 5 minute bucket values
  std::cout << std::endl;
  for (int i = 0; i < 2016; ++i) {
    uint32_t secs = i * 5 * 60;
    float s = pred_speeds.speed(0, secs);
    if (s < 0.0f) {
      throw std::runtime_error("Negative speed");
    }
  }
}

} // namespace

int main(void) {
  test::suite suite("predictedspeeds");

  suite.test(TEST_CASE(test_free_flow_speed));

  suite.test(TEST_CASE(test_decoding));

  suite.test(TEST_CASE(test_negative_speeds));

  return suite.tear_down();
}
