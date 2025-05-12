#include <iostream>

#include "baldr/predictedspeeds.h"
#include "midgard/util.h"

#include "test.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

// Check if the speed is within threshold for the test
constexpr uint32_t kSpeedErrorThreshold = 2;
inline bool within_threshold(const uint32_t v1, const uint32_t v2) {
  return (v2 > v1) ? (v2 - v1) < kSpeedErrorThreshold : (v1 - v2) < kSpeedErrorThreshold;
}

// normalized l1 norm of the vector
float normalized_l1_norm(const float* vec, uint32_t size) {
  assert(size > 0);
  float sum = 0.f;
  for (uint32_t i = 0; i < size; ++i)
    sum += fabs(vec[i]);
  return sum / size;
}

void try_free_flow_speed(const std::string& encoded_str,
                         const uint32_t exp_free_flow,
                         const uint32_t exp_constrained_flow) {
  auto decoded_data = decode64(encoded_str);
  const auto raw = reinterpret_cast<const unsigned char*>(decoded_data.data());
  std::size_t index = 0;
  uint32_t t = static_cast<std::uint32_t>(raw[index++] & 0x1f);
  EXPECT_EQ(t, 0);

  uint32_t free_flow_speed = static_cast<std::uint32_t>(raw[index++] & 0xff);
  EXPECT_EQ(free_flow_speed, exp_free_flow);

  uint32_t constrained_flow_speed = static_cast<std::uint32_t>(raw[index++] & 0xff);
  EXPECT_EQ(constrained_flow_speed, exp_constrained_flow);
}

TEST(PredictedSpeeds, test_free_flow_speed) {
  try_free_flow_speed("AAie", 8, 158);

  // Add additional cases below
  try_free_flow_speed("AACe", 0, 158);
}

TEST(PredictedSpeeds, test_decoding) {
  // base64 encoded string
  std::string encoded_speed_string =
      "AQXFAAkABAAhAAz/+//bABn/3wAMABsAEQAF//gAAAAdABQAEv/wABf//gAsAAkAKgAAACj/+gBDAAQAbAALAQQAKv63AAD/mwAM/87/7P/TAAX/2P/1//P//f/sAAn/z//xAA7//P/y//z/8v/x////+wAMABX/+f/6AA4AGQAEABX/9//vAAf/8gAfAAb/9AAFABH//P/0ABQABv/2////4//7//0AE//+//n/5AATAAcAAQAL/+v//P/3ABMAAAAU//L/+v/8AAAAEP/3AAsABQAE/9f/7AABAAwAAQAGABP//QAJ/+4AB//gABUAAf/+AAv/6P/oABP//gAAABX/5f/5AAT//v/5AAgABv/3AB7/6gAdAAL/+P/r//sACwADAAT/9wAE//MACAAK//cACv/4//sABAAA//j//P/7//H/9v/y//wACwAHAAYABv/4AAL/+QAKAB7//wAHABX/8wAQ/+wAFAAL/+7//AAIAAgADf/9AAz/4gAQ//X/9//+//j/9wAEAAz//wADAAc=";

  // Decode the base64 string and cast the data to a raw string of signed bytes
  auto decoded_str = decode64(encoded_speed_string);
  // HACK(mookerji): kDecodedSpeedSize+1 is the expected size (as opposed to kDecodedSpeedSize)
  // because we start reading from a 1-byte offset below at the little endian conversion. This is
  // actually a broken test fixture because we start reading from 0 in the actual CLI decoding in
  // src/mjolnir/valhalla_add_predicted_traffic.cc. To FIX this, we need to encode the speeds[]
  // array below and start the little endian conversion from 0 instead of 1.
  EXPECT_EQ(decoded_str.size(), kDecodedSpeedSize + 1);

  auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

  // Check that the first value pair == 1
  EXPECT_EQ(static_cast<std::int8_t>(raw[0]), 1) << "First value should be 1";

  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number (big endian).
  // Convert to little endian.
  int idx = 1;
  int16_t coefficients[kCoefficientCount];
  for (uint32_t i = 0; i < kCoefficientCount; ++i, idx += 2) {
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
    ASSERT_PRED2(within_threshold, s, speeds[i]) << "Speed outside of range";
  }
}

/**
 * Test to check for negative speeds in an encoded predicted speed string. If we find cases
 * where we see a negative speed we should trace it back to the input speeds and change the
 * encoded speed string here to see if the issue is valid.
 */
TEST(PredictedSpeeds, test_negative_speeds) {
  // base64 encoded string
  std::string encoded_speed_string =
      "AQRu//UAEAAC/+4AA//6//gAAwAFAA//9wAHAAH/4AAd/+wACwAH//0AGQAYAA7//wANAAL/9//mAAUACgATAAb/8v/2//8AC//1ABMAAAAGABX/9//0//0AAAAQAAIAAv/6////9gAJAAcACf/zAAQAAwAC//oACf/2//sADQAVABD/+QADAAcACf/2//gABwAHAAAABv/9AAf/+QAM//kAEAAE//r//wAMAAD/9AAN//D/7QAK//EAE//7AAkAAQAF//f/+AAB//z/6f/y//MAAP/6ABL//AATABX//wAFAAMAGv/2AAf//wAI//sACv/5AAb/8gAOAAYADv/5AAMACP////T/7gAH//P/+f/9//n/9f/0//0AAwAP//3/8gAA//8ACv////gAAgAHAAP//QALAAcAFAAA//8ABP/vAAIAEAAM/+3/9QAC//j//v/tABj/+wAA//sAC//6//0ABwAAAAoABgAMAAb/+P/3AAX/9//7//0ADP/sAAwAB//v/+3//wAMABAACgAF//o=";

  // Decode the base64 string and cast the data to a raw string of signed bytes
  auto decoded_str = decode64(encoded_speed_string);
  // HACK(mookerji): See note above.
  EXPECT_EQ(decoded_str.size(), kDecodedSpeedSize + 1);

  auto raw = reinterpret_cast<const int8_t*>(decoded_str.data());

  // Check that the first value pair == 1
  EXPECT_EQ(static_cast<std::int8_t>(raw[0]), 1) << "First value should be 1";

  // Create the coefficients. Each group of 2 bytes represents a signed, int16 number (big endian).
  // Convert to little endian.
  int idx = 1;
  int16_t coefficients[kCoefficientCount];
  for (uint32_t i = 0; i < kCoefficientCount; ++i, idx += 2) {
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
    ASSERT_GE(s, 0.0f) << "Negative speed";
  }
}

TEST(PredictedSpeeds, test_compress_decompress_accuracy) {
  // generate speed values for buckets
  std::array<float, kBucketsPerWeek> speeds;
  for (uint32_t i = 0; i < kBucketsPerWeek; ++i)
    speeds[i] = roundf(30.f + 15.f * sin(i / 20.f));

  // compress speed buckets
  auto compressed_speeds = compress_speed_buckets(speeds.data());

  // decompress speed buckets
  std::array<float, kBucketsPerWeek> decompressed_speeds;
  for (uint32_t i = 0; i < kBucketsPerWeek; ++i)
    decompressed_speeds[i] = decompress_speed_bucket(compressed_speeds.data(), i);

  std::array<float, kBucketsPerWeek> diff_speeds;
  for (uint32_t i = 0; i < kBucketsPerWeek; ++i)
    diff_speeds[i] = fabs(speeds[i] - decompressed_speeds[i]);

  // check that average error of decompressing no more than threshold
  float l1_err = normalized_l1_norm(diff_speeds.data(), diff_speeds.size());
  EXPECT_LE(l1_err, 1.f) << "Low decompression accuracy"; // <= 1 KPH

  // check that all decompressed speed values differ from original by no more than threshold
  float max_diff = *std::max_element(diff_speeds.begin(), diff_speeds.end());
  EXPECT_LE(max_diff, 2.f) << "Low decompression accuracy"; // <= 2 KPH
}

struct EncoderDecoderTest : public ::testing::Test {
  EncoderDecoderTest() {
    // fill in coefficients
    for (size_t i = 0; i < coefficients.size(); ++i)
      coefficients[i] = (i % 2 == 0) ? (10 * i) : (-10 * i);
    // set corresponding encoded string
    encoded =
        "AAD/9gAU/+IAKP/OADz/ugBQ/6YAZP+SAHj/fgCM/2oAoP9WALT/QgDI/y4A3P8aAPD/BgEE/vIBGP7eASz+ygFA/rYBVP6iAWj+jgF8/noBkP5mAaT+UgG4/j4BzP4qAeD+FgH0/gICCP3uAhz92gIw/cYCRP2yAlj9ngJs/YoCgP12ApT9YgKo/U4CvP06AtD9JgLk/RIC+Pz+Awz86gMg/NYDNPzCA0j8rgNc/JoDcPyGA4T8cgOY/F4DrPxKA8D8NgPU/CID6PwOA/z7+gQQ++YEJPvSBDj7vgRM+6oEYPuWBHT7ggSI+24EnPtaBLD7RgTE+zIE2PseBOz7CgUA+vYFFPriBSj6zgU8+roFUPqmBWT6kgV4+n4FjPpqBaD6VgW0+kIFyPouBdz6GgXw+gYGBPnyBhj53gYs+coGQPm2BlT5ogZo+Y4GfPl6BpD5Zgak+VIGuPk+Bsz5Kgbg+RYG9PkCBwj47gcc+NoHMPjGB0T4sgdY+J4HbPiKB4D4dgeU+GIHqPhOB7z4Og==";
  }

  std::array<int16_t, kCoefficientCount> coefficients;
  std::string encoded;
};

TEST_F(EncoderDecoderTest, test_speeds_encoder) {
  // encode coefficients: base64-string for array values in big endian format
  auto my_encoded = encode_compressed_speeds(coefficients.data());
  // check encoded string
  ASSERT_EQ(encoded, my_encoded) << "Incorrect encoded string";
}

TEST_F(EncoderDecoderTest, test_speeds_decoder) {
  // decode coefficients
  auto my_coefficients = decode_compressed_speeds(encoded);
  // check decoded values
  ASSERT_TRUE(std::equal(coefficients.begin(), coefficients.end(), my_coefficients.begin()))
      << "Incorrect decoded coefficients";
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
