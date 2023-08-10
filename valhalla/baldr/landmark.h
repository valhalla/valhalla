#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>

#include <valhalla/baldr/graphconstants.h>

namespace valhalla {
namespace baldr {

// obvious landmark types for vehicle routing
enum class LandmarkType : uint8_t {
  // these will almost always be obvious by their function and don't require a name
  fuel = 1,
  post_office = 2,
  police = 3,
  fire_station = 4,
  car_wash = 5,
  // could be obvious if its a chain but could also just be a building
  restaurant = 6,
  // this will always be a chain in which case the name is a required distinguisher (as they are often
  // next to each other on either side of a street)
  fast_food = 7,
  // this could be a chain making it easier to distinguish (starbucks) but could be a one off cafe in
  // which case the name will be required
  cafe = 8,
  // on the fence about this one, these are often obvious without a name, maybe should go in the upper
  // list?
  bank = 10,
  // same with this one, in some countries these are obvious and have the little plus sign, in america
  // its not the case and they can be different chains or could be part of a grocery store or
  // anything... speaking of grocery store, why isnt that on the list? also why arent department
  // stores on the list, surely stores would be a good landmark to use (again when they are named)
  pharmacy = 11,
  // again these could just be a random building with nothing other than a sign like "early years" or
  // something, they better have a name or it will be ambiguous
  kindergarten = 12,
  // same thing, often just a building
  bar = 13,
  // this could be very very big like a college campus, i think we should remove this
  hospital = 14,
  // same as bar could be just a building need at least a name to have a chance of seeing it
  pub = 16,
  // generic, could be many many types of clinics also often there are multiple of these located in
  // same "complex", you might have a dentist, orthodontist, ear nose throat and eye specialists all
  // next to each other.
  clinic = 17,
  // on the fence about this one, to me this would be in the same category as "museum" or something
  // where its so specialized maybe it doesnt need a name. what keeps me from say ing that is you
  // often cant tell from the outside what function these amenities serve until you read the name
  theatre = 16,
  // this could work without a name in my opinion, they often have a board out front that shows what
  // movies are playing which would probably remove the need for a name
  cinema = 17,
  // these are usually quite large as well but not usually multiple buildings, how often are they
  // located near one another though? if its often perhaps we need a name to differentiate between
  // this casino and the next one down on the next block? maybe that argument is a good argument for
  // everything has to have a name?
  casino = 18,
};

inline LandmarkType string_to_landmark_type(const std::string& s) {
  static const std::unordered_map<std::string, LandmarkType> string_to_landmark_type =
      {{"restaurant", LandmarkType::restaurant},
       {"fast_food", LandmarkType::fast_food},
       {"cafe", LandmarkType::cafe},
       {"fuel", LandmarkType::fuel},
       {"bank", LandmarkType::bank},
       {"pharmacy", LandmarkType::pharmacy},
       {"kindergarten", LandmarkType::kindergarten},
       {"bar", LandmarkType::bar},
       {"hospital", LandmarkType::hospital},
       {"post_office", LandmarkType::post_office},
       {"pub", LandmarkType::pub},
       {"clinic", LandmarkType::clinic},
       {"police", LandmarkType::police},
       {"fire_station", LandmarkType::fire_station},
       {"car_wash", LandmarkType::car_wash},
       {"theatre", LandmarkType::theatre},
       {"cinema", LandmarkType::cinema},
       {"casino", LandmarkType::casino}};

  auto it = string_to_landmark_type.find(s);
  if (it == string_to_landmark_type.cend()) {
    throw std::runtime_error("unknown landmark type");
  }
  return it->second;
}

inline int32_t deserialize_latlng(const char*& begin) {
  int32_t byte, shift = 0, result = 0;
  do {
    // take the least significant 7 bits shifted into place
    byte = int32_t(*begin++);
    result |= (byte & 0x7f) << shift;
    shift += 7;
    // if the most significant bit is set there is more to this number
  } while (byte & 0x80);

  return ((result & 1 ? ~result : result) >> 1);
}

inline std::string serialize_latlng(int number) {
  // get the sign bit down on the least significant end to
  // make the most significant bits mostly zeros
  std::string output{};

  number = number < 0 ? ~(static_cast<unsigned int>(number) << 1) : number << 1;
  // we take 7 bits of this at a time
  while (number > 0x7f) {
    // marking the most significant bit means there are more pieces to come
    int nextValue = (0x80 | (number & 0x7f));
    output.push_back(static_cast<char>(nextValue));
    number >>= 7;
  }
  // write the last chunk
  output.push_back(static_cast<char>(number & 0x7f));

  return output;
}

inline std::string encode_latlng(double d) {
  int int_d = static_cast<int>(round(d * 1e6));
  return serialize_latlng(int_d);
}

inline double decode_latlng(const char*& begin) {
  auto int_result = deserialize_latlng(begin);
  return (static_cast<double>(int_result) / 1e6);
}

struct Landmark {
  int64_t id;
  std::string name;
  LandmarkType type;
  double lng;
  double lat;

  std::string to_str() const;

  Landmark(const int64_t id,
           const std::string name,
           const LandmarkType type,
           const double lng,
           const double lat)
      : id(id), name(name), type(type), lng(lng), lat(lat) {
  }

  /**
   * Constructor: convert the given string to a Landmark object.
   * The input string should consist of 1 byte of LandmarkType,
   * some bytes for location (lng and lat) and landmark name.
   *
   * @param str The string to be converted.
   */
  Landmark(const std::string& str) {
    // ensure that the string has the minimum expected size to represent a Landmark
    if (str.size() < 3) { // TODO: any other size?
      throw std::runtime_error("Invalid Landmark string: too short");
    }

    // TODO: ensure that the first byte is in LandmarkType

    id = 0; // fake id

    type = static_cast<LandmarkType>(static_cast<uint8_t>(str[0]));

    const char* begin = str.data() + 1;
    lat = decode_latlng(begin);
    lng = decode_latlng(begin);

    name = begin;
  }
};

/**
 * Convert a Landmark object to a string which consists of 1 byte of kLandmark tag,
 * 1 byte of landmark type, dynamic bytes of location (lng and lat) and landmark name at last.
 *
 * @param landmark The Landmark object to be converted.
 */
inline std::string Landmark::to_str() const {
  std::string tagged_value(1, static_cast<std::string::value_type>(TaggedValue::kLandmark));

  tagged_value += std::string(1, static_cast<std::string::value_type>(type));
  tagged_value += encode_latlng(lat) + encode_latlng(lng);
  tagged_value += name;

  return tagged_value;
}

} // namespace baldr
} // namespace valhalla
