#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <valhalla/baldr/graphconstants.h>
#include <valhalla/midgard/util.h>

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
  bank = 9,
  // same with this one, in some countries these are obvious and have the little plus sign, in america
  // its not the case and they can be different chains or could be part of a grocery store or
  // anything... speaking of grocery store, why isnt that on the list? also why arent department
  // stores on the list, surely stores would be a good landmark to use (again when they are named)
  pharmacy = 10,
  // again these could just be a random building with nothing other than a sign like "early years" or
  // something, they better have a name or it will be ambiguous
  kindergarten = 11,
  // same thing, often just a building
  bar = 12,
  // this could be very very big like a college campus, i think we should remove this
  hospital = 13,
  // same as bar could be just a building need at least a name to have a chance of seeing it
  pub = 14,
  // generic, could be many many types of clinics also often there are multiple of these located in
  // same "complex", you might have a dentist, orthodontist, ear nose throat and eye specialists all
  // next to each other.
  clinic = 15,
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

constexpr uint8_t LandmarkTypeFirstValue = static_cast<uint8_t>(LandmarkType::fuel);
constexpr uint8_t LandmarkTypeLastValue = static_cast<uint8_t>(LandmarkType::casino);

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

struct Landmark {
  int64_t id;
  std::string name;
  LandmarkType type;
  double lng;
  double lat;

  Landmark(const int64_t id,
           const std::string name,
           const LandmarkType type,
           const double lng,
           const double lat)
      : id(id), name(name), type(type), lng(lng), lat(lat) {
  }

  /**
   * Constructor: convert the given string to a Landmark object.
   * The input string should consist of at least 9 bytes: 1 byte for landmark type,
   * 8 bytes for location (lng and lat), and the remaining for landmark name.
   *
   * @param str The string to be converted.
   */
  Landmark(const std::string& str) : id(0) {
    // needs at least 8 bytes for ll, 1 for type and 1 for the name
    if (str.size() < 10) {
      throw std::runtime_error("Invalid Landmark string: too short");
    }
    name = str.substr(9);

    // and that the type is valid
    if (str[8] < LandmarkTypeFirstValue || str[8] > LandmarkTypeLastValue) {
      throw std::logic_error("Invalid landmark type");
    }
    type = static_cast<LandmarkType>(str[8]);

    // extract the ll
    uint64_t ll = midgard::unaligned_read<uint64_t>(static_cast<const void*>(str.data()));
    lat = ((ll >> 32) & ((1ull << 31) - 1)) / static_cast<double>(1e7) - 90;
    lng = (ll & ((1ull << 32) - 1)) / static_cast<double>(1e7) - 180;
  }

  /**
   * Convert a Landmark object to a string.
   * The string consists of 1 byte of kLandmark tag, 1 byte of landmark type,
   * 8 bytes of location (lng and lat) and some bytes for landmark name at last.
   *
   * @param landmark The Landmark object to be converted.
   */
  std::string to_str() const {
    // lon,lat stuffed into 63 bits at 7digit precision
    uint64_t ll = uint64_t((lng + 180) * 1e7 + .5) | (uint64_t((lat + 90) * 1e7 + .5) << 32);
    std::string value(static_cast<const char*>(static_cast<void*>(&ll)), sizeof(ll));
    value.push_back(static_cast<std::string::value_type>(type));
    value += name;
    return value;
  }
};

} // namespace baldr
} // namespace valhalla
