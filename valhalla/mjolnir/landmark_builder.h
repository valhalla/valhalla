#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <unordered_map>

namespace valhalla {
namespace mjolnir {
using Landmark = std::tuple<int64_t, std::string, LandmarkType, double, double>;

/**
 * Convert a Landmark object to a string which consists of 1 byte for kLandmark tag,
 * 1 byte for landmark type, 4 bytes for location (lng and lat), and landmark name.
 *
 * @param landmark The Landmark object to be converted.
 */
std::string landmark_to_str(const Landmark& landmark) {
  std::string tagged_value(1, static_cast<char>(baldr::TaggedValue::kLandmark));
  tagged_value.push_back(static_cast<std::string::value_type>(std::get<2>(landmark)));
  uint32_t location = uint32_t((std::get<3>(landmark) + 180) * 1e7) << 16 |
                      uint32_t((std::get<4>(landmark) + 90) * 1e7) << 1; // leaves one spare bit
  tagged_value += std::string(static_cast<const char*>(static_cast<void*>(&location)), 4);
  tagged_value += std::get<1>(landmark);

  return tagged_value;
}

// Convert a string to a Landmark object
Landmark str_to_landmark(const std::string& str) {
  // Ensure that the string has the minimum expected size to represent a Landmark
  if (str.size() < 7) { // or 6 = 1 + 1 + 4?
    throw std::runtime_error("Invalid Landmark string: too short");
  }

  // Ensure that the first byte is the kLandmark tag
  if (str[0] != static_cast<char>(baldr::TaggedValue::kLandmark)) {
    throw std::runtime_error("Invalid Landmark string: missing kLandmark tag");
  }

  // Extract the LandmarkType (second byte in the string)
  LandmarkType landmark_type = static_cast<LandmarkType>(static_cast<uint8_t>(str[1]));

  // Extract the location (next 4 bytes) - lng and lat
  uint32_t location = 0;
  std::memcpy(&location, str.data() + 2, 4);
  double lng = (static_cast<double>((location >> 16) & 0x7FFFFFFF) / 1e7) - 180;
  double lat = (static_cast<double>((location >> 1) & 0x7FFFFFFF) / 1e7) - 90;

  // Extract the name (rest of the string after the first 6 bytes)
  std::string name = str.substr(6);

  return std::make_tuple(0, name, landmark_type, lng,
                         lat); // The first element (int64_t) is not used in this reverse conversion
}

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

struct LandmarkDatabase {
public:
  /**
   * LandmarkDatabase constructor.
   * Create a new LandmarkDatabase object that connects to the SQLite landmark database
   * with the given `db_name`. The `read_only` parameter determines whether the
   * database connection is read-only or read-write.
   *
   * @param db_name The file path of the SQLite database to connect to.
   * @param read_only Set to true to open the database in read-only mode, false for read-write.
   */
  LandmarkDatabase(const std::string& db_name, bool read_only);

  /**
   * Insert a new landmark into the database.
   * This function inserts a new landmark into the database with the given `name`, `type`,
   * `lng`, and `lat`. The landmark's primary key will be automatically assigned by the database.
   *
   * @param name The name of the landmark.
   * @param type The type of the landmark (LandmarkType enum).
   * @param lng The longitude of the landmark.
   * @param lat The latitude of the landmark.
   */
  void insert_landmark(const std::string& name,
                       const LandmarkType& type,
                       const double lng,
                       const double lat);

  /**
   * Retrieve a vector of landmarks from the database within the specified bounding box.
   * Bounding box is a rectangle area defined by min latitude, max latitude, min longitude, and max
   * longitude.
   *
   * @param minLat The minimum latitude of the bounding box.
   * @param minLong The minimum longitude of the bounding box.
   * @param maxLat The maximum latitude of the bounding box.
   * @param maxLong The maximum longitude of the bounding box.
   * @return A vector of Landmark objects within the specified bounding box.
   */
  std::vector<Landmark> get_landmarks_by_bbox(const double minLat,
                                              const double minLong,
                                              const double maxLat,
                                              const double maxLong);

  /**
   * Retrieve a vector of landmarks that match the provided primary keys.
   * NOTE: The return size may be less than the size of the input if some of the provided
   * primary keys do not exist in the database. In such cases, the function will skip the missing IDs
   * without throwing an exception.
   *
   * @param pkeys A vector of int64_t representing the primary keys of the landmarks to retrieve.
   * @return A vector of Landmark objects matching the given primary keys.
   */
  std::vector<Landmark> get_landmarks_by_ids(const std::vector<int64_t>& pkeys);

protected:
  struct db_pimpl;
  std::shared_ptr<db_pimpl> pimpl;
};

/**
 * Build a SQLite landmark database and insert landmarks data from the PBF files to the database.
 * This function reads landmark data from PBF files specified in the `input_files` vector.
 * It then parses the data to extract landmark nodes and stores the landmarks in the database.
 * The database is automatically built based on the configuration settings provided in `pt`.
 *
 * @param pt The configuration settings for the landmark building process.
 * @param input_files A vector of file paths to the PBF input files.
 * @return True if landmarks were successfully built and inserted into the database, false otherwise.
 */
bool BuildLandmarkFromPBF(const boost::property_tree::ptree& pt,
                          const std::vector<std::string>& input_files);

} // namespace mjolnir

} // namespace valhalla
