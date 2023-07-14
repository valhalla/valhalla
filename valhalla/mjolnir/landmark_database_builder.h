#pragma once

#include <memory>
#include <string>
#include <vector>

#include <string>
#include <unordered_map>
namespace valhalla {
namespace mjolnir {

// most 15 popular landmark types
enum class LandmarkType : uint8_t {
  NA = 0,
  parking = 1,
  bench = 2,
  parking_space = 3,
  place_of_worship = 4,
  restaurant = 5,
  waste_basket = 6,
  bicycle_parking = 7,
  fast_food = 8,
  cafe = 9,
  fuel = 10,
  shelter = 11,
  recycling = 12,
  toilets = 13,
  bank = 14,
  pharmacy = 15,
};

struct Landmark {
  std::string name;
  LandmarkType type;
  double lng;
  double lat;
};

struct LandmarkDatabase {
public:
  LandmarkDatabase(const std::string& db_name, bool read_only);
  void insert_landmark(const Landmark& landmark);
  std::vector<Landmark> get_landmarks_in_bounding_box(const double minLat,
                                                      const double minLong,
                                                      const double maxLat,
                                                      const double maxLong);

protected:
  struct db_pimpl;
  std::shared_ptr<db_pimpl> pimpl;
};

// class?
struct LandmarkParser {
public:
  static std::vector<Landmark> Parse(const std::vector<std::string>& input_files);
};

bool BuildLandmarkFromPBF(const std::vector<std::string>& input_files);

} // namespace mjolnir

} // namespace valhalla
