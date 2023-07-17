#pragma once

#include <memory>
#include <string>
#include <vector>

#include <string>
#include <unordered_map>
namespace valhalla {
namespace mjolnir {
static const std::string default_landmark_name = "";

// obvious landmark types for vehicle routing
enum class LandmarkType : uint8_t {
  null = 0,
  restaurant = 1,
  school = 2,
  fast_food = 3,
  cafe = 4,
  fuel = 5,
  bank = 6,
  pharmacy = 7,
  kindergarten = 8,
  bar = 9,
  hospital = 10,
  post_office = 11,
  pub = 12,
  clinic = 13,
  police = 14,
  fire_station = 15,
  car_wash = 16,
  charging_station = 17,
  college = 18,
  university = 19,
  theatre = 20,
  cinema = 21
};

inline LandmarkType string_to_landmark_type(const std::string& s) {
  static const std::unordered_map<std::string, LandmarkType> string_to_landmark_type =
      {{"restaurant", LandmarkType::restaurant},
       {"school", LandmarkType::school},
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
       {"charging_station", LandmarkType::charging_station},
       {"college", LandmarkType::college},
       {"university", LandmarkType::university},
       {"theatre", LandmarkType::theatre},
       {"cinema", LandmarkType::cinema}};

  auto it = string_to_landmark_type.find(s);
  if (it != string_to_landmark_type.cend()) {
    return it->second;
  }
  return LandmarkType::null;
}

inline std::string landmark_type_to_string(LandmarkType t) {
  static const std::unordered_map<uint8_t, std::string> landmark_type_to_string =
      {{static_cast<uint8_t>(LandmarkType::null), "na"},
       {static_cast<uint8_t>(LandmarkType::restaurant), "restaurant"},
       {static_cast<uint8_t>(LandmarkType::school), "school"},
       {static_cast<uint8_t>(LandmarkType::fast_food), "fast_food"},
       {static_cast<uint8_t>(LandmarkType::cafe), "cafe"},
       {static_cast<uint8_t>(LandmarkType::fuel), "fuel"},
       {static_cast<uint8_t>(LandmarkType::bank), "bank"},
       {static_cast<uint8_t>(LandmarkType::pharmacy), "pharmacy"},
       {static_cast<uint8_t>(LandmarkType::kindergarten), "kindergarten"},
       {static_cast<uint8_t>(LandmarkType::bar), "bar"},
       {static_cast<uint8_t>(LandmarkType::hospital), "hospital"},
       {static_cast<uint8_t>(LandmarkType::post_office), "post_office"},
       {static_cast<uint8_t>(LandmarkType::pub), "pub"},
       {static_cast<uint8_t>(LandmarkType::clinic), "clinic"},
       {static_cast<uint8_t>(LandmarkType::police), "police"},
       {static_cast<uint8_t>(LandmarkType::fire_station), "fire_station"},
       {static_cast<uint8_t>(LandmarkType::car_wash), "car_wash"},
       {static_cast<uint8_t>(LandmarkType::charging_station), "charging_station"},
       {static_cast<uint8_t>(LandmarkType::college), "college"},
       {static_cast<uint8_t>(LandmarkType::university), "university"},
       {static_cast<uint8_t>(LandmarkType::theatre), "theatre"},
       {static_cast<uint8_t>(LandmarkType::cinema), "cinema"}};

  auto it = landmark_type_to_string.find(static_cast<uint8_t>(t));
  if (it != landmark_type_to_string.cend()) {
    return it->second;
  }
  return "null";
}

struct Landmark {
  std::string name = default_landmark_name;
  LandmarkType type = LandmarkType::null;
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

bool BuildLandmarkFromPBF(const std::vector<std::string>& input_files, const std::string& db_name);

} // namespace mjolnir

} // namespace valhalla
