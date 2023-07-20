#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <string>
#include <unordered_map>

namespace valhalla {
namespace mjolnir {
static const std::string default_landmark_name = "";

// obvious landmark types for vehicle routing
enum class LandmarkType : uint8_t {
  restaurant = 1,
  fast_food = 2,
  cafe = 3,
  fuel = 4,
  bank = 5,
  pharmacy = 6,
  kindergarten = 7,
  bar = 8,
  hospital = 9,
  post_office = 10,
  pub = 11,
  clinic = 12,
  police = 13,
  fire_station = 14,
  car_wash = 15,
  theatre = 16,
  cinema = 17,
  bus_station = 18,
  biergarten = 19,
  casino = 20,
};

using Landmark = std::tuple<std::string, LandmarkType, double, double>;

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
       {"bus_station", LandmarkType::bus_station},
       {"biergarten", LandmarkType::biergarten},
       {"casino", LandmarkType::casino}};

  auto it = string_to_landmark_type.find(s);
  if (it == string_to_landmark_type.cend()) {
    throw std::runtime_error("unknown landmark type");
  }
  return it->second;
}

struct LandmarkDatabase {
public:
  LandmarkDatabase(const std::string& db_name, bool read_only);

  void insert_landmark(const std::string& name,
                       const LandmarkType& type,
                       const double lng,
                       const double lat);
  std::vector<Landmark> get_landmarks_in_bounding_box(const double minLat,
                                                      const double minLong,
                                                      const double maxLat,
                                                      const double maxLong);

protected:
  struct db_pimpl;
  std::shared_ptr<db_pimpl> pimpl;
};

bool BuildLandmarkFromPBF(const boost::property_tree::ptree& pt,
                          const std::vector<std::string>& input_files);

} // namespace mjolnir

} // namespace valhalla
