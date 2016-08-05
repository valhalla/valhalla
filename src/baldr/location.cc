#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/location.h"
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/util.h>

namespace valhalla {
namespace baldr {

Location::Location(const midgard::PointLL& latlng, const StopType& stoptype)
    : latlng_(latlng),
      stoptype_(stoptype) {
}

Location Location::FromPtree(const boost::property_tree::ptree& pt) {

  float lat = pt.get<float>("lat");
  if (lat < -90.0f || lat > 90.0f)
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
  float lon = midgard::circular_range_clamp<float>(pt.get<float>("lon"), -180, 180);

  Location location(
      { lon, lat },
      (pt.get<std::string>("type", "break") == "through" ?
        StopType::THROUGH : StopType::BREAK));

  location.date_time_ = pt.get_optional<std::string>("date_time");
  location.heading_ = pt.get_optional<int>("heading");
  location.way_id_ = pt.get_optional<uint64_t>("way_id");

  auto name = pt.get_optional<std::string>("name");
  if (name)
    location.name_ = *name;

  auto street = pt.get_optional<std::string>("street");
  if (street)
    location.street_ = *street;

  auto city = pt.get_optional<std::string>("city");
  if (city)
    location.city_ = *city;

  auto state = pt.get_optional<std::string>("state");
  if (state)
    location.state_ = *state;

  auto postal_code = pt.get_optional<std::string>("postal_code");
  if (postal_code)
    location.zip_ = *postal_code;

  auto country = pt.get_optional<std::string>("country");
  if (country)
    location.country_ = *country;

  return location;
}

Location Location::FromJson(const std::string& json) {
  std::stringstream stream;
  stream << json;
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(stream, pt);
  return FromPtree(pt);
}

Location Location::FromCsv(const std::string& csv) {
  //split it up into parts
  std::vector<std::string> parts;
  boost::algorithm::split(parts, csv, boost::algorithm::is_any_of(","));
  if (parts.size() < 2)
    throw std::runtime_error("Bad format for csv formated location");

  float lat = std::stof(parts[0]);
  if (lat < -90.0f || lat > 90.0f)
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
  float lon = midgard::circular_range_clamp<float>(std::stof(parts[1]), -180, 180);

  //make the lng, lat and check for info about the stop type
  Location l(
      { lon, lat },
      (parts.size() > 2 && parts[2] == "through" ?
          StopType::THROUGH : StopType::BREAK));

  return l;
}

bool Location::operator==(const Location& o) const {
  return latlng_ == o.latlng_ && stoptype_ == o.stoptype_ &&
         name_ == o.name_ && street_ == o.street_ && city_ == o.city_ &&
         state_ == o.state_ && zip_ == o.zip_ && country_ == o.country_ &&
         date_time_ == o.date_time_ && heading_ == o.heading_ && way_id_ == o.way_id_;
}

}
}
