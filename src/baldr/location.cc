
#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/location.h"
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>

namespace valhalla{
namespace baldr{

  Location::Location(const midgard::PointLL& latlng, const StopType& stoptype):latlng_(latlng), stoptype_(stoptype){
  }

Location Location::FromPtree(const boost::property_tree::ptree& pt) {

  Location location(
      { pt.get<float>("longitude"), pt.get<float>("latitude") },
      (pt.get<std::string>("type") == "through" ?
          StopType::THROUGH : StopType::BREAK));
  //LOG_INFO("LAT=" + std::to_string(location.latlng_.lat()));
  //LOG_INFO("LNG=" + std::to_string(location.latlng_.lng()));
  //LOG_INFO("TYPE=" + pt.get<std::string>("type"));

  auto heading_ptr = pt.get_optional<int>("heading");
  if (heading_ptr) {
    location.heading_ =  std::to_string(*heading_ptr);
    //LOG_INFO("HEADING=" + location.heading_);
  }

  auto name_ptr = pt.get_optional<std::string>("name");
  if (name_ptr) {
    location.name_ = *name_ptr;
    //LOG_INFO("NAME=" + location.name_);
  }

  auto street_ptr = pt.get_optional<std::string>("street");
  if (street_ptr) {
    location.street_ = *street_ptr;
    //LOG_INFO("STREET=" + location.street_);
  }

  auto city_ptr = pt.get_optional<std::string>("city");
  if (city_ptr) {
    location.city_ = *city_ptr;
    //LOG_INFO("CITY=" + location.city_);
  }

  auto state_ptr = pt.get_optional<std::string>("state");
  if (state_ptr) {
    location.state_ = *state_ptr;
    //LOG_INFO("STATE=" + location.state_);
  }

  auto postal_code_ptr = pt.get_optional<std::string>("postal_code");
  if (postal_code_ptr) {
    location.zip_ = *postal_code_ptr;
    //LOG_INFO("POSTAL_CODE=" + location.zip_);
  }

  auto country_ptr = pt.get_optional<std::string>("country");
  if (country_ptr) {
    location.country_ = *country_ptr;
    //LOG_INFO("COUNTRY=" + location.country_);
  }

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
    if(parts.size() < 2)
      throw std::runtime_error("Bad format for csv formated location");

    // Validate lat,lng input. Longitude must be between -360 and 360. Values outside
    // -180 to 180 are converted to lie within the range [-180,180]
    float lat = std::stof(parts[0]);
    if (lat < -90.0f || lat > 90.0f) {
      throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
    }
    float lng = std::stof(parts[1]);
    if (lng < -360.0f || lng > 360.0f) {
      throw std::runtime_error("Longitude must be in the range [-360, 360] degrees");
    }
    if (lng < -180.0f) {
      lng += 360.0f;
    } else if (lng > 180.0f) {
      lng -= 360.0f;
    }

    //make the lng, lat and check for info about the stop type
    Location l({lng, lat},
      (parts.size() > 2 && parts[2] == "through" ? StopType::THROUGH : StopType::BREAK));

    //grab some address info
    if (parts.size() > 3) {
      auto part = parts.begin() + 3;
      for(auto address : { &l.name_, &l.street_, &l.city_, &l.state_, &l.zip_, &l.country_ }) {
        if(part == parts.end())
          break;
        address->swap(*part);
        ++part;
      }
    }

    if (parts.size() > 9) {
      l.heading_ = parts[9];
    }

    return l;
  }

}
}
