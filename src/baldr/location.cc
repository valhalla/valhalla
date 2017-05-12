#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/pointll.h"
#include "midgard/logging.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

Location::Location(const midgard::PointLL& latlng, const StopType& stoptype, unsigned int minimum_reachability, unsigned long radius)
  : latlng_(latlng), stoptype_(stoptype), minimum_reachability_(minimum_reachability), radius_(radius) {
}

boost::property_tree::ptree Location::ToPtree() const {
  boost::property_tree::ptree location;
  location.put("lat", latlng_.lat());
  location.put("lon", latlng_.lng());
  if (stoptype_ == StopType::THROUGH)
    location.put("type", "through");
  else
    location.put("type", "break");

  if(!name_.empty())
    location.put("name", name_);
  if(!street_.empty())
    location.put("street", street_);
  if(!city_.empty())
    location.put("city", city_);
  if(!state_.empty())
    location.put("state", state_);
  if(!zip_.empty())
    location.put("postal_code", zip_);
  if(!country_.empty())
    location.put("country", country_);

  if(date_time_ && !(*date_time_).empty())
    location.put("date_time", *date_time_);
  if(heading_)
    location.put("heading", *heading_);
  if(heading_tolerance_)
    location.put("heading_tolerance", *heading_tolerance_);
  if(way_id_)
    location.put("way_id", *way_id_);

  location.put("minimum_reachability", minimum_reachability_);
  location.put("radius", radius_);

  return location;
}

rapidjson::Value Location::ToRapidJson(rapidjson::Document::AllocatorType& a) const {
  rapidjson::Value location(rapidjson::kObjectType);

  location.AddMember("lat", latlng_.lat(), a);
  location.AddMember("lon", latlng_.lng(), a);
  if (stoptype_ == StopType::THROUGH)
    location.AddMember("type", "through", a);
  else
    location.AddMember("type", "break", a);

  if(!name_.empty())
    location.AddMember("name", name_, a);
  if(!street_.empty())
    location.AddMember("street", street_, a);
  if(!city_.empty())
    location.AddMember("city", city_, a);
  if(!state_.empty())
    location.AddMember("state", state_, a);
  if(!zip_.empty())
    location.AddMember("postal_code", zip_, a);
  if(!country_.empty())
    location.AddMember("country", country_, a);
  if(date_time_ && !date_time_->empty())
    location.AddMember("date_time", *date_time_, a);
  if(heading_)
    location.AddMember("heading", *heading_, a);
  if(way_id_)
    location.AddMember("way_id", *way_id_, a);

  location.AddMember("minimum_reachability", minimum_reachability_, a);
  location.AddMember("radius", static_cast<unsigned int>(radius_), a);
  return location;
}

Location Location::FromPtree(const boost::property_tree::ptree& pt) {

  float lat = pt.get<float>("lat");
  if (lat < -90.0f || lat > 90.0f)
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
  float lon = midgard::circular_range_clamp<float>(pt.get<float>("lon"), -180, 180);

  Location location({ lon, lat },
    (pt.get<std::string>("type", "break") == "through" ?
      StopType::THROUGH : StopType::BREAK));

  location.name_ = pt.get<std::string>("name", "");
  location.street_ = pt.get<std::string>("street", "");
  location.city_ = pt.get<std::string>("city", "");
  location.state_ = pt.get<std::string>("state", "");
  location.zip_ = pt.get<std::string>("postal_code", "");
  location.country_ = pt.get<std::string>("country", "");

  location.date_time_ = pt.get_optional<std::string>("date_time");
  location.heading_ = pt.get_optional<float>("heading");
  location.heading_tolerance_ = pt.get_optional<float>("heading_tolerance");
  location.way_id_ = pt.get_optional<long double>("way_id");

  location.minimum_reachability_ = pt.get<unsigned int>("minimum_reachability", 50);
  location.radius_ = pt.get<unsigned long>("radius", 0);

  return location;
}

Location Location::FromRapidJson(const rapidjson::Value& d, unsigned int default_reachability, unsigned long default_radius){
  auto lat = GetOptionalFromRapidJson<float>(d, "/lat");
  if (! lat) throw std::runtime_error{"lat is missing"};

  if (*lat < -90.0f || *lat > 90.0f)
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");

  auto lon = GetOptionalFromRapidJson<float>(d, "/lon");
  if (! lon) throw std::runtime_error{"lon is missing"};

  lon = midgard::circular_range_clamp<float>(*lon, -180, 180);

  StopType stop_type{StopType::BREAK};
  auto stop_type_json = GetOptionalFromRapidJson<std::string>(d, "/type");
  if (stop_type_json && *stop_type_json == std::string("through")){
    stop_type = StopType::THROUGH;
  }

  Location location{{*lon,*lat}, stop_type};

  location.name_ = GetFromRapidJson<std::string>(d, "/name", "");
  location.street_ = GetFromRapidJson<std::string>(d, "/street", "");
  location.city_ = GetFromRapidJson<std::string>(d, "/city", "");
  location.state_ = GetFromRapidJson<std::string>(d, "/state", "");
  location.zip_ = GetFromRapidJson<std::string>(d, "/postal_code", "");
  location.country_ = GetFromRapidJson<std::string>(d, "/country", "");

  location.date_time_ = GetOptionalFromRapidJson<std::string>(d, "/date_time");
  location.heading_ = GetOptionalFromRapidJson<int>(d, "/heading");
  location.way_id_ = GetOptionalFromRapidJson<uint64_t>(d, "/way_id");

  location.minimum_reachability_ = GetFromRapidJson<unsigned int>(d, "/minimum_reachability", default_reachability);
  location.radius_ = GetFromRapidJson<unsigned int>(d, "/radius", default_radius);

  return location;
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
  Location l({ lon, lat },
    (parts.size() > 2 && parts[2] == "through" ?
      StopType::THROUGH : StopType::BREAK));

  return l;
}

bool Location::operator==(const Location& o) const {
  return latlng_ == o.latlng_ && stoptype_ == o.stoptype_ &&
         name_ == o.name_ && street_ == o.street_ && city_ == o.city_ &&
         state_ == o.state_ && zip_ == o.zip_ && country_ == o.country_ &&
         date_time_ == o.date_time_ && heading_ == o.heading_ &&
         heading_tolerance_ == o.heading_tolerance_ && way_id_ == o.way_id_
         && minimum_reachability_ == o.minimum_reachability_ && radius_ == o.radius_;
}

}
}
