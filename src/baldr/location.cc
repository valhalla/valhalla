#include <stdexcept>

#include "baldr/location.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

namespace valhalla {
namespace baldr {

Location::Location(const midgard::PointLL& latlng,
                   const StopType& stoptype,
                   unsigned int minimum_reachability,
                   unsigned long radius)
    : latlng_(latlng), stoptype_(stoptype), minimum_reachability_(minimum_reachability),
      radius_(radius) {
}

rapidjson::Value Location::ToRapidJson(rapidjson::Document::AllocatorType& a) const {
  rapidjson::Value location(rapidjson::kObjectType);

  location.AddMember("lat", latlng_.lat(), a);
  location.AddMember("lon", latlng_.lng(), a);
  if (stoptype_ == StopType::THROUGH) {
    location.AddMember("type", "through", a);
  } else {
    location.AddMember("type", "break", a);
  }

  if (!name_.empty()) {
    location.AddMember("name", name_, a);
  }
  if (!street_.empty()) {
    location.AddMember("street", street_, a);
  }
  if (!city_.empty()) {
    location.AddMember("city", city_, a);
  }
  if (!state_.empty()) {
    location.AddMember("state", state_, a);
  }
  if (!zip_.empty()) {
    location.AddMember("postal_code", zip_, a);
  }
  if (!country_.empty()) {
    location.AddMember("country", country_, a);
  }
  if (date_time_ && !date_time_->empty()) {
    location.AddMember("date_time", *date_time_, a);
  }
  if (heading_) {
    location.AddMember("heading", *heading_, a);
  }
  if (heading_tolerance_) {
    location.AddMember("heading_tolerance", *heading_tolerance_, a);
  }
  if (node_snap_tolerance_) {
    location.AddMember("node_snap_tolerance", *node_snap_tolerance_, a);
  }
  if (way_id_) {
    location.AddMember("way_id", *way_id_, a);
  }

  location.AddMember("minimum_reachability", minimum_reachability_, a);
  location.AddMember("radius", static_cast<unsigned int>(radius_), a);
  return location;
}

Location Location::FromRapidJson(const rapidjson::Value& d,
                                 unsigned int default_reachability,
                                 unsigned long default_radius) {
  auto lat = rapidjson::get_optional<float>(d, "/lat");
  if (!lat) {
    throw std::runtime_error{"lat is missing"};
  };

  if (*lat < -90.0f || *lat > 90.0f) {
    throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
  }

  auto lon = rapidjson::get_optional<float>(d, "/lon");
  if (!lon) {
    throw std::runtime_error{"lon is missing"};
  };

  lon = midgard::circular_range_clamp<float>(*lon, -180, 180);

  StopType stop_type{StopType::BREAK};
  auto stop_type_json = rapidjson::get_optional<std::string>(d, "/type");
  if (stop_type_json && *stop_type_json == std::string("through")) {
    stop_type = StopType::THROUGH;
  }

  Location location{{*lon, *lat}, stop_type};

  location.name_ = rapidjson::get<std::string>(d, "/name", "");
  location.street_ = rapidjson::get<std::string>(d, "/street", "");
  location.city_ = rapidjson::get<std::string>(d, "/city", "");
  location.state_ = rapidjson::get<std::string>(d, "/state", "");
  location.zip_ = rapidjson::get<std::string>(d, "/postal_code", "");
  location.country_ = rapidjson::get<std::string>(d, "/country", "");

  location.date_time_ = rapidjson::get_optional<std::string>(d, "/date_time");
  location.heading_ = rapidjson::get_optional<int>(d, "/heading");
  location.heading_tolerance_ = rapidjson::get_optional<int>(d, "/heading_tolerance");
  location.node_snap_tolerance_ = rapidjson::get_optional<float>(d, "/node_snap_tolerance");
  location.way_id_ = rapidjson::get_optional<uint64_t>(d, "/way_id");

  location.minimum_reachability_ =
      rapidjson::get<unsigned int>(d, "/minimum_reachability", default_reachability);
  location.radius_ = rapidjson::get<unsigned int>(d, "/radius", default_radius);

  return location;
}

bool Location::operator==(const Location& o) const {
  return latlng_ == o.latlng_ && stoptype_ == o.stoptype_ && name_ == o.name_ &&
         street_ == o.street_ && city_ == o.city_ && state_ == o.state_ && zip_ == o.zip_ &&
         country_ == o.country_ && date_time_ == o.date_time_ && heading_ == o.heading_ &&
         heading_tolerance_ == o.heading_tolerance_ &&
         node_snap_tolerance_ == o.node_snap_tolerance_ && way_id_ == o.way_id_ &&
         minimum_reachability_ == o.minimum_reachability_ && radius_ == o.radius_;
}

} // namespace baldr
} // namespace valhalla
