#pragma once

#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/rapidjson_utils.h"
#include "midgard/constants.h"
#include "midgard/logging.h"

#include <cstdint>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

struct VehicleSpeeds {
  uint32_t auto_;
  uint32_t truck_;

  VehicleSpeeds() : auto_(0), truck_(0){};
};

enum class LegalSpeedDomain : int8_t {
  kInvalid = -1,
  kRural = 0,
  kUrban = 1,
  kMotorway = 2,
  kLivingStreet = 3,
  kTrunk = 4,
  kService = 5,
  kFallback = 6
};

inline LegalSpeedDomain stringToLegalSpeedDomain(const std::string& s) {
  static const std::unordered_map<std::string, LegalSpeedDomain> stringToLegalSpeedDomain =
      {{"rural", LegalSpeedDomain::kRural},       {"urban", LegalSpeedDomain::kUrban},
       {"motorway", LegalSpeedDomain::kMotorway}, {"living street", LegalSpeedDomain::kLivingStreet},
       {"trunk", LegalSpeedDomain::kTrunk},       {"service road", LegalSpeedDomain::kService},
       {"fallback", LegalSpeedDomain::kFallback}};

  auto found = stringToLegalSpeedDomain.find(s);
  if (found == stringToLegalSpeedDomain.end()) {
    return LegalSpeedDomain::kInvalid;
  } else {
    return found->second;
  }
}

inline uint32_t parseOSMSpeedString(const std::string& s) {
  int num_ix = 0;

  if (s.empty())
    return 0;

  // see how many digits we find from beginning of the string
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (isdigit(c)) {
      num_ix++;
    } else {
      break;
    }
  }

  // if the value is not numeric, check if it's "walk" else default to 0
  uint32_t speed = num_ix < 1 ? s == "walk" ? 10 : 0 : stoi(s.substr(0, num_ix));

  if (s.size() > 3 && s.substr(s.size() - 3) == "mph") {
    speed = static_cast<uint32_t>(std::round(speed * kMPHtoKPH));
  }

  return speed;
};

// structure that holds legal speed limits for auto/truck for urban/rural
// regions and some road classes/uses
struct SimpleLegalSpeed {
  VehicleSpeeds urban;
  VehicleSpeeds rural;
  VehicleSpeeds motorway;
  VehicleSpeeds trunk;
  VehicleSpeeds living_street;
  VehicleSpeeds service;
  VehicleSpeeds fallback;
  SimpleLegalSpeed(const rapidjson::GenericArray<true, rapidjson::Value>& arr) {

    for (const auto& entry : arr) {

      LegalSpeedDomain domain = entry.HasMember("name")
                                    ? stringToLegalSpeedDomain(entry["name"].GetString())
                                    : LegalSpeedDomain::kFallback;

      auto tags = entry["tags"].GetObject();
      std::string truck_speed_str =
          tags.HasMember("maxspeed:hgv") ? tags["maxspeed:hgv"].GetString() : "";
      std::string speed_str = tags.HasMember("maxspeed") ? tags["maxspeed"].GetString() : "";
      uint32_t truck_speed = parseOSMSpeedString(truck_speed_str);
      uint32_t auto_speed = parseOSMSpeedString(speed_str);
      switch (domain) {
        case LegalSpeedDomain::kRural:
          rural.auto_ = auto_speed;
          rural.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kUrban:
          urban.auto_ = auto_speed;
          urban.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kMotorway:
          motorway.auto_ = auto_speed;
          motorway.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kLivingStreet:
          living_street.auto_ = auto_speed;
          living_street.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kTrunk:
          trunk.auto_ = auto_speed;
          trunk.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kService:
          service.auto_ = auto_speed;
          service.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kFallback:
          fallback.auto_ = auto_speed;
          fallback.truck_ = truck_speed;
          break;
        case LegalSpeedDomain::kInvalid:
          break;
        default:
          break;
      }
    }
  }
};

class SimpleLegalSpeedAssigner {
protected:
  // maps admin code to legal speed limits
  std::unordered_map<std::string, SimpleLegalSpeed> legal_speeds_map_;
  bool update_speed_;

public:
  /**
   * @param legal_speeds_file optional path to legal speed config
   * @param update_speed      whether to actually update the edge speeds or simply return it
   *                          so it can be set as speed_limit
   */
  SimpleLegalSpeedAssigner(const boost::optional<std::string>& legal_speeds_file,
                           const bool update_speed) {
    update_speed_ = update_speed;

    if (!legal_speeds_file) {
      LOG_WARN("Disabled legal default speed assignment from config, no file specified.");
      return;
    }

    try {
      auto doc = rapidjson::read_json(*legal_speeds_file);
      if (doc.HasParseError())
        throw std::runtime_error("malformed json");
      if (!doc.IsObject())
        throw std::runtime_error("must be a json object");
      if (!doc.GetObject().HasMember("speedLimitsByCountryCode"))
        throw std::runtime_error("missing speedLimitsByCountryCode member");

      auto outer = doc.GetObject()["speedLimitsByCountryCode"].GetObject();
      // loop over each country/state array
      for (const auto& [key, value] : outer) {
        std::string code = key.GetString();

        // parse the array
        SimpleLegalSpeed ls(value.GetArray());
        if (legal_speeds_map_.count(code))
          throw std::runtime_error("Duplicate country/state entry");

        // save the parsed speeds in map so they can be looked up by admin code
        legal_speeds_map_.emplace(code, ls);
      }
    } // something went wrong with parsing or opening the file
    catch (const std::exception& e) {
      LOG_WARN(std::string("Disabled legal default speeds assignment from config: ") + e.what());
      legal_speeds_map_.clear();
    } // something else was thrown
    catch (...) {
      LOG_WARN("Disabled legal default speeds assignment from config: unknown error");
      legal_speeds_map_.clear();
    }
    LOG_INFO("Enabled legal default speeds assignment from config: " + *legal_speeds_file);
  }

  /**
   * Determines the vehicle and truck speeds of a given edge if the legal speed config contains
   * entries for the admin the edge lies within. Is only concerned with
   * urban/rural legal speed limits as well as for simple road classes. Only actually sets the speed
   * if update_speed was set to true in the constructor.
   *
   * @param directededge the directed edge whose speed will be updated
   * @param density              Relative road density.
   * @param country_code         2 letter country code
   * @param state_code           2 letter state code
   * @return returns the updated (auto) speed or 0 if unchanged
   */
  uint32_t update_speed(DirectedEdge& directededge,
                        const uint32_t density,
                        const std::string& country_code,
                        const std::string& state_code) const {

    // return early if both truck and auto speed are tagged speeds or if the edge use
    if ((directededge.speed_type() == SpeedType::kTagged && directededge.truck_speed() > 0) ||
        directededge.use() == Use::kFerry || directededge.use() == Use::kRailFerry ||
        directededge.use() == Use::kRail || !(directededge.forwardaccess() & kVehicularAccess)) {
      return 0;
    }

    bool speed_changed = false;

    // try to find state then country
    auto found = legal_speeds_map_.find(country_code + "-" + state_code);
    if (found == legal_speeds_map_.end())
      found = legal_speeds_map_.find(country_code);
    // none was found, bail
    if (found == legal_speeds_map_.end())
      return 0;

    auto ls = found->second;
    auto speed = directededge.speed();
    auto truck_speed = directededge.truck_speed();

    // start with density
    if (density > kMaxRuralDensity) {
      if (ls.urban.auto_) {
        speed = ls.urban.auto_;
      } else if (ls.fallback.auto_) {
        speed = ls.fallback.auto_;
      }

      // truck: try hgv specific limit, fall back to auto speed limit
      // try to fall back to "fallback" speed if no urban speed limits were found
      if (ls.urban.truck_) {
        truck_speed = ls.urban.truck_;
      } else if (ls.urban.auto_) {
        truck_speed = ls.urban.auto_;
      } else if (ls.fallback.truck_) {
        truck_speed = ls.fallback.truck_;
      } else if (ls.fallback.auto_) {
        truck_speed = ls.fallback.auto_;
      }

    } else {
      if (ls.rural.auto_) {
        speed = ls.rural.auto_;
      } else if (ls.fallback.auto_) {
        speed = ls.fallback.auto_;
      }

      // truck: try hgv specific limit, fall back to auto speed limit
      // try to fall back to "fallback" speed if no rural speed limits were found
      if (ls.rural.truck_) {
        truck_speed = ls.rural.truck_;
      } else if (ls.rural.auto_) {
        truck_speed = ls.rural.auto_;
      } else if (ls.fallback.truck_) {
        truck_speed = ls.fallback.truck_;
      } else if (ls.fallback.auto_) {
        truck_speed = ls.fallback.auto_;
      }
    }

    // then look for more specific rules based on road class/use
    if (directededge.classification() == valhalla::baldr::RoadClass::kMotorway) {
      if (ls.motorway.auto_)
        speed = ls.motorway.auto_;

      // truck: try hgv specific limit, fall back to auto speed limit
      if (ls.motorway.truck_) {
        truck_speed = ls.motorway.truck_;
      } else if (ls.motorway.auto_) {
        truck_speed = ls.motorway.auto_;
      }
    } else if (directededge.use() == Use::kLivingStreet) {
      if (ls.living_street.auto_)
        speed = ls.living_street.auto_;

      if (ls.living_street.truck_) {
        truck_speed = ls.living_street.truck_;
      } else if (ls.living_street.auto_) {
        truck_speed = ls.living_street.auto_;
      }
    } else if (directededge.classification() == valhalla::baldr::RoadClass::kTrunk) {
      if (ls.trunk.auto_)
        speed = ls.trunk.auto_;

      if (ls.trunk.truck_) {
        truck_speed = ls.trunk.truck_;
      } else if (ls.trunk.auto_) {
        truck_speed = ls.trunk.auto_;
      }
    } else if (directededge.use() == Use::kServiceRoad) {
      if (ls.service.auto_)
        speed = ls.service.auto_;

      if (ls.service.truck_) {
        truck_speed = ls.service.truck_;
      } else if (ls.service.auto_) {
        truck_speed = ls.service.auto_;
      }
    }

    if (directededge.speed_type() == SpeedType::kClassified) {
      speed_changed |= directededge.speed() != speed;
      if (update_speed_)
        directededge.set_speed(speed);
    }

    if (!directededge.truck_speed() && update_speed_) {
      directededge.set_truck_speed(truck_speed);
    }
    return speed_changed ? directededge.speed() : 0;
  }
};
