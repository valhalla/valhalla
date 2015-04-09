
#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "baldr/location.h"
#include <valhalla/midgard/pointll.h>

namespace valhalla{
namespace baldr{

  Location::Location(const midgard::PointLL& latlng, const StopType& stoptype):latlng_(latlng), stoptype_(stoptype){
  }

  Location Location::FromGeoJson(const std::string& geojson){
    throw std::runtime_error("Location serialization from geojson is not yet implemented");
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
    auto part = parts.begin() + 2;
    for(auto address : { &l.name_, &l.street_, &l.city_, &l.state_, &l.zip_, &l.country_, &l.phone_, &l.url_ }) {
      if(part == parts.end())
        break;
      address->swap(*part);
      ++part;
    }

    if (parts.size() > 11) {
      l.heading_ = parts[11];
    }

    return l;
  }

}
}
