
#include <stdexcept>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "baldr/location.h"
#include <valhalla/midgard/pointll.h>

namespace valhalla{
namespace baldr{

  Location::Location():stoptype_(StopType::BREAK){
  }

  Location::Location(const midgard::PointLL& latlng, const StopType& stoptype):latlng_(latlng), stoptype_(stoptype){
  }

  Location Location::FromGeoJson(const std::string& geojson){
    Location location;
    throw std::runtime_error("Location serialization from geojson is not yet implemented");
    return location;
  }

  Location Location::FromCsv(const std::string& csv) {
    //split it up into parts
    std::vector<std::string> parts;
    boost::algorithm::split(parts, csv, boost::algorithm::is_any_of(","), boost::algorithm::token_compress_on);
    if(parts.size() < 2)
      throw std::runtime_error("Bad format for csv formated location");

    //make the lat, lng
    midgard::PointLL ll(std::stof(parts[0]), std::stof(parts[1]));

    //check for info about the stop type
    if(parts.size() > 2 && parts[2] == "through")
      return Location(ll, StopType::THROUGH);
    return Location(ll);
  }

}
}
