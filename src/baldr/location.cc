
#include <stdexcept>
#include "baldr/location.h"

namespace valhalla{
namespace baldr{

  Location::Location():stoptype_(StopType::BREAK){
  }

  Location::Location(const midgard::PointLL& latlng, const StopType& stoptype):latlng_(latlng), stoptype_(stoptype){
  }

  Location::Location(const std::string geojson):stoptype_(StopType::BREAK){
    throw std::runtime_error("Location serialization from geojson is not yet implemented");
  }

}
}
