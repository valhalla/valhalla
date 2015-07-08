#ifndef __VALHALLA_SKADI_SERVICE_H__
#define __VALHALLA_SKADI_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
namespace baldr {
namespace skadi {

  json::ArrayPtr serialize_shape(const std::vector<midgard::PointLL&> shape);

  json::ArrayPtr serialize_shape(const std::string encoded_polyline);

  std::vector<std::vector<float>> getelevation(const midgard::PointLL & latlng, const midgard::PointLL & latlng2);

  void run_service(const boost::property_tree::ptree& config);
}
}
}


#endif //__VALHALLA_SKADI_SERVICE_H__
