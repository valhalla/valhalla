#ifndef __VALHALLA_SKADI_SERVICE_H__
#define __VALHALLA_SKADI_SERVICE_H__

#include <vector>

#include <boost/property_tree/ptree.hpp>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {
namespace skadi {


  void run_service(const boost::property_tree::ptree& config);

  json::MapPtr serialize_shape(const std::vector<PointLL>& shape);
}
}


#endif //__VALHALLA_SKADI_SERVICE_H__
