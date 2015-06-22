#ifndef __VALHALLA_LOKI_SERVICE_H__
#define __VALHALLA_LOKI_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
  namespace loki {

    void run_service(const boost::property_tree::ptree& config);

  }
}


#endif //__VALHALLA_LOKI_SERVICE_H__
