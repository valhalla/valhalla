#ifndef __VALHALLA_TYR_SERVICE_H__
#define __VALHALLA_TYR_SERVICE_H__

#include <boost/property_tree/ptree.hpp>

namespace valhalla {
  namespace tyr {

    void run_service(const boost::property_tree::ptree& config);

  }
}


#endif //__VALHALLA_TYR_SERVICE_H__
