// -*- mode: c++ -*-

#ifndef __MMP_SERVICE_H__
#define __MMP_SERVICE_H__

#include <boost/property_tree/ptree.hpp>


namespace valhalla{

namespace meili {

void run_service(const boost::property_tree::ptree& config);

}

}



#endif //__MMP_SERVICE_H__
