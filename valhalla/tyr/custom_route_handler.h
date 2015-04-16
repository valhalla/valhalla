#ifndef VALHALLA_TYR_CUSTOM_ROUTE_HANDLER_H_
#define VALHALLA_TYR_CUSTOM_ROUTE_HANDLER_H_

#include <valhalla/tyr/route_handler.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/baldr/graphreader.h>
#include <string>
#include <memory>

namespace valhalla{
namespace tyr{

class CustomRouteHandler : public Handler {
 public:
  /**
   * Parses json request data to be used as options for the action
   *
   * @param config   where the config file resides
   * @param json     the request data
   * @return a handler object ready to act
   */
  CustomRouteHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& dict_request);

  /**
   * Don't expose the default constructor
   */
  CustomRouteHandler() = delete;

  /**
   * Destructor
   */
  virtual ~CustomRouteHandler();

  /**
   * Actually run the route and return the json representation
   * @return string the json representation of the route mirroring the format of OSRM for now
   */
  virtual std::string Action();

 protected:
  bool km_units_;
  std::string units_;
  valhalla::sif::cost_ptr_t cost_;
  std::unique_ptr<valhalla::baldr::GraphReader> reader_;
  boost::property_tree::ptree directions_options_ptree_;
};

}
}

#endif  // VALHALLA_TYR_CUSTOM_ROUTE_HANDLER_H_
