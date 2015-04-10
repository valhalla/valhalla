#ifndef VALHALLA_TYR_ROUTE_HANDLER_H_
#define VALHALLA_TYR_ROUTE_HANDLER_H_

#include <valhalla/tyr/handler.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/baldr/graphreader.h>
#include <string>
#include <memory>

namespace valhalla{
namespace tyr{

// Base route handler - intended to handle simple requests for OSRM
// compatibility mode. Valhalla JSON inputs use the CustomRouteHandler.
class RouteHandler : public Handler {
 public:
  /**
   * Parses json request data to be used as options for the action
   *
   * @param config   where the config file resides
   * @param json     the request data
   * @return a handler object ready to act
   */
  RouteHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& dict_request);

  /**
   * Don't expose the default constructor
   */
  RouteHandler() = delete;

  /**
   * Destructor
   */
  virtual ~RouteHandler();

  /**
   * Actually run the route and return the json representation
   * @return string the json representation of the route mirroring the format of OSRM for now
   */
  virtual std::string Action();

 protected:
  valhalla::sif::cost_ptr_t cost_;
  std::unique_ptr<valhalla::baldr::GraphReader> reader_;
};

}
}

#endif  // VALHALLA_TYR_ROUTE_HANDLER_H_
