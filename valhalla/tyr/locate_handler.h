#ifndef VALHALLA_TYR_LOCATE_HANDLER_H_
#define VALHALLA_TYR_LOCATE_HANDLER_H_

#include <valhalla/tyr/handler.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/baldr/graphreader.h>

namespace valhalla{
namespace tyr{

class LocateHandler : public Handler {
 public:
  /**
   * Parses json request data to be used as options for the action
   *
   * @param config   where the config file resides
   * @param json     the request data
   * @return a handler object ready to act
   */
  LocateHandler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& dict_request);

  /**
   * Don't expose the default constructor
   */
  LocateHandler() = delete;

  /**
   * Destructor
   */
  virtual ~LocateHandler();

  /**
   * Find the closest node to the input point
   * @return string the json representation of the route mirroring the format of OSRM for now
   */
  virtual std::string Action();

 protected:
  std::vector<baldr::Location> locations_;
  valhalla::sif::cost_ptr_t cost_;
  std::unique_ptr<valhalla::baldr::GraphReader> reader_;
};

}
}

#endif  // VALHALLA_TYR_LOCATE_HANDLER_H_
