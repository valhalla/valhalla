#ifndef VALHALLA_TYR_LOCATE_HANDLER_H_
#define VALHALLA_TYR_LOCATE_HANDLER_H_

#include <valhalla/tyr/handler.h>

namespace valhalla{
namespace tyr{

class LocateHandler : public Handler {
 public:
  /**
   * Parses json request data to be used as options for the action
   *
   * @param config   where the config file resides
   * @param dict     the request data
   * @return a handler object ready to act
   */
  LocateHandler(const std::string& config, const boost::python::dict& dict_request);

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
};

}
}

#endif  // VALHALLA_TYR_LOCATE_HANDLER_H_
