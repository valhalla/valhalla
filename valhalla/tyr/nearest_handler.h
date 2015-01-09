#ifndef VALHALLA_TYR_NEAREST_HANDLER_H_
#define VALHALLA_TYR_NEAREST_HANDLER_H_

#include <valhalla/tyr/handler.h>

namespace valhalla{
namespace tyr{

class NearestHandler : public Handler {
 public:
  /**
   * We use all the same constructors etc as our parent
   */
  using Handler::Handler;

  /**
   * Destructor
   */
  virtual ~NearestHandler();

  /**
   * Find the closest point alone an edge in the route network, may be a node or an interpolated point
   * @return string the json representation of the route mirroring the format of OSRM for now
   */
  virtual std::string Action();

 protected:
};

}
}

#endif  // VALHALLA_TYR_NEAREST_HANDLER_H_
