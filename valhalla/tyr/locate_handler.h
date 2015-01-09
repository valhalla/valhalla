#ifndef VALHALLA_TYR_LOCATE_HANDLER_H_
#define VALHALLA_TYR_LOCATE_HANDLER_H_

namespace valhalla{
namespace tyr{

class LocateHandler : public Handler {
 public:
  /**
   * We use all the same constructors etc as our parent
   */
  using Handler::Handler;

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
