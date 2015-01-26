#ifndef VALHALLA_TYR_HANDLER_H_
#define VALHALLA_TYR_HANDLER_H_

#include <boost/python/dict.hpp>
#include <boost/optional.hpp>
#include <string>
#include <valhalla/baldr/location.h>

namespace valhalla{
namespace tyr{

class Handler {
 public:
  /**
   * Parses json request data to be used as options for the action
   *
   * @param config   where the config file resides
   * @param dict     the request data
   * @return a handler object ready to act
   */
  Handler(const std::string& config, const boost::python::dict& dict_request);

  /**
   * Don't expose the default constructor
   */
  Handler() = delete;

  /**
   * Destructor
   */
  virtual ~Handler();

  /**
   * Do the action the handler is supposed to, base class doesn't implement this
   */
  virtual std::string Action() = 0;

 protected:

  std::string config_;
  std::vector<baldr::Location> locations_;
  boost::optional<std::string> jsonp_;

};

}
}

#endif  // VALHALLA_TYR_HANDLER_H_
