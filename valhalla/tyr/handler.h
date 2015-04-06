#ifndef VALHALLA_TYR_HANDLER_H_
#define VALHALLA_TYR_HANDLER_H_

#include <boost/property_tree/ptree.hpp>
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
   * @param json     the request data
   * @return a handler object ready to act
   */
  Handler(const boost::property_tree::ptree& config, const boost::property_tree::ptree& request);

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

  boost::property_tree::ptree config_;
  std::vector<baldr::Location> locations_;
  boost::optional<std::string> jsonp_;

};

}
}

#endif  // VALHALLA_TYR_HANDLER_H_
