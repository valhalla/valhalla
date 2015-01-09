#ifndef VALHALLA_THOR_COSTFACTORY_H_
#define VALHALLA_THOR_COSTFACTORY_H_

#include <map>
#include <memory>

// Include derived cost methods?
#include "thor/autocost.h"
#include "thor/bicyclecost.h"
#include "thor/pedestriancost.h"

namespace valhalla {
namespace thor {

/**
* Generic factory class for creating objects based on type name.
*/
template <class heuristic_t>
class CostFactory {
 public:
  typedef std::shared_ptr<heuristic_t> heuristic_ptr_t;
  //TODO: might want to have some configurable params to each heuristic type
  typedef heuristic_ptr_t (*factory_function_t)(/*pt::ptree const& config*/);

  /**
   * Constructor
   */
  CostFactory()
    : factory_funcs_() {
  }

  /**
   * Register the callback to create this type of heuristic
   *
   * @param name       the name of the heuristic that the function creates
   * @param function   the function pointer to call to actually create the heuristic object
   */
  void Register(const std::string& name, factory_function_t function) {
    factory_funcs_.emplace(name, function);
  }

  /**
   * Make a heuristic from its name
   *
   * @param name    the name of the heuristic to create
   * TODO: add configuration for the heuristic options
   */
  heuristic_ptr_t Create(const std::string& name/*, pt::ptree const& config*/) const {
    auto itr = factory_funcs_.find(name);
    if (itr == factory_funcs_.end()) {
      throw std::runtime_error("Unrecognized heuristic name: " + name);
    }
    //create the heuristic using the function pointer
    return itr->second();
  }

 private:
  std::map<std::string, factory_function_t> factory_funcs_;
};

}
}

#endif // VALHALLA_THOR_COSTFACTORY_H_
