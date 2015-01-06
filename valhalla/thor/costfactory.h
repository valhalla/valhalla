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
template <typename T>
class CostFactory {
 public:
  typedef std::map<std::string, DynamicCost*> func_map_t;

  /**
   * Constructor
   */
  CostFactory()
    : factory_funcs_() {
  }

  /**
   * Register
   */
  CostFactory& Register(const std::string& type, DynamicCost* obj) {
    factory_funcs_.insert(std::make_pair(type, obj));
    return (*this);
  }

  DynamicCost* Create(const std::string& type) const {
    auto itr = factory_funcs_.find(type);
    if (itr == factory_funcs_.end()) {
      throw std::runtime_error("Unrecognized type: " + type);
    }
    return itr->second;
  }

 private:
  func_map_t factory_funcs_;
};

}
}

#endif // VALHALLA_THOR_COSTFACTORY_H_
