#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include "lua.h"
#include <lualib.h>
#include <lauxlib.h>
}

#include <string>
#include <map>

// Represents the key/values of an object
typedef std::map<std::string, std::string> Tags;

namespace valhalla {
namespace mjolnir {

/**
 */
class LuaTagTransform {
 public:
  /**
   * Constructor
   */
  LuaTagTransform();

  /**
   * Destructor
   */
  ~LuaTagTransform();

  void Init() const;

  void SetLuaWayFunc(std::string luawayfunc);

  void SetLuaNodeFunc(std::string luanodefunc);

  std::string GetLuaWayFunc() const;

  std::string GetLuaNodeFunc() const;

  void CheckLuaFuncExists(const std::string &func_name) const;

  Tags TransformInLua(bool isWay, const Tags &tags);

 protected:

  lua_State *luastate_;

  std::string luanodefunc_;
  std::string luawayfunc_;

};

}
}

#endif  // VALHALLA_MJOLNIR_LUA_H
