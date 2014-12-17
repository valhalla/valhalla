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

  void OpenLib() const;

  void SetLuaWayFunc(std::string luawayfunc);

  void SetLuaNodeFunc(std::string luanodefunc);

  std::string GetLuaWayFunc() const;

  std::string GetLuaNodeFunc() const;

  void SetLuaWayScript(std::string luawayscript);

  void SetLuaNodeScript(std::string luanodescript);

  std::string GetLuaWayScript() const;

  std::string GetLuaNodeScript() const;

  Tags TransformInLua(bool isWay, const Tags &tags);

 protected:

  lua_State *waystate_;
  lua_State *nodestate_;

  std::string luanodefunc_;
  std::string luawayfunc_;

  std::string luanodescript_;
  std::string luawayscript_;

};

}
}

#endif  // VALHALLA_MJOLNIR_LUA_H
