#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <valhalla/mjolnir/osmdata.h>

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

  void SetLuaRelationFunc(std::string luarelationfunc);

  std::string GetLuaWayFunc() const;

  std::string GetLuaNodeFunc() const;

  std::string GetLuaRelationFunc() const;

  void SetLuaWayScript(std::string luawayscript);

  void SetLuaNodeScript(std::string luanodescript);

  void SetLuaRelationScript(std::string luarelationscript);

  std::string GetLuaWayScript() const;

  std::string GetLuaNodeScript() const;

  std::string GetLuaRelationScript() const;

  Tags TransformInLua(OSMType type, const Tags &tags);

 protected:

  lua_State *waystate_;
  lua_State *nodestate_;
  lua_State *relationstate_;

  std::string luawayfunc_;
  std::string luanodefunc_;
  std::string luarelationfunc_;

  std::string luawayscript_;
  std::string luanodescript_;
  std::string luarelationscript_;

};

}
}

#endif  // VALHALLA_MJOLNIR_LUA_H
