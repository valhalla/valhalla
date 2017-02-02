#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}

#include <valhalla/mjolnir/osmdata.h>

#include <string>
#include <unordered_map>

namespace valhalla {
namespace mjolnir {

using Tags = std::unordered_map<std::string, std::string>;

/**
 */
class LuaTagTransform {
 public:

  /**
   * Constructor
   * @param lua   the string containing the lua code
   */
  LuaTagTransform(const std::string& lua);

  ~LuaTagTransform();

  Tags Transform(OSMType type, const Tags &tags);

 protected:

  lua_State* state_;

};

}
}

#endif  // VALHALLA_MJOLNIR_LUA_H
