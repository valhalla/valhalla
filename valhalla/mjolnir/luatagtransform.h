#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

#include <valhalla/mjolnir/osmdata.h>

#include <robin_hood.h>

#include <string>

namespace valhalla {
namespace mjolnir {

using Tags = robin_hood::unordered_map<std::string, std::string>;

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

  Tags Transform(OSMType type, uint64_t osmid, const Tags& tags);

protected:
  lua_State* state_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_LUA_H
