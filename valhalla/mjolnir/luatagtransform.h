#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

#include <valhalla/mjolnir/osmdata.h>

#include <string>
#include <valhalla/midgard/small_flat_map.h>

namespace valhalla {
namespace mjolnir {

using Tags = midgard::SmallFlatMap<std::string, std::string>;

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

  Tags Transform(OSMType type, const Tags& tags);

protected:
  lua_State* state_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_LUA_H
