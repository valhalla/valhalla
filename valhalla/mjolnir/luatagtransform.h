#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

#include <valhalla/mjolnir/osmdata.h>

#include <ankerl/unordered_dense.h>
#include <osmium/osm/tag.hpp>

#include <string>

namespace valhalla {
namespace mjolnir {

using Tags = ankerl::unordered_dense::map<std::string, std::string>;

/**
 */
class LuaTagTransform {
public:
  /**
   * Constructor
   * @param lua   the string containing the lua code
   */
  LuaTagTransform(const std::string& lua);

  LuaTagTransform(const LuaTagTransform&) = delete;
  LuaTagTransform& operator=(const LuaTagTransform&) = delete;
  LuaTagTransform(LuaTagTransform&&) = delete;
  LuaTagTransform& operator=(LuaTagTransform&&) = delete;

  ~LuaTagTransform();

  Tags Transform(OSMType type, uint64_t osmid, const osmium::TagList& tags);

protected:
  lua_State* state_;
};

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_LUA_H
