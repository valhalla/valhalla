#ifndef VALHALLA_MJOLNIR_LUA_H
#define VALHALLA_MJOLNIR_LUA_H

#include <valhalla/mjolnir/osmdata.h>

#include <ankerl/unordered_dense.h>
#include <osmium/osm/tag.hpp>

#include <string>

struct lua_State;

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
