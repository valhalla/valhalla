#include "mjolnir/luatagtransform.h"

#include "midgard/logging.h"
#include "mjolnir/osmdata.h"
#include <boost/format.hpp>
#include <stdexcept>

using namespace valhalla::mjolnir;

namespace {

const std::string LUA_NODE_PROC = "nodes_proc";
const std::string LUA_WAY_PROC = "ways_proc";
const std::string LUA_REL_PROC = "rels_proc";

void CheckLuaFuncExists(lua_State* state, const std::string& func_name) {

  lua_getglobal(state, func_name.c_str());

  if (!lua_isfunction(state, -1)) {
    throw std::runtime_error(
        (boost::format("Lua script does not contain a function %1%.") % func_name).str());
  }
  lua_pop(state, 1);
}

std::string to_string(OSMType t) {
  if (t == OSMType::kNode) {
    return "node";
  } else if (t == OSMType::kWay) {
    return "way";
  } else {
    return "relation";
  }
}

} // namespace

LuaTagTransform::LuaTagTransform(const std::string& lua) {
  // create a new lua state
  state_ = luaL_newstate();
  luaL_openlibs(state_);
  luaL_dostring(state_, lua.c_str());

  // check that various functions exist
  CheckLuaFuncExists(state_, LUA_NODE_PROC);
  CheckLuaFuncExists(state_, LUA_WAY_PROC);
  CheckLuaFuncExists(state_, LUA_REL_PROC);
}

LuaTagTransform::~LuaTagTransform() {
  if (state_ != NULL) {
    lua_close(state_);
  }
}

Tags LuaTagTransform::Transform(OSMType type, uint64_t osmid, const Tags& maptags) {

  // grab the proper function out of the lua code
  Tags result;
  const std::string& lua_func =
      type == OSMType::kNode ? LUA_NODE_PROC : (type == OSMType::kWay ? LUA_WAY_PROC : LUA_REL_PROC);

  try {
    // grab the function
    lua_getglobal(state_, lua_func.c_str());

    // set up the lua table (map)
    int count = 0;
    lua_newtable(state_);
    for (const auto& tag : maptags) {
      lua_pushstring(state_, tag.first.c_str());
      lua_pushstring(state_, tag.second.c_str());
      lua_rawset(state_, -3);
      count++;
    }

    // tell lua how many items are in the map
    lua_pushinteger(state_, count);

    // call lua
    if (lua_pcall(state_, 2, type == OSMType::kWay ? 4 : 2, 0)) {
      const char* lua_error_message = lua_tostring(state_, 1);
      // if the Lua code fails hard, such as throwing an interpreter error or
      // running out of memory, then this indicates a programming logic error
      // and the program should stop with as informative an error message as
      // it's possible to give.
      throw std::runtime_error((boost::format("Failed to execute lua function "
                                              "for basic tag processing in %1% %2%: %3%") %
                                to_string(type) % osmid % lua_error_message)
                                   .str());
    }

    // TODO:  if we dont care about it we stop looking.  Look for filter = 1
    /*if(lua_tonumber(state, 1))
      return result;*/

    // osm2pgsql has extra info for roads and polygons which we dont care about
    if (type == OSMType::kWay) {
      lua_pop(state_, 1);
      lua_pop(state_, 1);
    }

    // pull out the keys and values into a map
    lua_pushnil(state_);
    while (lua_next(state_, -2) != 0) {
      const char* key = lua_tostring(state_, -2);
      if (key == nullptr) {
        LOG_ERROR((boost::format("Invalid key in Lua function: %1%.") % lua_func).str());
        break;
      }
      const char* value = lua_tostring(state_, -1);
      if (value == nullptr) {
        LOG_ERROR((boost::format("Invalid value in Lua function: %1%.") % lua_func).str());
        break;
      }
      result[key] = value;
      lua_pop(state_, 1);
    }

    // pull out an int which if its 1 means we dont care about this way/node
    int filter = lua_tointeger(state_, -2);
    lua_pop(state_, 2);

    if (filter) {
      result.clear();
    }
  } catch (std::exception& e) {
    LOG_ERROR((boost::format("Exception in Lua function: %1%: %2%") % lua_func % e.what()).str());
  } catch (...) {
    LOG_ERROR((boost::format("Unknown exception in Lua function: %1%.") % lua_func).str());
  }

  return result;
}
