#include "luatagtransform.h"

#include <stdexcept>
#include <boost/format.hpp>

using namespace valhalla::mjolnir;

LuaTagTransform::LuaTagTransform()
{
  // Create a new lua state
  luastate_ = luaL_newstate();

}

LuaTagTransform::~LuaTagTransform(){
  if (luastate_ != NULL) {

    lua_close(luastate_);

  }
}

void LuaTagTransform::Init() const {

  luaL_openlibs(luastate_);

  //TODO:add lua script to options.
  std::string tag_transform_script = "/data/valhalla/import/osm2pgsql/edges.lua";
  luaL_dofile(luastate_, tag_transform_script.c_str());
  CheckLuaFuncExists("ways_proc");

  tag_transform_script = "/data/valhalla/import/osm2pgsql/vertices.lua";
  luaL_dofile(luastate_, tag_transform_script.c_str());
  CheckLuaFuncExists("nodes_proc");
}

void LuaTagTransform::SetLuaWayFunc(std::string luawayfunc) {
  luawayfunc_ = luawayfunc;
}

void LuaTagTransform::SetLuaNodeFunc(std::string luanodefunc) {
  luanodefunc_ = luanodefunc;
}

std::string LuaTagTransform::GetLuaWayFunc() const {
  return luawayfunc_;
}

std::string LuaTagTransform::GetLuaNodeFunc() const {
  return luanodefunc_;
}

void LuaTagTransform::CheckLuaFuncExists(const std::string &func_name) const {

  lua_getglobal(luastate_, func_name.c_str());

  if (!lua_isfunction (luastate_, -1)) {
    throw std::runtime_error((boost::format("Lua script does not contain a function %1%")
    % func_name).str());
  }
  lua_pop(luastate_,1);
}

Tags LuaTagTransform::TransformInLua(bool isWay, const Tags &maptags) {

  Tags result;

  int filter;
  int count = 0;
  struct keyval *item;
  const char * key, * value;

  int polygon = 0;
  int roads = 0;

  //TODO::add boost options.
  if (isWay)
    lua_getglobal(luastate_,"ways_proc");
  else
    lua_getglobal(luastate_,"nodes_proc");

  lua_newtable(luastate_);    /* key value table */

  for (auto tag : maptags) {
    lua_pushstring(luastate_, tag.first.c_str());
    lua_pushstring(luastate_, tag.second.c_str());
    lua_rawset(luastate_, -3);
    count++;
  }

  lua_pushinteger(luastate_, count);

  if (lua_pcall(luastate_,2,4,0)) {
    fprintf(stderr, "Failed to execute lua function for basic tag processing: %s\n", lua_tostring(luastate_, -1));
    /* lua function failed */
  }

  roads = lua_tointeger(luastate_, -1);
  lua_pop(luastate_,1);
  polygon = lua_tointeger(luastate_, -1);
  lua_pop(luastate_,1);

  lua_pushnil(luastate_);
  while (lua_next(luastate_,-2) != 0) {
    key = lua_tostring(luastate_,-2);
    value = lua_tostring(luastate_,-1);

    result[key] = value;

    lua_pop(luastate_,1);
  }

  filter = lua_tointeger(luastate_, -2);

  lua_pop(luastate_,2);

  return result;

}

