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

void LuaTagTransform::OpenLib() const {

  luaL_openlibs(luastate_);

  //check if way script and function exists.
  luaL_dofile(luastate_, luawayscript_.c_str());
  CheckLuaFuncExists(luawayfunc_);

  //check if node script and function exists.
  luaL_dofile(luastate_, luanodescript_.c_str());
  CheckLuaFuncExists(luanodefunc_);
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

void LuaTagTransform::SetLuaWayScript(std::string luawayscript) {
  luawayscript_ = luawayscript;
}

void LuaTagTransform::SetLuaNodeScript(std::string luanodescript) {
  luanodescript_ = luanodescript;
}

std::string LuaTagTransform::GetLuaWayScript() const {
  return luawayscript_;
}

std::string LuaTagTransform::GetLuaNodeScript() const {
  return luanodescript_;
}

void LuaTagTransform::CheckLuaFuncExists(const std::string &func_name) const {

  lua_getglobal(luastate_, func_name.c_str());

  if (!lua_isfunction (luastate_, -1)) {
    throw std::runtime_error((boost::format("Lua script does not contain a function %1%.")
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
    lua_getglobal(luastate_,luawayfunc_.c_str());
  else
    lua_getglobal(luastate_,luanodefunc_.c_str());

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

