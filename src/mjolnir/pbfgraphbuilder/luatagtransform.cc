#include "luatagtransform.h"

#include <stdexcept>
#include <boost/format.hpp>

using namespace valhalla::mjolnir;

namespace {

void CheckLuaFuncExists(lua_State* state, const std::string &func_name) {

  lua_getglobal(state, func_name.c_str());

  if (!lua_isfunction (state, -1)) {
    throw std::runtime_error((boost::format("Lua script does not contain a function %1%.")
    % func_name).str());
  }
  lua_pop(state,1);
}

}

LuaTagTransform::LuaTagTransform()
{
  // Create a new lua state
  waystate_ = luaL_newstate();
  nodestate_ = luaL_newstate();

}

LuaTagTransform::~LuaTagTransform(){

  if (waystate_ != NULL) {
    lua_close(waystate_);
  }
  if (nodestate_ != NULL) {
    lua_close(nodestate_);
  }
}

void LuaTagTransform::OpenLib() const {

  luaL_openlibs(waystate_);
  luaL_openlibs(nodestate_);

  //check if way script and function exists.
  luaL_dofile(waystate_, luawayscript_.c_str());
  CheckLuaFuncExists(waystate_, luawayfunc_);

  //check if node script and function exists.
  luaL_dofile(nodestate_, luanodescript_.c_str());
  CheckLuaFuncExists(nodestate_, luanodefunc_);
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

namespace{
void stackdump_g(lua_State* l)
{
    int i;
    int top = lua_gettop(l);

    printf("total in stack %d\n",top);

    for (i = 1; i <= top; i++)
    {  /* repeat for each level */
        int t = lua_type(l, i);
        switch (t) {
            case LUA_TSTRING:  /* strings */
                printf("string: '%s'\n", lua_tostring(l, i));
                break;
            case LUA_TBOOLEAN:  /* booleans */
                printf("boolean %s\n",lua_toboolean(l, i) ? "true" : "false");
                break;
            case LUA_TNUMBER:  /* numbers */
                printf("number: %g\n", lua_tonumber(l, i));
                break;
            default:  /* other values */
                printf("%s\n", lua_typename(l, t));
                break;
        }
        printf("  ");  /* put a separator */
    }
    printf("\n");  /* end the listing */
}
}

Tags LuaTagTransform::TransformInLua(bool isWay, const Tags &maptags) {

  //grab the proper function out of the lua code
  lua_State* state;
  if (isWay){
    lua_getglobal(waystate_,luawayfunc_.c_str());
    state = waystate_;
  }
  else {
    lua_getglobal(nodestate_,luanodefunc_.c_str());
    state = nodestate_;
  }

  //set up the lua table (map)
  int count = 0;
  lua_newtable(state);
  for (const auto& tag : maptags) {
    lua_pushstring(state, tag.first.c_str());
    lua_pushstring(state, tag.second.c_str());
    lua_rawset(state, -3);
    count++;
  }

  //tell lua how many items are in the map
  lua_pushinteger(state, count);

  //call lua
  if (lua_pcall(state, 2, isWay ? 4 : 2, 0)) {
    fprintf(stderr, "Failed to execute lua function for basic tag processing: %s\n", lua_tostring(state, -1));
  }

  //if we dont care about it we stop looking
  Tags result;
  /*if(lua_tonumber(state, 1))
    return result;*/

  //osm2pgsql has extra info for roads and polygons which we dont care about
  if(isWay) {
    lua_pop(state,1);
    lua_pop(state,1);
  }

  //pull out the keys and values into a map
  lua_pushnil(state);
  while (lua_next(state,-2) != 0) {
    const char* key = lua_tostring(state,-2);
    const char* value = lua_tostring(state,-1);
    result[key] = value;
    lua_pop(state,1);
  }

  //pull out an int which if its 1 means we dont care about this way/node
  int filter = lua_tointeger(state, -2);
  lua_pop(state,2);

  return result;
}

