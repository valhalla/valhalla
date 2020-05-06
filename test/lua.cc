#include "test.h"

#include <string>

#include "mjolnir/graph_lua_proc.h"
#include "mjolnir/luatagtransform.h"
#include "mjolnir/osmdata.h"

using namespace valhalla;

namespace {
TEST(Lua, ZeroMantissa) {
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  mjolnir::Tags tags;
  // Check that decimals are properly parsed
  tags.insert({"highway", "primary"});
  tags.insert({"maxheight", "2.0"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, tags);
  ASSERT_FLOAT_EQ(2.0f, std::stof(results["maxheight"]));
}
} // namespace

// TODO: sweet jesus add more tests of this class!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
