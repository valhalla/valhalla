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
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags);
  ASSERT_FLOAT_EQ(2.0f, std::stof(results["maxheight"]));
}

void assert_height_parses(std::string maxheight, float expected) {
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  mjolnir::Tags tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight", maxheight});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags);
  ASSERT_TRUE(results.count("maxheight") == 1);
  ASSERT_FLOAT_EQ(expected, std::stof(results["maxheight"]));
}

TEST(Lua, DefaultMeters) {
  // default for a number without units should be meters
  assert_height_parses("1.0", 1.0f);
  assert_height_parses("1", 1.0f);
  assert_height_parses("1.1", 1.1f);
}

TEST(Lua, ExplicitMeters) {
  // check all the common ways of writing meters
  assert_height_parses("1m", 1.0f);

  // there could be spaces, or not.
  assert_height_parses("1.1 m", 1.1f);
  assert_height_parses("1.1m", 1.1f);
  assert_height_parses("1 m", 1.0f);

  // but the units can change if they're plural
  assert_height_parses("1 meter", 1.0f);
  assert_height_parses("2meters", 2.0f);
}

TEST(Lua, UnitsCaseInsensitive) {
  // case doesn't matter for units
  assert_height_parses("1 METERS", 1.0f);
}

TEST(Lua, Centimeters) {
  // yes, this exists in the data...
  assert_height_parses("100cm", 1.0f);
}

TEST(Lua, EuropeanDecimal) {
  // it's common to see numbers written using the European convention of a
  // comma as the decimal separator.
  assert_height_parses("1,1", 1.1f);
}

TEST(Lua, FeetAndInches) {
  // there are a bunch of different commonly-used ways of writing heights in
  // feet and inches in OSM. here's a few, cropped from taginfo and generalised
  // to their generic formats.
  assert_height_parses("1ft1in", 0.33f);
  assert_height_parses("2 feet 1 inch", 0.64f);
  assert_height_parses("1 foot 6 inches", 0.46f);
  assert_height_parses("1.1 ft", 0.34f);
  assert_height_parses("1 ft", 0.3f);

  // feet and inches can also be written with single and double quotes (' and ")
  // and even with two single quotes (' and '').
  assert_height_parses("1'", 0.3f);
  assert_height_parses("1.1\"", 0.03f);
  assert_height_parses("1' 1\"", 0.33f);
  assert_height_parses("1'1\"", 0.33f);
  assert_height_parses("1'1''", 0.33f);
}

TEST(Lua, NumberDoublePeriod) {
  // Way 25494427 version 14 has a "maxheight" tag value of 3..35 with the two
  // dots. This probably shouldn't be parsed?
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  mjolnir::Tags tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight", "3..35"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags);

  // check that the maxheight isn't present...
  ASSERT_TRUE(results.count("maxheight") == 0);

  // ... but that the results aren't completely empty
  ASSERT_TRUE(results.size() > 0);
}

TEST(Lua, TestForwardBackward) {
  // Way 25494427 version 14 has a "maxheight" tag value of 3..35 with the two
  // dots. This probably shouldn't be parsed?
  mjolnir::LuaTagTransform lua(std::string(lua_graph_lua, lua_graph_lua + lua_graph_lua_len));

  mjolnir::Tags tags;
  tags.insert({"highway", "tertiary"});
  tags.insert({"maxheight:forward", "1"});
  tags.insert({"maxheight:backward", "1"});
  tags.insert({"maxlength:forward", "1"});
  tags.insert({"maxlength:backward", "1"});
  tags.insert({"maxwidth:forward", "1"});
  tags.insert({"maxwidth:backward", "1"});
  tags.insert({"maxweight:forward", "1"});
  tags.insert({"maxweight:backward", "1"});
  auto results = lua.Transform(mjolnir::OSMType::kWay, 1, tags);

  // check that the maxheight is present...
  ASSERT_TRUE(results.count("maxheight_forward") == 1);
  ASSERT_TRUE(results.count("maxheight_backward") == 1);
  ASSERT_TRUE(results.count("maxlength_forward") == 1);
  ASSERT_TRUE(results.count("maxlength_backward") == 1);
  ASSERT_TRUE(results.count("maxwidth_forward") == 1);
  ASSERT_TRUE(results.count("maxwidth_backward") == 1);
  ASSERT_TRUE(results.count("maxweight_forward") == 1);
  ASSERT_TRUE(results.count("maxweight_backward") == 1);
}
} // namespace

// TODO: sweet jesus add more tests of this class!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
