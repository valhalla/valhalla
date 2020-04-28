#include "baldr/signinfo.h"

#include "mjolnir/graphbuilder.h"

#include "test.h"

using namespace std;
using namespace valhalla::mjolnir;
using valhalla::mjolnir::GraphBuilder;

namespace {

TEST(Refs, Basic) {

  std::string way_refs = "US 21;US 321";
  std::string rel_refs = "US 21|west;US 321|north";

  std::string output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "US 21 west;US 321 north");

  way_refs = "I 94;US 21;US 321";
  rel_refs = "US 21|west;US 321|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "I 94;US 21 west;US 321 north");

  way_refs = "I 26;US 21;US 321";
  rel_refs = "US 21;US 321";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "I 26;US 21;US 321");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "US 21|north;US 321|south";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "US 21 north;I 95;US 321 south");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "US 21;I 95;US 321");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "I 95|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "US 21;I 95 north;US 321");

  way_refs = "I 99;US 220;US 322";
  rel_refs = "US 322|west;I 99|south;US 220|south;ADHS O|south";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_EQ(output, "I 99 south;US 220 south;US 322 west");

  way_refs = "";
  rel_refs = "I 95|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  EXPECT_TRUE(output.empty());
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
