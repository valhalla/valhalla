#include "baldr/signinfo.h"

#include "test.h"

#include "mjolnir/graphbuilder.h"

using namespace std;
using namespace valhalla::mjolnir;
using valhalla::mjolnir::GraphBuilder;

namespace {

void RefsTest() {

  std::string way_refs = "US 21;US 321";
  std::string rel_refs = "US 21|west;US 321|north";

  std::string output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "US 21 west;US 321 north")
    throw std::runtime_error("US 21 west;US 321 north failed.");

  way_refs = "I 94;US 21;US 321";
  rel_refs = "US 21|west;US 321|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "I 94;US 21 west;US 321 north")
    throw std::runtime_error("I 94;US 21 west;US 321 north failed.");

  way_refs = "I 26;US 21;US 321";
  rel_refs = "US 21;US 321";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "I 26;US 21;US 321")
    throw std::runtime_error("I 26;US 21;US 321 failed.");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "US 21|north;US 321|south";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "US 21 north;I 95;US 321 south")
    throw std::runtime_error("US 21 north;I 95;US 321 south failed.");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "US 21;I 95;US 321")
    throw std::runtime_error("US 21;I 95;US 321 failed.");

  way_refs = "US 21;I 95;US 321";
  rel_refs = "I 95|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "US 21;I 95 north;US 321")
    throw std::runtime_error("US 21;I 95 north;US 321 failed.");

  way_refs = "I 99;US 220;US 322";
  rel_refs = "US 322|west;I 99|south;US 220|south;ADHS O|south";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (output != "I 99 south;US 220 south;US 322 west")
    throw std::runtime_error("I 99 south;US 220 south;US 322 west failed.");

  way_refs = "";
  rel_refs = "I 95|north";

  output = GraphBuilder::GetRef(way_refs, rel_refs);

  if (!output.empty())
    throw std::runtime_error("Empty test failed.");
}

} // namespace

int main() {
  test::suite suite("refs");

  // Test setting and getting on random sizes of bit tables
  suite.test(TEST_CASE(RefsTest));

  return suite.tear_down();
}
