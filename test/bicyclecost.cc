#include "baldr/directededge.h"
#include "baldr/graphid.h"
#include "sif/bicyclecost.h"
#include "worker.h"

#include <gtest/gtest.h>

#include <string>

using namespace valhalla;
using namespace valhalla::sif;

namespace {

TEST(BicycleCost, CandidateSurfaceLimits) {
  baldr::DirectedEdge edge{};
  edge.set_forwardaccess(baldr::kBicycleAccess);
  edge.set_reverseaccess(baldr::kBicycleAccess);
  edge.set_use(baldr::Use::kPath);
  const baldr::graph_tile_ptr tile;
  const baldr::GraphId edgeid{0, 2, 0};

  for (const std::string type : {"road", "cross", "hybrid", "mountain"}) {
    for (const float avoid_bad_surfaces : {0.0f, 0.25f, 1.0f}) {
      SCOPED_TRACE(type + " avoid_bad_surfaces=" + std::to_string(avoid_bad_surfaces));
      Api request;
      ParseApi(R"({"costing":"bicycle","costing_options":{"bicycle":{"bicycle_type":")" + type +
                   R"(","avoid_bad_surfaces":)" + std::to_string(avoid_bad_surfaces) + "}}}",
               valhalla::Options::route, request);
      const auto cost = CreateBicycleCost(request.options().costings().at(Costing::bicycle));

      edge.set_surface(baldr::Surface::kImpassable);
      EXPECT_TRUE(cost->IsAccessible(&edge));
      EXPECT_FALSE(cost->Allowed(&edge, tile, edgeid));

      edge.set_surface(baldr::Surface::kPavedSmooth);
      EXPECT_TRUE(cost->Allowed(&edge, tile, edgeid));

      edge.set_surface(baldr::Surface::kPath);
      EXPECT_EQ(cost->Allowed(&edge, tile, edgeid), avoid_bad_surfaces < 1.0f || type == "mountain");
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
