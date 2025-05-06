#include "baldr/transitroute.h"
#include "test.h"

using namespace std;
using namespace valhalla::baldr;

// Expected size is 40 bytes. Since there are still "spare" bits
// we want to alert if somehow any change grows this structure size
constexpr size_t kTransitRouteExpectedSize = 40;

namespace {

TEST(TransitRoute, Sizeof) {
  EXPECT_EQ(sizeof(TransitRoute), kTransitRouteExpectedSize);
}

TEST(TransitRoute, TestWriteRead) {
  // Test building a transit route and reading back values
  TransitType route_type = TransitType::kMetro;
  TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, 888, 999);

  EXPECT_EQ(route.route_type(), route_type);
  EXPECT_EQ(route.one_stop_offset(), 111);
  EXPECT_EQ(route.op_by_onestop_id_offset(), 222);
  EXPECT_EQ(route.op_by_name_offset(), 333);
  EXPECT_EQ(route.op_by_website_offset(), 444);
  EXPECT_EQ(route.route_color(), 555);
  EXPECT_EQ(route.route_text_color(), 666);
  EXPECT_EQ(route.short_name_offset(), 777);
  EXPECT_EQ(route.long_name_offset(), 888);
  EXPECT_EQ(route.desc_offset(), 999);

  // Test bounds for each text offset
  EXPECT_THROW(TransitRoute route(route_type, kMaxNameOffset + 1, 222, 333, 444, 555, 666, 777, 888,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, kMaxNameOffset + 1, 333, 444, 555, 666, 777, 888,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, 222, kMaxNameOffset + 1, 444, 555, 666, 777, 888,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, 222, 333, kMaxNameOffset + 1, 555, 666, 777, 888,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, kMaxNameOffset + 1, 888,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, kMaxNameOffset + 1,
                                  999),
               std::runtime_error);
  EXPECT_THROW(TransitRoute route(route_type, 111, 222, 333, 444, 555, 666, 777, 888,
                                  kMaxNameOffset + 1),
               std::runtime_error);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
