#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/proto/api.pb.h>

namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

namespace {} // namespace

namespace valhalla {
namespace incidents {

// process the locate output
void get_locate_req(Api& request,
                    const baldr::OpenLR::LocationReferencePoint& lrp,
                    const bool flip_bearing) {
  const auto bearing = flip_bearing ? (static_cast<uint32_t>(lrp.bearing) + 180U) % 360U
                                    : static_cast<uint32_t>(lrp.bearing);

  auto* loc = request.mutable_options()->mutable_locations()->Add();
  loc->mutable_ll()->set_lng(lrp.longitude);
  loc->mutable_ll()->set_lat(lrp.latitude);
  loc->set_radius(15);
  loc->set_heading(bearing);
  loc->set_heading_tolerance(20U);
  loc->mutable_search_filter()->set_min_road_class(valhalla::RoadClass::kResidential);
  loc->mutable_search_filter()->set_max_road_class(static_cast<RoadClass>(lrp.frc));
};

void get_route_req(Api& request,
                   const std::pair<vm::PointLL, uint32_t>&& a_pt,
                   const std::pair<vm::PointLL, uint32_t>&& b_pt,
                   const bool use_bearing) {

  for (const auto& pt : {a_pt, b_pt}) {
    auto* loc = request.mutable_options()->mutable_locations()->Add();
    loc->mutable_ll()->set_lng(std::get<0>(pt).lng());
    loc->mutable_ll()->set_lat(std::get<0>(pt).lat());
    loc->set_radius(0);
    if (use_bearing) {
      loc->set_heading(std::get<1>(pt));
      loc->set_heading_tolerance(20U);
    }
  }

  if (use_bearing) {
    request.mutable_options()->mutable_locations(0)->set_heading(std::get<1>(a_pt));
    request.mutable_options()->mutable_locations(0)->set_heading_tolerance(20);
  }
}
} // namespace incidents
} // namespace valhalla