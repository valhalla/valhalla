#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/proto/api.pb.h>

namespace vb = valhalla::baldr;

namespace {} // namespace

namespace valhalla {
namespace incidents {

// process the locate output
void get_locate_req(Api& request,
                    const vb::OpenLR::LocationReferencePoint& lrp,
                    const bool flip_bearing) {
  const auto bearing = flip_bearing ? (static_cast<uint32_t>(lrp.bearing) + 180UL) % 360UL
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

} // namespace incidents
} // namespace valhalla