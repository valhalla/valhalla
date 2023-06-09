#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/proto/api.pb.h>

namespace vb = valhalla::baldr;

namespace {
static valhalla::Api get_base_req() {
  valhalla::Api base_request;
  base_request.mutable_options()->set_action(valhalla::Options::locate);
  base_request.mutable_options()->set_costing_type(valhalla::Costing::auto_);
  base_request.mutable_options()->set_verbose(true);

  return base_request;
}

} // namespace

namespace valhalla {
namespace incidents {

// process the locate output
Api get_locate_req(const vb::OpenLR::LocationReferencePoint& lrp, bool flip_bearing) {
  const auto bearing = flip_bearing ? (static_cast<uint32_t>(lrp.bearing) + 180U) % 360U
                                    : static_cast<uint32_t>(lrp.bearing);

  auto request = get_base_req();
  auto* loc = request.mutable_options()->mutable_locations()->Add();
  loc->mutable_ll()->set_lng(lrp.longitude);
  loc->mutable_ll()->set_lng(lrp.latitude);
  loc->set_radius(15);
  loc->set_heading(bearing);
  loc->set_heading_tolerance(20U);
  loc->mutable_search_filter()->set_min_road_class(valhalla::RoadClass::kResidential);
  loc->mutable_search_filter()->set_max_road_class(static_cast<RoadClass>(lrp.frc));

  return request;
};

} // namespace incidents
} // namespace valhalla