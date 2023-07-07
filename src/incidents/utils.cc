#include <valhalla/baldr/openlr.h>
#include <valhalla/incidents/utils.h>
#include <valhalla/midgard/encoded.h>
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
  loc->set_radius(5);
  loc->set_heading(bearing);
  loc->set_heading_tolerance(20U);
  loc->mutable_search_filter()->set_min_road_class(valhalla::RoadClass::kResidential);
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
  }

  if (use_bearing) {
    request.mutable_options()->mutable_locations(0)->set_heading(std::get<1>(a_pt));
    request.mutable_options()->mutable_locations(0)->set_heading_tolerance(20U);
  }
}

void print_route(const valhalla::TripLeg& leg) {
  rapidjson::writer_wrapper_t writer(32768); // reserve 32 kb
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");

  writer.set_precision(6);

  const auto pts = vm::decode<std::vector<vm::PointLL>>(leg.shape());

  writer.start_object(); // single feature
  writer("type", "Feature");
  writer.start_object("geometry");
  writer("type", "LineString");
  writer.start_array("coordinates");

  for (const auto& coord : pts) {
    writer.start_array(); // single coordinate
    writer(coord.lng());
    writer(coord.lat());
    writer.end_array(); // single coordinate
  }

  writer.end_array();  // coordinates
  writer.end_object(); // geometry

  writer.start_object("properties");
  writer.end_object(); // properties
  writer.end_object(); // single feature

  writer.end_array();  // features
  writer.end_object(); // FeatureCollection

  std::cerr << writer.get_buffer() << std::endl;
}
} // namespace incidents
} // namespace valhalla