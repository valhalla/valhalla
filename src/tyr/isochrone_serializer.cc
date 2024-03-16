
#include "baldr/json.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "thor/worker.h"
#include "tyr/serializers.h"

#include <cmath>
#include <sstream>
#include <utility>

#ifdef ENABLE_GDAL
#include <gdal_priv.h>
#endif

using namespace valhalla::baldr::json;

namespace {

// allows us to only ever register the driver once per process without having to put it
// in every executable that might call into this code
struct gdal_singleton_t {
  static const gdal_singleton_t& get() {
    static const gdal_singleton_t instance;
    return instance;
  }

private:
  gdal_singleton_t() {
#ifdef ENABLE_GDAL
    GDALRegister_GTiff();
#endif
  }
};

using rgba_t = std::tuple<float, float, float>;

using namespace valhalla;
using namespace tyr;
using namespace midgard;
using contour_t = std::list<PointLL>;                 // single ring
using feature_t = std::list<contour_t>;               // rings per interval
using contours_t = std::vector<std::list<feature_t>>; // all rings
using contour_group_t = std::vector<const contour_t*>;
using grouped_contours_t = std::vector<contour_group_t>;
// dimension, value (seconds/meters), name (time/distance), color
using contour_interval_t = std::tuple<size_t, float, std::string, std::string>;

grouped_contours_t GroupContours(const bool polygons, const feature_t& contours) {
  grouped_contours_t results;

  // if the user requested linestrings, we'll give them linestrings
  if (!polygons || contours.size() < 2) {
    std::for_each(contours.begin(), contours.end(),
                  [&results](const contour_t& c) { results.push_back({&c}); });
    return results;
  }

  // inner rings have negative area
  auto isExteriorRing = [](const contour_t& c) -> bool { return polygon_area(c) > 0; };

  std::vector<const contour_t*> inner_ptrs;
  std::for_each(contours.begin(), contours.end(),
                [&results, &inner_ptrs, isExteriorRing](const contour_t& c) {
                  if (isExteriorRing(c)) {
                    results.push_back({&c});
                  } else {
                    inner_ptrs.push_back(&c);
                  }
                });

  // exactly one exterior ring, so all inners go in one group
  if (results.size() == 1) {
    std::for_each(inner_ptrs.begin(), inner_ptrs.end(),
                  [&results](const contour_t* c) { results[0].push_back(c); });
    return results;
  }

  // iterate over outer rings and for each inner ring check if the inner ring is within the exterior
  // ring
  for (const auto* inner : inner_ptrs) {

    // get the first point of the ring (could be any though)
    const PointLL& inner_pt = inner->front();
    bool found_exterior = false;
    // go over exterior rings from smallest to largest
    for (size_t i = results.size(); i > 0; --i) {
      const contour_t& ext = *results[i - 1][0];

      // inner is within exterior ring if any of its points lies within the exterior ring
      // if (inner_pt.WithinPolygon(ext)) {
      if (point_in_poly(inner_pt, ext)) {
        results[i - 1].push_back(inner);
        found_exterior = true;
        break;
      }
    }

    // TODO: reverse winding and make it an exterior ring
    if (!found_exterior) {
      LOG_WARN("No exterior ring contour found for inner contour.");
    }
  }

  return results;
}

std::string getIntervalColor(std::vector<contour_interval_t>& intervals, size_t interval_idx) {
  std::stringstream hex;
  // color was supplied
  if (!std::get<3>(intervals[interval_idx]).empty()) {
    hex << "#" << std::get<3>(intervals[interval_idx]);
  } // or we computed it..
  else {
    auto h = interval_idx * (150.f / intervals.size());
    auto c = .5f;
    auto x = c * (1 - std::abs(std::fmod(h / 60.f, 2.f) - 1));
    auto m = .25f;
    rgba_t color = h < 60 ? rgba_t{m + c, m + x, m}
                          : (h < 120 ? rgba_t{m + x, m + c, m} : rgba_t{m, m + c, m + x});
    hex << "#" << std::hex << static_cast<int>(std::get<0>(color) * 255 + .5f) << std::hex
        << static_cast<int>(std::get<1>(color) * 255 + .5f) << std::hex
        << static_cast<int>(std::get<2>(color) * 255 + .5f);
  }
  return hex.str();
}

void addLocations(Api& request, valhalla::baldr::json::ArrayPtr& features) {
  int idx = 0;
  for (const auto& location : request.options().locations()) {
    // first add all snapped points as MultiPoint feature per origin point
    auto snapped_points_array = array({});
    std::unordered_set<PointLL> snapped_points;
    for (const auto& path_edge : location.correlation().edges()) {
      const PointLL& snapped_current = PointLL(path_edge.ll().lng(), path_edge.ll().lat());
      // remove duplicates of path_edges in case the snapped object is a node
      if (snapped_points.insert(snapped_current).second) {
        snapped_points_array->push_back(
            array({fixed_t{snapped_current.lng(), 6}, fixed_t{snapped_current.lat(), 6}}));
      }
    };
    features->emplace_back(map({{"type", std::string("Feature")},
                                {"properties", map({{"type", std::string("snapped")},
                                                    {"location_index", static_cast<uint64_t>(idx)}})},
                                {"geometry", map({{"type", std::string("MultiPoint")},
                                                  {"coordinates", snapped_points_array}})}}));

    // then each user input point as separate Point feature
    const valhalla::LatLng& input_latlng = location.ll();
    const auto input_array = array({fixed_t{input_latlng.lng(), 6}, fixed_t{input_latlng.lat(), 6}});
    features->emplace_back(
        map({{"type", std::string("Feature")},
             {"properties",
              map({{"type", std::string("input")}, {"location_index", static_cast<uint64_t>(idx)}})},
             {"geometry", map({{"type", std::string("Point")}, {"coordinates", input_array}})}}));
    idx++;
  }
}

#ifdef ENABLE_GDAL
// get a temporary file name suffix for GDAL's virtual file system
std::string GenerateTmpFName() {
  std::stringstream ss;
  ss << "/vsimem/" << std::this_thread::get_id() << "_"
     << std::chrono::high_resolution_clock::now().time_since_epoch().count();
  return ss.str();
}

std::string serializeGeoTIFF(Api& request, const std::shared_ptr<const GriddedData<2>>& isogrid) {

  // time, distance
  std::vector<bool> metrics{false, false};
  for (auto& contour : request.options().contours()) {
    metrics[0] = metrics[0] || contour.has_time_case();
    metrics[1] = metrics[1] || contour.has_distance_case();
  }

  auto box = isogrid->MinExtent();
  int32_t ext_x = box[2] - box[0];
  int32_t ext_y = box[3] - box[1];

  // for GDALs virtual fs
  std::string name = GenerateTmpFName();

  auto nbands = std::count(metrics.begin(), metrics.end(), true);
  char** geotiff_options = NULL;
  geotiff_options = CSLSetNameValue(geotiff_options, "COMPRESS", "PACKBITS");

  gdal_singleton_t::get();
  auto driver_manager = GetGDALDriverManager();
  auto geotiff_driver = driver_manager->GetDriverByName("GTiff");
  auto geotiff_dataset =
      geotiff_driver->Create(name.c_str(), ext_x, ext_y, nbands, GDT_UInt16, geotiff_options);

  OGRSpatialReference spatial_ref;
  spatial_ref.SetWellKnownGeogCS("EPSG:4326");
  double geo_transform[6] = {isogrid->TileBounds(isogrid->TileId(box[0], box[1])).minx(), // minx
                             isogrid->TileSize(),
                             0,
                             isogrid->TileBounds(isogrid->TileId(box[0], box[1])).miny(), // miny
                             0,
                             isogrid->TileSize()};

  geotiff_dataset->SetGeoTransform(geo_transform);
  geotiff_dataset->SetSpatialRef(const_cast<OGRSpatialReference*>(&spatial_ref));

  for (size_t metric_idx = 0; metric_idx < metrics.size(); ++metric_idx) {
    if (!metrics[metric_idx])
      continue; // only create bands for requested metrics
    uint16_t* data = new uint16_t[ext_x * ext_y];

    // seconds or 10 meter steps
    float scale_factor = metric_idx == 0 ? 60 : 100;
    for (int32_t i = 0; i < ext_y; ++i) {
      for (int32_t j = 0; j < ext_x; ++j) {
        auto tileid = isogrid->TileId(j + box[0], i + box[1]);
        data[i * ext_x + j] =
            static_cast<uint16_t>(isogrid->DataAt(tileid, metric_idx) * scale_factor);
      }
    }
    auto band = geotiff_dataset->GetRasterBand(nbands == 2 ? (metric_idx + 1) : 1);
    band->SetNoDataValue(std::numeric_limits<uint16_t>::max());
    band->SetDescription(metric_idx == 0 ? "Time (seconds)" : "Distance (10m)");

    CPLErr err = band->RasterIO(GF_Write, 0, 0, ext_x, ext_y, data, ext_x, ext_y, GDT_UInt16, 0, 0);

    delete[] data;

    if (err != CE_None) {
      throw valhalla_exception_t{599, "Unknown error when writing GeoTIFF."};
    }
  }

  GDALClose(geotiff_dataset);
  vsi_l_offset bufferlength;
  GByte* bytes = VSIGetMemFileBuffer(name.c_str(), &bufferlength, TRUE);

  // TODO: there's gotta be way to do this without copying
  std::string data(reinterpret_cast<char*>(bytes), bufferlength);

  return data;
};
#endif

std::string serializeIsochroneJson(Api& request,
                                   std::vector<contour_interval_t>& intervals,
                                   contours_t& contours,
                                   bool show_locations,
                                   bool polygons) {
  // for each contour interval
  int i = 0;
  auto features = array({});
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& interval_contours = contours[contour_index];

    std::string hex = getIntervalColor(intervals, i);
    ++i;

    // for each feature on that interval
    for (const auto& feature : interval_contours) {
      grouped_contours_t groups = GroupContours(polygons, feature);
      auto geom = array({});
      // each group is a polygon consisting of an exterior ring and possibly inner rings
      for (const auto& group : groups) {
        auto poly = array({});
        for (const auto& ring : group) {
          auto ring_coords = array({});
          for (const auto& pair : *ring) {
            ring_coords->push_back(array({fixed_t{pair.lng(), 6}, fixed_t{pair.lat(), 6}}));
          }
          if (polygons) {
            poly->emplace_back(ring_coords);
          } else {
            poly = ring_coords;
          }
        }
        geom->emplace_back(poly);
      }

      // add a feature
      features->emplace_back(map({
          {"type", std::string("Feature")},
          {"geometry",
           map({
               {"type", std::string(polygons ? groups.size() > 1 ? "MultiPolygon" : "Polygon"
                                             : "LineString")},
               {"coordinates",
                polygons && geom->size() > 1 ? geom : geom->at(0)}, // unwrap linestring, or polygon
                                                                    // if there's only one
           })},
          {"properties", map({
                             {"metric", std::get<2>(interval)},
                             {"contour", baldr::json::float_t{std::get<1>(interval)}},
                             {"color", hex},                     // lines
                             {"fill", hex},                      // geojson.io polys
                             {"fillColor", hex},                 // leaflet polys
                             {"opacity", fixed_t{.33f, 2}},      // lines
                             {"fill-opacity", fixed_t{.33f, 2}}, // geojson.io polys
                             {"fillOpacity", fixed_t{.33f, 2}},  // leaflet polys
                         })},
      }));
    }
  }

  if (show_locations)
    addLocations(request, features);

  auto feature_collection = map({
      {"type", std::string("FeatureCollection")},
      {"features", features},
  });

  if (request.options().has_id_case()) {
    feature_collection->emplace("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    feature_collection->emplace("warnings", serializeWarnings(request));
  }

  std::stringstream ss;
  ss << *feature_collection;

  return ss.str();
}

std::string serializeIsochronePbf(Api& request,
                                  std::vector<contour_interval_t>& intervals,
                                  const contours_t& contours) {
  // construct pbf output
  Isochrone& isochrone = *request.mutable_isochrone();

  // construct contours
  for (size_t isoline_index = 0; isoline_index < contours.size(); ++isoline_index) {
    const auto& contour = contours[isoline_index];
    const auto& interval = intervals[isoline_index];

    auto* interval_pbf = isochrone.mutable_intervals()->Add();
    interval_pbf->set_metric(std::get<2>(interval) == "time" ? Isochrone::time : Isochrone::distance);

    interval_pbf->set_metric_value(std::get<1>(interval));

    // for each feature
    for (const auto& feature : contour) {
      grouped_contours_t groups = GroupContours(true, feature);

      // for each group of rings (first is outer, rest is inner)
      for (const std::vector<const contour_t*>& group_ptr : groups) {
        auto* contour_pbf = interval_pbf->mutable_contours()->Add();

        // construct a geometry
        for (const std::list<PointLL>* ring : group_ptr) {
          std::cerr << "Rings: " << ring->size() << std::endl;

          auto* geom = contour_pbf->mutable_geometries()->Add();
          for (PointLL pair : *ring) {
            geom->add_coords(round(pair.lng() * 1e6));
            geom->add_coords(round(pair.lat() * 1e6));
          }
        }
      }
    }
  }

  return serializePbf(request);
}

} // namespace

namespace valhalla {
namespace tyr {

std::string serializeIsochrones(Api& request,
                                std::vector<midgard::GriddedData<2>::contour_interval_t>& intervals,
                                const std::shared_ptr<const midgard::GriddedData<2>>& isogrid) {

  // only generate if json or pbf output is requested
  contours_t contours;

  switch (request.options().format()) {
    case Options_Format_pbf:
    case Options_Format_json:
      // we have parallel vectors of contour properties and the actual geojson features
      // this method sorts the contour specifications by metric (time or distance) and then by value
      // with the largest values coming first. eg (60min, 30min, 10min, 40km, 10km)
      contours =
          isogrid->GenerateContours(intervals, request.options().polygons(),
                                    request.options().denoise(), request.options().generalize());
      return request.options().format() == Options_Format_json
                 ? serializeIsochroneJson(request, intervals, contours,
                                          request.options().show_locations(),
                                          request.options().polygons())
                 : serializeIsochronePbf(request, intervals, contours);

#ifdef ENABLE_GDAL
    case Options_Format_geotiff:
      return serializeGeoTIFF(request, isogrid);
#endif
    default:
      throw;
  }
}
} // namespace tyr
} // namespace valhalla
