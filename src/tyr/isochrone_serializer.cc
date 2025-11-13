#include "baldr/rapidjson_utils.h"
#include "exceptions.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "tyr/serializers.h"

#ifdef ENABLE_GEOTIFF
#include <geotiff.h>
#include <geotiffio.h>
#include <tiffio.h>
#include <xtiffio.h>
#endif

#include <cmath>
#include <cstring>
#include <limits>
#include <ranges>
#include <sstream>
#include <utility>
#include <vector>

#ifndef TIFFTAG_GDAL_METADATA
#define TIFFTAG_GDAL_METADATA 42112
#endif

#ifndef TIFFTAG_GDAL_NODATA
#define TIFFTAG_GDAL_NODATA 42113
#endif

using namespace valhalla::baldr::json;

namespace {

#ifdef ENABLE_GEOTIFF
#endif

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

void addLocations(Api& request, rapidjson::writer_wrapper_t& writer) {
  int idx = 0;
  for (const auto& location : request.options().locations()) {
    writer.start_object(); // feature 1
    writer.start_object("geometry");
    writer.start_array("coordinates");
    // first add all snapped points as MultiPoint feature per origin point
    std::unordered_set<PointLL> snapped_points;
    writer.set_precision(tyr::kCoordinatePrecision);
    for (const auto& path_edge : location.correlation().edges()) {
      const PointLL& snapped_current = PointLL(path_edge.ll().lng(), path_edge.ll().lat());
      // remove duplicates of path_edges in case the snapped object is a node
      if (snapped_points.insert(snapped_current).second) {
        writer.start_array();
        writer(snapped_current.lng());
        writer(snapped_current.lat());
        writer.end_array();
      }
    };
    writer.end_array(); // coordinates
    writer("type", "MultiPoint");
    writer.end_object(); // geometry
    writer.start_object("properties");
    writer("location_index", static_cast<uint64_t>(idx));
    writer("type", "snapped");
    writer.end_object(); // properties
    writer("type", "Feature");
    writer.end_object(); // feature 1

    writer.start_object(); // feature 2
    writer.start_object("geometry");
    // then each user input point as separate Point feature
    const valhalla::LatLng& input_latlng = location.ll();
    writer.start_array("coordinates");
    writer(input_latlng.lng());
    writer(input_latlng.lat());
    writer.end_array(); // coordinates
    writer("type", "Point");
    writer.end_object(); // geometry
    writer.start_object("properties");
    writer("location_index", static_cast<uint64_t>(idx));
    writer("type", "input");
    writer.end_object(); // properties
    writer("type", "Feature");
    writer.end_object(); // feature 2
    idx++;
  }
}

#ifdef ENABLE_GEOTIFF
// describe the two custom GDAL tags to libtiff
// FIELD_CUSTOM: ASCII, variable length
constexpr TIFFFieldInfo TIFF_GDAL_INFO[] = {{TIFFTAG_GDAL_METADATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM,
                                             1, 0, const_cast<char*>("GDAL_METADATA")},
                                            {TIFFTAG_GDAL_NODATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1,
                                             0, const_cast<char*>("GDAL_NODATA")}};

class MemTIFFStream {
public:
  // used as callbacks in lib(geo)tiff
  static tmsize_t read(thandle_t handle, void* buf, tmsize_t size) {
    auto* self = static_cast<MemTIFFStream*>(handle);
    if (self->pos_ >= self->buffer_.size())
      return 0;
    tmsize_t can = std::min<tmsize_t>(size, self->buffer_.size() - self->pos_);
    std::memcpy(buf, self->buffer_.data() + self->pos_, static_cast<size_t>(can));
    self->pos_ += can;
    return can;
  }

  static tmsize_t write(thandle_t handle, void* buf, tmsize_t size) {
    auto* self = static_cast<MemTIFFStream*>(handle);
    if (self->pos_ + size > self->buffer_.size())
      self->buffer_.resize(static_cast<size_t>(self->pos_ + size));
    std::memcpy(self->buffer_.data() + self->pos_, buf, static_cast<size_t>(size));
    self->pos_ += size;
    return size;
  }

  static toff_t seek(thandle_t handle, toff_t off, int whence) {
    auto* self = static_cast<MemTIFFStream*>(handle);
    toff_t newpos = self->pos_;
    if (whence == SEEK_SET)
      newpos = off;
    else if (whence == SEEK_CUR)
      newpos += off;
    else if (whence == SEEK_END)
      newpos = static_cast<toff_t>(self->buffer_.size()) + off;
    self->pos_ = newpos;
    return self->pos_;
  }

  static toff_t size(thandle_t handle) {
    auto* self = static_cast<MemTIFFStream*>(handle);
    return static_cast<toff_t>(self->buffer_.size());
  }

  static int close(thandle_t /*handle*/) {
    return 0;
  }

  thandle_t handle() noexcept {
    return reinterpret_cast<thandle_t>(this);
  }
  std::string str() const {
    return std::string(reinterpret_cast<const char*>(buffer_.data()), buffer_.size());
  }

private:
  std::vector<uint8_t> buffer_;
  toff_t pos_ = 0;
};

void register_gdal_custom_tags(TIFF* tif) {
  TIFFMergeFieldInfo(tif, TIFF_GDAL_INFO, sizeof(TIFF_GDAL_INFO) / sizeof(TIFF_GDAL_INFO[0]));
}
// Serialize GeoTIFF via lib(geo)tiff
std::string serializeGeoTIFF(Api& request, const std::shared_ptr<const GriddedData<2>>& isogrid) {

  // time, distance
  std::vector<bool> metrics{false, false};
  for (auto& contour : request.options().contours()) {
    metrics[0] = metrics[0] || contour.has_time_case();
    metrics[1] = metrics[1] || contour.has_distance_case();
  }
  const uint8_t valid_bands = static_cast<uint16_t>(std::count(metrics.begin(), metrics.end(), true));

  auto box = isogrid->MinExtent();
  int32_t ext_x = box[2] - box[0];
  int32_t ext_y = box[3] - box[1];

  // collect data from isogrid
  std::vector<std::vector<uint16_t>> planes(metrics.size());
  for (const auto metric_idx : std::views::iota(0U, metrics.size())) {
    if (!metrics[metric_idx]) {
      continue;
    }
    auto& current_plane = planes[metric_idx];
    current_plane.resize(static_cast<size_t>(ext_x) * static_cast<size_t>(ext_y));

    // seconds or 10 meter steps
    const float scale_factor = (metric_idx == 0) ? 60.f : 100.f;
    for (const auto i : std::views::iota(0, ext_y)) {
      for (const auto j : std::views::iota(0, ext_x)) {
        auto tileid = isogrid->TileId(j + box[0], i + box[1]);
        // flip Y so first row is top
        current_plane[(ext_y - 1 - i) * ext_x + j] =
            static_cast<uint16_t>(isogrid->DataAt(tileid, metric_idx) * scale_factor);
      }
    }
  }

  // in-memory TIFF
  MemTIFFStream mem;
  TIFF* tif =
      XTIFFClientOpen("mem", "w", mem.handle(), MemTIFFStream::read, MemTIFFStream::write,
                      MemTIFFStream::seek, MemTIFFStream::close, MemTIFFStream::size,
                      static_cast<TIFFMapFileProc>(nullptr), static_cast<TIFFUnmapFileProc>(nullptr));

  // Core tags
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(ext_x));
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(ext_y));
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, valid_bands);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_SEPARATE); // one plane per band
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC,
               PHOTOMETRIC_MINISBLACK); // required, hints at B/W display
  TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_PACKBITS);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
  TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);

  // we let libtiff decide how to optimize binning, more interesting for compression
  uint32_t rowsperstrip = TIFFDefaultStripSize(tif, 0);
  if (rowsperstrip == 0 || rowsperstrip > static_cast<uint32_t>(ext_y))
    rowsperstrip = static_cast<uint32_t>(ext_y);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, rowsperstrip);

  // it seems geotiff as we know them have some gdal custom tags
  // TODO: write NODATA values:
  //   - it _should_ be possible with the metadata xml below, e.g.
  //     <Item name="NODATA_VALUES">{} {}</Item> </GDALMetadata>)", isogrid->MaxValue(0),
  //     isogrid->MaxValue(1));
  //   - or initialize the iso_grid with max uint16_t and set TIFFSetField(tif, TIFFTAG_GDAL_NODATA,
  //   "65535");
  register_gdal_custom_tags(tif);
  std::ostringstream gdal_xml;
  gdal_xml << "<GDALMetadata>\n";
  if (metrics[0])
    gdal_xml << "  <Item name=\"time_seconds\" sample=\"0\">Time (seconds)</Item>\n";
  if (metrics[1])
    gdal_xml << "  <Item name=\"distance_decameters\" sample=\""
             << (metrics[0] ? 1 : 0) // distance band index in the written order
             << "\">Distance (10m)</Item>\n";
  gdal_xml << "</GDALMetadata>";
  TIFFSetField(tif, TIFFTAG_GDAL_METADATA, gdal_xml.str().c_str());

  std::string desc;
  if (valid_bands == 2) {
    desc = "Band 1: Time (seconds)\nBand 2: Distance (10m)";
  } else {
    desc = metrics[0] ? "Band 1: Time (seconds)" : "Band 1: Distance (10m)";
  }
  TIFFSetField(tif, TIFFTAG_IMAGEDESCRIPTION, desc.c_str());

  // write geotiff keys
  GTIF* gtif = GTIFNew(tif);
  if (!gtif) {
    XTIFFClose(tif);
    throw valhalla_exception_t{599, "Failed to create GeoTIFF."};
  }
  GTIFKeySet(gtif, GTModelTypeGeoKey, TYPE_SHORT, 1, ModelTypeGeographic);
  GTIFKeySet(gtif, GeographicTypeGeoKey, TYPE_SHORT, 1, GCS_WGS_84);
  GTIFKeySet(gtif, GeogAngularUnitsGeoKey, TYPE_SHORT, 1, Angular_Degree);

  if (!GTIFWriteKeys(gtif)) {
    GTIFFree(gtif);
    XTIFFClose(tif);
    throw valhalla_exception_t{599, "Failed to write GeoTIFF."};
  }
  GTIFFree(gtif);

  // define the origin and pixel scale in real-world coordinates
  const std::array<double, 3> pixel_scale = {static_cast<double>(isogrid->TileSize()), // X scale
                                             static_cast<double>(isogrid->TileSize()), // Y scale
                                             0.0};
  const std::array<double, 6> tiepoints =
      {0.0,
       0.0,
       0.0, // upper-left anchor
       static_cast<double>(isogrid->TileBounds(isogrid->TileId(box[0], box[1])).minx()),
       static_cast<double>(isogrid->TileBounds(isogrid->TileId(box[0], box[3])).maxy()),
       0.0};
  TIFFSetField(tif, TIFFTAG_GEOPIXELSCALE, 3, pixel_scale.data());
  TIFFSetField(tif, TIFFTAG_GEOTIEPOINTS, 6, tiepoints.data());

  // write each band
  uint32_t band_idx = 0;
  for (const auto metric_idx : std::views::iota(0U, metrics.size())) {
    if (!metrics[metric_idx]) {
      continue;
    }
    for (int32_t row = 0; row < ext_y; ++row) {
      uint16_t* rowptr = &planes[metric_idx][static_cast<size_t>(row * ext_x)];
      if (TIFFWriteScanline(tif, reinterpret_cast<void*>(rowptr), static_cast<uint32_t>(row),
                            band_idx) < 0) {
        throw valhalla_exception_t{599, "Failed to write GeoTIFF."};
      }
    }
    band_idx++;
  }

  // Finalize
  if (!TIFFWriteDirectory(tif)) {
    XTIFFClose(tif);
    throw valhalla_exception_t{599, "Failed to write GeoTIFF."};
  }
  XTIFFClose(tif);

  // Return memory buffer
  return mem.str();
}
#endif

std::string serializeIsochroneJson(Api& request,
                                   std::vector<contour_interval_t>& intervals,
                                   contours_t& contours,
                                   bool show_locations,
                                   bool polygons) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object(); // feature_collection
  writer.start_array("features");
  // for each contour interval
  int i = 0;
  assert(intervals.size() == contours.size());
  for (size_t contour_index = 0; contour_index < intervals.size(); ++contour_index) {
    const auto& interval = intervals[contour_index];
    const auto& interval_contours = contours[contour_index];

    std::string hex = getIntervalColor(intervals, i);
    ++i;

    // for each feature on that interval
    for (const auto& feature : interval_contours) {
      writer.start_object(); // feature

      writer.set_precision(2);
      writer.start_object("properties");
      writer("fill-opacity", .33f); // geojson.io polys
      writer("fillColor", hex);     // leaflet polys
      writer("opacity", .33f);      // lines
      writer("fill", hex);          // geojson.io polys
      writer("fillOpacity", .33f);  // leaflet polys
      writer("color", hex);         // lines
      writer("contour", std::get<1>(interval));
      writer("metric", std::get<2>(interval));
      writer.end_object(); // properties

      writer.start_object("geometry");
      writer.start_array("coordinates");
      grouped_contours_t groups = GroupContours(polygons, feature);
      // each group is a polygon consisting of an exterior ring and possibly inner rings
      writer.set_precision(tyr::kCoordinatePrecision);
      for (const auto& group : groups) {
        if (polygons && groups.size() > 1) // start a MultiPolygon?
          writer.start_array();            // poly
        for (const auto& ring : group) {
          if (polygons)
            writer.start_array(); // ring_coords
          for (const auto& pair : *ring) {
            writer.start_array();
            writer(pair.lng());
            writer(pair.lat());
            writer.end_array();
          }
          if (polygons)
            writer.end_array(); // ring_coords
        }
        if (polygons && groups.size() > 1)
          writer.end_array(); // poly
      }

      writer.end_array(); // coordinates
      writer("type", polygons ? groups.size() > 1 ? "MultiPolygon" : "Polygon" : "LineString");
      writer.end_object(); // geometry

      writer("type", "Feature");
      writer.end_object(); // feature
    }
  }

  if (show_locations)
    addLocations(request, writer);
  writer.end_array(); // features

  writer("type", "FeatureCollection");

  if (request.options().has_id_case()) {
    writer("id", request.options().id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    serializeWarnings(request, writer);
  }

  writer.end_object(); // feature_collection
  return writer.get_buffer();
}

std::string serializeIsochronePbf(Api& request,
                                  std::vector<contour_interval_t>& intervals,
                                  const contours_t& contours) {
  // construct pbf output
  auto& isochrone = *request.mutable_isochrone();

  // construct contours
  for (size_t isoline_index = 0; isoline_index < contours.size(); ++isoline_index) {
    const auto& contour = contours[isoline_index];
    const auto& interval = intervals[isoline_index];

    auto* interval_pbf = isochrone.mutable_intervals()->Add();
    interval_pbf->set_metric(std::get<2>(interval) == "time" ? valhalla::Isochrone::time
                                                             : valhalla::Isochrone::distance);

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
#ifdef ENABLE_GEOTIFF
    case Options_Format_geotiff:
      return serializeGeoTIFF(request, isogrid);
#endif
    default:
      throw;
  }
}
} // namespace tyr
} // namespace valhalla
