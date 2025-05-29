#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "thor/matrixalgorithm.h"
#include "tyr/serializers.h"

#include <cstdint>

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::thor;

namespace {

void serialize_duration(const valhalla::Matrix& matrix,
                        rapidjson::writer_wrapper_t& writer,
                        size_t start_td,
                        const size_t td_count) {
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for time in matrix result
    if (matrix.times()[i] != kMaxCost) {
      writer(static_cast<uint64_t>(matrix.times()[i]));
    } else {
      writer(nullptr);
    }
  }
}

void serialize_distance(const valhalla::Matrix& matrix,
                        rapidjson::writer_wrapper_t& writer,
                        size_t start_td,
                        const size_t td_count,
                        double distance_scale) {
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for distance in matrix result
    if (matrix.distances()[i] != kMaxCost) {
      writer(static_cast<uint64_t>(matrix.distances()[i]) * distance_scale);
    } else {
      writer(nullptr);
    }
  }
}

void serialize_shape(const valhalla::Matrix& matrix,
                     rapidjson::writer_wrapper_t& writer,
                     const size_t start_td,
                     const size_t td_count,
                     const ShapeFormat shape_format) {
  // TODO(nils): shapes aren't implemented yet in TDMatrix
  if (shape_format == no_shape || (matrix.algorithm() != Matrix::CostMatrix))
    return;

  for (size_t i = start_td; i < start_td + td_count; ++i) {
    switch (shape_format) {
      // even if it source == target or no route found, we want to emplace an element
      case geojson:
        if (!matrix.shapes()[i].empty()) {
          writer.start_object();
          tyr::geojson_shape(decode<std::vector<PointLL>>(matrix.shapes()[i]), writer);
          writer.end_object();
        } else
          writer(nullptr);
        break;
      default:
        // this covers the polylines
        writer(matrix.shapes()[i]);
    }
  }
}
} // namespace

namespace osrm_serializers {

// Serialize route response in OSRM compatible format.
std::string serialize(const Api& request) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.start_object();
  const auto& options = request.options();

  // If here then the matrix succeeded. Set status code to OK and serialize
  // waypoints (locations).
  writer("code", "Ok");
  writer.start_array("sources");
  osrm::waypoints(options.sources(), writer);
  writer.end_array();
  writer.start_array("destinations");
  osrm::waypoints(options.targets(), writer);
  writer.end_array();
  writer.start_array("durations");
  for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
    const auto first_td = source_index * options.targets_size();
    writer.start_array();
    serialize_duration(request.matrix(), writer, first_td, options.targets_size());
    writer.end_array();
  }
  writer.end_array();

  writer.start_array("distances");
  for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
    const auto first_td = source_index * options.targets_size();
    writer.start_array();
    serialize_distance(request.matrix(), writer, first_td, options.targets_size(), 1.0);
    writer.end_array();
  }
  writer.end_array();
  writer("algorithm", MatrixAlgoToString(request.matrix().algorithm()));

  writer.end_object();
  return writer.get_buffer();
}
} // namespace osrm_serializers

namespace valhalla_serializers {

void locations(const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
               rapidjson::writer_wrapper_t& writer) {
  for (const auto& location : locations) {
    if (location.correlation().edges().size() == 0) {
      writer(nullptr);
    } else {
      auto& corr_ll = location.correlation().edges(0).ll();
      writer.start_object();
      writer.set_precision(tyr::kCoordinatePrecision);
      writer("lat", corr_ll.lat());
      writer("lon", corr_ll.lng());
    }
    writer.end_object();
  }
}

void serialize_row(const valhalla::Matrix& matrix,
                   rapidjson::writer_wrapper_t& writer,
                   size_t start_td,
                   const size_t td_count,
                   const size_t source_index,
                   const size_t target_index,
                   const double distance_scale,
                   const ShapeFormat shape_format) {
  writer.start_array();
  for (size_t i = start_td; i < start_td + td_count; ++i) {
    // check to make sure a route was found; if not, return null for distance & time in matrix
    // result
    const auto time = matrix.times()[i];
    const auto& date_time = matrix.date_times()[i];
    const auto& time_zone_offset = matrix.time_zone_offsets()[i];
    const auto& time_zone_name = matrix.time_zone_names()[i];
    const auto& begin_lat = matrix.begin_lat()[i];
    const auto& begin_lon = matrix.begin_lon()[i];
    const auto& end_lat = matrix.end_lat()[i];
    const auto& end_lon = matrix.end_lon()[i];
    const auto& begin_heading = matrix.begin_heading()[i];
    const auto& end_heading = matrix.end_heading()[i];
    writer.start_object();
    if (time != kMaxCost) {
      writer("from_index", source_index);
      writer("to_index", target_index + (i - start_td));
      writer("time", static_cast<uint64_t>(time));
      writer("distance", static_cast<double>(matrix.distances()[i] * distance_scale));
      if (!date_time.empty()) {
        writer("date_time", date_time);
      }

      if (!time_zone_offset.empty()) {
        writer("time_zone_offset", time_zone_offset);
      }

      if (!time_zone_name.empty()) {
        writer("time_zone_name", time_zone_name);
      }

      writer.set_precision(1);
      if (begin_heading != kInvalidHeading) {
        writer("begin_heading", begin_heading);
      }

      if (end_heading != kInvalidHeading) {
        writer("end_heading", end_heading);
      }
      writer.set_precision(tyr::kCoordinatePrecision);
      if (begin_lat != INVALID_LL) {
        writer("begin_lat", begin_lat);
      }
      if (begin_lon != INVALID_LL) {
        writer("begin_lon", begin_lon);
      }
      if (end_lat != INVALID_LL) {
        writer("end_lat", end_lat);
      }
      if (end_lon != INVALID_LL) {
        writer("end_lon", end_lon);
      }
      writer.set_precision(tyr::kDefaultPrecision);
      if (matrix.shapes().size() && shape_format != no_shape) {
        // TODO(nils): tdmatrices don't have "shape" support yet
        if (!matrix.shapes()[i].empty()) {
          switch (shape_format) {
            case geojson:
              writer.start_object("shape");
              tyr::geojson_shape(decode<std::vector<PointLL>>(matrix.shapes()[i]), writer);
              writer.end_object();
              break;
            default:
              writer("shape", matrix.shapes()[i]);
          }
        }
      }
    } else {
      writer("from_index", source_index);
      writer("to_index", target_index + (i - start_td));
      writer("time", nullptr);
      writer("distance", nullptr);
    }
    writer.end_object();
  }
  writer.end_array();
}

std::string serialize(const Api& request, double distance_scale) {
  rapidjson::writer_wrapper_t writer(4096);
  writer.set_precision(tyr::kDefaultPrecision);
  writer.start_object();
  const auto& options = request.options();

  if (options.verbose()) {
    writer.start_array("sources_to_targets");
    for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
      serialize_row(request.matrix(), writer, source_index * options.targets_size(),
                    options.targets_size(), source_index, 0, distance_scale, options.shape_format());
    }
    writer.end_array(); // sources_to_targets

    writer.start_array("sources");
    locations(options.sources(), writer);
    writer.end_array();
    writer.start_array("targets");
    locations(options.targets(), writer);
    writer.end_array();
  } // slim it down
  else {
    writer.start_object("sources_to_targets");

    writer.start_array("durations");
    for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
      const auto first_td = source_index * options.targets_size();
      writer.start_array();
      serialize_duration(request.matrix(), writer, first_td, options.targets_size());
      writer.end_array();
    }
    writer.end_array();

    writer.start_array("distances");
    for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
      const auto first_td = source_index * options.targets_size();
      writer.start_array();
      serialize_distance(request.matrix(), writer, first_td, options.targets_size(), distance_scale);
      writer.end_array();
    }
    writer.end_array();

    if (!(options.shape_format() == no_shape ||
          (request.matrix().algorithm() != Matrix::CostMatrix))) {
      writer.start_array("shapes");
      for (int source_index = 0; source_index < options.sources_size(); ++source_index) {
        const auto first_td = source_index * options.targets_size();
        writer.start_array();
        serialize_shape(request.matrix(), writer, first_td, options.targets_size(),
                        options.shape_format());
        writer.end_array();
      }
      writer.end_array();
    }

    writer.end_object(); // sources_to_targets
  }
  writer("units", Options_Units_Enum_Name(options.units()));
  writer("algorithm", MatrixAlgoToString(request.matrix().algorithm()));

  if (options.has_id_case()) {
    writer("id", options.id());
  }

  // add warnings to json response
  if (request.info().warnings_size() >= 1) {
    tyr::serializeWarnings(request, writer);
  }

  writer.end_object();
  return writer.get_buffer();
}
} // namespace valhalla_serializers

namespace valhalla {
namespace tyr {

std::string serializeMatrix(Api& request) {
  double distance_scale = (request.options().units() == Options::miles) ? kMilePerMeter : kKmPerMeter;

  // error if we failed finding any connection
  // dont bother serializing in case of /expansion request
  if (std::all_of(request.matrix().times().begin(), request.matrix().times().end(),
                  [](const float& time) { return time == kMaxCost; })) {
    throw valhalla_exception_t(442);
  } else if (request.options().action() == Options_Action_expansion) {
    return "";
  }

  switch (request.options().format()) {
    case Options_Format_osrm:
      return osrm_serializers::serialize(request);
    case Options_Format_json:
      return valhalla_serializers::serialize(request, distance_scale);
    case Options_Format_pbf:
      return serializePbf(request);
    default:
      throw;
  }
}

} // namespace tyr
} // namespace valhalla
