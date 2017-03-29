#include "meili/geojson_reader.h"

namespace valhalla {

namespace meili {


inline bool
is_geometry(const rapidjson::Value& value)
{ return value.IsObject() && value.HasMember("coordinates"); }


inline bool
is_feature(const rapidjson::Value& value)
{
  // Strictly speak a GeoJSON feature must have "id" and "properties",
  // but in our case they are optional. A full example is as folllows:
  // {"id": 1, "type": "Feature", "geometry": GEOMETRY, "properties": {"times": [], "radius": []}}

  // We follow Postel's Law: be liberal in what you accept
  return value.IsObject()
      // && object.HasMember("type")
      // && std::string(object["type"].GetString()) == "Feature"
      && value.HasMember("geometry");
}


inline bool
is_feature_collection(const rapidjson::Value& value)
{
  return value.IsObject()
      && value.HasMember("type") && value["type"] == "FeatureCollection"
      && value.HasMember("features") && value["features"].IsArray();
}


inline bool
is_geometry_collection(const rapidjson::Value& value)
{
  return value.IsObject()
      && value.HasMember("type") && value["type"] == "GeometryCollection"
      && value.HasMember("geometries") && value["geometries"].IsArray();
}


std::vector<midgard::PointLL>
read_coordinates(const rapidjson::Value& geometry)
{
  if (!is_geometry(geometry)) {
    throw SequenceParseError("Invalid GeoJSON geometry");
  }

  // Parse coordinates
  if (!geometry.HasMember("coordinates")) {
    throw SequenceParseError("Invalid GeoJSON geometry: coordinates not found");
  }

  const auto& coordinates = geometry["coordinates"];
  if (!coordinates.IsArray()) {
    throw SequenceParseError("Invalid GeoJSON geometry: coordindates is not an array of coordinates");
  }

  std::vector<midgard::PointLL> coords;
  for (rapidjson::SizeType i = 0; i < coordinates.Size(); i++) {
    const auto& coordinate = coordinates[i];
    if (!coordinate.IsArray()
        || coordinate.Size() != 2
        || !coordinate[0].IsNumber()
        || !coordinate[1].IsNumber()) {
      throw SequenceParseError("Invalid GeoJSON geometry: coordindate at "
                               + std::to_string(i)
                               + " is not a valid coordinate (a array of two numbers)");
    }
    const auto lng = coordinate[0].GetDouble(),
               lat = coordinate[1].GetDouble();
    coords.emplace_back(lng, lat);
  }

  return coords;
}


rapidjson::Document
parse_json(const char* text)
{
  rapidjson::Document document;

  document.Parse(text);
  if (document.HasParseError()) {
    std::string message(GetParseError_En(document.GetParseError()));
    throw SequenceParseError("Unable to parse JSON body: " + message);
  }

  return document;
}


GeoJSONReader::GeoJSONReader(float default_gps_accuracy, float default_search_radius)
    : default_gps_accuracy_(default_gps_accuracy),
      default_search_radius_(default_search_radius) {}


std::vector<Measurement>
GeoJSONReader::ReadGeometry(const rapidjson::Value& value) const
{
  std::vector<Measurement> sequence;
  const auto& coords = read_coordinates(value);
  for (const auto& coord: coords) {
    sequence.emplace_back(coord, default_gps_accuracy_, default_search_radius_);
  }
  return sequence;
}


std::vector<Measurement>
GeoJSONReader::ReadFeature(const rapidjson::Value& value) const
{
  if (!is_feature(value)) {
    throw SequenceParseError("Invalid GeoJSON feature");
  }

  float default_gps_accuracy = default_gps_accuracy_,
       default_search_radius = default_search_radius_;
  const auto& coords = read_coordinates(value["geometry"]);

  // TODO: add timestamp
  // TODO: limit gps_accuracy and search_radius
  std::vector<float> gps_accuracy, search_radius;
  if (value.HasMember("properties")) {
    const auto& properties = value["properties"];

    if (properties.HasMember("gps_accuracy")) {
      const auto& gps_accuracy_value = properties["gps_accuracy"];
      if (gps_accuracy_value.IsArray()) {
        for (rapidjson::SizeType i = 0; i < gps_accuracy_value.Size(); i++) {
          // TODO: check overflow
          gps_accuracy.emplace_back(gps_accuracy_value[i].GetDouble());
        }
      } else if (gps_accuracy_value.IsNumber()) {
        // TODO: check overflow
        default_gps_accuracy = gps_accuracy_value.GetDouble();
      }
    }

    if (properties.HasMember("search_radius")) {
      const auto& search_radius_value = properties["search_radius"];
      if (search_radius_value.IsArray()) {
        for (rapidjson::SizeType i = 0; i < search_radius_value.Size(); i++) {
          // TODO: check overflow
          search_radius.emplace_back(search_radius_value[i].GetDouble());
        }
      } else if (search_radius_value.IsNumber()) {
        // TODO: check overflow
        default_search_radius = search_radius_value.GetDouble();
      }
    }
  }

  std::vector<Measurement> measurements;
  size_t i = 0;
  for (const auto& coord: coords) {
    measurements.emplace_back(coords[i],
                              i < gps_accuracy.size()? gps_accuracy[i] : default_gps_accuracy,
                              i < search_radius.size()? search_radius[i] : default_search_radius);
    i++;
  }

  return measurements;
}


bool GeoJSONReader::Read(const std::string& string,
                         std::vector<std::vector<Measurement>>& sequences) const
{
  bool is_collection;
  const auto& document = parse_json(string.c_str());

  if (is_geometry(document)) {
    sequences.push_back(ReadGeometry(document));
    is_collection = false;

  } else if (is_geometry_collection(document)) {
    const auto& geometries = document["geometries"];
    for (size_t i = 0; i < geometries.Size(); i++) {
      sequences.push_back(ReadGeometry(geometries[i]));
    }
    is_collection = true;

  } else if (is_feature(document)) {
    sequences.push_back(ReadFeature(document));
    is_collection = false;

  } else if (is_feature_collection(document)) {
    const auto& features = document["features"];
    for (size_t i = 0; i < features.Size(); i++) {
      sequences.push_back(ReadFeature(features[i]));
    }
    is_collection = true;

  } else {
    throw SequenceParseError("Invalid GeoJSON object: expect either Feature or Geometry");
  }

  return is_collection;
}

}

}
