#include "loki/serializer_common.h"
#include "proto/trip.pb.h"
#include "proto_conversions.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace valhalla {
namespace loki {
namespace incidents {

void serializeIncidentProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer,
                                 const TripLeg::ValhallaIncident& incident,
                                 const std::string& iso_3166_1_alpha2,
                                 const std::string& streets_v8_class,
                                 const std::string& key_prefix) {

  const auto& meta = incident.metadata();
  writer.Key(key_prefix + "id");
  writer.Int64(meta.id());
  if (meta.has_type() && meta.type()) {
    writer.Key(key_prefix + "type");
    writer.String(std::string(valhalla::incidentTypeToString(meta.type())));
  }
  if (!iso_3166_1_alpha2.empty()) {
    writer.Key(key_prefix + "iso_3166_1_alpha2");
    writer.String(iso_3166_1_alpha2);
  }
  if (meta.has_description() && !meta.description().empty()) {
    writer.Key(key_prefix + "description");
    writer.String(meta.description());
  }
  if (meta.has_creation_time()) {
    writer.Key(key_prefix + "creation_time");
    writer.String(valhalla::time_to_string(meta.creation_time()));
  }
  if (meta.has_start_time()) {
    writer.Key(key_prefix + "start_time");
    writer.String(time_to_string(meta.start_time()));
  }
  if (meta.has_end_time()) {
    writer.Key(key_prefix + "end_time");
    writer.String(time_to_string(meta.end_time()));
  }
  if (meta.has_impact()) {
    writer.Key(key_prefix + "impact");
    writer.String(std::string(valhalla::incidentImpactToString(meta.impact())));
  }
  if (meta.has_sub_type() && !meta.sub_type().empty()) {
    writer.Key(key_prefix + "sub_type");
    writer.String(meta.sub_type());
  }
  if (meta.has_sub_type_description() && !meta.sub_type_description().empty()) {
    writer.Key(key_prefix + "sub_type_description");
    writer.String(meta.sub_type_description());
  }
  if (meta.alertc_codes_size() > 0) {
    writer.Key(key_prefix + "alertc_codes");
    writer.StartArray();
    for (const auto& alertc_code : meta.alertc_codes()) {
      writer.Int(static_cast<uint64_t>(alertc_code));
    }
    writer.EndArray();
  }
  {
    writer.Key(key_prefix + "lanes_blocked");
    writer.StartArray();
    for (const auto& blocked_lane : meta.lanes_blocked()) {
      writer.String(blocked_lane);
    }
    writer.EndArray();
  }
  if (meta.has_road_closed()) {
    writer.Key(key_prefix + "closed");
    writer.Bool(meta.road_closed());
  }
  if (!streets_v8_class.empty()) {
    writer.Key(key_prefix + "class");
    writer.String(streets_v8_class);
  }

  if (meta.has_congestion()) {
    writer.Key(key_prefix + "congestion");
    writer.StartObject();
    writer.Key("value");
    writer.Int(meta.congestion().value());
    writer.EndObject();
  }

  // TODO Add
  //"geometry_index_start": 42,
  //"geometry_index_end": 42
}

} // namespace incidents
} // namespace loki
} // namespace valhalla
