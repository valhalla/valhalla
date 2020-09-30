#ifndef VALHALLA_LOKI_SERIALIZER_COMMON
#define VALHALLA_LOKI_SERIALIZER_COMMON

#include "proto/incidents.pb.h"
#include "proto/trip.pb.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace valhalla {
namespace loki {
namespace incidents {

void serializeIncidentProperties(rapidjson::Writer<rapidjson::StringBuffer>& writer,
                                 const TripLeg::ValhallaIncident& incident,
                                 const std::string& iso_3166_1_alpha2,
                                 const std::string& streets_v8_class,
                                 const std::string& key_prefix);

} // namespace incidents
} // namespace loki
} // namespace valhalla

#endif
