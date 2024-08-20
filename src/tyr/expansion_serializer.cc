
#include "baldr/rapidjson_utils.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace rapidjson;

namespace valhalla {
namespace tyr {
std::string serializeExpansion(Api& request, const std::string& algo) {
  if (request.options().format() == Options::Format::Options_Format_pbf)
    return serializePbf(request);

  // form GeoJSON
  writer_wrapper_t writer(1024 * 1024);
  writer.start_object();
  writer("type", "FeatureCollection");
  writer.start_array("features");
  writer.set_precision(6);

  std::unordered_set<Options::ExpansionProperties> exp_props;
  for (const auto& prop : request.options().expansion_properties()) {
    exp_props.insert(static_cast<Options_ExpansionProperties>(prop));
  }
  auto expansion = request.expansion();
  for (int i = 0; i < expansion.geometries().size(); ++i) {
    // create features
    writer.start_object(); // feature object
    writer("type", "Feature");

    writer.start_object("geometry");
    writer("type", "LineString");
    writer.start_array("coordinates");

    // make the geom
    const auto& geom = expansion.geometries(i);
    for (int j = 0; j < geom.coords().size() - 1; j += 2) {
      writer.start_array();
      writer(static_cast<double>((geom.coords(j) / 1e6)));
      writer(static_cast<double>((geom.coords(j + 1) / 1e6)));
      writer.end_array();
    }

    writer.end_array();  // coordinates
    writer.end_object(); // geometry

    writer.start_object("properties");
    // no properties asked for, don't collect any
    if (!exp_props.size()) {
      writer.end_object(); // properties
      writer.end_object(); // feature
      continue;
    }

    // make the properties
    if (exp_props.count(Options_ExpansionProperties_duration)) {
      writer("duration", static_cast<uint64_t>(expansion.durations(i)));
    }
    if (exp_props.count(Options_ExpansionProperties_distance)) {
      writer("distance", static_cast<uint64_t>(expansion.distances(i)));
    }
    if (exp_props.count(Options_ExpansionProperties_cost)) {
      writer("cost", static_cast<uint64_t>(expansion.costs(i)));
    }
    if (exp_props.count(Options_ExpansionProperties_edge_status))
      writer("edge_status", Expansion_EdgeStatus_Enum_Name(expansion.edge_status(i)));
    if (exp_props.count(Options_ExpansionProperties_edge_id))
      writer("edge_id", static_cast<uint64_t>(expansion.edge_id(i)));
    if (exp_props.count(Options_ExpansionProperties_pred_edge_id))
      writer("pred_edge_id", static_cast<uint64_t>(expansion.pred_edge_id(i)));
    if (exp_props.count(Options_ExpansionProperties_expansion_type))
      writer("expansion_type", static_cast<uint64_t>(expansion.expansion_type(i)));

    writer.end_object(); // properties
    writer.end_object(); // feature
  }

  // close the GeoJSON
  writer.end_array(); // features
  writer.start_object("properties");
  writer("algorithm", algo);
  writer.end_object();
  writer.end_object(); // object

  return writer.get_buffer();
}
} // namespace tyr
} // namespace valhalla