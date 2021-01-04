#ifndef VALHALLA_LOKI_POLYGON_SEARCH_H_
#define VALHALLA_LOKI_POLYGON_SEARCH_H_

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>

#include <functional>

namespace valhalla {
namespace loki {

double GetRingsArea(const google::protobuf::RepeatedPtrField<Options::AvoidPolygon>& rings_pbf);

} // namespace loki
} // namespace valhalla

#endif // VALHALLA_LOKI_POLYGON_SEARCH_H_
