#ifndef VALHALLA_ODIN_NARRATIVEBUILDER_H_
#define VALHALLA_ODIN_NARRATIVEBUILDER_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>

namespace valhalla{
namespace odin{

class NarrativeBuilder {
 public:
  NarrativeBuilder(TripPath& trip_path);

  void Build();

 protected:
  TripPath& trip_path_;

  // TODO - temps for initial end to end test
  std::vector<std::vector<std::string>> maneuver_names_;
  std::vector<float> maneuver_distance_;


};

}
}

#endif  // VALHALLA_ODIN_NARRATIVEBUILDER_H_
