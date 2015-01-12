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

};

}
}

#endif  // VALHALLA_ODIN_NARRATIVEBUILDER_H_
