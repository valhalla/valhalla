#ifndef VALHALLA_ODIN_NARRATIVEBUILDER_H_
#define VALHALLA_ODIN_NARRATIVEBUILDER_H_

#include <valhalla/proto/trippath.pb.h>

namespace valhalla{
namespace odin{

class NarrativeBuilder {
 public:
  NarrativeBuilder(TripPath& tripPath);

 protected:
  TripPath& trip_path_;
};

}
}

#endif  // VALHALLA_ODIN_NARRATIVEBUILDER_H_
