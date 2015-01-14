#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <vector>

#include <valhalla/proto/trippath.pb.h>

namespace valhalla {
namespace odin {

class EnhancedTripPath : public TripPath {
 public:

  TripPath_Edge* GetPrevEdge(const int nodeIndex, int delta = 1);

  TripPath_Edge* GetCurrEdge(const int nodeIndex);

  TripPath_Edge* GetNextEdge(const int nodeIndex, int delta = 1);

  bool IsValidNodeIndex(int nodeIndex);

  bool IsFirstNodeIndex(int nodeIndex);

  bool IsLastNodeIndex(int nodeIndex);

 protected:
  EnhancedTripPath();

};

}
}

#endif  // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
