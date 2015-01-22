#ifndef VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
#define VALHALLA_ODIN_ENHANCEDTRIPPATH_H_

#include <valhalla/proto/trippath.pb.h>

namespace valhalla {
namespace odin {

class EnhancedTripPath;
class EnhancedTripPath_Edge;

class EnhancedTripPath : public TripPath {
 public:

  EnhancedTripPath_Edge* GetPrevEdge(const int nodeIndex, int delta = 1);

  EnhancedTripPath_Edge* GetCurrEdge(const int nodeIndex);

  EnhancedTripPath_Edge* GetNextEdge(const int nodeIndex, int delta = 1);

  bool IsValidNodeIndex(int nodeIndex) const;

  bool IsFirstNodeIndex(int nodeIndex) const;

  bool IsLastNodeIndex(int nodeIndex) const;

  int GetLastNodeIndex() const;

 protected:
  EnhancedTripPath();

};

class EnhancedTripPath_Edge : public TripPath_Edge {
 public:

  bool IsUnnamed() const;

 protected:
  EnhancedTripPath_Edge();

};

}
}

#endif  // VALHALLA_ODIN_ENHANCEDTRIPPATH_H_
