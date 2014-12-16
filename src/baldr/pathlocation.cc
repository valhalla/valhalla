#include "baldr/pathlocation.h"

namespace valhalla{
namespace baldr{

  PathLocation::PathLocation(const Location& location):location_(location), vertex_(false){
  }

  bool PathLocation::IsVertex() const {
    for(auto& edge : edges_){
      if(0 < edge.dist_ && edge.dist_ < 1)
      {
        return (vertex_ = false);
      }
    }
    return (vertex_ = true);
  }


}
}
