#include <iostream>

#include "odin/maneuversbuilder.h"

namespace valhalla {
namespace odin {

ManeuversBuilder::ManeuversBuilder(EnhancedTripPath* etp)
    : trip_path_(etp) {
}

void ManeuversBuilder::Build() {
  std::cout << __FILE__ << ":" << __LINE__ << std::endl;
  std::cout << "BUILD ---------------------------------------------" << std::endl;
  std::cout << "trip_path_->node_size()=" << trip_path_->node_size() << std::endl;
  std::cout << "BUILD ---------------------------------------------" << std::endl;
  for (int i = (trip_path_->node_size() - 1); i >= 0; --i) {
    auto* prevEdge = trip_path_->GetPrevEdge(i);
    auto* currEdge = trip_path_->GetCurrEdge(i);
    auto* nextEdge = trip_path_->GetNextEdge(i);
    std::cout << i << ":  ";
    if (prevEdge)
      std::cout << "prevEdge=" << ((prevEdge->name_size() > 0) ?
          prevEdge->name(0) : "unnamed");
    else
      std::cout << "prevEdge=NONE";

    if (currEdge)
      std::cout << "  | currEdge=" << ((currEdge->name_size() > 0) ?
          currEdge->name(0) : "unnamed");
    else
      std::cout << "  | currEdge=NONE";

    if (nextEdge)
      std::cout << "  | nextEdge=" << ((nextEdge->name_size() > 0) ?
          nextEdge->name(0) : "unnamed") << std::endl;
    else
      std::cout << "  | nextEdge=NONE" << std::endl;

    std::cout << "---------------------------------------------" << std::endl;
  }
}

}
}
