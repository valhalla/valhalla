#include "baldr/edgeinfo.h"

namespace valhalla{
namespace baldr{
  EdgeInfo::EdgeInfo() {
  }

  const GraphId& EdgeInfo::nodea() const {
    return nodea_;
  }

  const GraphId& EdgeInfo::nodeb() const {
    return nodeb_;
  }

  const std::vector<PointLL>& EdgeInfo::shape() const {
    return shape_;
  }

  const std::vector<std::string>& EdgeInfo::nameindexes() const {
    return nameindexes_;
  }
}
}
