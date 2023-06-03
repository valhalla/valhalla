
#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/incidents/edge_matcher.h>
#include <valhalla/incidents/worker.h>

namespace valhalla {
namespace incidents {

bool incident_worker_t::incidents(IncidentsAction action, rapidjson::Document& req) {

  switch (action) {
    case IncidentsAction::UPDATE:
      update_traffic(req);
      LOG_WARN("/update request");
      break;
    case IncidentsAction::DELETE:
      update_traffic(req);
      LOG_WARN("/delete request");
      break;
    case IncidentsAction::RESET:
      update_traffic(req);
      LOG_WARN("/reset request");
      break;
    default:
      LOG_ERROR("Can't be!");
      break;
  }
  return true;
}
} // namespace incidents
} // namespace valhalla
