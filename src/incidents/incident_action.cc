
#include "baldr/rapidjson_utils.h"
#include "incidents/worker.h"

namespace valhalla {
namespace incidents {

bool incident_worker_t::incidents(IncidentsAction action, rapidjson::Document& req) {

  // TODO: do smth with the openlr strings
  // for (const auto& _ : req.GetArray()) {
  //  const auto openlr_str = _.GetString();
  //}
  switch (action) {
    case IncidentsAction::UPDATE:
      LOG_WARN("/update request");
      break;
    case IncidentsAction::DELETE:
      LOG_WARN("/delete request");
      break;
    case IncidentsAction::RESET:
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
