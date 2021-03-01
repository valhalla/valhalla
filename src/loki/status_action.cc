#include "loki/worker.h"

namespace valhalla {
namespace loki {
std::string loki_worker_t::status(Api& request) const {
#ifdef HAVE_HTTP
  // if we are in the process of shutting down we signal that here
  // should react by draining traffic (though they are likely doing this as they are usually the ones
  // who sent us the request to shutdown)
  if (prime_server::draining() || prime_server::shutting_down()) {
    throw valhalla_exception_t{102};
  }
#endif
  return "{}";
}
} // namespace loki
} // namespace valhalla
