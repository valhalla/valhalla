#ifndef __VALHALLA_ODIN_SERVICE_H__
#define __VALHALLA_ODIN_SERVICE_H__

#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/worker.h>

namespace valhalla {
namespace odin {

#ifdef HAVE_HTTP
void run_service(const boost::property_tree::ptree& config);
#endif

class odin_worker_t : public service_worker_t {
public:
  odin_worker_t(const boost::property_tree::ptree& config);
  virtual ~odin_worker_t();
#ifdef HAVE_HTTP
  virtual worker_t::result_t work(const std::list<zmq::message_t>& job,
                                  void* request_info,
                                  const std::function<void()>& interupt) override;
#endif
  virtual void cleanup() override;

  std::list<TripDirections> narrate(const valhalla_request_t& request,
                                    std::list<TripPath>& legs) const;
};
} // namespace odin
} // namespace valhalla

#endif //__VALHALLA_ODIN_SERVICE_H__
