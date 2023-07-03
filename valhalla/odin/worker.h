#ifndef __VALHALLA_ODIN_SERVICE_H__
#define __VALHALLA_ODIN_SERVICE_H__

#include <valhalla/odin/markup_formatter.h>
#include <valhalla/proto/api.pb.h>
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
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interupt) override;
#endif

  /**
   * Creates maneuvers and narrative for the path and serializes the entire response to bytes
   * @param request   the request with the filled out trip
   * @return a string of bytes representing the payload, depends on request.options.format
   */
  std::string narrate(Api& request) const;
  void status(Api& request) const;

protected:
  MarkupFormatter markup_formatter_;

private:
  std::string service_name() const override {
    return "odin";
  }
};
} // namespace odin
} // namespace valhalla

#endif //__VALHALLA_ODIN_SERVICE_H__
