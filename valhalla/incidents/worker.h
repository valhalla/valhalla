#ifndef __VALHALLA_INCIDENTS_SERVICE_H__
#define __VALHALLA_INCIDENTS_SERVICE_H__

#include <boost/property_tree/ptree.hpp>
#include <prime_server/prime_server.hpp>

#include <valhalla/tyr/actor.h>
#include <valhalla/worker.h>

const prime_server::headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};

namespace valhalla {
namespace incidents {

using tile_edges_t = std::unordered_map<uint32_t, std::vector<uint32_t>>;

void run_service(const boost::property_tree::ptree& config);

class incident_worker_t : public service_worker_t {
public:
  enum IncidentsAction { NONE = 0, UPDATE = 1, DELETE = 2, RESET = 3 };
  incident_worker_t(const boost::property_tree::ptree& config,
                    const std::shared_ptr<baldr::GraphReader>& graph_reader = {});
  virtual prime_server::worker_t::result_t work(const std::list<zmq::message_t>& job,
                                                void* request_info,
                                                const std::function<void()>& interrupt) override;

  virtual void cleanup() override;
  void set_interrupt(const std::function<void()>* interrupt) override;

protected:
  bool incidents(IncidentsAction action, rapidjson::Document& req);
  std::vector<baldr::GraphId> update_traffic(const rapidjson::Document& req_doc);
  unsigned int thread_count;
  boost::property_tree::ptree config;
  std::shared_ptr<baldr::GraphReader> reader;
  std::vector<tyr::actor_t> actors;

private:
  std::string service_name() const override {
    return "incidents";
  }

  prime_server::worker_t::result_t
  to_incident_response(bool success, prime_server::http_request_info_t& request_info) const {
    prime_server::headers_t headers{CORS, worker::JSON_MIME};
    prime_server::worker_t::result_t result{false, std::list<std::string>(), ""};
    auto status_code = success ? 204U : 400U;
    prime_server::http_response_t response(status_code, "", "", headers);
    response.from_info(request_info);
    result.messages.emplace_back(response.to_string());
    return result;
  }
};
} // namespace incidents
} // namespace valhalla

#endif //__VALHALLA_INCIDENTS_SERVICE_H__
