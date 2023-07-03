
#include "valhalla/incidents/worker.h"
#include "valhalla/tyr/actor.h"
#include "valhalla/baldr/rapidjson_utils.h"

using namespace prime_server;

namespace {

worker_t::result_t serialize_incident_error(const valhalla::valhalla_exception_t& exception,
                                            prime_server::http_request_info_t& request_info) {
  std::stringstream body;
  rapidjson::writer_wrapper_t writer(4096);

  // do the writer
  writer.start_object();

  writer("status", exception.http_message);
  writer("status_code", static_cast<uint64_t>(exception.http_code));
  writer("error", std::string(exception.message));
  writer("error_code", static_cast<uint64_t>(exception.code));

  writer.end_object();

  worker_t::result_t result{false, std::list<std::string>(), ""};
  http_response_t response(exception.http_code, exception.http_message, writer.get_buffer(),
                           headers_t{CORS, valhalla::worker::JSON_MIME});
  response.from_info(request_info);
  result.messages.emplace_back(response.to_string());

  return result;
}

} // anonymous namespace

namespace valhalla {
namespace incidents {
incident_worker_t::incident_worker_t(const boost::property_tree::ptree& config_tree)
    : service_worker_t(config_tree), config(config_tree),
      reader(std::make_shared<baldr::GraphReader>(config.get_child("mjolnir"), nullptr, false)) {

  // set the available threads and initialize as many actors as needed
  // thread_count =
  //    std::max(static_cast<unsigned int>(1),
  //             config.get<unsigned int>("mjolnir.concurrency",
  //             std::thread::hardware_concurrency()));
  thread_count = 1;
  LOG_INFO("Running incident action with " + std::to_string(thread_count) + " threads...");

  actors.reserve(thread_count);
  for (uint32_t i = 0; i < thread_count; i++) {
    actors.emplace_back(tyr::actor_t(config, false));
  }

  // signal that the worker started successfully
  started();
}

worker_t::result_t incident_worker_t::work(const std::list<zmq::message_t>& job,
                                           void* request_info,
                                           const std::function<void()>& interrupt_function) {

  // get request info and make sure to record any metrics before we are done
  auto& info = *static_cast<http_request_info_t*>(request_info);
  LOG_INFO("Got Incident Request " + std::to_string(info.id));
  worker_t::result_t result{true, {}, {}};
  try {
    // Set the interrupt function
    service_worker_t::set_interrupt(&interrupt_function);

    // request parsing
    auto http_request =
        http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

    // validate request
    if (http_request.method != method_t::POST) {
      throw valhalla_exception_t{101};
    } else if (http_request.body.empty()) {
      throw valhalla_exception_t{100};
    }

    // Parse the body into a JSON doc
    rapidjson::Document document;
    document.Parse(http_request.body.c_str());

    auto action{IncidentsAction::NONE};
    if (http_request.path == "/update") {
      action = IncidentsAction::UPDATE;
    } else if (http_request.path == "/delete") {
      action = IncidentsAction::DELETE;
    } else if (http_request.path == "/reset") {
      action = IncidentsAction::RESET;
    } else if (http_request.path == "/geojson/matches") {
      action = IncidentsAction::GEOJSON_MATCHES;
    } else if (http_request.path == "/geojson/openlr") {
      action = IncidentsAction::GEOJSON_OPENLR;
    } else {
      throw valhalla_exception_t{106, "/update, /delete, /reset, /geojson/matches, /geojson/openlr"};
    }
    result = to_incident_response(incident_worker_t::incidents(action, document), info);

  } catch (const valhalla_exception_t& e) {
    LOG_WARN("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_incident_error(e, info);
  } catch (const std::exception& e) {
    LOG_ERROR("400::" + std::string(e.what()) + " request_id=" + std::to_string(info.id));
    result = serialize_incident_error({499, std::string(e.what())}, info);
  }

  return result;
}

void run_service(const boost::property_tree::ptree& config) {
  // gracefully shutdown when asked via SIGTERM
  quiesce(config.get<unsigned int>("httpd.service.drain_seconds", 28U),
          config.get<unsigned int>("httpd.service.shutting_seconds", 1U));

  // or returns just location information back to the server
  auto loopback_endpoint = config.get<std::string>("httpd.service.loopback");
  auto interrupt_endpoint = config.get<std::string>("httpd.service.interrupt");

  // listen for requests
  zmq::context_t context;
  incident_worker_t incident_worker(config);
  worker_t worker(context, "ipc:///tmp/incidents_out", "ipc:///dev/null", loopback_endpoint,
                  interrupt_endpoint,
                  std::bind(&incident_worker_t::work, std::ref(incident_worker),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                  std::bind(&incident_worker_t::cleanup, std::ref(incident_worker)));
  worker.work();
}

void incident_worker_t::cleanup() {
}

void incident_worker_t::set_interrupt(const std::function<void()>* interrupt_function) {
  interrupt = interrupt_function;
  reader->SetInterrupt(interrupt);
}

} // namespace incidents
} // namespace valhalla
