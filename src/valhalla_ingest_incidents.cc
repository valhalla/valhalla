#include <boost/property_tree/ptree.hpp>

#include "baldr/rapidjson_utils.h"
#include "config.h"
#include "incidents/worker.h"
#include "midgard/logging.h"
#include "tyr/actor.h"
#include <cxxopts.hpp>
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>

using namespace prime_server;

int main(int argc, char** argv) {
  // args
  filesystem::path config_file_path;
  boost::property_tree::ptree pt;

  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_ingest_indicidents",
      "valhalla_ingest_indicidents " VALHALLA_VERSION "\n\n"
      "Starts a server to ingest incident data via HTTP from intermediate software,\n"
      "which consumes raw incident data, e.g. by TomTom. Will write incidents speed\n"
      "updates into the configured traffic.tar file.\n\n");

    using namespace std::string_literals;
    options.add_options()
      ("h,help", "Print this help message.")
      ("c,config", "Path to the configuration file", cxxopts::value<std::string>())
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>());
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("version")) {
      std::cout << "valhalla_ingest_indicidents " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    // Read the config file
    if (result.count("inline-config")) {
      std::stringstream ss;
      ss << result["inline-config"].as<std::string>();
      rapidjson::read_json(ss, pt);
    } else if (result.count("config") &&
               filesystem::is_regular_file(
                   config_file_path = filesystem::path(result["config"].as<std::string>()))) {
      rapidjson::read_json(config_file_path.string(), pt);
    } else {
      std::cerr << "Configuration is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // gracefully shutdown when asked via SIGTERM
  prime_server::quiesce(pt.get<unsigned int>("httpd.service.drain_seconds", 28U),
                        pt.get<unsigned int>("httpd.service.shutting_seconds", 1U));

  std::string listen = pt.get<std::string>("httpd.service.listen");
  std::string loopback = pt.get<std::string>("httpd.service.loopback");
  std::string interrupt = pt.get<std::string>("httpd.service.interrupt");
  uint32_t request_timeout = pt.get<uint32_t>("httpd.service.timeout_seconds");

  // check the server endpoint
  if (listen.find("tcp://") != 0) {
    if (listen.find("ipc://") != 0) {
      LOG_ERROR("You must listen on either tcp://ip:port or ipc://some_socket_file");
      return EXIT_FAILURE;
    } else {
      LOG_WARN("Listening on a domain socket limits the server to local requests");
    }
  }

  // configure logging
  auto logging_subtree = pt.get_child_optional("loki.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }
  auto nthreads =
      std::max(static_cast<std::uint32_t>(1),
               pt.get<std::uint32_t>("mjolnir.concurrency", std::thread::hardware_concurrency()));

  // setup the server & proxy within this process
  zmq::context_t context;
  std::thread server_thread = std::thread(
      std::bind(&http_server_t::serve,
                http_server_t(context, listen, "ipc:///tmp/incidents_in", loopback, interrupt, true,
                              DEFAULT_MAX_REQUEST_SIZE, request_timeout)));

  std::thread incident_proxy_thread(
      std::bind(&proxy_t::forward,
                proxy_t(context, "ipc:///tmp/incidents_in", "ipc:///tmp/incidents_out")));
  incident_proxy_thread.detach();
  std::list<std::thread> indicent_worker_threads;
  for (size_t i = 0; i < nthreads; ++i) {
    indicent_worker_threads.emplace_back(valhalla::incidents::run_service, pt);
    indicent_worker_threads.back().detach();
  }

  // wait forever (or for interrupt)
  server_thread.join();

  return 0;
}
