#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <thread>
#include <unordered_set>

#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#ifdef HAVE_HTTP
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
using namespace prime_server;
#endif

#include "midgard/logging.h"

#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/actor.h"

int main(int argc, char** argv) {
#ifdef HAVE_HTTP
  if (argc < 2 || argc > 4) {
    LOG_ERROR("Usage: " + std::string(argv[0]) + " config/file.json [concurrency]");
    LOG_ERROR("Usage: " + std::string(argv[0]) + " config/file.json action json_request");
    return 1;
  }
#else
  if (argc < 4) {
    LOG_ERROR("Usage: " + std::string(argv[0]) + " config/file.json action json_request");
    return 1;
  }
#endif

  // config file
  // TODO: validate the config
  std::string config_file(argv[1]);
  boost::property_tree::ptree config;
  rapidjson::read_json(config_file, config);

  // one shot direct request mode
  if (argc == 4) {
    // because we want the program output to go only to stdout we force any logging to be stderr
    valhalla::midgard::logging::Configure({{"type", "std_err"}});

    // setup an object that can answer the request
    valhalla::tyr::actor_t actor(config);

    // figure out which action
    valhalla::Options::Action action;
    if (!valhalla::Options_Action_Enum_Parse(argv[2], &action)) {
      std::cerr << "Unknown action" << std::endl;
      return 1;
    }

    // if argv[3] is a file, then use its content as request, otherwise use it directly
    std::string request_str;
    try {
      std::ifstream request_file(argv[3]);
      if (request_file) {
        request_str = std::string((std::istreambuf_iterator<char>(request_file)),
                                  std::istreambuf_iterator<char>());
      } else {
        request_str = argv[3];
      }
    } catch (const std::exception& e) {
      LOG_ERROR(e.what());
      return 1;
    } catch (...) {
      LOG_ERROR("Unknown exception thrown while reading request string or file");
      return 1;
    }

    // do the right action
    valhalla::Api request;
    try {
      switch (action) {
        case valhalla::Options::route:
          std::cout << actor.route(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::locate:
          std::cout << actor.locate(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::sources_to_targets:
          std::cout << actor.matrix(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::optimized_route:
          std::cout << actor.optimized_route(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::isochrone:
          std::cout << actor.isochrone(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::trace_route:
          std::cout << actor.trace_route(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::trace_attributes:
          std::cout << actor.trace_attributes(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::height:
          std::cout << actor.height(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::transit_available:
          std::cout << actor.transit_available(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::expansion:
          std::cout << actor.expansion(request_str, nullptr, &request) << std::endl;
          break;
        case valhalla::Options::status:
          std::cout << actor.status(request_str, nullptr, &request) << std::endl;
          break;
        default:
          std::cerr << "Unknown action" << std::endl;
          return 1;
      }
    } // request processing error specific error condition
    catch (const valhalla::valhalla_exception_t& ve) {
      std::cout << valhalla::jsonify_error(ve, request) << std::endl;
      return 1;
    } // it was a regular exception!?
    catch (const std::exception& e) {
      std::cout << jsonify_error({599, std::string(e.what())}, request) << std::endl;
      return 1;
    } // anything else
    catch (...) {
      std::cout << jsonify_error({599, std::string("Unknown exception thrown")}, request)
                << std::endl;
      return 1;
    }

    // we are done
    return 0;
  }

#ifdef HAVE_HTTP
  // gracefully shutdown when asked via SIGTERM
  prime_server::quiesce(config.get<unsigned int>("httpd.service.drain_seconds", 28),
                        config.get<unsigned int>("httpd.service.shutting_seconds", 1));

  // grab the endpoints
  std::string listen = config.get<std::string>("httpd.service.listen");
  std::string loopback = config.get<std::string>("httpd.service.loopback");
  std::string interrupt = config.get<std::string>("httpd.service.interrupt");
  std::string loki_proxy = config.get<std::string>("loki.service.proxy");
  std::string thor_proxy = config.get<std::string>("thor.service.proxy");
  std::string odin_proxy = config.get<std::string>("odin.service.proxy");
  // TODO: add multipoint accumulator worker

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
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      config.get_child_optional("tyr.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // number of workers to use at each stage
  auto worker_concurrency = std::thread::hardware_concurrency();
  if (argc > 2) {
    worker_concurrency = std::stoul(argv[2]);
  }

  // setup the cluster within this process
  zmq::context_t context;
  std::thread server_thread =
      std::thread(std::bind(&http_server_t::serve, http_server_t(context, listen, loki_proxy + "_in",
                                                                 loopback, interrupt, true)));

  // loki layer
  std::thread loki_proxy_thread(
      std::bind(&proxy_t::forward, proxy_t(context, loki_proxy + "_in", loki_proxy + "_out")));
  loki_proxy_thread.detach();
  std::list<std::thread> loki_worker_threads;
  for (size_t i = 0; i < worker_concurrency; ++i) {
    loki_worker_threads.emplace_back(valhalla::loki::run_service, config);
    loki_worker_threads.back().detach();
  }

  // thor layer
  std::thread thor_proxy_thread(
      std::bind(&proxy_t::forward, proxy_t(context, thor_proxy + "_in", thor_proxy + "_out")));
  thor_proxy_thread.detach();
  std::list<std::thread> thor_worker_threads;
  for (size_t i = 0; i < worker_concurrency; ++i) {
    thor_worker_threads.emplace_back(valhalla::thor::run_service, config);
    thor_worker_threads.back().detach();
  }

  // odin layer
  std::thread odin_proxy_thread(
      std::bind(&proxy_t::forward, proxy_t(context, odin_proxy + "_in", odin_proxy + "_out")));
  odin_proxy_thread.detach();
  std::list<std::thread> odin_worker_threads;
  for (size_t i = 0; i < worker_concurrency; ++i) {
    odin_worker_threads.emplace_back(valhalla::odin::run_service, config);
    odin_worker_threads.back().detach();
  }

  // TODO: add multipoint accumulator

  // wait forever (or for interrupt)
  server_thread.join();
#endif

  return 0;
}
