#include <cxxopts.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <string>
#include <thread>
#ifdef ENABLE_SERVICES
#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
using namespace prime_server;
#endif

#include "argparse_utils.h"
#include "config.h"
#include "loki/worker.h"
#include "midgard/logging.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/actor.h"

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  std::vector<std::string> pos_args;
  boost::property_tree::ptree config;

  cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION +
          "\n\n"
          "a program to run a multi-threaded Valhalla HTTP service powered by https://github.com/kevinkreiser/prime_server\n");

  // clang-format off
  options.add_options()
    ("h,help", "Print this help message.")
    ("v,version","Print the version of this software.")
    // no optional args bcs of pre-cxxopts backwards-compatibility
    ("pos_args", "positional arguments", cxxopts::value<std::vector<std::string>>(pos_args));
  // clang-format on

  try {
    options.parse_positional({"pos_args"});
    options.positional_help("CONFIG_JSON [CONCURRENCY] or CONFIG_JSON ACTION JSON_REQUEST");
    auto result = options.parse(argc, argv);
    // We set up conf & num_threads ourselves
    if (!parse_common_args(program, options, result, nullptr, "", false))
      return EXIT_SUCCESS;

#ifdef ENABLE_SERVICES
    if (pos_args.size() < 1 || pos_args.size() > 3) {
      throw cxxopts::exceptions::exception("[FATAL] Too many or few arguments, see --help:\n");
    }
#else
    if (pos_args.size() != 3) {
      throw cxxopts::exceptions::exception("");
    }
#endif

    // get the config
    const std::string& config_file = pos_args[0];
    config = valhalla::config(config_file);

  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cout << options.help() << "\n";
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // one shot direct request mode
  if (pos_args.size() == 3) {
    // because we want the program output to go only to stdout we force any logging to be stderr
    valhalla::midgard::logging::Configure({{"type", "std_err"}});

    const std::string &action_arg = pos_args[1], request_arg = pos_args[2];

    // setup an object that can answer the request
    valhalla::tyr::actor_t actor(config);

    // figure out which action
    valhalla::Options::Action action;
    if (!valhalla::Options_Action_Enum_Parse(action_arg, &action)) {
      std::cerr << "Unknown action" << std::endl;
      return 1;
    }

    // if argv[3] is a file, then use its content as request, otherwise use it directly
    std::string request_str;
    try {
      std::ifstream request_file(request_arg);
      if (request_file) {
        request_str = std::string((std::istreambuf_iterator<char>(request_file)),
                                  std::istreambuf_iterator<char>());
      } else {
        request_str = request_arg;
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
      std::cout << valhalla::serialize_error(ve, request) << std::endl;
      return 1;
    } // it was a regular exception!?
    catch (const std::exception& e) {
      std::cout << serialize_error({599, std::string(e.what())}, request) << std::endl;
      return 1;
    } // anything else
    catch (...) {
      std::cout << serialize_error({599, std::string("Unknown exception thrown")}, request)
                << std::endl;
      return 1;
    }

    // we are done
    return 0;
  }

#ifdef ENABLE_SERVICES
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
  auto logging_subtree = config.get_child_optional("loki.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // number of workers to use at each stage
  auto worker_concurrency =
      pos_args.size() < 2 ? std::thread::hardware_concurrency() : std::stoul(pos_args[1]);

  uint32_t request_timeout = config.get<uint32_t>("httpd.service.timeout_seconds");

  // setup the cluster within this process
  zmq::context_t context;
  std::thread server_thread =
      std::thread(std::bind(&http_server_t::serve,
                            http_server_t(context, listen, loki_proxy + "_in", loopback, interrupt,
                                          true, DEFAULT_MAX_REQUEST_SIZE, request_timeout)));

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
