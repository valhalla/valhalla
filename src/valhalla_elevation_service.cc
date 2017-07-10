#include <thread>
#include <functional>
#include <string>
#include <list>
#include <set>
#include <iostream>
#include <unordered_set>
#include <memory>
#include <stdexcept>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <prime_server/prime_server.hpp>
#include <prime_server/http_protocol.hpp>
using namespace prime_server;

#include "midgard/logging.h"

#include "skadi/worker.h"

int main(int argc, char** argv) {

  if(argc < 2) {
    LOG_ERROR("Usage: " + std::string(argv[0]) + " config/file.json [concurrency]");
    return 1;
  }

  //config file
  //TODO: validate the config
  std::string config_file(argv[1]);
  boost::property_tree::ptree config;
  boost::property_tree::read_json(config_file, config);

  //grab the endpoints
  std::string listen = config.get<std::string>("httpd.service.listen");
  std::string loopback = config.get<std::string>("httpd.service.loopback");
  std::string interrupt = config.get<std::string>("httpd.service.interrupt");
  std::string skadi_proxy = config.get<std::string>("skadi.service.proxy");

  //check the server endpoint
  if(listen.find("tcp://") != 0) {
    if(listen.find("icp://") != 0) {
      LOG_ERROR("You must listen on either tcp://ip:port or ipc://some_socket_file");
      return EXIT_FAILURE;
    }
    else
      LOG_WARN("Listening on a domain socket limits the server to local requests");
  }

  //number of workers to use at each stage
  auto worker_concurrency = std::thread::hardware_concurrency();
  if(argc > 2)
    worker_concurrency = std::stoul(argv[2]);

  //setup the cluster within this process
  zmq::context_t context;
  std::thread server_thread = std::thread(std::bind(&http_server_t::serve,
    http_server_t(context, listen, skadi_proxy + "_in", loopback, interrupt, true)));

  //skadi layer
  std::thread skadi_proxy_thread(std::bind(&proxy_t::forward, proxy_t(context, skadi_proxy + "_in", skadi_proxy + "_out")));
  skadi_proxy_thread.detach();
  std::list<std::thread> skadi_worker_threads;
  for(size_t i = 0; i < worker_concurrency; ++i) {
    skadi_worker_threads.emplace_back(valhalla::skadi::run_service, config);
    skadi_worker_threads.back().detach();
  }

  //wait forever (or for interrupt)
  server_thread.join();

  return 0;
}

