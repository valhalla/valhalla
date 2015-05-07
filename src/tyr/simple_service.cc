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

#include <valhalla/midgard/logging.h>
#include <valhalla/tyr/route_handler.h>
#include <valhalla/tyr/custom_route_handler.h>
#include <valhalla/tyr/locate_handler.h>
#include <valhalla/tyr/nearest_handler.h>
using namespace valhalla::tyr;

namespace {

  const std::string ROUTE("/route");
  const std::string VIA_ROUTE("/via_route");
  const std::string LOCATE("/locate");
  const std::string NEAREST("/nearest");

  boost::property_tree::ptree from_request(const http_request_t& request) {
    boost::property_tree::ptree pt;

    //throw the json into the ptree
    auto json = request.query.find("json");
    if(json != request.query.end() && json->second.size()) {
      std::istringstream is(json->second.front());
      boost::property_tree::read_json(is, pt);
    }

    //throw the query params into the ptree
    for(const auto& kv : request.query) {
      //skip json or empty entries
      if(kv.first == "json" || kv.second.size() == 0)
        continue;

      //turn single value entries into single key value
      if(kv.second.size() == 1) {
        pt.add(kv.first, kv.second.front());
        continue;
      }

      //make an array of values for this key
      boost::property_tree::ptree array;
      for(const auto& value : kv.second) {
        boost::property_tree::ptree element;
        element.put("", value);
        array.push_back(std::make_pair("", element));
      }
      pt.add_child(kv.first, array);
    }

    return pt;
  }

  class simple_handler_t {
   public:
    simple_handler_t(const std::string config_file) {
      boost::property_tree::read_json(config_file, config);
    }
    worker_t::result_t handle(const std::list<zmq::message_t>& job, void* request_info) {
      //request should look like:
      //  /[route|via_route|locate|nearest]?loc=&json=&jsonp=
      worker_t::result_t result{false};
      try{
        //request parsing
        auto request = http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());
        auto request_pt = from_request(request);
        std::shared_ptr<Handler> handler;
        if(request.path == ROUTE)
          handler.reset(new CustomRouteHandler(config, request_pt));
        else if(request.path == VIA_ROUTE)
          handler.reset(new RouteHandler(config, request_pt));
        else if(request.path == LOCATE)
          handler.reset(new LocateHandler(config, request_pt));
        else if(request.path == VIA_ROUTE)
          handler.reset(new NearestHandler(config, request_pt));
        else
         throw std::runtime_error("Try any of: '/route' '/via_route' '/locate' '/nearest'"); //TODO: 404

        auto body = handler->Action();
        http_response_t response(200, "OK", body);
        response.from_info(static_cast<http_request_t::info_t*>(request_info));
        result.messages.emplace_back(response.to_string());
      }
      catch(const std::exception& e) {
        http_response_t response(400, "Bad Request", e.what());
        response.from_info(static_cast<http_request_t::info_t*>(request_info));
        result.messages.emplace_back(response.to_string());
      }
      return result;
    }
   private:
    boost::property_tree::ptree config;
  };

}

int main(int argc, char** argv) {

  if(argc < 2) {
    LOG_ERROR("Usage: " + std::string(argv[0]) + " server_listen_endpoint config/file.json [concurrency]");
    return 1;
  }

  //server endpoint
  std::string server_endpoint(argv[1]);
  if(server_endpoint.find("tcp://") != 0) {
    if(server_endpoint.find("icp://") != 0) {
      LOG_ERROR("You must listen on either tcp://ip:port or ipc://some_socket_file");
      return EXIT_FAILURE;
    }
    else
      LOG_WARN("Listening on a domain socket limits the server to local requests");
  }

  //config file
  //TODO: validate the config
  std::string config_file(argv[2]);

  //number of workers to use at each stage
  auto worker_concurrency = std::thread::hardware_concurrency();
  if(argc > 3)
    worker_concurrency = std::stoul(argv[3]);

  //change these to tcp://known.ip.address.with:port if you want to do this across machines
  zmq::context_t context;
  std::string result_endpoint = "ipc://result_endpoint";
  std::string handler_endpoint = "ipc://handler_endpoint";

  //server
  std::thread server_thread = std::thread(std::bind(&http_server_t::serve,
    http_server_t(context, server_endpoint, handler_endpoint + "_upstream", result_endpoint)));

  //load balancer for handler
  std::thread handler_proxy(std::bind(&proxy_t::forward, proxy_t(context, handler_endpoint + "_upstream", handler_endpoint + "_downstream")));
  handler_proxy.detach();

  //request handlers
  std::list<std::thread> handler_worker_threads;
  for(size_t i = 0; i < worker_concurrency; ++i) {
    handler_worker_threads.emplace_back(std::bind(&worker_t::work,
      worker_t(context, handler_endpoint + "_downstream", "ipc://null_endpoint", result_endpoint,
      std::bind(&simple_handler_t::handle, simple_handler_t(config_file), std::placeholders::_1, std::placeholders::_2)
    )));
    handler_worker_threads.back().detach();
  }

  //TODO: should we listen for SIGINT and terminate gracefully/exit(0)?
  server_thread.join();

  return 0;
}

