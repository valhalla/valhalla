#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "odin/worker.h"

int main(int argc, char** argv) {

  if(argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " conf/valhalla.json" << std::endl;
    return 1;
  }

  //config file
  std::string config_file(argv[1]);
  boost::property_tree::ptree config;
  boost::property_tree::read_json(config_file, config);

  //run the service worker
  valhalla::odin::run_service(config);

  return 0;
}
