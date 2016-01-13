#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "mmp/service.h"


int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cerr << "usage: service CONFIG" << std::endl;
    return 1;
  }

  std::string filename(argv[1]);
  boost::property_tree::ptree config;
  boost::property_tree::read_json(filename, config);
  mmp::run_service(config);

  return 0;
}
