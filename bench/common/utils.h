#ifndef VALHALLA_BENCH_UTILS_H_
#define VALHALLA_BENCH_UTILS_H_

#include <iostream>
#include <sstream>

#include "sif/costfactory.h"
#include "sif/dynamiccost.h"
#include "valhalla/worker.h"

namespace valhalla {

using sif::cost_ptr_t;
using sif::CostFactory;
using sif::DynamicCost;

namespace bench {

std::string LoadFile(const std::string& filename) {
  std::stringstream ss;
  std::string line;
  std::ifstream input_file;
  input_file.open(filename.c_str());
  while (std::getline(input_file, line)) {
    ss << line;
  }
  return ss.str();
}

cost_ptr_t MakeCosting(const std::string& cost_mode) {
  Options options;
  for (int i = 0; i < Costing_MAX; ++i) {
    options.add_costing_options();
  }
  Costing costing;
  Costing_Enum_Parse(cost_mode, &costing);
  options.set_costing(costing);
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  return factory.Create(options);
}
} // namespace bench
} // namespace valhalla

#endif // VALHALLA_BENCH_UTILS_H_
