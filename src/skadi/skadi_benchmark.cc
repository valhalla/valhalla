#include <fstream>
#include <list>
#include <utility>

#include <valhalla/midgard/logging.h>

#include "skadi/sample.h"

int main(int argc, char** argv) {

  if(argc < 2)
    throw std::runtime_error("No data source specified");
  if(argc < 3)
    throw std::runtime_error("No coordinate postings provided");

  LOG_INFO("Loading data source");
  valhalla::skadi::sample sample(argv[1]);

  LOG_INFO("Loading coordinate postings");
  std::list<std::pair<double, double> > postings{{}};
  std::ifstream file(argv[2]);
  while(file >> postings.back().first >> postings.back().second)
    postings.emplace_back();
  postings.pop_back();

  LOG_INFO("Sampling at postings");
  for(const auto& p : postings)
    sample.get(p);

  LOG_INFO("Finished");

  return EXIT_SUCCESS;
}
