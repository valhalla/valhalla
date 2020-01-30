#include "test.h"

#include "mjolnir/graphbuilder.h"

#include <algorithm>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

using namespace std;
using namespace valhalla::mjolnir;

namespace {}

// TODO: sweet jesus add more tests of this class!

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
