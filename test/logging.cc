#include "midgard/logging.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <future>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

#include "test.h"

using namespace valhalla::midgard;

namespace {

size_t work() {
  std::ostringstream s;
  s << "hi my name is: " << std::this_thread::get_id();

  for (size_t i = 0; i < 2; ++i) {
    // std::async is pretty uninteresting unless you make things yield
    LOG_ERROR(s.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG_WARN(s.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG_INFO(s.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG_DEBUG(s.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG_TRACE(s.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    valhalla::midgard::logging::Log(s.str(), " [CUSTOM] ");
  }
  return 10;
}

TEST(Logging, FileLoggerTest) {
  // get rid of it first so we don't append
  std::remove("test/thread_file_log_test.log");

  // configure bogusly
  EXPECT_THROW(logging::Configure({{"type", "file"},
                                   {"file_name", "test/thread_file_log_test.log"},
                                   {"reopen_interval", "opi-903"}}),
               std::exception);

  // configure properly
  logging::Configure(
      {{"type", "file"}, {"file_name", "test/thread_file_log_test.log"}, {"reopen_interval", "1"}});

  // start up some threads
  std::vector<std::future<size_t>> results;
  for (size_t i = 0; i < 4; ++i) {
    results.emplace_back(std::async(std::launch::async, work));
  }

  for (auto& result : results) {
    ASSERT_NO_THROW(result.get());
  }

  // wait for logger to close and reopen the file
  LOG_TRACE("force log close/reopen");
  std::this_thread::sleep_for(std::chrono::milliseconds(1010));

  // open up the file and make sure it looks right
  std::ifstream file("test/thread_file_log_test.log");

  std::string line;
  size_t error = 0, warn = 0, info = 0, debug = 0, trace = 0, custom = 0;
  while (std::getline(file, line)) {
    error += (line.find(" [ERROR] ") != std::string::npos);
    warn += (line.find(" [WARN] ") != std::string::npos);
    info += (line.find(" [INFO] ") != std::string::npos);
    debug += (line.find(" [DEBUG] ") != std::string::npos);
    trace += (line.find(" [TRACE] ") != std::string::npos);
    custom += (line.find(" [CUSTOM] ") != std::string::npos);
    line.clear();
  }
  size_t line_count = error + warn + info + debug + trace;
  EXPECT_EQ(line_count, 41); // todo: or should it be 40 as it was in the comment?

  EXPECT_EQ(error, 8);
  EXPECT_EQ(warn, 8);
  EXPECT_EQ(info, 8);
  EXPECT_EQ(debug, 8);
  EXPECT_EQ(trace, 9);
  EXPECT_EQ(custom, 8);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
