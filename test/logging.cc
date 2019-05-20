#include "midgard/logging.h"
#include "test.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <functional>
#include <future>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

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

void ThreadFileLoggerTest() {
  // get rid of it first so we don't append
  std::remove("test/thread_file_log_test.log");

  // configure bogusly
  try {
    logging::Configure({{"type", "file"},
                        {"file_name", "test/thread_file_log_test.log"},
                        {"reopen_interval", "opi-903"}});
    throw std::runtime_error("Configuring with non numeric reopen_interval should have thrown");
  } catch (...) {}
  // configure properly
  logging::Configure(
      {{"type", "file"}, {"file_name", "test/thread_file_log_test.log"}, {"reopen_interval", "1"}});

  // start up some threads
  std::vector<std::future<size_t>> results;
  for (size_t i = 0; i < 4; ++i) {
    results.emplace_back(std::async(std::launch::async, work));
  }

  // dont really care about the results but we can pretend
  int exit_code = 0;
  for (auto& result : results) {
    try {
      size_t count = result.get();
    } catch (std::exception& e) { exit_code++; }
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
  if (line_count != 41)
    throw std::runtime_error("Log should have exactly 40 lines but had " +
                             std::to_string(line_count));
  if (error != 8 || warn != 8 || info != 8 || debug != 8 || trace != 9 || custom != 8)
    throw std::runtime_error("Wrong distribution of log messages");
}

} // namespace

int main() {
  test::suite suite("logging");

  // check file logging
  suite.test(TEST_CASE(ThreadFileLoggerTest));

  return suite.tear_down();
}
