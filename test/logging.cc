#include "midgard/logging.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
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

TEST(Logging, FileLoggerTest) {
  std::string base_path = "test/file_log_test.log";

  // Clean up any existing files first to ensure clean test state
  std::remove(base_path.c_str());
  for (int i = 1; i <= 4; ++i) {
    std::string archived_file = base_path + "." + std::to_string(i);
    std::remove(archived_file.c_str());
  }
  // configure bogusly
  EXPECT_THROW(logging::Configure({{"type", "file"},
                                   {"file_name", "test/thread_file_log_test.log"},
                                   {"reopen_interval", "opi-903"}}),
               std::exception);

  // configure properly
  logging::Configure({{"type", "file"},
                      {"file_name", base_path},
                      {"reopen_interval", "1"},
                      {"max_file_size", "4000"}, // Small size to trigger rolling
                      {"max_archived_files", "3"}});

  // start up some threads
  std::vector<std::future<size_t>> results;
  results.reserve(4);
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
  std::ifstream file(base_path);

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

  // Test log rolling
  std::string message =
      "This message should be long enough to trigger log rolling when multiple are written.";

  for (int i = 0; i < 200; ++i) {
    LOG_INFO("Test message " + std::to_string(i) + ": " + message);
  }

  // Give logger time to finish writing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify rolling occurred
  bool current_exists = std::filesystem::exists(base_path);
  EXPECT_TRUE(current_exists) << "Current log file should exist";

  // Check for archived files
  size_t archived_count = 0;
  for (int i = 1; i <= 5; ++i) {
    std::string archived_file = base_path + "." + std::to_string(i);
    if (std::filesystem::exists(archived_file)) {
      archived_count++;
    }
  }

  EXPECT_GT(archived_count, 0) << "Should have at least one archived file";

  // Clean up
  std::remove(base_path.c_str());
  for (int i = 1; i <= 4; ++i) {
    std::string archived_file = base_path + "." + std::to_string(i);
    std::remove(archived_file.c_str());
  }
}

} // namespace

