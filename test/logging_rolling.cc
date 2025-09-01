#include "midgard/logging.h"
#include "test.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>

using namespace valhalla::midgard;

namespace {
TEST(Logging, FileLoggerRollingTest) {
  // Clean up any existing test files
  std::string base_path = "test/rolling_test.log";
  std::remove(base_path.c_str());
  for (int i = 1; i <= 5; ++i) {
    std::remove((base_path + "." + std::to_string(i)).c_str());
  }

  logging::Configure({{"type", "file"},
                      {"file_name", base_path},
                      {"max_file_size", "500"}, // Small size to trigger rolling
                      {"max_archived_files", "3"}});

  std::string message =
      "This message should be long enough to trigger log rolling when multiple are written.";

  for (int i = 0; i < 15; ++i) {
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
  for (int i = 1; i <= 5; ++i) {
    std::remove((base_path + "." + std::to_string(i)).c_str());
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
