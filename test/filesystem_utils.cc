
#include "filesystem_utils.h"
#include "test.h"

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace vfs = valhalla::filesystem_utils;
namespace stdfs = std::filesystem;

TEST(Filesystem, concurrent_folder_create_delete) {

  const std::string nested_subdir = stdfs::path{"test/k/l/m/n/o/p/q/r/s/t"};
  const std::string base_subdir = stdfs::path{"test/k"};

  if (stdfs::is_directory(base_subdir))
    stdfs::remove_all(base_subdir);

  // Use half the available cores so we don't place too much extra pressure
  // on the CI test runs which are already run in parallel.
  size_t num_threads = std::thread::hardware_concurrency() / 2;

  // There's no point in running this test if it will be single threaded.
  if (num_threads < 2)
    return;

  // start_race_mutex protects the start_race_event and the start_race
  // boolean. Condition variables can be awoken "spuriously", hence the
  // start_race boolean.
  std::mutex start_race_mutex;
  bool start_race = false;
  std::condition_variable start_race_event;

  auto create_folder = [&](bool& ready_flag) {
    SCOPED_TRACE("Had trouble creating path " + nested_subdir);
    // a simple (cheesy?) way for each thread to announce they are ready
    ready_flag = true;

    // All threads will queue up at start_race_event.wait(), only
    // being released when the "race manager" sets start_race to true
    // and calls start_race_event.notify_all().
    std::unique_lock<std::mutex> start_race_lock(start_race_mutex);
    start_race_event.wait(start_race_lock, [&] { return start_race; });
    start_race_lock.unlock();

    stdfs::create_directories(nested_subdir);
    ASSERT_TRUE(stdfs::exists(nested_subdir));
  };

  const int num_iters = 50;
  for (int j = 0; j < num_iters; j++) {
    // concurrent folder creation
    // Start some threads. Wait for each to declare themselves ready.
    start_race = false;
    std::vector<std::unique_ptr<std::thread>> threads(num_threads);
    for (size_t i = 0; i < num_threads; i++) {
      bool ready_flag = false;
      threads[i] = std::make_unique<std::thread>(create_folder, std::ref(ready_flag));
      while (!ready_flag) {
        std::this_thread::yield();
      }
    }

    // By this point we know all threads have been created and are ready.
    // Announce the start of the race by first setting the "start_race"
    // cv boolean and then calling "notify_all()".
    start_race_mutex.lock();
    start_race = true;
    start_race_mutex.unlock();
    start_race_event.notify_all();

    for (size_t i = 0; i < num_threads; i++) {
      threads[i]->join();
    }

    ASSERT_TRUE(stdfs::exists(nested_subdir));
  }

  stdfs::remove_all(base_subdir);
}

TEST(Filesystem, has_data_member_type) {
  auto arr = vfs::has_data<std::array<int, 3>>::value;
  EXPECT_TRUE(arr);

  auto vc = vfs::has_data<std::vector<int>>::value;
  EXPECT_TRUE(vc);

  auto str = vfs::has_data<std::string>::value;
  EXPECT_TRUE(str);

  auto umap = vfs::has_data<std::unordered_map<int, int>>::value;
  EXPECT_FALSE(umap);

  auto integer = vfs::has_data<int>::value;
  EXPECT_FALSE(integer);

  auto lst = vfs::has_data<std::list<int>>::value;
  EXPECT_FALSE(lst);
}

TEST(Filesystem, save_file_valid_input) {
  std::vector<std::string> tests{"/tmp/save_file_input/utrecht_tiles/0/003/196.gp",
                                 "/tmp/save_file_input/utrecht_tiles/1/051/305.gph",
                                 "/tmp/save_file_input/utrecht_tiles/2/000/818/660.gph"};

  for (const auto& test : tests) {
    EXPECT_TRUE(vfs::save<std::string>(test));
  }

  stdfs::remove_all("/tmp/save_file_input/");
}

TEST(Filesystem, save_file_invalid_input) {
  std::vector<std::filesystem::path> tests{"", "/etc/", "/tmp/", "/var/"};

  for (const auto& test : tests)
    EXPECT_FALSE(vfs::save<std::string>(test)) << "FAILED " << test;
}