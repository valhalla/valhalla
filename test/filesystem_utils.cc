
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
  std::vector<std::string> tests{"", "/etc/", "/tmp/", "/var/"};

  for (const auto& test : tests)
    EXPECT_FALSE(filesystem::save<std::string>(test)) << "FAILED " << test;
}