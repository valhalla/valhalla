#include "filesystem.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "test.h"

// TODO: move this out of the test and into the filesystem replacement
// and then remove filesystem from mjolnir as well
#ifdef _WIN32
#include <windows.h>
int mkdir(const char* dir_name, long) {
  if (!CreateDirectory(dir_name, NULL))
    return -1;
  return 0;
}
constexpr int perm755 = 0;
#else
constexpr int perm755 = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
#endif

namespace {

TEST(Filesystem, exists) {
  // drop a file
  { std::fstream fs(".touched_file", std::ios::out); }
  // check this dir is there
  EXPECT_TRUE(filesystem::exists(".")) << "the current directory doesnt exist?";
  EXPECT_TRUE(filesystem::exists(".touched_file")) << "couldn't create a file?";
}

TEST(Filesystem, directory) {
  // todo: any specific reason it was repeated 2 times?
  EXPECT_TRUE(filesystem::is_directory(".")) << "the current directory isnt a directory?";
  EXPECT_TRUE(filesystem::is_directory(".")) << "the current directory isnt a directory?";
}

TEST(Filesystem, regular_file) {
  EXPECT_TRUE(filesystem::is_regular_file(".touched_file"));
}

void try_mkdir(const std::string& p) {
  EXPECT_EQ(mkdir(p.c_str(), perm755), 0) << "couldnt create directory";
}

TEST(Filesystem, recursive_directory_listing) {
  // make a directory tree with these entries
  std::string s(1, filesystem::path::preferred_separator);
  std::vector<std::string> dirs{"foo", "foo" + s + "qux", "foo" + s + "quux"};
  std::vector<std::string> files{"foo" + s + "bar", "foo" + s + "baz",
                                 "foo" + s + "quux" + s + "corge"};

  // create them
  for (const auto& d : dirs)
    try_mkdir(d.c_str());
  for (const auto& f : files) {
    std::fstream fs(f, std::ios::out);
    filesystem::path p(f);
    fs.write(p.filename().c_str(), p.filename().string().size());
  }
  // unlike the posix find command the directory iterator doesnt give you access to
  // root or starting directory just the stuff under it so lets pop that off
  dirs.erase(dirs.begin());
  // go get whats there
  for (filesystem::recursive_directory_iterator i("foo"), end; i != end; ++i) {
    if (i->is_directory()) {
      auto pos = std::remove(dirs.begin(), dirs.end(), i->path().string());
      ASSERT_NE(pos, dirs.end()) << "unexpected directory";
      dirs.erase(pos, dirs.end());
      ASSERT_THROW(i->file_size(), std::runtime_error);
    } else if (i->is_regular_file()) {
      auto pos = std::remove(files.begin(), files.end(), i->path().string());
      ASSERT_NE(pos, files.end()) << "unexpected file";
      files.erase(pos, files.end());
      // force it to stat the file
      ASSERT_EQ(i->file_size(), i->path().filename().string().size());
      // check the cached size
      ASSERT_EQ(i->file_size(), i->path().filename().string().size());
    } else {
      FAIL() << "unexpected entry type";
    }
  }
  // if we didnt get everything we have a problem
  EXPECT_TRUE(dirs.empty());
  EXPECT_TRUE(files.empty());

  // cleanup the stuff we made, 2 tests in one ;o)
  filesystem::remove_all("foo");
  EXPECT_TRUE(!filesystem::exists("foo"));
}

TEST(Filesystem, remove_any) {
  // delete non existent thing
  EXPECT_FALSE(filesystem::remove(".foobar")) << "Deleting nonexistent item should return false";

  // make and delete a file
  { std::fstream fs(".foobar", std::ios::out); }
  EXPECT_TRUE(filesystem::remove(".foobar"));
  EXPECT_FALSE(filesystem::exists(".foobar")) << ".foobar file should have been deleted";

  // make and delete a file
  try_mkdir(".foobar");
  EXPECT_TRUE(filesystem::remove(".foobar"));
  EXPECT_FALSE(filesystem::exists(".foobar")) << ".foobar dir should have been deleted";
}

TEST(Filesystem, parent_path) {
  std::vector<filesystem::path> in{{"/"},   {"/foo/bar"}, {"/foo/../"}, {"/foo/bar/../f"},
                                   {"./f"}, {"foo/bar/f"}};
  std::vector<filesystem::path> out{
      {""}, {"/foo"}, {"/foo/.."}, {"/foo/bar/.."}, {"."}, {"foo/bar"},
  };
  for (const auto& i : in) {
    auto a = i.parent_path();
    EXPECT_EQ(a.string(), out[&i - &in.front()].string()) << "wrong parent path";
  }
}

TEST(Filesystem, extension) {
  std::vector<filesystem::path>
      in{{"/foo/bar.txt"},      {"/foo/bar."},        {"/foo/bar"},         {"/foo/bar.txt/bar.cc"},
         {"/foo/bar.txt/bar."}, {"/foo/bar.txt/bar"}, {"/foo/."},           {"/foo/.."},
         {"/foo/.hidden"},      {"/foo/..bar"},       {"/foo/bar.baz.qux"}, {"..."},
         {"/baz/.foo.bar"}};
  std::vector<filesystem::path> out{{".txt"}, {"."}, {""},     {".cc"},  {"."}, {""},    {""},
                                    {""},     {""},  {".bar"}, {".qux"}, {"."}, {".bar"}};
  for (const auto& i : in) {
    auto a = i.extension();
    EXPECT_EQ(a.string(), out[&i - &in.front()].string()) << "wrong extension";
  }
}

TEST(Filesystem, file_size) {
}

TEST(Filesystem, concurrent_folder_create_delete) {

  const char* create_subdir = "test/a/b/c/d/e/f/g/h/i/j";
  const char* base_subdir = "test/a";

  auto create_folder = [&]() {
    bool success = filesystem::create_directories(create_subdir);
    EXPECT_TRUE(success);
    EXPECT_TRUE(filesystem::exists(create_subdir));
  };

  auto remove_folder = [&]() {
    std::uintmax_t delete_count = filesystem::remove_all(base_subdir);
    // I've found that two+ threads will claim that they've deleted the same
    // file object. This results in double+ counting and prevents me from
    // asserting anything related to the delete_count. All we can be sure
    // is the following:
    EXPECT_TRUE(!filesystem::exists(base_subdir));
  };

  size_t num_threads = std::thread::hardware_concurrency();

  const int num_iters = 50;
  for (int j = 0; j < num_iters; j++) {
    // concurrent folder creation
    {
      std::vector<std::unique_ptr<std::thread>> threads(num_threads);
      for (size_t i = 0; i < num_threads; i++)
        threads[i].reset(new std::thread(create_folder));
      for (size_t i = 0; i < num_threads; i++)
        threads[i]->join();
      EXPECT_TRUE(filesystem::exists(create_subdir));
    }

    // populate each subfolder with num_files_per_folder text files.
    {
      const int num_files_per_folder = 10;
      std::string folder_name = "test";
      for (char subdir = 'a'; subdir < 'k'; subdir++) {
        folder_name = folder_name + "/" + subdir;
        for (size_t i = 0; i < num_files_per_folder; i++) {
          std::string filename = folder_name + "/" + std::to_string(i) + ".txt";
          FILE* f = fopen(filename.c_str(), "w");
          fprintf(f, "hello");
          fclose(f);
        }
      }
    }

    // Tell every thread to delete the same "test/a" folder. Each will
    // take some portion of the effort decided simply by who gets there first.
    // The key aspect is that this succeeds without error and the "test/a"
    // folder is successfully deleted.
    {
      std::vector<std::unique_ptr<std::thread>> threads(num_threads);
      for (size_t i = 0; i < num_threads; i++)
        threads[i].reset(new std::thread(remove_folder));
      for (size_t i = 0; i < num_threads; i++)
        threads[i]->join();
      EXPECT_TRUE(!filesystem::exists(base_subdir));
    }
  }
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
