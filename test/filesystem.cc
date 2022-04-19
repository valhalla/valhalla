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
  ASSERT_TRUE(filesystem::exists(".")) << "the current directory doesnt exist?";
  ASSERT_TRUE(filesystem::exists(".touched_file")) << "couldn't create a file?";
}

TEST(Filesystem, directory) {
  // todo: any specific reason it was repeated 2 times?
  ASSERT_TRUE(filesystem::is_directory(".")) << "the current directory isnt a directory?";
  ASSERT_TRUE(filesystem::is_directory(".")) << "the current directory isnt a directory?";
}

TEST(Filesystem, regular_file) {
  ASSERT_TRUE(filesystem::is_regular_file(".touched_file"));
}

void try_mkdir(const std::string& p) {
  ASSERT_EQ(mkdir(p.c_str(), perm755), 0) << "couldnt create directory " << p;
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
  ASSERT_TRUE(dirs.empty());
  ASSERT_TRUE(files.empty());

  // cleanup the stuff we made, 2 tests in one ;o)
  filesystem::remove_all("foo");
  ASSERT_TRUE(!filesystem::exists("foo"));
}

TEST(Filesystem, remove_any) {
  // delete non existent thing
  ASSERT_FALSE(filesystem::remove(".foobar")) << "Deleting nonexistent item should return false";

  // make and delete a file
  { std::fstream fs(".foobar", std::ios::out); }
  ASSERT_TRUE(filesystem::remove(".foobar"));
  ASSERT_FALSE(filesystem::exists(".foobar")) << ".foobar file should have been deleted";

  // make and delete a file
  try_mkdir(".foobar");
  ASSERT_TRUE(filesystem::remove(".foobar"));
  ASSERT_FALSE(filesystem::exists(".foobar")) << ".foobar dir should have been deleted";
}

TEST(Filesystem, parent_path) {
  std::vector<filesystem::path> in{{"/"},   {"/foo/bar"}, {"/foo/../"}, {"/foo/bar/../f"},
                                   {"./f"}, {"foo/bar/f"}};
  std::vector<filesystem::path> out{
      {""}, {"/foo"}, {"/foo/.."}, {"/foo/bar/.."}, {"."}, {"foo/bar"},
  };
  for (const auto& i : in) {
    auto a = i.parent_path();
    ASSERT_EQ(a.string(), out[&i - &in.front()].string()) << "wrong parent path";
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
    ASSERT_EQ(a.string(), out[&i - &in.front()].string()) << "wrong extension";
  }
}

TEST(Filesystem, file_size) {
}

TEST(Filesystem, concurrent_folder_create_delete) {

  const std::string nested_subdir = "test/a/b/c/d/e/f/g/h/i/j";
  const std::string base_subdir = "test/a";

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
    // a simple (cheesy?) way for each thread to announce they are ready
    ready_flag = true;

    // All threads will queue up at start_race_event.wait(), only
    // being released when the "race manager" sets start_race to true
    // and calls start_race_event.notify_all().
    std::unique_lock<std::mutex> start_race_lock(start_race_mutex);
    start_race_event.wait(start_race_lock, [&] { return start_race; });
    start_race_mutex.unlock();

    bool success = filesystem::create_directories(nested_subdir);
    ASSERT_TRUE(success);
    ASSERT_TRUE(filesystem::exists(nested_subdir));
  };

  auto remove_folder = [&](bool& ready_flag) {
    // a simple (cheesy?) way for each thread to announce they are ready
    ready_flag = true;

    // All threads will queue up at start_race_event.wait(), only
    // being released when the "race manager" sets start_race to true
    // and calls start_race_event.notify_all().
    std::unique_lock<std::mutex> start_lock(start_race_mutex);
    start_race_event.wait(start_lock, [&] { return start_race; });
    start_race_mutex.unlock();

    std::uintmax_t delete_count = filesystem::remove_all(base_subdir);
    // I've found that two+ threads will claim that they've deleted the same
    // file object. This results in double+ counting and prevents me from
    // asserting anything related to the delete_count. All we can be sure
    // is the following:
    ASSERT_TRUE(!filesystem::exists(base_subdir));
  };

  const int num_iters = 50;
  for (int j = 0; j < num_iters; j++) {
    // concurrent folder creation
    {
      // Start some threads. Wait for each to declare themselves ready.
      start_race = false;
      std::vector<std::unique_ptr<std::thread>> threads(num_threads);
      for (size_t i = 0; i < num_threads; i++) {
        bool ready_flag = false;
        threads[i].reset(new std::thread(create_folder, std::ref(ready_flag)));
        while (!ready_flag) {
          std::this_thread::yield();
        }
      }

      // By this point we know all threads have been created and are ready.
      // Announce the start of the race by first setting the "race_started"
      // cv boolean and then calling "notify_all()".
      start_race_mutex.lock();
      start_race = true;
      start_race_mutex.unlock();
      start_race_event.notify_all();

      for (size_t i = 0; i < num_threads; i++) {
        threads[i]->join();
      }

      ASSERT_TRUE(filesystem::exists(nested_subdir));
    }

    // populate each subfolder with num_files_per_folder text files.
    {
      const int num_files_per_folder = 10;
      std::string folder_name = "test";
      for (char subdir = 'a'; subdir < 'k'; subdir++) {
        folder_name = folder_name + "/" + subdir;
        for (size_t i = 0; i < num_files_per_folder; i++) {
          std::string filename = folder_name + "/" + std::to_string(i) + ".txt";
          std::ofstream f(filename.c_str(), std::ios::out);
          f << "hello";
          f.close();
        }
      }
    }

    // Tell every thread to delete the same "test/a" folder. Each will
    // take some portion of the effort decided simply by who gets there first.
    // The key aspect is that this succeeds without error and the "test/a"
    // folder is successfully deleted.
    {
      // Start some threads. Wait for each to declare themselves ready.
      start_race = false;
      std::vector<std::unique_ptr<std::thread>> threads(num_threads);
      for (size_t i = 0; i < num_threads; i++) {
        bool ready_flag = false;
        threads[i].reset(new std::thread(remove_folder, std::ref(ready_flag)));
        while (!ready_flag) {
          std::this_thread::yield();
        }
      }

      // By this point we know all threads have been created and are ready.
      // Announce the start of the race by first setting the "race_started"
      // cv boolean and then calling "notify_all()".
      start_race_mutex.lock();
      start_race = true;
      start_race_mutex.unlock();
      start_race_event.notify_all();

      for (size_t i = 0; i < num_threads; i++) {
        threads[i]->join();
      }

      ASSERT_TRUE(!filesystem::exists(base_subdir));
    }
  }
}

TEST(Filesystem, has_data_member_type) {
  auto arr = filesystem::has_data<std::array<int, 3>>::value;
  EXPECT_TRUE(arr);

  auto vc = filesystem::has_data<std::vector<int>>::value;
  EXPECT_TRUE(vc);

  auto str = filesystem::has_data<std::string>::value;
  EXPECT_TRUE(str);

  auto umap = filesystem::has_data<std::unordered_map<int, int>>::value;
  EXPECT_FALSE(umap);

  auto integer = filesystem::has_data<int>::value;
  EXPECT_FALSE(integer);

  auto lst = filesystem::has_data<std::list<int>>::value;
  EXPECT_FALSE(lst);
}

TEST(Filesystem, save_file_valid_input) {
  std::vector<std::string> tests{"/tmp/save_file_input/utrecht_tiles/0/003/196.gp",
                                 "/tmp/save_file_input/utrecht_tiles/1/051/305.gph",
                                 "/tmp/save_file_input/utrecht_tiles/2/000/818/660.gph"};

  std::size_t cnt{1};
  for (const auto& test : tests) {
    EXPECT_TRUE(filesystem::save<std::string>(test));
    EXPECT_EQ(filesystem::get_files("/tmp/save_file_input/utrecht_tiles").size(), cnt++);
  }

  filesystem::remove_all("/tmp/save_file_input/");
}

TEST(Filesystem, save_file_invalid_input) {
  std::vector<std::string> tests{"", "/etc/", "/tmp/", "/var/"};

  for (const auto& test : tests)
    EXPECT_FALSE(filesystem::save<std::string>(test)) << "FAILED " << test;
}

TEST(Filesystem, get_files_valid_input) {
  std::vector<std::string> tests{"/tmp/save_file_input/utrecht_tiles/0/003/196.gph",
                                 "/tmp/save_file_input/utrecht_tiles/1/051/305.gph",
                                 "/tmp/save_file_input/utrecht_tiles/2/000/818/660.gph"};

  std::size_t cnt{1};
  for (const auto& test : tests) {
    EXPECT_TRUE(filesystem::save<std::string>(test));
    EXPECT_EQ(filesystem::get_files("/tmp/save_file_input/utrecht_tiles").size(), cnt++);
  }

  if (filesystem::create_directories("/tmp/save_file_invalid"))
    EXPECT_TRUE(filesystem::get_files("/tmp/save_file_invalid").empty());

  filesystem::remove_all("/tmp/save_file_input/");
  filesystem::remove_all("/tmp/save_file_invalid/");
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
