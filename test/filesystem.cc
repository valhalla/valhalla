#include "filesystem.h"
#include "test.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

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

void exists() {
  // drop a file
  { std::fstream fs(".touched_file", std::ios::out); }
  // check this dir is there
  if (!filesystem::exists("."))
    throw std::logic_error("the current directory doesnt exist?");
  if (!filesystem::exists(".touched_file"))
    throw std::logic_error("couldn't create a file?");
}
void directory() {
  if (!filesystem::is_directory("."))
    throw std::logic_error("the current directory isnt a directory?");
  if (!filesystem::is_directory("."))
    throw std::logic_error("the current directory isnt a directory?");
}
void regular_file() {
  if (!filesystem::is_regular_file(".touched_file"))
    throw std::logic_error("the current directory isnt a directory?");
}
void try_mkdir(const std::string& p) {
  if (mkdir(p.c_str(), perm755))
    throw std::runtime_error("couldnt create directory");
}
void recursive_directory_listing() {
  // make a directory tree with these entries
  std::string s(1, filesystem::path::preferred_separator);
  std::vector<std::string> dirs{"foo", "foo" + s + "qux", "foo" + s + "quux"};
  std::vector<std::string> files{"foo" + s + "bar", "foo" + s + "bar",
                                 "foo" + s + "quux" + s + "corge"};

  // create them
  for (const auto& d : dirs)
    try_mkdir(d.c_str());
  for (const auto& f : files)
    std::fstream fs(f, std::ios::out);
  // unlike the posix find command the directory iterator doesnt give you access to
  // root or starting directory just the stuff under it so lets pop that off
  dirs.erase(dirs.begin());
  // go get whats there
  for (filesystem::recursive_directory_iterator i("foo"), end; i != end; ++i) {
    if (i->is_directory()) {
      auto pos = std::remove(dirs.begin(), dirs.end(), i->path().string());
      if (pos == dirs.end())
        throw std::logic_error("unexpected directory");
      dirs.erase(pos, dirs.end());
    } else if (i->is_regular_file()) {
      auto pos = std::remove(files.begin(), files.end(), i->path().string());
      if (pos == files.end())
        throw std::logic_error("unexpected file");
      files.erase(pos, files.end());
    } else {
      throw std::logic_error("unexpected entry type");
    }
  }
  // if we didnt get everything we have a problem
  if (!dirs.empty() || !files.empty())
    throw std::logic_error("we could find all files or dirs");

  // cleanup the stuff we made, 2 tests in one ;o)
  if (!filesystem::remove_all("foo"))
    throw std::logic_error("why cant we delete the stuff we just made");
}

void remove_any() {
  // delete non existant thing
  if (filesystem::remove(".foobar"))
    throw std::logic_error(".foobar should not exist");

  // make and delete a file
  { std::fstream fs(".foobar", std::ios::out); }
  if (!filesystem::remove(".foobar") || filesystem::exists(".foobar"))
    throw std::logic_error(".foobar file should have been deleted");

  // make and delete a file
  try_mkdir(".foobar");
  if (!filesystem::remove(".foobar") || filesystem::exists(".foobar"))
    throw std::logic_error(".foobar dir should have been deleted");
}

} // namespace

int main() {
  test::suite suite("filesystem");

  suite.test(TEST_CASE(exists));

  suite.test(TEST_CASE(directory));

  suite.test(TEST_CASE(regular_file));

  suite.test(TEST_CASE(recursive_directory_listing));

  suite.test(TEST_CASE(remove_any));

  return suite.tear_down();
}
