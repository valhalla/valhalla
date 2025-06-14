#include <sys/stat.h>

#include <chrono>
#include <filesystem>

#ifdef _WIN32
#define FS_MTIME(st_stat) st_stat.st_mtime
#elif __APPLE__
#define FS_MTIME(st_stat) st_stat.st_mtime
#else
#define FS_MTIME(st_stat) st_stat.st_mtim.tv_sec
#endif

namespace std::filesystem {
inline std::time_t last_write_time_t(const std::filesystem::path& p) {
  // note, in C++20 there's a proper chrono::clock_cast so we can use filesystem::last_write_time
  struct stat s;
  if (stat(p.c_str(), &s) != 0)
    throw std::runtime_error("could not stat " + p.string());
  return FS_MTIME(s);
}
} // namespace std::filesystem
