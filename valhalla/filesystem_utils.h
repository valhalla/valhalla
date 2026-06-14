#pragma once

#include <sys/stat.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#ifdef _WIN32
#define FS_MTIME(st_stat) st_stat.st_mtime
#elif __APPLE__
#define FS_MTIME(st_stat) st_stat.st_mtime
#else
#define FS_MTIME(st_stat) st_stat.st_mtim.tv_sec
#endif

namespace valhalla::filesystem_utils {

inline std::time_t last_write_time_t(const std::filesystem::path& p) {
  // note, in C++20 there's a proper chrono::clock_cast so we can use filesystem::last_write_time
  struct stat s;
  if (stat(p.string().c_str(), &s) != 0)
    throw std::runtime_error("could not stat " + p.string());
  return FS_MTIME(s);
}

// satisfied by contiguous containers that expose a raw byte pointer via .data()
template <typename T> concept has_data = requires(const T& t) {
  t.data();
};

/**
 * Renames `from` to `to`, replacing `to` if it exists, mirroring POSIX rename:
 * readers still holding the old `to` open keep reading it while new opens see the new
 * file. On Windows this uses POSIX-semantics rename, falling back to a non-atomic
 * replace on filesystems that lack it (FAT/exFAT, some SMB shares). Cannot replace a
 * memory-mapped target.
 *
 * @param from the existing file to rename
 * @param to   the destination path, replaced if it already exists
 * @param ec   set to the failure reason, cleared on success
 */
void rename_replace(const std::filesystem::path& from,
                    const std::filesystem::path& to,
                    std::error_code& ec);

/**
 * Saves data to the supplied path. Will replace the contents in case fpath already exists.
 * Will create a new directory if directory did not exist before hand.
 *
 * @param fpath the absolute or relative path to the file you want to save
 * @param data  the data you want to save, wrapped in a container
 *
 * @returns A boolean indicating whether saving the file was successful.
 * */
template <has_data Container>
inline bool save(const std::filesystem::path& fpath, const Container& data = {}) {
  if (fpath.empty())
    return false;

  auto dir = fpath.parent_path();

  // if the path is not a directory or it doesn't exist and we can't create it for some reason
  if ((!std::filesystem::exists(dir) && !std::filesystem::create_directories(dir)) ||
      !std::filesystem::is_directory(dir))
    return false;

  auto generate_tmp_suffix = []() -> std::string {
    std::stringstream ss;
    ss << ".tmp_" << std::this_thread::get_id() << "_"
       << std::chrono::high_resolution_clock::now().time_since_epoch().count();
    return ss.str();
  };

  std::filesystem::path tmp_location;
  // Technically this is a race condition but its super unlikely (famous last words)
  // TODO(nils): what am I missing here? why the while loop?
  //   how can tmp_location be empty after 1st iteration? how can a file be created in the loop?
  while (tmp_location.empty() || std::filesystem::exists(tmp_location)) {
    tmp_location = fpath;
    tmp_location += generate_tmp_suffix();
  }

  std::ofstream file(tmp_location, std::ios::out | std::ios::binary | std::ios::ate);
  file.write(data.data(), data.size());
  file.close();

  if (file.fail()) {
    std::filesystem::remove(tmp_location);
    return false;
  }

  std::error_code ec;
  rename_replace(tmp_location, fpath, ec);
  if (ec) {
    std::filesystem::remove(tmp_location);
    return false;
  }

  return true;
}
} // namespace valhalla::filesystem_utils
