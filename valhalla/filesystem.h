#pragma once
/**
 * NOTE: This is a slim version of std::filesystem (boost::filesytem) containing just
 * the parts that we need for recursive directory listing, file types and exists check
 * we've just stubbed out the interface such that its basically a subset of the official
 * version missing the stuff we dont make use of. it should work on posix and windows
 */

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <memory>
#include <string>
#include <sys/stat.h>
#ifdef _MSC_VER
#include <direct.h> // _mkdir
#include <fcntl.h>
#include <io.h> // _chsize
#else
#include <unistd.h>
#endif
#include <vector>

namespace filesystem {

class path {
public:
#if defined(_WIN32) || defined(__CYGWIN__)
  static constexpr char preferred_separator = L'\\';
#else
  static constexpr char preferred_separator = L'/';
#endif
  path(const char* path_name) : path(std::string(path_name)) {
  }
  path(const std::string& path_name) : path_name_(path_name), separators_() {
    // TODO: squash repeated separators
    // delineate the path
    for (size_t npos = path_name_.find_first_of(preferred_separator); npos != std::string::npos;
         npos = path_name_.find_first_of(preferred_separator, npos + 1))
      separators_.push_back(npos);
  }
  path filename() const {
    if (separators_.empty())
      return *this;
    return path(path_name_.substr(separators_.back() + 1));
  }
  const char* c_str() const {
    return path_name_.c_str();
  }
  const std::string& string() const {
    return path_name_;
  }
  path& replace_filename(const path& filename) {
    // empty or just a file name then copy
    if (separators_.empty())
      return *this = filename;

    // TODO: if its just the root_name we replace that

    // take everything from the last separator
    path_name_.erase(separators_.back());
    separators_.pop_back();
    return *this /= filename;
  }
  bool operator==(const path& rhs) const {
    return path_name_ == rhs.path_name_;
  }
  path& operator/=(const path& rhs) {
    if (!rhs.path_name_.empty()) {
      // update the path
      auto len = path_name_.size();
      if (!path_name_.empty() && path_name_.back() != preferred_separator &&
          rhs.path_name_.front() != preferred_separator) {
        separators_.push_back(path_name_.size());
        path_name_.push_back(preferred_separator);
        ++len;
      }
      path_name_ += rhs.path_name_;
      // update the parts bookkeeping
      auto sep_len = separators_.size();
      separators_.insert(separators_.end(), rhs.separators_.begin(), rhs.separators_.end());
      for (auto i = separators_.begin() + sep_len; i != separators_.end(); ++i)
        (*i) += len;
    }
    return *this;
  }

  friend bool create_directories(const path&);

private:
  std::string path_name_;
  std::vector<size_t> separators_;
};

class directory_iterator;
class recursive_directory_iterator;
class directory_entry {
public:
  directory_entry(const filesystem::path& path) : directory_entry(path, false) {
  }
  bool exists() const {
    return entry_ != nullptr;
  }
  bool is_directory() const {
    return exists() && entry_->d_type == DT_DIR;
  }
  bool is_regular_file() const {
    return exists() && entry_->d_type == DT_REG;
  }
  bool is_symlink() const {
    return exists() && entry_->d_type == DT_LNK;
  }
  const filesystem::path& path() const {
    return path_;
  }
  bool operator==(const directory_entry& rhs) const {
    return (!entry_ && !rhs.entry_) || memcmp(entry_.get(), rhs.entry_.get(), sizeof(dirent)) == 0;
  }

private:
  friend directory_iterator;
  friend recursive_directory_iterator;
  directory_entry(const filesystem::path& path, bool iterate)
      : dir_(nullptr), entry_(nullptr), path_(path) {
    // stat it first in case its not a directory
    struct stat s;
    if (stat(path_.c_str(), &s) == 0) {
      // if it is a directory and we are going to iterate over it
      if (S_ISDIR(s.st_mode) && iterate) {
        dir_.reset(opendir(path_.c_str()), [](DIR* d) { closedir(d); });
        return;
      }
      // make a dirent from stat info for starting out
      auto filename = path_.filename();
      entry_.reset(new dirent);
      // entry_->d_reclen =
      // entry_->d_off =
      entry_->d_ino = s.st_ino;
      strcpy(entry_->d_name, filename.c_str());
#ifdef _DIRENT_HAVE_D_NAMLEN
      entry_->d_namlen = filename.string().size();
#endif
      entry_->d_type = mode_to_type(s.st_mode);
    }
  }

  // On POXIX, d_type is char.
  // On Windows, d_type is int (values are larger than 8-bit integer).
  decltype(::dirent::d_type) mode_to_type(decltype(::stat::st_mode) mode) {
    if (S_ISREG(mode))
      return DT_REG;
    else if (S_ISDIR(mode))
      return DT_DIR;
    else if (S_ISFIFO(mode))
      return DT_FIFO;
    else if (S_ISSOCK(mode))
      return DT_SOCK;
    else if (S_ISCHR(mode))
      return DT_CHR;
    else if (S_ISBLK(mode))
      return DT_BLK;
    else if (S_ISLNK(mode))
      return DT_LNK;
    else
      return DT_UNKNOWN;
  }
  dirent* next() {
    // if we can scan
    if (dir_) {
      bool first_entry = entry_ == nullptr;
      // we have to skip . and ..
      do {
        entry_.reset(readdir(dir_.get()), [](dirent*) {});
      } while (entry_ && (strcmp(entry_->d_name, ".") == 0 || strcmp(entry_->d_name, "..") == 0));
      // update the path
      if (entry_) {
        if (first_entry)
          path_ /= entry_->d_name;
        else
          path_.replace_filename(entry_->d_name);
        // fix the type if its unknown
        struct stat s;
        if (entry_->d_type == DT_UNKNOWN && stat(path_.c_str(), &s) == 0)
          entry_->d_type = mode_to_type(s.st_mode);
      }
    }
    return entry_.get();
  }
  std::shared_ptr<DIR> dir_;
  std::shared_ptr<dirent> entry_;
  filesystem::path path_;
};

class directory_iterator {
public:
  directory_iterator(const filesystem::path& path) : entry_(new directory_entry(path, true)) {
    // if its not a dir or it is empty
    if (!entry_->dir_ || !entry_->next())
      entry_.reset();
  }
  directory_iterator() : entry_() {
  }
  const directory_entry& operator*() const {
    return *entry_;
  }
  const directory_entry* operator->() const {
    return entry_.get();
  }
  directory_iterator& operator++() {
    if (entry_ && !entry_->next())
      entry_.reset();
    return *this;
  }

private:
  std::shared_ptr<directory_entry> entry_;
  friend bool operator==(const directory_iterator& lhs, const directory_iterator& rhs);
};

inline bool operator==(const directory_iterator& lhs, const directory_iterator& rhs) {
  return (lhs.entry_ && rhs.entry_ && *lhs.entry_ == *rhs.entry_) || (lhs.entry_ == rhs.entry_);
}

inline bool operator!=(const directory_iterator& lhs, const directory_iterator& rhs) {
  return !(lhs == rhs);
}

// NOTE: follows links by default..
class recursive_directory_iterator {
public:
  recursive_directory_iterator(const filesystem::path& path) : stack_() {
    stack_.emplace_back(new directory_entry(path, true));
    // if wasn't an iterable directory or it was empty then we are at the end
    if (!stack_.back()->dir_ || !stack_.back()->next())
      stack_.clear();
  }
  recursive_directory_iterator() : stack_() {
  }
  const directory_entry& operator*() const {
    return *stack_.back();
  }
  const directory_entry* operator->() const {
    return stack_.back().get();
  }
  recursive_directory_iterator& operator++() {
    // if we have something to iterate
    if (!stack_.empty()) {
      // if its a directory we deepen our depth first search
      if (stack_.back()->is_directory())
        stack_.emplace_back(new directory_entry(stack_.back()->path_, true));
      // try to go horizontally and if not go up
      while (!stack_.empty() && !stack_.back()->next())
        stack_.pop_back();
    }
    return *this;
  }

private:
  std::vector<std::shared_ptr<directory_entry>> stack_;
  friend bool operator==(const recursive_directory_iterator& lhs,
                         const recursive_directory_iterator& rhs);
};

inline bool operator==(const recursive_directory_iterator& lhs,
                       const recursive_directory_iterator& rhs) {
  // somewhat weak check but for practical purposes should be fine given inode is unique
  return lhs.stack_.size() == rhs.stack_.size() &&
         (lhs.stack_.empty() || *lhs.stack_.back() == *rhs.stack_.back());
}

inline bool operator!=(const recursive_directory_iterator& lhs,
                       const recursive_directory_iterator& rhs) {
  return !(lhs == rhs);
}

inline bool exists(const path& p) {
  return directory_entry(p).exists();
}

inline bool is_directory(const path& p) {
  return directory_entry(p).is_directory();
}

inline bool is_regular_file(const path& p) {
  return directory_entry(p).is_regular_file();
}

inline bool create_directories(const path& p) {
  // name no work to do
  if (p.path_name_.empty())
    return true;

  // for each piece of the path
  struct stat s;
  for (size_t i = 0; i <= p.separators_.size(); ++i) {
    auto sep = i < p.separators_.size() ? p.separators_[i] + 1 : p.path_name_.size();
    // if this piece doesnt exist
    auto partial = p.path_name_.substr(0, sep);
    if (stat(partial.c_str(), &s) != 0) {
      // create this piece with filesystem::permissions::all
#ifdef _MSC_VER
      if (_mkdir(partial.c_str()) != 0) {
#else
      if (mkdir(partial.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
#endif
        return false;
        // throw std::runtime_error(std::string("Failed to create path: ") + strerror(errno));
      }
    } // if it did exist but wasnt a directory not good
    else if (!S_ISDIR(s.st_mode)) {
      return false;
      // throw std::runtime_error("Path exists and is not a directory");
    }
  }

  // make it!
  return true;
}

inline void resize_file(const path& p, std::uintmax_t new_size) {

#ifdef _MSC_VER
  auto truncate = [](char const* filepath, std::uintmax_t length) -> int {
    // _chsize expects value in range of signed long
    if (length > (std::numeric_limits<long>::max)())
      return false;

    int fd{0};
    if (::_sopen_s(&fd, filepath, _O_RDWR | _O_CREAT, _SH_DENYNO, _S_IREAD | _S_IWRITE) != 0)
      return false;

    int const rc = ::_chsize(fd, static_cast<long>(length));
    _close(fd);
    return rc == 0;
  };
#endif

  if (truncate(p.c_str(), new_size))
    throw std::runtime_error(std::string("Failed to resize path: ") + strerror(errno));
}

inline bool remove(const path& p) {
  return ::remove(p.c_str()) == 0;
}

inline bool remove_all(const path& p) {
  // for each entry in this directory
  for (directory_iterator i(p), end; i != end; ++i) {
    // if its a directory we recurse depth first
    if (i->is_directory()) {
      if (!remove_all(i->path()))
        return false;
    } // otherwise its a file or link try to delete it
    else if (!remove(i->path()))
      return false;
  }
  // delete the root
  return remove(p);
}

} // namespace filesystem