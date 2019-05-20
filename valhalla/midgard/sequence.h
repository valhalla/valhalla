#ifndef VALHALLA_MJOLNIR_SEQUENCE_H_
#define VALHALLA_MJOLNIR_SEQUENCE_H_

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _MSC_VER
#include <io.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif // _MSC_VER
#include <fcntl.h>

// if we are on android
#ifdef __ANDROID__
// we didnt get a posix alias until 23
#if __ANDROID_API__ < 23
#define posix_madvise madvise
#endif
// we didnt get these posix aliases until M
#if __ANDROID_API__ < __ANDROID_API_M__
#define POSIX_MADV_NORMAL MADV_NORMAL
#define POSIX_MADV_RANDOM MADV_RANDOM
#define POSIX_MADV_SEQUENTIAL MADV_SEQUENTIAL
#define POSIX_MADV_WILLNEED MADV_WILLNEED
#define POSIX_MADV_DONTNEED MADV_DONTNEED
#endif
#endif

#ifdef _MSC_VER
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN 1
#endif
#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE 1
#endif
#ifndef _CRT_NONSTDC_NO_WARNINGS
#define _CRT_NONSTDC_NO_WARNINGS 1
#endif
#include <windows.h>

#define PROT_READ 0x01
#define PROT_WRITE 0x02
#define MAP_SHARED 0x01
#define MAP_PRIVATE 0x02
#define MAP_ANONYMOUS 0x20
#define MAP_FAILED (reinterpret_cast<void*>(static_cast<LONG_PTR>(-1)))
#define POSIX_MADV_NORMAL 0     // ignored
#define POSIX_MADV_SEQUENTIAL 2 // ignored

inline void* mmap(void* addr, size_t length, int prot, int flags, int fd, long long offset) {
  (void)addr; // ignored

  DWORD access = 0;
  DWORD protect = 0;
  if ((prot & PROT_WRITE)) {
    if ((flags & MAP_PRIVATE)) {
      access = FILE_MAP_COPY;
      protect = PAGE_WRITECOPY;
    } else {
      access = FILE_MAP_WRITE;
      protect = PAGE_READWRITE;
    }
  } else if ((prot & PROT_READ)) {
    access = FILE_MAP_READ;
    protect = PAGE_READONLY;
  }

  HANDLE file_handle = INVALID_HANDLE_VALUE;
  if ((fd != -1) && (flags & MAP_ANONYMOUS) == 0) {
    file_handle = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
  }

  void* map = nullptr;
  HANDLE mapped_handle = ::CreateFileMapping(file_handle, nullptr, protect, 0, 0, nullptr);
  if (mapped_handle) {
    map = ::MapViewOfFile(mapped_handle, access, (DWORD)((offset >> 32) & 0xffffffffUL),
                          (DWORD)(offset & 0xffffffffUL), 0);

    BOOL rc = FALSE;
    rc = ::CloseHandle(mapped_handle); // release internal reference to mapped view
  }

  if (!map) {
    return static_cast<void*>(static_cast<char*>(MAP_FAILED));
  }

  return static_cast<void*>(static_cast<char*>(map));
}

inline int munmap(void* addr, size_t length) {
  (void)length; // ignored
  if (::UnmapViewOfFile(addr) == 0) {
    return -1;
  }
  return 0;
}

#endif // _MSC_VER

namespace valhalla {
namespace midgard {

template <class T> class mem_map {
public:
  // non-copyable
  mem_map(mem_map&&) = default;
  mem_map& operator=(mem_map&&) = default;
  mem_map(const mem_map&) = delete;
  mem_map& operator=(const mem_map&) = delete;

  // default constructable to nothing loaded
  mem_map() : ptr(nullptr), count(0), file_name("") {
  }

  // construct with file
  mem_map(const std::string& file_name, size_t size, int advice = POSIX_MADV_NORMAL)
      : ptr(nullptr), count(0), file_name("") {
    map(file_name, size, advice);
  }

  // unmap when done
  ~mem_map() {
    unmap();
  }

  // create a new file to map with a given size
  void create(const std::string& new_file_name, size_t new_count, int advice = POSIX_MADV_NORMAL) {
    auto target_size = new_count * sizeof(T);
    struct stat s;
    if (stat(new_file_name.c_str(), &s) || s.st_size != target_size) {
      // open, create and truncate the file
      std::ofstream f(new_file_name, std::ios::binary | std::ios::out | std::ios::trunc);
      // seek to the new size and put a null char
      f.seekp(new_count * sizeof(T) - 1);
      f.write("\0", 1);
    }
    // map it
    map(new_file_name, new_count, advice);
  }

  // reset to another file or another size
  void map(const std::string& new_file_name, size_t new_count, int advice = POSIX_MADV_NORMAL) {
    // just in case there was already something
    unmap();

    // has to be something to map
    if (new_count > 0) {
      auto fd =
#if defined(_MSC_VER)
          _open(new_file_name.c_str(), O_RDWR, 0);
#else
          open(new_file_name.c_str(), O_RDWR, 0);
#endif
      if (fd == -1) {
        throw std::runtime_error(new_file_name + "(open): " + strerror(errno));
      }
      ptr = mmap(nullptr, new_count * sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (ptr == MAP_FAILED) {
        throw std::runtime_error(new_file_name + "(mmap): " + strerror(errno));
      }

#if defined(_MSC_VER)
      auto cl = _close(fd);
#else
      auto cl = close(fd);
      posix_madvise(ptr, new_count * sizeof(T), advice);
#endif
      if (cl == -1) {
        throw std::runtime_error(new_file_name + "(close): " + strerror(errno));
      }
      count = new_count;
      file_name = new_file_name;
    }
  }

  // drop the map
  void unmap() {
    // has to be something to unmap
    if (ptr) {
      // unmap
      auto un = munmap(ptr, count * sizeof(T));
      if (un == -1) {
        throw std::runtime_error(file_name + "(munmap): " + strerror(errno));
      }

      // clear
      ptr = nullptr;
      count = 0;
      file_name = "";
    }
  }

  T* get() const {
    return static_cast<T*>(ptr);
  }

  operator T*() {
    return static_cast<T*>(ptr);
  }

  operator const T*() const {
    return static_cast<const T*>(ptr);
  }

  operator bool() const {
    return ptr != nullptr;
  }

  size_t size() const {
    return count;
  }

  const std::string& name() const {
    return file_name;
  }

protected:
  void* ptr;
  size_t count;
  std::string file_name;
};

template <class T> class sequence {
public:
  // static_assert(std::is_pod<T>::value, "sequence requires POD types for now");
  static const size_t npos = -1;

  sequence() = delete;

  sequence(const sequence&) = delete;

  sequence(const std::string& file_name,
           bool create = false,
           size_t write_buffer_size = 1024 * 1024 * 32 / sizeof(T))
      : file(new std::fstream(file_name,
                              std::ios_base::binary | std::ios_base::in | std::ios_base::out |
                                  (create ? std::ios_base::trunc : std::ios_base::ate))),
        file_name(file_name) {

    // crack open the file
    if (!*file) {
      throw std::runtime_error("sequence: " + file_name + ": " + strerror(errno));
    }
    auto end = file->tellg();
    auto element_count = static_cast<std::streamoff>(std::ceil(end / sizeof(T)));
    if (end != static_cast<decltype(end)>(element_count * sizeof(T))) {
      throw std::runtime_error("sequence: " + file_name + " has an incorrect size for type");
    }
    write_buffer.reserve(write_buffer_size ? write_buffer_size : 1);

    // memory map the file for reading
    memmap.map(file_name, element_count);
  }

  ~sequence() {
    // finish writing whatever it was to file
    flush();
  }

  // add an element to the sequence
  void push_back(const T& obj) {
    write_buffer.push_back(obj);
    // push it to the file
    if (write_buffer.size() == write_buffer.capacity()) {
      flush();
    }
  }

  // finds the first matching object by scanning O(n)
  // assumes nothing about the order of the file
  // the predicate should be something like an equality check
  size_t find_first_of(const T& target,
                       const std::function<bool(const T&, const T&)>& predicate,
                       size_t start_index = 0) {
    flush();
    // keep looking while we have stuff to look at
    while (start_index < memmap.size()) {
      T candidate = memmap ? *(static_cast<const T*>(memmap) + start_index) : (*this)[start_index];
      if (predicate(target, candidate)) {
        return start_index;
      }
      ++start_index;
    }
    return npos;
  }

  // sort the file based on the predicate
  void sort(const std::function<bool(const T&, const T&)>& predicate,
            size_t buffer_size = 1024 * 1024 * 512 / sizeof(T)) {
    using queue_t =
        std::map<T, std::list<std::fstream>::iterator, std::function<bool(const T&, const T&)>>;

    flush();
    // if no elements we are done
    if (memmap.size() == 0) {
      return;
    }
    std::sort(static_cast<T*>(memmap), static_cast<T*>(memmap) + memmap.size(), predicate);
    return;
  }

  // perform an volatile operation on all the items of this sequence
  void transform(const std::function<void(T&)>& predicate) {
    flush();
    for (size_t i = 0; i < memmap.size(); ++i) {
      // grab the element
      auto element = at(i);
      // grab the underlying item
      auto item = *element;
      // transform it
      predicate(item);
      // put it back
      element = item;
    }
  }

  // perform a non-volatile operation on all the items of this sequence
  void enumerate(const std::function<void(const T&)>& predicate) {
    flush();
    // grab each element and do something with it
    for (size_t i = 0; i < memmap.size(); ++i) {
      predicate(*(at(i)));
    }
  }

  // force writing whatever we have in the write_buffer to file
  void flush() {
    if (write_buffer.size()) {
      file->seekg(0, file->end);
      file->write(static_cast<const char*>(static_cast<const void*>(write_buffer.data())),
                  write_buffer.size() * sizeof(T));
      file->flush();
      memmap.map(file_name, memmap.size() + write_buffer.size());
      write_buffer.clear();
    }
  }

  // how many things have been written so far
  size_t size() const {
    return memmap.size() + write_buffer.size();
  }

  // a read/writeable object within the sequence, accessed through memory mapped file
  struct iterator {
    friend class sequence;

  public:
    // static_assert(std::is_pod<T>::value, "sequence_element requires POD types for now");
    iterator() = delete;
    iterator& operator=(const iterator& other) {
      parent = other.parent;
      index = other.index;
      return *this;
    }
    iterator& operator=(const T& other) {
      // If index is beyond the end of the mmap buffer, then
      // access items that may be in the write_buffer.
      if (index >= parent->memmap.size()) {
        parent->write_buffer[index - parent->memmap.size()] = other;
      } else {
        *(static_cast<T*>(parent->memmap) + index) = other;
      }
      return *this;
    }
    operator T() {
      // If index is beyond the end of the mmap buffer, then
      // access items that may be in the write_buffer.
      if (index >= parent->memmap.size()) {
        return parent->write_buffer.at(index - parent->memmap.size());
      } else {
        return *(static_cast<T*>(parent->memmap) + index);
      }
    }
    T operator*() {
      return operator T();
    }
    iterator& operator++() {
      ++index;
      return *this;
    }
    iterator operator++(int) {
      auto other = *this;
      ++index;
      return other;
    }
    iterator& operator+=(size_t offset) {
      index += offset;
      return *this;
    }
    iterator operator+(size_t offset) {
      auto other = *this;
      other.index += offset;
      return other;
    }
    iterator& operator--() {
      --index;
      return *this;
    }
    iterator operator--(int) {
      auto other = *this;
      --index;
      return other;
    }
    iterator& operator-=(size_t offset) {
      index -= offset;
      return *this;
    }
    iterator operator-(size_t offset) {
      auto other = *this;
      other.index -= offset;
      return other;
    }
    bool operator==(const iterator& other) const {
      return parent == other.parent && index == other.index;
    }
    bool operator!=(const iterator& other) const {
      return index != other.index || parent != other.parent;
    }
    size_t operator-(const iterator& other) const {
      return index - other.index;
    }
    size_t position() const {
      return index;
    }

  protected:
    iterator(sequence* base, size_t offset) : parent(base), index(offset) {
    }
    sequence* parent;
    size_t index;
  };

  // search for an object using binary search O(logn)
  // assumes the file was written in sorted order
  // the predicate should be something like a less than or greater than check
  iterator find(const T& target, const std::function<bool(const T&, const T&)>& predicate) {
    flush();
    // if no elements we are done
    if (memmap.size() == 0) {
      return end();
    }
    // if we did find it return the iterator to it
    auto* found = std::lower_bound(static_cast<const T*>(memmap),
                                   static_cast<const T*>(memmap) + memmap.size(), target, predicate);
    if (!(predicate(target, *found) || predicate(*found, target))) {
      return at(found - static_cast<const T*>(memmap));
    }
    // we didnt find it
    return end();
  }

  iterator at(size_t index) {
    // dump to file and make an element
    return iterator(this, index);
  }

  // write/read at certain index
  iterator operator[](size_t index) {
    return at(index);
  }

  // write/read at the beginning
  T front() {
    return *at(0);
  }

  // write/read at the end
  T back() {
    flush();
    return *iterator(this, memmap.size() - 1);
  }

  // first iterator
  iterator begin() {
    return at(0);
  }

  // invalid end iterator
  iterator end() {
    return iterator(this, memmap.size() + write_buffer.size());
  }

protected:
  std::shared_ptr<std::fstream> file;
  std::string file_name;
  std::vector<T> write_buffer;
  mem_map<T> memmap;
};

struct tar {
  struct header_t {
    char name[100];
    char mode[8];
    char uid[8];
    char gid[8];
    char size[12];
    char mtime[12];
    char chksum[8];
    char typeflag;
    char linkname[100];
    char magic[6];
    char version[2];
    char uname[32];
    char gname[32];
    char devmajor[8];
    char devminor[8];
    char prefix[155];
    char padding[12];

    static uint64_t octal_to_int(const char* data, size_t size = 12) {
      const unsigned char* ptr = (const unsigned char*)data + size;
      uint64_t sum = 0;
      uint64_t multiplier = 1;
      // Skip everything after the last NUL/space character
      // In some TAR archives the size field has non-trailing NULs/spaces, so this is necessary
      const unsigned char* check = ptr; // This is used to check where the last NUL/space char is
      for (; check >= (unsigned char*)data; check--) {
        if ((*check) == 0 || (*check) == ' ') {
          ptr = check - 1;
        }
      }
      for (; ptr >= (unsigned char*)data; ptr--) {
        sum += ((*ptr) - 48) * multiplier;
        multiplier *= 8;
      }
      return sum;
    }
    bool is_ustar() const {
      return (memcmp("ustar", magic, 5) == 0);
    }
    size_t get_file_size() const {
      return octal_to_int(size);
    }
    bool blank() const {
      constexpr header_t BLANK{};
      return !memcmp(this, &BLANK, sizeof(header_t));
    }
    bool verify() const {
      // make a copy and blank the checksum
      header_t temp = *this;
      memset(temp.chksum, ' ', 8);
      int64_t usum = 0, sum = 0;
      // compute the checksum
      for (int i = 0; i < sizeof(header_t); i++) {
        usum += ((unsigned char*)&temp)[i];
        sum += ((char*)&temp)[i];
      }
      // check if its right
      uint64_t rsum = octal_to_int(chksum);
      return rsum == usum || rsum == sum;
    }
  };

  tar(const std::string& tar_file, bool regular_files_only = true)
      : tar_file(tar_file), corrupt_blocks(0) {
    // get the file size
    struct stat s;
    if (stat(tar_file.c_str(), &s))
      throw std::runtime_error("(stat): " + tar_file + " " + strerror(errno));
    if (s.st_size == 0 || (s.st_size % sizeof(header_t)) != 0) {
      throw std::runtime_error(tar_file + "(stat): invalid archive size " +
                               std::to_string(s.st_size) + " with header size " +
                               std::to_string(sizeof(header_t)));
      return;
    }

    // map the file
    mm.map(tar_file, s.st_size);

    // rip through the tar to see whats in it noting that most tars end with 2 empty blocks
    // but we can concatenate tars and get empty blocks in between so we'll just be pretty
    // lax about it and we'll count the ones we cant make sense of
    const char* position = mm.get();
    while (position < mm.get() + mm.size()) {
      // get the header for this file
      const header_t* h = static_cast<const header_t*>(static_cast<const void*>(position));
      position += sizeof(header_t);
      // if it doesnt checkout ignore it and move on one block at a time
      if (!h->verify()) {
        corrupt_blocks += !h->blank();
        continue;
      }
      auto size = h->get_file_size();
      // do we record entry file or not
      if (!regular_files_only || (h->typeflag == '0' || h->typeflag == '\0')) {
        contents.emplace(std::piecewise_construct, std::forward_as_tuple(std::string{h->name}),
                         std::forward_as_tuple(position, size));
      }
      // every entry's data is rounded to the nearst header_t sized "block"
      auto blocks = static_cast<size_t>(std::ceil(static_cast<double>(size) / sizeof(header_t)));
      position += blocks * sizeof(header_t);
    }
  }

  std::string tar_file;
  mem_map<char> mm;
  using entry_name_t = std::string;
  using entry_location_t = std::pair<const char*, size_t>;
  std::unordered_map<entry_name_t, entry_location_t> contents;
  size_t corrupt_blocks;
};

} // namespace midgard
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_SEQUENCE_H_
