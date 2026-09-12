#include "midgard/file_io.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace {

// Requests are chunked so a single transfer never exceeds what the platform accepts in one call.
constexpr size_t kMaxTransfer = 64 * 1024 * 1024;

// the factories name the intent; the flag is an implementation detail of the platform call
uintptr_t open_native(const std::string& path, bool replace) {
#ifdef _WIN32
  // CreateFile denies sharing by default where the CRT does not, and the same file gets mapped
  // elsewhere while a handle is held, so the modes have to be spelled out
  const HANDLE h =
      ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                    replace ? CREATE_ALWAYS : OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (h == INVALID_HANDLE_VALUE) {
    throw std::runtime_error("file_handle: cannot open " + path + ": windows error " +
                             std::to_string(::GetLastError()));
  }
  return reinterpret_cast<uintptr_t>(h);
#else
  const int fd = ::open(path.c_str(), O_RDWR | (replace ? (O_CREAT | O_TRUNC) : 0), 0644);
  if (fd == -1) {
    throw std::runtime_error("file_handle: cannot open " + path + ": " + strerror(errno));
  }
  return static_cast<uintptr_t>(fd);
#endif
}

#ifdef _WIN32
[[noreturn]] void fail(const char* what) {
  throw std::runtime_error(std::string("file_handle::") + what + ": windows error " +
                           std::to_string(::GetLastError()));
}
#else
[[noreturn]] void fail(const char* what) {
  throw std::runtime_error(std::string("file_handle::") + what + ": " + strerror(errno));
}
#endif

} // namespace

namespace valhalla {
namespace midgard {

file_handle file_handle::open(const std::string& path) {
  return file_handle(open_native(path, false));
}

file_handle file_handle::create(const std::string& path) {
  return file_handle(open_native(path, true));
}

file_handle::~file_handle() {
  if (handle_ == kEmpty) {
    return; // moved from, the file belongs to someone else now
  }
  // nothing useful is left to do with a failure here, and a destructor must not throw
#ifdef _WIN32
  ::CloseHandle(reinterpret_cast<HANDLE>(handle_));
#else
  ::close(static_cast<int>(handle_));
#endif
}

void file_handle::read_bytes_at(void* destination, size_t bytes, uint64_t offset) const {
  auto* out = static_cast<char*>(destination);
#ifdef _WIN32
  const HANDLE h = reinterpret_cast<HANDLE>(handle_);
#else
  const int fd = static_cast<int>(handle_);
#endif
  while (bytes) {
    const size_t want = bytes < kMaxTransfer ? bytes : kMaxTransfer;
#ifdef _WIN32
    OVERLAPPED ov{};
    ov.Offset = static_cast<DWORD>(offset & 0xffffffffull);
    ov.OffsetHigh = static_cast<DWORD>((offset >> 32) & 0xffffffffull);
    DWORD got = 0;
    if (!::ReadFile(h, out, static_cast<DWORD>(want), &got, &ov) || got == 0) {
      fail("read_at");
    }
#else
    const ssize_t got = ::pread(fd, out, want, static_cast<off_t>(offset));
    if (got <= 0) {
      fail("read_at");
    }
#endif
    out += got;
    offset += static_cast<uint64_t>(got);
    bytes -= static_cast<size_t>(got);
  }
}

void file_handle::write_bytes_at(const void* source, size_t bytes, uint64_t offset) {
  const auto* in = static_cast<const char*>(source);
#ifdef _WIN32
  const HANDLE h = reinterpret_cast<HANDLE>(handle_);
#else
  const int fd = static_cast<int>(handle_);
#endif
  while (bytes) {
    const size_t want = bytes < kMaxTransfer ? bytes : kMaxTransfer;
#ifdef _WIN32
    OVERLAPPED ov{};
    ov.Offset = static_cast<DWORD>(offset & 0xffffffffull);
    ov.OffsetHigh = static_cast<DWORD>((offset >> 32) & 0xffffffffull);
    DWORD put = 0;
    if (!::WriteFile(h, in, static_cast<DWORD>(want), &put, &ov) || put == 0) {
      fail("write_at");
    }
#else
    const ssize_t put = ::pwrite(fd, in, want, static_cast<off_t>(offset));
    if (put <= 0) {
      fail("write_at");
    }
#endif
    in += put;
    offset += static_cast<uint64_t>(put);
    bytes -= static_cast<size_t>(put);
  }
}

} // namespace midgard
} // namespace valhalla
