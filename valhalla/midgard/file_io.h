#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

namespace valhalla {
namespace midgard {

template <class T>
inline constexpr bool is_byte_copyable_v =
    std::is_trivially_copy_constructible_v<T>&& std::is_trivially_destructible_v<T>;

/**
 * Thin owning wrapper over the platform's file API. Abstracts platform-specific API for offsetted
 * read and write operations that mutate no inner state and thus could be done concurrently.
 */
class file_handle {
public:
  // Opens a file that must already exist. Throws otherwise.
  static file_handle open(const std::string& path);
  // Creates the file, emptying it if it is already there.
  static file_handle create(const std::string& path);

  ~file_handle();

  file_handle(file_handle&& other) noexcept : handle_(other.handle_) {
    other.handle_ = kEmpty;
  }
  file_handle& operator=(file_handle&& other) noexcept {
    file_handle local(std::move(other));
    std::swap(handle_, local.handle_);
    return *this;
  }
  file_handle(const file_handle&) = delete;
  file_handle& operator=(const file_handle&) = delete;

  template <class T> void read_at(std::span<T> destination, uint64_t offset_bytes) const {
    static_assert(is_byte_copyable_v<T>, "file_handle moves raw bytes");
    static_assert(!std::is_const_v<T>, "read_at needs a writable destination");
    read_bytes_at(destination.data(), destination.size_bytes(), offset_bytes);
  }

  template <class T> void write_at(std::span<T> source, uint64_t offset_bytes) {
    static_assert(is_byte_copyable_v<std::remove_const_t<T>>, "file_handle moves raw bytes");
    write_bytes_at(source.data(), source.size_bytes(), offset_bytes);
  }

private:
  // Constructor is private. Use `open` or `create` functions instead
  explicit file_handle(uintptr_t handle) : handle_(handle) {
  }

  void read_bytes_at(void* destination, size_t bytes, uint64_t offset) const;
  void write_bytes_at(const void* source, size_t bytes, uint64_t offset);

  // Platform-specific, file descriptor on POSIX and HANDLE on Windows.
  static constexpr uintptr_t kEmpty = ~uintptr_t(0);
  uintptr_t handle_;
};

} // namespace midgard
} // namespace valhalla
