#pragma once

#include <vector>
#include <fstream>
#include <thread>

namespace valhalla {
namespace midgard {

// `sequence<T>`-compatible writer that offloads writing to a separate thread
template <class T> class sequence_writer {
  std::ofstream file;

  // Buffers that are swapped between the worker thread and the user thread.
  // It can be considered as a channel with capacity of 1.
  std::vector<T> write_buffer;
  std::vector<T> shadow_buffer;

  // Items counter for a `size()` method
  size_t count = 0;

  // Guards `shadow_buffer` and the `running` flag
  std::mutex queue_mutex;
  // Gets notified when the main thread has written to the shadow buffer
  std::condition_variable write_cv;
  // Gets notified when the shadow buffer is written to the file and is empty
  std::condition_variable written_cv;

  bool running = true;
  std::thread worker;

public:
  // Constructor - opens the file for writing and starts worker thread
  explicit sequence_writer(const std::string& filename, size_t buffer_size = 1024 * 1024 * 32 / sizeof(T))
      : file(filename, std::ios_base::binary | std::ios_base::trunc) {
    if (!file) {
      throw std::runtime_error("sequence_writer: " + filename + ": " + strerror(errno));
    }

    write_buffer.reserve(buffer_size);
    shadow_buffer.reserve(buffer_size);

    worker = std::thread(&sequence_writer::write_shadow_buffer, this);
  }

  // Non-copyable
  sequence_writer(const sequence_writer&) = delete;
  sequence_writer& operator=(const sequence_writer&) = delete;

  ~sequence_writer() {
    flush();
    {
      std::lock_guard lock(queue_mutex);
      running = false;
      write_cv.notify_one();
    }
    worker.join();
  }

  // Add an element to the sequence
  void push_back(const T& item) {
    write_buffer.push_back(item);
    count += 1;

    // If buffer is full, send it to the worker thread
    if (write_buffer.size() >= write_buffer.capacity()) {
      std::unique_lock lock(queue_mutex);
      written_cv.wait(lock, [this]() { return shadow_buffer.empty(); });
      std::swap(write_buffer, shadow_buffer);
      write_cv.notify_one();
    }
  }

  size_t size() const {
    return count;
  }

  // Flush any pending writes and wait until they're complete
  void flush() {
    std::unique_lock lock(queue_mutex);
    written_cv.wait(lock, [this]() { return shadow_buffer.empty(); });

    if (!write_buffer.empty()) {
      std::swap(write_buffer, shadow_buffer);
      write_cv.notify_one();

      // Wait until the worker thread has written the shadow buffer
      written_cv.wait(lock, [this]() { return shadow_buffer.empty(); });
    }
  }

private:
  void write_shadow_buffer() {
    std::unique_lock lock(queue_mutex);
    while (true) {
      write_cv.wait(lock, [this]() { return !shadow_buffer.empty() || !running; });
      if (shadow_buffer.empty() && !running) {
        break; // Exit if we're shutting down and buffer is empty
      }

      file.write(static_cast<const char*>(static_cast<const void*>(shadow_buffer.data())),
                 shadow_buffer.size() * sizeof(T));
      file.flush();
      shadow_buffer.clear();

      written_cv.notify_all();
    }
  }
};

} // namespace midgard
} // namespace valhalla
