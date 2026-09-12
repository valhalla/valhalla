#include "midgard/file_io.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using namespace valhalla::midgard;

namespace {

// mirrors the shape sort_sequence moves around: trivially copyable, not a multiple of a word
struct record {
  uint64_t key;
  uint32_t a, b;
  uint64_t pad[3];
  bool operator==(const record&) const = default;
};
static_assert(sizeof(record) == 40, "layout");

static_assert(std::is_move_constructible_v<file_handle>, "handles are movable");
static_assert(std::is_move_assignable_v<file_handle>, "handles are movable");
static_assert(!std::is_copy_constructible_v<file_handle>, "handles own their file");
static_assert(!std::is_copy_assignable_v<file_handle>, "handles own their file");
static_assert(!std::is_default_constructible_v<file_handle>, "a handle always has a file");

struct scoped_file {
  explicit scoped_file(
      const std::string& stem = ::testing::UnitTest::GetInstance()->current_test_info()->name())
      : path((std::filesystem::temp_directory_path() / ("valhalla_file_io_" + stem)).string()) {
    std::filesystem::remove(path);
  }
  ~scoped_file() {
    std::filesystem::remove(path);
  }
  std::string path;
};

std::vector<record> make_records(size_t count, uint64_t seed = 0) {
  std::vector<record> v(count);
  for (size_t i = 0; i < count; ++i) {
    v[i] = record{seed + i, static_cast<uint32_t>(i), static_cast<uint32_t>(i * 2), {i, i, i}};
  }
  return v;
}

std::vector<record> slice(const std::vector<record>& v, size_t at, size_t count) {
  return {v.begin() + at, v.begin() + at + count};
}

std::vector<record> read_records(const file_handle& h, size_t count, size_t at = 0) {
  std::vector<record> v(count);
  h.read_at(std::span(v), at * sizeof(record));
  return v;
}

size_t records_in(const std::string& path) {
  return std::filesystem::file_size(path) / sizeof(record);
}

TEST(FileHandle, TransfersAtOffsets) {
  scoped_file f;
  const auto all = make_records(100, 7);
  auto h = file_handle::create(f.path);
  h.write_at(std::span(all), 0);
  EXPECT_EQ(records_in(f.path), all.size());
  EXPECT_EQ(read_records(h, all.size()), all);
  EXPECT_EQ(read_records(h, 10, 30), slice(all, 30, 10));

  // overwriting a window leaves everything around it alone
  const auto replacement = make_records(10, 9999);
  h.write_at(std::span(replacement), 30 * sizeof(record));
  const auto after = read_records(h, all.size());
  EXPECT_EQ(slice(after, 0, 30), slice(all, 0, 30));
  EXPECT_EQ(slice(after, 30, 10), replacement);
  EXPECT_EQ(slice(after, 40, 60), slice(all, 40, 60));
}

// sort_sequence relies on this: the merge never pre-sizes its output, partitions just write their
// own ranges and the file has to end up exactly as long as they cover
TEST(FileHandle, ExtendsAFileThatAlreadyHasContent) {
  scoped_file f;
  const auto head = make_records(20, 1);
  { file_handle::create(f.path).write_at(std::span(head), 0); }
  const auto tail = make_records(5, 2);
  auto h = file_handle::open(f.path); // opened, not replaced
  h.write_at(std::span(tail), 100 * sizeof(record));

  EXPECT_EQ(records_in(f.path), 105u);
  const auto all = read_records(h, 105);
  EXPECT_EQ(slice(all, 0, 20), head);
  EXPECT_EQ(slice(all, 20, 80), std::vector<record>(80)); // the gap reads as zeros
  EXPECT_EQ(slice(all, 100, 5), tail);
}

TEST(FileHandle, TruncateEmptiesAnExistingFile) {
  scoped_file f;
  const auto initial = make_records(100, 3);
  { file_handle::create(f.path).write_at(std::span(initial), 0); }
  EXPECT_EQ(records_in(f.path), 100u);
  { const auto h = file_handle::create(f.path); }
  EXPECT_EQ(records_in(f.path), 0u);
}

TEST(FileHandle, ThrowsWhenTheFileIsMissing) {
  scoped_file f;
  try {
    const auto h = file_handle::open(f.path);
    FAIL();
  } catch (const std::runtime_error& e) {
    // the one failure that still names the file, since the caller handed it to us
    EXPECT_NE(std::string(e.what()).find(f.path), std::string::npos);
  }
}

TEST(FileHandle, ThrowsWhenReadingPastTheEnd) {
  scoped_file f;
  auto h = file_handle::create(f.path);
  std::vector<record> buffer(20);
  h.write_at(std::span(buffer).first(10), 0); // only ten records exist
  EXPECT_THROW(h.read_at(std::span(buffer), 0), std::runtime_error);
}

TEST(FileHandle, EmptyTransfersDoNothing) {
  scoped_file f;
  auto h = file_handle::create(f.path);
  std::vector<record> none;
  EXPECT_NO_THROW(h.write_at(std::span(none), 0));
  EXPECT_NO_THROW(h.read_at(std::span(none), 1 << 20)); // no bytes move, so the offset is moot
  EXPECT_EQ(records_in(f.path), 0u);
}

// transfers longer than the internal chunk size loop, so the partial transfer bookkeeping runs
TEST(FileHandle, ChunksTransfersLargerThanOneRequest) {
  scoped_file f;
  std::vector<uint64_t> written((64ull << 20) / sizeof(uint64_t) + 1024); // just over 64 MiB
  std::iota(written.begin(), written.end(), 0);

  auto h = file_handle::create(f.path);
  h.write_at(std::span(written), 0);
  EXPECT_EQ(std::filesystem::file_size(f.path), written.size() * sizeof(uint64_t));

  std::vector<uint64_t> read(written.size());
  h.read_at(std::span(read), 0);
  EXPECT_EQ(read, written);
}

// the property the whole design rests on: positioned transfers touch no shared cursor, so one
// handle serves any number of threads
TEST(FileHandle, OneHandleServesConcurrentWriters) {
  scoped_file f;
  constexpr size_t kThreads = 8, kPer = 4096;

  auto h = file_handle::create(f.path);
  std::vector<std::thread> threads;
  for (size_t t = 0; t < kThreads; ++t) {
    // back to front, so the file is extended well before the gaps are filled
    const size_t slot = kThreads - 1 - t;
    threads.emplace_back([&h, slot]() {
      const auto mine = make_records(kPer, slot * 1000000);
      h.write_at(std::span(mine), slot * kPer * sizeof(record));
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(records_in(f.path), kThreads * kPer);
  const auto all = read_records(h, kThreads * kPer);
  for (size_t t = 0; t < kThreads; ++t) {
    EXPECT_EQ(slice(all, t * kPer, kPer), make_records(kPer, t * 1000000)) << "slot " << t;
  }
}

TEST(FileHandle, OneHandleServesConcurrentReaders) {
  scoped_file f;
  const auto written = make_records(8192, 11);
  auto h = file_handle::create(f.path);
  h.write_at(std::span(written), 0);

  std::atomic<size_t> mismatches(0);
  std::vector<std::thread> threads;
  threads.reserve(8);
  for (size_t t = 0; t < 8; ++t) {
    threads.emplace_back([&h, &written, &mismatches, t]() {
      const size_t at = t * 512; // overlapping windows, so they genuinely contend
      for (int repeat = 0; repeat < 16; ++repeat) {
        if (read_records(h, 2048, at) != slice(written, at, 2048)) {
          ++mismatches;
        }
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(mismatches.load(), 0u);
}

// guards a double close: if the moved from handle still owned the descriptor it would shut it
// while the destination is still using it
TEST(FileHandle, DestroyingTheMovedFromHandleLeavesTheFileUsable) {
  scoped_file f;
  const auto written = make_records(64, 2);

  std::optional<file_handle> survivor;
  {
    auto source = file_handle::create(f.path);
    source.write_at(std::span(written), 0);
    survivor.emplace(std::move(source));
  }
  EXPECT_EQ(read_records(*survivor, written.size()), written);
}

TEST(FileHandle, MoveAssignmentAdoptsTheNewFileAndSurvivesSelfAssignment) {
  scoped_file first("move_assign_first");
  scoped_file second("move_assign_second");
  const auto twos = make_records(32, 200);
  {
    const auto ones = make_records(32, 100);
    file_handle::create(first.path).write_at(std::span(ones), 0);
    file_handle::create(second.path).write_at(std::span(twos), 0);
  }

  auto h = file_handle::open(first.path);
  h = file_handle::open(second.path); // the first descriptor is released here
  EXPECT_EQ(read_records(h, twos.size()), twos);

  auto& alias = h;
  h = std::move(alias); // self assignment must not close the descriptor it is about to keep
  EXPECT_EQ(read_records(h, twos.size()), twos);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
