#include <vector>
#include <fstream>
#include <memory>
#include <functional>
#include <queue>
#include <iostream>
#include <algorithm>

const int kPartSize = 10000;


const auto in_mode = std::ios::in;
const auto out_mode = std::ios::out;
//const auto in_mode = std::ios::binary;
//const auto out_mode = std::ios::binary;

template <typename T>
inline T Read(std::istream& is);


template <typename T>
inline void Write(std::ostream& os, T value);

class BufferedReader {
public:
//    explicit BufferedReader(std::istream& is) : is_(is) {}
  BufferedReader() {}

  explicit BufferedReader(std::string filename, std::ios::openmode mode = in_mode) : is_(std::move(filename), mode) {}

  void open(std::string filename, std::ios::openmode mode) {
    is_.open(std::move(filename), mode);
  }

  void close() {
    is_.close();
  }

  template <class T>
  T Read() {
    if (ptr_ + sizeof(T) > end_) {
      std::copy(ptr_, end_, buf_);
      int tail = end_ - ptr_;
      is_.read(buf_ + tail, ptr_ - buf_);
      ptr_ = buf_;
    }
    T value = *reinterpret_cast<T*>(ptr_);
    ptr_ += sizeof(T);
    return value;
  }

private:
  static const size_t kInitialSize = 1000;
  char buf_[kInitialSize];
  char* end_ = buf_ + kInitialSize;
  char* ptr_ = end_;
  std::ifstream is_;
};

class BufferedWriter {
public:
  BufferedWriter() {}

  explicit BufferedWriter(std::string filename, std::ios::openmode mode = out_mode) : os_(std::move(filename), mode) {}

  void open(std::string filename, std::ios::openmode mode) {
    os_.open(std::move(filename), mode);
  }

  void close() {
    if (ptr_ != buf_) {
      os_.write(buf_, ptr_ - buf_);
    }
  }

  template <class T>
  void Write(T t) {
    if (ptr_ + sizeof(T) > end_) {
      os_.write(buf_, ptr_ - buf_);
      ptr_ = buf_;
    }
    *reinterpret_cast<T*>(ptr_) = t;
    ptr_ += sizeof(T);
  }

  ~BufferedWriter() {
    close();
  }

private:
  static const size_t kInitialSize = 1000;
  char buf_[kInitialSize];
  char* end_ = buf_ + kInitialSize;
  char* ptr_ = buf_;
  std::ofstream os_;
};


template <typename T>
T Read(std::istream& is) {
  T value;
  is.read(reinterpret_cast<char*>(&value), sizeof(value));
  return value;
}

template <typename T>
T Read(BufferedReader& reader) {
  return reader.Read<T>();
}


template <typename T>
void Write(std::ostream& os, T value) {
  os.write(reinterpret_cast<char*>(&value), sizeof(value));
}

template <typename T>
void Write(BufferedWriter& writer, T value) {
  writer.Write(value);
}


template <class SizeType, class T>
class ExternalSort {
public:
  ExternalSort(std::string input, std::string output, std::function<void(T&)> modify)
      : input_file_(std::move(input)),
        output_file_(std::move(output)),
        modify_(modify)
  {}



  void Sort(bool skip_final_size = false) {
    auto files = MakeParts();
    int file_count = files.size();

    while (true) {
      std::vector<BufferedReader> fins;
      std::vector<int64_t> sizes;
      std::priority_queue<
          std::pair<T, int>,
          std::vector<std::pair<T, int>>,
          std::greater<std::pair<T, int>>
      > pq;

      auto part_count = std::min(static_cast<int>(files.size()), kPartSize);
      fins.reserve(part_count);
      sizes.reserve(part_count);

      SizeType size_sum = 0;
      for (size_t i = 0; i < part_count; ++i) {
        fins.emplace_back(files.front(), in_mode);
        files.pop();

        auto size = Read<SizeType>(fins.back());
        size_sum += size;

        pq.emplace(Read<T>(fins.back()), i);
        sizes.push_back(size - 1);
      }

      auto output_name = files.empty() ? output_file_ : std::string("_") + std::to_string(file_count++);
      BufferedWriter fout(output_name, out_mode);
      if (!files.empty() || !skip_final_size) {
        Write(fout, size_sum);
      }

      while (!pq.empty()) {
        T value;
        int file_index;
        std::tie(value, file_index) = pq.top();
        pq.pop();

        Write(fout, value);
        if (sizes[file_index]) {
          pq.emplace(Read<T>(fins[file_index]), file_index);
          --sizes[file_index];
        }
      }
      files.push(output_name);

      if (files.size() == 1) {
        break;
      }
    }
  }

private:
  std::string input_file_, output_file_;
  std::function<void(T&)> modify_;

  std::vector<T> ReadPart(BufferedReader& is, SizeType part_size) {
    std::vector<T> part;
    part.reserve(part_size);
    while (part_size--) {
      part.push_back(Read<T>(is));
      modify_(part.back());
    }
    return part;
  }

  void WritePart(BufferedWriter& os, const std::vector<T>& part) {
    Write(os, static_cast<SizeType>(part.size()));
    for (const auto& i : part) {
      Write(os, i);
    }
  }

  std::queue<std::string> MakeParts() {
    BufferedReader fin(input_file_, in_mode);
    SizeType element_count = Read<SizeType>(fin);

    int64_t part_count = 0;
    std::queue<std::string> files;
    while (element_count) {
      SizeType part_size = std::min(static_cast<int>(element_count), kPartSize);
      auto part = ReadPart(fin, part_size);
      std::sort(part.begin(), part.end());

      auto filename = std::string("_") + std::to_string(part_count);
      files.push(filename);
      BufferedWriter fout(filename, out_mode);
      WritePart(fout, part);

      ++part_count;
      element_count -= part_size;
    }
    return files;
  }
};


//  std::string by_index = std::string("by_index") + kExtension;
//  ExternalSort<int, std::pair<int, int>>(
//      std::string("input") + kExtension,
//      by_index,
//      [](std::pair<int, int>&) {}
//  ).Sort();
