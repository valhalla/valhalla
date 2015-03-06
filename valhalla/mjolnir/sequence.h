#ifndef VALHALLA_MJOLNIR_SEQUENCE_H_
#define VALHALLA_MJOLNIR_SEQUENCE_H_

#include <fstream>
#include <string>
#include <vector>
#include <type_traits>
#include <iterator>
#include <cmath>
#include <memory>
#include <cerrno>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <list>
#include <map>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

namespace valhalla{
namespace mjolnir{

template <class T>
struct sequence {
 public:
  //static_assert(std::is_pod<T>::value, "sequence requires POD types for now");
  static const size_t npos = -1;

  sequence() = delete;

  sequence(const sequence&) = delete;

  sequence(const std::string& file_name, bool create = false, bool use_mmap = true, size_t write_buffer_size = 1024 * 1024 * 32 / sizeof(T)):
    file(new std::fstream(file_name, std::ios_base::binary | std::ios_base::in | std::ios_base::out | (create ? std::ios_base::trunc : std::ios_base::ate))),
    file_name(file_name), mmap_handle(nullptr) {

    //crack open the file
    if(!*file)
      throw std::runtime_error(file_name + ": " + strerror(errno));
    auto end = file->tellg(); //file_size(file_name);
    element_count = std::ceil<size_t>(end / sizeof(T));
    if(end != element_count * sizeof(T))
      throw std::runtime_error("This file has an incorrect size for type");
    write_buffer.reserve(write_buffer_size ? write_buffer_size : 1);

    //memory map the file for reading
    if(use_mmap)
      remmap(element_count, element_count);
  }

  ~sequence() {
    //finish writing whatever it was to file
    flush();
    //lose the memory map
    if(mmap_handle)
      munmap(mmap_handle, element_count * sizeof(T));
  }

  //add an element to the sequence
  void push_back(const T& obj) {
    write_buffer.push_back(obj);
    //push it to the file
    if(write_buffer.size() == write_buffer.capacity())
      flush();
  }

  //search for an object using binary search O(logn)
  //assumes the file was written in sorted order
  //the predicate should be something like a less than or greater than check
  bool find(T& target, const std::function<bool (const T&, const T&)>& predicate) {
    flush();
    //if no elements we are done
    if(element_count == 0)
      return false;
    //if its a memory map use built in
    else if(mmap_handle) {
      T original = target;
      target = *std::lower_bound(static_cast<const T*>(mmap_handle), static_cast<const T*>(mmap_handle) + element_count, original, predicate);
      return !(predicate(original, target) || predicate(target, original));
    }

    //binary search within the file to find a section of the file that should contain it
    size_t low = 0, high = element_count - 1;
    T original = target;
    while(low <= high) {
      //grab the object in the middle of the range
      auto mid = (low + high) / 2;
      file->seekg(mid * sizeof(T));
      file->read(static_cast<char*>(static_cast<void*>(&target)), sizeof(T));
      //done
      if(!predicate(original, target) && !predicate(target, original))
        return true;
      //if this was the last chance to find it and we didnt
      if(low == high)
        return false;
      //search the upper half
      else if(predicate(target, original))
        low = mid + 1;
      //search the lower half
      else
        high = mid - 1;
    }
    //didn't find it
    return false;
  }

  //finds the first matching object by scanning O(n)
  //assumes nothing about the order of the file
  //the predicate should be something like an equality check
  size_t find_first_of(const T& target, const std::function<bool (const T&, const T&)>& predicate, size_t start_index = 0) {
    flush();
    //keep looking while we have stuff to look at
    while(start_index < element_count) {
      T candidate = mmap_handle ? *(static_cast<const T*>(mmap_handle) + start_index) : (*this)[start_index];
      if(predicate(target, candidate))
        return start_index;
      ++start_index;
    }
    return npos;
  }

  //sort the file based on the predicate
  void sort(const std::function<bool (const T&, const T&)>& predicate, size_t buffer_size = 1024 * 1024 * 512 / sizeof(T)) {
    using queue_t = std::map<T, std::list<std::fstream>::iterator, std::function<bool (const T&, const T&)> >;

    flush();
    //if no elements we are done
    if(element_count == 0)
      return;
    //if its a memory map use built in
    else if(mmap_handle) {
      std::sort(static_cast<T*>(mmap_handle), static_cast<T*>(mmap_handle) + element_count, predicate);
      return;
    }

    //write take a chunk at a time, sort it, and write it to a temp file
    std::vector<T> tmp_buffer(buffer_size);
    size_t chunk;
    std::list<std::string> tmp_names;
    for(size_t pos = 0, i = 0; pos < element_count; pos += chunk, ++i) {
      //grab a chunk and sort it
      file->seekg(pos * sizeof(T));
      chunk = element_count - pos > buffer_size ? buffer_size : element_count - pos;
      file->read(static_cast<char*>(static_cast<void*>(tmp_buffer.data())), chunk * sizeof(T));
      std::sort(tmp_buffer.begin(), tmp_buffer.begin() + chunk, predicate);
      //write it to a temp file so we can merge later
      std::string tmp_name = std::to_string(i) + "." + file_name;
      std::fstream tmp_file(tmp_name, std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
      if(!tmp_file)
        throw std::runtime_error(tmp_name + ": " + strerror(errno));
      tmp_names.emplace_back(std::move(tmp_name));
      tmp_file.write(static_cast<const char*>(static_cast<const void*>(tmp_buffer.data())), chunk * sizeof(T));
    }

    //special case when we only had one file to sort because it was so few elements
    std::string sorted_file_name = tmp_names.back();
    if(tmp_names.size() > 1){

      //a way to read a single value from a temp file and get it into the queue
      auto read_one = [](std::list<std::fstream>::iterator& tmp_file, queue_t& queue){
        T value;
        tmp_file->read(static_cast<char*>(static_cast<void*>(&value)), sizeof(T));
        if(!tmp_file->eof())
          queue.emplace(std::piecewise_construct, std::forward_as_tuple(value), std::forward_as_tuple(tmp_file));
      };

      //crack open all the temp files
      std::list<std::fstream> tmp_files;
      for(const auto& tmp_name : tmp_names) {
        tmp_files.emplace_back(tmp_name, std::ios_base::binary | std::ios_base::in);
        if(!tmp_files.back())
          throw std::runtime_error(tmp_name + ": " + strerror(errno));
      }

      //write out a single sorted file by taking the next best value from any of the temp files
      sorted_file_name = "sorted." + file_name;
      std::fstream sorted_file(sorted_file_name, std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
      queue_t queue(predicate);
      //prime the queue with the initial values
      for(std::list<std::fstream>::iterator tmp_file = tmp_files.begin(); tmp_file != tmp_files.end(); ++tmp_file)
        read_one(tmp_file, queue);
      //while we have something to write out
      size_t written = 0;
      while(queue.size()){
        sorted_file.write(static_cast<const char*>(static_cast<const void*>(&queue.begin()->first)), sizeof(T));
        std::list<std::fstream>::iterator& tmp_file = queue.begin()->second;
        queue.erase(queue.begin());
        read_one(tmp_file, queue);
        if(++written % write_buffer.capacity() == 0){
          written = 0;
          sorted_file.flush();
        }
      }
      //done with these
      for(const auto& tmp_name : tmp_names) {
        if(std::remove(tmp_name.c_str()) != 0)
          throw std::runtime_error("Couldn't remove file: " + tmp_name);
      }
    }

    //lose the unsorted file
    file->close();
    if(std::remove(file_name.c_str()) != 0)
      throw std::runtime_error("Couldn't remove file: " + file_name);
    if(std::rename(sorted_file_name.c_str(), file_name.c_str()) != 0)
      throw std::runtime_error("Couldn't move file " + sorted_file_name + " to file " + file_name);
    file->open(file_name, std::ios_base::binary | std::ios_base::in | std::ios_base::out);
  }

  //perform an volatile operation on all the items of this sequence
  void transform(const std::function<void (T&)>& predicate) {
    flush();
    for(size_t i = 0; i < element_count; ++i) {
      //grab the element
      auto element = at(i);
      //grab the underlying item
      auto item = *element;
      //transform it
      predicate(item);
      //put it back
      element = item;
    }
  }

  //perform a non-volatile operation on all the items of this sequence
  void enumerate(const std::function<void (const T&)>& predicate) {
    flush();
    //grab each element and do something with it
    for(size_t i = 0; i < element_count; ++i)
      predicate(*at(i));
  }

  //force writing whatever we have in the write_buffer to file
  void flush() {
    if(write_buffer.size()) {
      file->seekg(0, file->end);
      file->write(static_cast<const char*>(static_cast<const void*>(write_buffer.data())), write_buffer.size() * sizeof(T));
      file->flush();
      if(mmap_handle)
        remmap(element_count, element_count + write_buffer.size());
      element_count += write_buffer.size();
      write_buffer.clear();
    }
  }

  //how many things have been written so far
  size_t size() const {
    return element_count + write_buffer.size();
  }

  //a read/writeable object within the sequence, accessed either through memory or file
  struct iterator {
    friend class sequence;
   public:
    //static_assert(std::is_pod<T>::value, "sequence_element requires POD types for now");
    iterator() = delete;
    iterator& operator=(const iterator& other) {
      parent = other.parent;
      index = other.index;
      return *this;
    }
    iterator& operator=(const T& other) {
      if(parent->mmap_handle) {
        (static_cast<T*>(parent->mmap_handle) + index) = other;
        return *this;
      }

      parent->file->seekg(index * sizeof(T));
      parent->file->write(static_cast<const char*>(static_cast<const void*>(&other)), sizeof(T));
      return *this;
    }
    operator T() {
      if(parent->mmap_handle)
        return *(static_cast<const T*>(parent->mmap_handle) + index);

      T result;
      parent->file->seekg(index * sizeof(T));
      parent->file->read(static_cast<char*>(static_cast<void*>(&result)), sizeof(T));
      return result;
    }
    T operator*() {
      return operator T();
    }
    iterator& operator++() {
      parent->flush();
      ++index;
      return *this;
    }
    iterator operator++(int) {
      parent->flush();
      auto other = *this;
      ++index;
      return other;
    }
    iterator& operator+=(size_t offset) {
      parent->flush();
      index += offset;
      return *this;
    }
    iterator operator+(size_t offset) {
      parent->flush();
      auto other = *this;
      other.index += offset;
      return *this;
    }
    iterator& operator--() {
      parent->flush();
      --index;
      return *this;
    }
    iterator operator--(int) {
      parent->flush();
      auto other = *this;
      --index;
      return other;
    }
    iterator& operator-=(size_t offset) {
      parent->flush();
      index -= offset;
      return *this;
    }
    iterator operator-(size_t offset) {
      parent->flush();
      auto other = *this;
      other.index -= offset;
      return *this;
    }
    bool operator==(const iterator& other) const {
      return parent == other.parent && index == other.index;
    }
    size_t operator-(const iterator& other) const {
      return index - other.index;
    }
    size_t position() const {
      return index;
    }
   protected:
    iterator(sequence* parent, size_t offset): parent(parent), index(index) {}
    sequence* parent;
    size_t index;
  };

  iterator at(size_t index) {
    //dump to file and make an element
    flush();
    return iterator(this, index);
  }

  //write/read at certain index
  iterator operator[](size_t index) {
    return at(index);
  }

  //write/read at the beginning
  T front() {
    return *at(0);
  }

  //write/read at the end
  T back() {
    flush();
    return *iterator(this, element_count - 1);
  }

  //first iterator
  iterator begin() {
    return at(0);
  }

  //invalid end iterator
  iterator end() {
    flush();
    return iterator(this, element_count);
  }

 protected:

  //redo the memory map
  void remmap(size_t old_size, size_t new_size) {
    //has to be something to unmap
    if(mmap_handle) {
      auto un = munmap(mmap_handle, old_size * sizeof(T));
      if(un == -1)
        throw std::runtime_error(file_name + "(munmap): " + strerror(errno));
    }
    //has to be something to remap
    if(new_size > 0) {
      auto fd = open(file_name.c_str(), O_RDWR, 0);
      if(fd == -1)
        throw std::runtime_error(file_name + "(open): " + strerror(errno));
      mmap_handle = mmap(nullptr, new_size * sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if(mmap_handle == MAP_FAILED)
        throw std::runtime_error(file_name + "(mmap): " + strerror(errno));
      auto cl = close(fd);
      if(cl == -1)
        throw std::runtime_error(file_name + "(close): " + strerror(errno));
    }
  }

  std::shared_ptr<std::fstream> file;
  std::string file_name;
  std::vector<T> write_buffer;
  size_t element_count;
  void* mmap_handle;
};

}
}

#endif //VALHALLA_MJOLNIR_SEQUENCE_H_
