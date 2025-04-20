#pragma once

#include <sqlite3.h>

#include <string_view>

namespace valhalla {
namespace mjolnir {

// RAII wrapper around sqlite3 and spatialite connection
class Sqlite3 final {
  sqlite3* db = nullptr;
  void* spatialite = nullptr;

  // constructor is private, use `open` instead
  Sqlite3(sqlite3* db, void* spatialite) : db(db), spatialite(spatialite) {
  }

public:
  static Sqlite3 open(const std::string& path,
                      int flags = SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX);
  // Default constructor that creates an empty Sqlite3 instance
  Sqlite3() : db(nullptr), spatialite(nullptr) {
  }
  ~Sqlite3();

  // This class cannot be copied, but can be moved
  Sqlite3(Sqlite3 const&) = delete;
  Sqlite3& operator=(Sqlite3 const&) = delete;
  Sqlite3(Sqlite3&& other) noexcept : db(other.db), spatialite(other.spatialite) {
    other.db = nullptr;
    other.spatialite = nullptr;
  }
  Sqlite3& operator=(Sqlite3&& other) noexcept {
    // move and swap idiom via local variable
    Sqlite3 local = std::move(other);
    std::swap(db, local.db);
    return *this;
  }

  // It might happen that DB fails to open and this method allow to check it.
  explicit operator bool() const {
    return db != nullptr;
  }

  sqlite3* get() {
    return db;
  }
};

} // namespace mjolnir
} // namespace valhalla
