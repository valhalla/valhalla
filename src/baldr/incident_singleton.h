#pragma once

#include "baldr/graphreader.h"
#include "midgard/sequence.h"

#include <chrono>
#include <condition_variable>
#include <ctime>
#include <memory>
#include <mutex>
#include <thread>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

namespace {

#ifdef _WIN32
#define MTIME(st_stat) st_stat.st_mtime
#elif __APPLE__
#define MTIME(st_stat) st_stat.st_mtime
#else
#define MTIME(st_stat) st_stat.st_mtim.tv_sec
#endif

constexpr time_t DEFAULT_MAX_LOADING_LATENCY = 60;
constexpr size_t DEFAULT_MAX_LATENT_COUNT = 5;

/**
 * Read the contents of a file into an incident tile
 * @param filename   name of the file on the file system to read into memory
 * @return a shared pointer with the data of the tile or an empty pointer if it could not be read
 */
std::shared_ptr<const valhalla::IncidentsTile> read_tile(const std::string& filename) {
  // crack the file open
  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_WARN("Incident Watcher failed to open " + filename);
    return {};
  }

  // prepare a stream for parsing
  std::string buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  google::protobuf::io::ArrayInputStream as(static_cast<const void*>(buffer.c_str()), buffer.size());
  google::protobuf::io::CodedInputStream cs(
      static_cast<google::protobuf::io::ZeroCopyInputStream*>(&as));

  // try to parse the stream
  std::shared_ptr<valhalla::IncidentsTile> tile(new valhalla::IncidentsTile);
  if (!tile->ParseFromCodedStream(&cs)) {
    LOG_WARN("Incident Watcher failed to parse " + filename);
    return {};
  }

  // dont store empty tiles no point
  if (tile->locations_size() == 0) {
    return {};
  }

  // hand back something that isnt modifyable
  return std::const_pointer_cast<const valhalla::IncidentsTile>(tile);
}

struct incident_singleton_t {
protected:
  // parameter pack to share state between daemon thread and singleton instance
  struct state_t {
    std::atomic<bool> lock_free;
    std::condition_variable signal;
    std::mutex mutex;
    std::unordered_map<uint64_t, std::shared_ptr<const valhalla::IncidentsTile>> cache;
  };
  // we use a shared_ptr to wrap the state between the watcher thread and the main threads singleton
  // instance. this gives the responsibility to the last living thread to deallocate the state object.
  // if we didn't do this, and the watcher thread were still running when the singleton instance got
  // destructed, then the watcher would be making use of a deallocated state object. this way, if the
  // watcher is last to die it own the lifetime of the state and if the singleton is the last to die
  // it owns the lifetime of the state. note that we still need to use atomics inside the state as
  // only the shared_ptr itself is thread safe, not the thing it points to
  std::shared_ptr<state_t> state;
  // daemon thread to watch for new incident data
  std::thread watcher;

  /**
   * Singleton private constructor that static function uses to instantiate the singleton
   * @param config   lets the daemon thread know where/how to look for incidents
   * @param tileset  an mmapped graph tileset (ie static) allows incident loading to be lock-free
   */
  incident_singleton_t(const boost::property_tree::ptree& config,
                       const std::unordered_set<valhalla::baldr::GraphId>& tileset)
      : state{new state_t{}},
        watcher(watch, config, std::cref(tileset), state, std::numeric_limits<size_t>::infinity()) {
    auto max_loading_latency =
        config.get<time_t>("incident_max_loading_latency", DEFAULT_MAX_LOADING_LATENCY);
    // see if the thread can start up and do a pass to load all the incidents
    std::unique_lock<std::mutex> lock(state->mutex);
    auto when = std::chrono::system_clock::now() + std::chrono::seconds(max_loading_latency);
    if (state->signal.wait_until(lock, when) == std::cv_status::timeout) {
      throw std::runtime_error("Unable to initialize incident watcher in the configured time period");
    }
    // let this run forever
    watcher.detach();
  }

  /**
   * Updates the tile in the states cache
   * @param state     the state to update
   * @param tile_id   the tile id we are loading
   * @param path      the path to the tile file
   * @return the shared_ptr that owns the tile memory
   */
  static bool update_tile(const std::shared_ptr<state_t>& state,
                          const valhalla::baldr::GraphId& tile_id,
                          const std::string& path) {
    // load the tile
    auto tile = read_tile(path);
    // see if we have a slot
    auto found = state->cache.find(tile_id);
    // if we dont have a slot make one, this must be synchronized
    if (found == state->cache.cend()) {
      // this shouldnt happen in lock free mode but can if you put unexpected tiles in the log/dir
      if (state->lock_free.load()) {
        LOG_WARN("Incident watcher skipped " + std::to_string(tile_id) +
                 " because it was not found in the configured tile extract");
        return false;
      }
      // actually make a spot in the cache synchronously
      try {
        std::unique_lock<std::mutex>(state->mutex);
        found = state->cache.insert({tile_id, {}}).first;
      } // if anything went wrong we have to catch it so that the mutex is unlocked
      catch (...) {
        LOG_ERROR("Incident watcher failed to add incident tile to cache");
      }
    }
    // atomically store the tile shared_ptr, could be actually nullptr when there are no incidents
    std::atomic_store_explicit(&found->second, tile, std::memory_order_release);
    LOG_DEBUG("Incident watcher " + std::string(tile ? "loaded " : "unloaded ") +
              std::to_string(tile_id));
    return true;
  }

  /**
   * Thread work function that continually checks for updates to incident tiles. The updates come in
   * the form of timestamps in a memory mapped file of 64bit integers (one per tile). The first 25
   * bits are the tile id and level (as a normal graphid) the remaining 39 bits are the timestamp.
   * When the timestamp for the last check is older than a timestamp for a given file that file is
   * replaced with whatever its contents are on disk.
   *
   * The thread begins by deciding whether its just scaning the directory (works for a small number of
   * incidents) or using a memory mapped changelog to communicate about which incidents chnaged last.
   * If the latter is used we preallocate
   * @param config    lets the function know where to look for incidents and desired update frequency
   * @param tileset   if the tileset is static we can use lockfree mode
   * @param state     used for interthread communication
   * @param max_runs  controls the maximum number of updates that will occur, infinity runs forever
   */
  static void watch(boost::property_tree::ptree config,
                    std::unordered_set<valhalla::baldr::GraphId> tileset,
                    std::shared_ptr<state_t> state,
                    size_t max_runs) {
    LOG_INFO("Incident watcher started");
    // try to configure for changelog mode
    std::unique_ptr<valhalla::midgard::sequence<uint64_t>> changelog;
    filesystem::path inc_log_path(config.get<std::string>("incident_log", ""));
    filesystem::path inc_dir;
    try {
      changelog.reset(new decltype(changelog)::element_type(inc_log_path.string()));
      LOG_INFO("Incident watcher configured for mmap changelog mode");
    } // check for a directory scan mode config
    catch (...) {
      inc_dir = filesystem::path(config.get<std::string>("incident_dir", ""));
      if (!filesystem::is_directory(inc_dir)) {
        inc_dir = {};
      } else {
        LOG_INFO("Incident watcher configured for directory scan mode");
      }
    }

    // bail if there is nothing to do
    if (!changelog && inc_dir.string().empty()) {
      LOG_INFO("Incident watcher disabled");
      state->signal.notify_one();
      return;
    }

    // a static tileset allows us to preallocate the cache entries which makes everything lock-free
    // for a planet extract this should be about 200000 * 8 * 2 == 3MB of ram
    state->lock_free.store(!tileset.empty());
    state->cache.reserve(tileset.size());
    for (const auto& tile_id : tileset) {
      state->cache[tile_id] = {};
    }

    // some setup for continuous operation
    size_t run_count = 0;
    time_t last_scan = 0;
    time_t max_loading_latency =
        config.get<time_t>("incident_max_loading_latency", DEFAULT_MAX_LOADING_LATENCY);

    // wait for someone to tell us to stop
    do {
      // TODO: its possible that a tiles update time is newer than even this scans start time
      // this happens when the tile is updated during the loop. in that case its possible that
      // the current iteration will load the tile and that it will again be loaded in the next
      auto current_scan = time(nullptr);
      size_t update_count = 0;

      // we are in memory map mode
      if (changelog) {
        // reload the log if the tileset isnt static
        if (!state->lock_free.load())
          changelog.reset(new decltype(changelog)::element_type(inc_log_path.string()));
        // check all of the timestamps/tile_ids
        for (auto entry : *changelog) {
          // check the timestamp part of the tile id to see if its newer
          // spare last 39 bits are the timestamp, leaves us with something like 17k years
          uint64_t timestamp = (entry >> 25) & ((uint64_t(1) << 39) - 1);
          if (last_scan <= timestamp) {
            // first 25 bits are the tile id
            valhalla::baldr::GraphId tile_id(((uint64_t(1) << 46) - 1) & entry);
            // concoct a file name from the tile_id
            auto file_location = inc_log_path;
            file_location.replace_filename(
                valhalla::baldr::GraphTile::FileSuffix(tile_id, ".pbf", true));
            // update the tile
            update_count += update_tile(state, tile_id, file_location.string());
          }
        }
      } // we are in directory scan mode
      else if (!inc_dir.string().empty()) {
        // check all of the files
        for (filesystem::recursive_directory_iterator i(inc_dir), end; i != end; ++i) {
          try {
            // if the tile was updated since the last time we scanned we load it
            struct stat s;
            valhalla::baldr::GraphId tile_id;
            if (i->is_regular_file() &&
                (tile_id = valhalla::baldr::GraphTile::GetTileId(i->path().string())).Is_Valid() &&
                stat(i->path().c_str(), &s) == 0 && last_scan <= MTIME(s)) {
              // update the tile
              update_count += update_tile(state, tile_id, i->path().string());
            }
          } catch (...) { LOG_WARN("Incident watcher ignoring " + i->path().string()); }
        }
      }

      // if this round finished but was slower than we want
      last_scan = current_scan;
      auto latency = time(nullptr) - current_scan;
      auto wait = 0;
      if (latency > max_loading_latency) {
        LOG_WARN("Incident watcher is not meeting max loading latency requirement");
      } // this round finished fast enough, changelog mode is cheap so wait only a second
      else {
        wait = changelog ? 1 : max_loading_latency - latency;
      }
      LOG_INFO("Incident watcher loaded " + std::to_string(update_count) + " tiles in " +
               std::to_string(latency) + " seconds");

      // signal to the constructor that we completed our first batch
      if (run_count++ == 0) {
        LOG_INFO("Incident watcher initialized");
        state->signal.notify_one();
      }

      // wait just a little before we check again
      std::this_thread::sleep_for(std::chrono::seconds(wait));
    } while (max_runs == std::numeric_limits<size_t>::infinity() || run_count < max_runs);

    LOG_INFO("Incident watcher has stopped");
  }

public:
  /**
   * Get an incident tile, this method is never called unless the config dictates it
   * @param tile_id   the tile id specifies which tile you want
   * @param config    only needed on first call, configures the incident loading
   * @param tileset   only needed on first call, configures the incident loading
   * @return a shared_ptr to the incident tile or an empty shared_ptr when none exists
   */
  static std::shared_ptr<const valhalla::IncidentsTile>
  get(const valhalla::baldr::GraphId& tile_id,
      const boost::property_tree::ptree& config = {},
      const std::unordered_set<valhalla::baldr::GraphId>& tileset = {}) {
    // spawn a daemon to watch for incidents
    static incident_singleton_t singleton{config, tileset};

    // return the tile from the cache or an empty one if its not there
    auto scoped_lock = singleton.state->lock_free.load()
                           ? std::unique_lock<std::mutex>()
                           : std::unique_lock<std::mutex>(singleton.state->mutex);
    auto found = singleton.state->cache.find(tile_id);
    if (found == singleton.state->cache.cend()) {
      return {};
    }
    auto tile = std::atomic_load_explicit(&found->second, std::memory_order_acquire);
    return tile;
  }
};
} // namespace
