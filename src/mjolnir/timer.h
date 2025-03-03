#ifndef VALHALLA_MJOLNIR_TIMER_H_
#define VALHALLA_MJOLNIR_TIMER_H_

#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <valhalla/midgard/logging.h>
#include <vector>

namespace valhalla {
namespace mjolnir {

// Forward declarations
class BuildTimingStatistics;

/**
 * Utility function to format milliseconds into a human-readable string
 * @param ms Time in milliseconds
 * @param include_ms Whether to include milliseconds in output
 * @return Formatted duration string
 */
inline std::string format_duration(int64_t ms, bool include_ms = false) {
  std::stringstream ss;

  if (ms >= 3600000) { // Time to begine a Netflix series(I recommend DrHouse cause it never ends)
    int64_t h = ms / 3600000;
    ms %= 3600000;
    int64_t m = ms / 60000;
    ms %= 60000;
    int64_t s = ms / 1000;
    ms %= 1000;
    ss << h << "h " << m << "m " << s << "s";
    if (include_ms)
      ss << " " << ms << "ms";
  } else if (ms >= 60000) { // mite want to grab a coffe while you wait
    int64_t m = ms / 60000;
    ms %= 60000;
    int64_t s = ms / 1000;
    ms %= 1000;
    ss << m << "m " << s << "s";
    if (include_ms)
      ss << " " << ms << "ms";
  } else if (ms >= 1000) { // ok this is resonable i gess
    int64_t s = ms / 1000;
    ms %= 1000;
    ss << s << "s";
    if (include_ms)
      ss << " " << ms << "ms";
  } else { // zoom zoom, fast as lighting mcqueen
    ss << ms << "ms";
  }

  return ss.str();
}

/**
 * A utility class for standardized timing measurements within Valhalla's tile building process.
 * Tracks how long your code takes to run so you can prove to your boss it's not your fault.
 */
class BuildStageTimer {
public:
  /**
   * Constructor - starts the timer and logs the beginning of a stage
   * @param stage_name Name of the stage (ideally matches BuildStage enum)
   * @param is_substage Whether this timer is for a substage
   * @param substage_name Optional substage name (defaults to stage_name)
   */
  BuildStageTimer(const std::string& stage_name,
                  bool is_substage = false,
                  const std::string& substage_name = "")
      : stage_name_(stage_name), substage_name_(substage_name.empty() ? stage_name : substage_name),
        is_substage_(is_substage), start_(std::chrono::high_resolution_clock::now()),
        stopped_(false) {

    std::stringstream ss;
    ss << "[BUILD_TIMER] " << (is_substage_ ? "SUBSTAGE_START" : "STAGE_START") << " "
       << get_full_name() << " ...";
    LOG_INFO(ss.str());
  }

  /**
   * Destructor - automatically stops timer if not already stopped
   * Because no one remembers to clean up their messes
   */
  ~BuildStageTimer() {
    if (!stopped_) {
      stop();
    }
  }

  /**
   * Manually stop timing and log result
   * @param success Whether the stage succeeded or failed miserably
   * @return Time elapsed in milliseconds (how long you've been suffering)
   */
  int64_t stop(bool success = true) {
    if (stopped_) {
      return elapsed_ms_;
    }

    auto end = std::chrono::high_resolution_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count();
    stopped_ = true;

    log_duration(success);
    record_timing();

    return elapsed_ms_;
  }

  /**
   * Get elapsed time without stopping timer
   * For when you're curious but not ready to commit
   * @return Milliseconds elapsed so far
   */
  int64_t elapsed_ms() const {
    if (stopped_) {
      return elapsed_ms_;
    }
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count();
  }

  /**
   * Get full stage name including parent stage if it's a substage
   * @return Formatted stage name
   */
  std::string get_full_name() const {
    if (is_substage_) {
      return stage_name_ + "." + substage_name_;
    }
    return stage_name_;
  }

private:
  std::string stage_name_;
  std::string substage_name_;
  bool is_substage_;
  std::chrono::high_resolution_clock::time_point start_;
  bool stopped_;
  int64_t elapsed_ms_ = 0;

  // Formats and logs the timing results
  void log_duration(bool success) const {
    // Use the common formatting function
    std::string duration_str = format_duration(elapsed_ms_, true);

    std::stringstream ss;
    ss << "[BUILD_TIMER] " << (is_substage_ ? "SUBSTAGE_COMPLETE" : "STAGE_COMPLETE") << " "
       << get_full_name() << ": " << duration_str << " (" << elapsed_ms_ << "ms)"
       << " [" << (success ? "SUCCESS" : "FAILED") << "]";

    LOG_INFO(ss.str());

    if (!is_substage_) {
      LOG_INFO("--------------------------------------------------------");
    }
  }

  // Records timing to global statistics
  void record_timing();
};

/**
 * Global statistics collector for build timings.
 * The big brother that watches all your timers.
 */
class BuildTimingStatistics {
public:
  /**
   * Singleton accessor
   * @return Reference to the global instance
   */
  static BuildTimingStatistics& instance() {
    static BuildTimingStatistics instance;
    return instance;
  }

  /**
   * Record a timing entry
   * @param stage_name Stage/substage identifier
   * @param is_substage Whether this is a substage
   * @param elapsed_ms Time elapsed in milliseconds
   */
  void record(const std::string& stage_name, bool is_substage, int64_t elapsed_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    timings_[stage_name].push_back(elapsed_ms);

    if (!is_substage) {
      major_stage_total_ms_ += elapsed_ms;
    }
  }

  /**
   * Log a summary of all collected timings
   * Aka "The Wall of Shame"
   */
  void log_summary() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Sort stages by how much of your life they wasted
    std::vector<std::pair<std::string, int64_t>> sorted_timings;
    for (const auto& [name, times] : timings_) {
      int64_t stage_total = 0;
      for (int64_t time : times) {
        stage_total += time;
      }
      sorted_timings.push_back({name, stage_total});
    }

    std::sort(sorted_timings.begin(), sorted_timings.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    LOG_INFO("============= BUILD TIMING SUMMARY =============");
    LOG_INFO("Total build time: " + format_duration(major_stage_total_ms_));
    LOG_INFO("Stages by total time (descending):");

    for (const auto& [stage_name, total_ms] : sorted_timings) {
      int count = timings_[stage_name].size();
      int64_t average_ms = total_ms / count;

      // Calculate percentage of total time for major stages
      double percentage = 0.0;
      if (major_stage_total_ms_ > 0 && stage_name.find('.') == std::string::npos) {
        percentage = (static_cast<double>(total_ms) / major_stage_total_ms_) * 100.0;
      }

      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << stage_name << ": " << format_duration(total_ms);

      if (count > 1) {
        ss << " (avg: " << format_duration(average_ms) << ", count: " << count << ")";
      }

      if (percentage > 0.0) {
        ss << " [" << percentage << "% of total]";
      }

      LOG_INFO(ss.str());
    }

    LOG_INFO("==============================================");
  }

private:
  BuildTimingStatistics() = default;

  std::mutex mutex_;
  std::map<std::string, std::vector<int64_t>> timings_;
  int64_t major_stage_total_ms_ = 0;
};

// Implementation of record_timing method
inline void BuildStageTimer::record_timing() {
  BuildTimingStatistics::instance().record(get_full_name(), is_substage_, elapsed_ms_);
}

/**
 * Convenience function to log the timing summary at the end of the build
 * Call this when you want to see where all your CPU cycles went
 */
inline void log_build_timing_summary() {
  BuildTimingStatistics::instance().log_summary();
}

// Convenience macros for even lazier developers
#define VALHALLA_BEGIN_STAGE_TIMER(stage_name)                                                       \
  auto stage_timer_##stage_name =                                                                    \
      std::make_unique<valhalla::mjolnir::BuildStageTimer>(#stage_name, false)

#define VALHALLA_BEGIN_SUBSTAGE_TIMER(stage_name, substage_name)                                     \
  auto stage_timer_##substage_name =                                                                 \
      std::make_unique<valhalla::mjolnir::BuildStageTimer>(#stage_name, true, #substage_name)

#define VALHALLA_END_STAGE_TIMER(stage_name, success) stage_timer_##stage_name->stop(success)

#define VALHALLA_END_SUBSTAGE_TIMER(substage_name, success) stage_timer_##substage_name->stop(success)

} // namespace mjolnir
} // namespace valhalla

#endif // VALHALLA_MJOLNIR_TIMER_H_