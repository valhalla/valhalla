#include "midgard/logging.h"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#ifdef __ANDROID__
#include <android/log.h>
#endif

namespace {

// Highlight special tags according to their levels.
const std::string& highlight(const std::string& value) {
  const static std::unordered_map<std::string, std::string>
      highlighted{{"[ERROR]", "\x1b[31;1m[ERROR]\x1b[0m"},
                  {"[WARN]", "\x1b[33;1m[WARN]\x1b[0m"},
                  {"[INFO]", "\x1b[32;1m[INFO]\x1b[0m"},
                  {"[DEBUG]", "\x1b[34;1m[DEBUG]\x1b[0m"},
                  {"[TRACE]", "\x1b[37;1m[TRACE]\x1b[0m"}};

  auto it = highlighted.find(value);
  return (it == highlighted.end()) ? value : it->second;
}

// Some compilers don't support enum hash specialization, so provide our own one.
struct EnumHash {
  template <typename Type> auto operator()(const Type& value) const {
    using UnderlyingType = std::underlying_type_t<Type>;
    return std::hash<UnderlyingType>()(static_cast<UnderlyingType>(value));
  }
};

// LogLevel to string map.
// Is used to save backward compatibility with previous logger version.
const std::unordered_map<valhalla::midgard::logging::LogLevel, std::string, EnumHash>
    level_tags{{valhalla::midgard::logging::LogLevel::ERROR, "ERROR"},
               {valhalla::midgard::logging::LogLevel::WARN, "WARN"},
               {valhalla::midgard::logging::LogLevel::INFO, "INFO"},
               {valhalla::midgard::logging::LogLevel::DEBUG, "DEBUG"},
               {valhalla::midgard::logging::LogLevel::TRACE, "TRACE"}};

#ifdef __ANDROID__
// Determine android log priority using message tags.
// Only first level tag is used to determine priority. All other are ignored.
android_LogPriority android_priority(const Tags& tags) {
  static const std::unordered_map<std::string, android_LogPriority>
      android_levels{{"ERROR", ANDROID_LOG_ERROR},
                     {"WARN", ANDROID_LOG_WARN},
                     {"INFO", ANDROID_LOG_INFO},
                     {"DEBUG", ANDROID_LOG_DEBUG},
                     {"TRACE", ANDROID_LOG_VERBOSE}};

  // Get first appropriate tag and return its log priority.
  for (const auto& tag : tags.tags()) {
    if (tag.special()) {
      auto it = android_levels.find();
      if (it != android_levels.end())
        return it->second;
    }
  }

  return ANDROID_LOG_INFO;
}
#endif

inline std::tm* get_gmtime(const std::time_t* time, std::tm* tm) {
#ifdef _MSC_VER
  // MSVC gmtime() already returns tm allocated in thread-local storage
  if (gmtime_s(tm, time) == 0)
    return tm;
  else
    return nullptr;
#else
  return gmtime_r(time, tm);
#endif
}

// returns formatted to: 'year/mo/dy hr:mn:sc.xxxxxx'
std::string TimeStamp() {
  // get the time
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(tp);
  std::tm gmt{};
  get_gmtime(&tt, &gmt);
  using sec_t = std::chrono::duration<double>;
  std::chrono::duration<double> fractional_seconds =
      (tp - std::chrono::system_clock::from_time_t(tt)) + std::chrono::seconds(gmt.tm_sec);
  // format the string
  std::string buffer("year/mo/dy hr:mn:sc.xxxxxx");
  sprintf(&buffer.front(), "%04d/%02d/%02d %02d:%02d:%09.6f", gmt.tm_year + 1900, gmt.tm_mon + 1,
          gmt.tm_mday, gmt.tm_hour, gmt.tm_min, fractional_seconds.count());
  return buffer;
}
} // namespace

namespace valhalla {
namespace midgard {

namespace logging {

const std::string& LevelTag(valhalla::midgard::logging::LogLevel level) {
  return level_tags.find(level)->second;
}

Tags operator+(Tags lhs, const Tags& rhs) {
  return lhs += rhs;
}

void Logger::log(std::unique_ptr<Message>&& message) {
  // Processor can be reconfigured during message enqueuing, so protect it.
  std::lock_guard<std::mutex> lock(processor_mutex_);
  processor_->push(std::move(message));
}

// Create default tag with its current timestamp.
Tags Logger::defaultTags() const {
  return Tags();
}

// Std stream worker.
class StdWorker {
public:
  // Construct worker using its configuration.
  explicit StdWorker(const LoggingConfig& config, std::ostream& stream)
      : highlighted_(config.find("color") != config.end() && config.find("color")->second == "true"),
        stream_(stream) {
  }

  void operator()(const Message& message) const {
#ifdef __ANDROID__
    const auto& tags = message.tags();
    __android_log_print(android_priority(tags), "valhalla", "%s\n",
                        (tags.str() + ": " + message.serialize()).c_str());
#else
    std::stringstream ss;
    ss << TimeStamp() << " ";
    const auto& tags = message.tags();
    if (highlighted_) {
      for (const auto& tag : tags.tags()) {
        std::string enclosed = "[" + tag.str() + "]";
        ss << (tag.custom() ? enclosed : highlight(enclosed));
      }
    } else {
      ss << message.tags().str();
    }

    if (!message.tags().empty())
      ss << " ";
    ss << message.serialize() << std::endl;
    stream_ << ss.rdbuf() << std::flush;
#endif
  }

protected:
  // Determines if special tags should be highlighted.
  const bool highlighted_;
  std::ostream& stream_;
};

// Stdout worker.
class StdOutWorker : public StdWorker {
public:
  // Construct worker using its configuration.
  explicit StdOutWorker(const LoggingConfig& config) : StdWorker(config, std::cout) {
  }
};

// Stderr worker.
class StdErrWorker : public StdWorker {
public:
  // Construct worker using its configuration.
  explicit StdErrWorker(const LoggingConfig& config) : StdWorker(config, std::cerr) {
  }
};

// Text file worker.
class TextFileWorker {
public:
  // Construct worker using its configuration.
  explicit TextFileWorker(const LoggingConfig& config) {
    // Grab the file name.
    auto name = config.find("file_name");
    if (name == config.end()) {
      throw std::runtime_error("No output file provided to file logger");
    }
    file_name = name->second;

    // Open file.
    open();
  }

  void operator()(const Message& message) {
    const auto& tags = message.tags();
    std::stringstream ss;
    ss << TimeStamp() << " " << tags.str();
    if (!tags.empty())
      ss << " ";
    ss << message.serialize() << std::endl;
    file << ss.rdbuf() << std::flush;
  }

protected:
  // Open file to be written to.
  void open() {
    file.open(file_name, std::fstream::out | std::fstream::app);
    if (!file.good())
      throw std::runtime_error("Unable to open log file");
  }

  // File name and stream to be written to.
  std::string file_name;
  std::fstream file;
};

} // namespace logging

// statically get a logger
logging::Logger& logging::GetLogger() {
  static valhalla::midgard::logging::Logger
      logger(valhalla::midgard::logging::Worker::create<valhalla::midgard::logging::StdOutWorker>(
                 "stdout"),
             valhalla::midgard::logging::Worker::create<valhalla::midgard::logging::StdErrWorker>(
                 "stderr"),
             valhalla::midgard::logging::Worker::create<valhalla::midgard::logging::TextFileWorker>(
                 "file"));
  return logger;
}

// statically configure logging
void logging::Configure(const LoggingConfig& config) {
  GetLogger().configure(config);
}

} // namespace midgard
} // namespace valhalla
