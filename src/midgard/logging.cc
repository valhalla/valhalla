#include "midgard/logging.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#ifdef __ANDROID__
#include <android/log.h>
#endif

namespace {

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

// the Log levels we support
struct EnumHasher {
  template <typename T> std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};
const std::unordered_map<valhalla::midgard::logging::LogLevel, std::string, EnumHasher>
    uncolored{{valhalla::midgard::logging::LogLevel::ERROR, " [ERROR] "},
              {valhalla::midgard::logging::LogLevel::WARN, " [WARN] "},
              {valhalla::midgard::logging::LogLevel::INFO, " [INFO] "},
              {valhalla::midgard::logging::LogLevel::DEBUG, " [DEBUG] "},
              {valhalla::midgard::logging::LogLevel::TRACE, " [TRACE] "}};
const std::unordered_map<valhalla::midgard::logging::LogLevel, std::string, EnumHasher>
    colored{{valhalla::midgard::logging::LogLevel::ERROR, " \x1b[31;1m[ERROR]\x1b[0m "},
            {valhalla::midgard::logging::LogLevel::WARN, " \x1b[33;1m[WARN]\x1b[0m "},
            {valhalla::midgard::logging::LogLevel::INFO, " \x1b[32;1m[INFO]\x1b[0m "},
            {valhalla::midgard::logging::LogLevel::DEBUG, " \x1b[34;1m[DEBUG]\x1b[0m "},
            {valhalla::midgard::logging::LogLevel::TRACE, " \x1b[37;1m[TRACE]\x1b[0m "}};
#ifdef __ANDROID__
const std::unordered_map<valhalla::midgard::logging::LogLevel, android_LogPriority, EnumHasher>
    android_levels{{valhalla::midgard::logging::LogLevel::ERROR, ANDROID_LOG_ERROR},
                   {valhalla::midgard::logging::LogLevel::WARN, ANDROID_LOG_WARN},
                   {valhalla::midgard::logging::LogLevel::INFO, ANDROID_LOG_INFO},
                   {valhalla::midgard::logging::LogLevel::DEBUG, ANDROID_LOG_DEBUG},
                   {valhalla::midgard::logging::LogLevel::TRACE, ANDROID_LOG_VERBOSE}};
#endif

} // namespace

namespace valhalla {
namespace midgard {

namespace logging {

// a factory that can create loggers (that derive from 'logger') via function pointers
// this way you could make your own logger that sends log messages to who knows where
Logger* LoggerFactory::Produce(const LoggingConfig& config) const {
  // grab the type
  auto type = config.find("type");
  if (type == config.end()) {
    throw std::runtime_error("Logging factory configuration requires a type of logger");
  }
  // grab the logger
  auto found = find(type->second);
  if (found != end()) {
    return found->second(config);
  }
  // couldn't get a logger
  throw std::runtime_error("Couldn't produce logger for type: " + type->second);
}

// statically get a factory
LoggerFactory& GetFactory() {
  static LoggerFactory factory_singleton{};
  return factory_singleton;
}

// register your custom loggers here
bool RegisterLogger(const std::string& name, LoggerCreator function_ptr) {
  auto success = GetFactory().emplace(name, function_ptr);
  return success.second;
}

// logger base class, not pure virtual so you can use as a null logger if you want
Logger::Logger(const LoggingConfig& config){};
Logger::~Logger(){};
void Logger::Log(const std::string&, const LogLevel){};
void Logger::Log(const std::string&, const std::string&){};
bool logger_registered = RegisterLogger("", [](const LoggingConfig& config) {
  Logger* l = new Logger(config);
  return l;
});

// logger that writes to standard out
class StdOutLogger : public Logger {
public:
  StdOutLogger() = delete;
  StdOutLogger(const LoggingConfig& config)
      : Logger(config),
        levels(config.find("color") != config.end() && config.find("color")->second == "true"
                   ? colored
                   : uncolored) {
  }
  virtual void Log(const std::string& message, const LogLevel level) {
#ifdef __ANDROID__
    __android_log_print(android_levels.find(level)->second, "valhalla", "%s", message.c_str());
#else
    Log(message, levels.find(level)->second);
#endif
  }
  virtual void Log(const std::string& message, const std::string& custom_directive = " [TRACE] ") {
#ifdef __ANDROID__
    __android_log_print(ANDROID_LOG_INFO, "valhalla", "%s", message.c_str());
#else
    std::string output;
    output.reserve(message.length() + 64);
    output.append(TimeStamp());
    output.append(custom_directive);
    output.append(message);
    output.push_back('\n');
    // cout is thread safe, to avoid multiple threads interleaving on one line
    // though, we make sure to only call the << operator once on std::cout
    // otherwise the << operators from different threads could interleave
    // obviously we dont care if flushes interleave
    std::cout << output;
    std::cout.flush();
#endif
  }

protected:
  const std::unordered_map<LogLevel, std::string, EnumHasher> levels;
};
bool std_out_logger_registered = RegisterLogger("std_out", [](const LoggingConfig& config) {
  Logger* l = new StdOutLogger(config);
  return l;
});

class StdErrLogger : public StdOutLogger {
  using StdOutLogger::StdOutLogger;
  virtual void Log(const std::string& message, const std::string& custom_directive = " [TRACE] ") {
#ifdef __ANDROID__
    __android_log_print(ANDROID_LOG_ERROR, "valhalla", "%s", message.c_str());
#else
    std::string output;
    output.reserve(message.length() + 64);
    output.append(TimeStamp());
    output.append(custom_directive);
    output.append(message);
    output.push_back('\n');
    std::cerr << output;
    std::cerr.flush();
#endif
  }
};
bool std_err_logger_registered = RegisterLogger("std_err", [](const LoggingConfig& config) {
  Logger* l = new StdErrLogger(config);
  return l;
});

// TODO: add log rolling
// logger that writes to file
class FileLogger : public Logger {
public:
  FileLogger() = delete;
  FileLogger(const LoggingConfig& config) : Logger(config) {
    // grab the file name
    auto name = config.find("file_name");
    if (name == config.end()) {
      throw std::runtime_error("No output file provided to file logger");
    }
    file_name = name->second;

    // if we specify an interval
    reopen_interval = std::chrono::seconds(300);
    auto interval = config.find("reopen_interval");
    if (interval != config.end()) {
      try {
        reopen_interval = std::chrono::seconds(std::stoul(interval->second));
      } catch (...) {
        throw std::runtime_error(interval->second + " is not a valid reopen interval");
      }
    }

    // crack the file open
    ReOpen();
  }
  virtual void Log(const std::string& message, const LogLevel level) {
    Log(message, uncolored.find(level)->second);
  }
  virtual void Log(const std::string& message, const std::string& custom_directive = " [TRACE] ") {
    std::string output;
    output.reserve(message.length() + 64);
    output.append(TimeStamp());
    output.append(custom_directive);
    output.append(message);
    output.push_back('\n');
    lock.lock();
    file << output;
    file.flush();
    lock.unlock();
    ReOpen();
  }

protected:
  void ReOpen() {
    // TODO: use CLOCK_MONOTONIC_COARSE
    // check if it should be closed and reopened
    auto now = std::chrono::system_clock::now();
    lock.lock();
    if (now - last_reopen > reopen_interval) {
      last_reopen = now;
      try {
        file.close();
      } catch (...) {}
      try {
        file.open(file_name, std::ofstream::out | std::ofstream::app);
        last_reopen = std::chrono::system_clock::now();
      } catch (std::exception& e) {
        try {
          file.close();
        } catch (...) {}
        throw e;
      }
    }
    lock.unlock();
  }
  std::string file_name;
  std::ofstream file;
  std::chrono::seconds reopen_interval;
  std::chrono::system_clock::time_point last_reopen;
};
bool file_logger_registered = RegisterLogger("file", [](const LoggingConfig& config) {
  Logger* l = new FileLogger(config);
  return l;
});

} // namespace logging

// statically get a logger using the factory
logging::Logger& logging::GetLogger(const LoggingConfig& config) {
  static std::unique_ptr<Logger> singleton(GetFactory().Produce(config));
  return *singleton;
}

// statically log manually
void logging::Log(const std::string& message, const logging::LogLevel level) {
  GetLogger().Log(message, level);
}

// statically log manually
void logging::Log(const std::string& message, const std::string& custom_directive) {
  GetLogger().Log(message, custom_directive);
}

// statically configure logging
void logging::Configure(const LoggingConfig& config) {
  GetLogger(config);
}

} // namespace midgard
} // namespace valhalla
