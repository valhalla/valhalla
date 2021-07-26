#ifndef VALHALLA_MIDGARD_LOGGING_H_
#define VALHALLA_MIDGARD_LOGGING_H_

#include <mutex>
#include <string>
#include <unordered_map>

namespace valhalla {
namespace midgard {

namespace logging {

// a factory that can create loggers (that derive from 'logger') via function pointers
// this way you could make your own logger that sends log messages to who knows where
class Logger;
using LoggingConfig = std::unordered_map<std::string, std::string>;
using LoggerCreator = Logger* (*)(const LoggingConfig&);
class LoggerFactory : public std::unordered_map<std::string, LoggerCreator> {
public:
  Logger* Produce(const LoggingConfig& config) const;
};

// register your custom loggers here
bool RegisterLogger(const std::string& name, LoggerCreator function_ptr);

// the Log levels we support
enum class LogLevel : char { LogTrace, LogDebug, LogInfo, LogWarn, LogError };

// logger base class, not pure virtual so you can use as a null logger if you want
class Logger {
public:
  Logger() = delete;
  Logger(const LoggingConfig& config);
  virtual ~Logger();
  virtual void Log(const std::string&, const LogLevel);
  virtual void Log(const std::string&, const std::string& custom_directive = " [TRACE] ");

protected:
  std::mutex lock;
};

// statically get a logger using the factory
Logger& GetLogger(const LoggingConfig& config = {{"type", "std_out"}, {"color", "true"}});

// statically log manually without the macros below
void Log(const std::string&, const LogLevel);
void Log(const std::string&, const std::string& custom_directive = " [TRACE] ");

// statically configure logging
// try something like:
// logging::Configure({ {"type", "std_out"}, {"color", ""} })
// logging::Configure({ {"type", "file"}, {"file_name", "test.log"}, {"reopen_interval", "1"} })
void Configure(const LoggingConfig& config);

// guarding against redefinitions
#ifndef LOG_ERROR
#ifndef LOG_WARN
#ifndef LOG_INFO
#ifndef LOG_DEBUG
#ifndef LOG_TRACE

// convenience macros stand out when reading code
// default to seeing INFO and up if nothing was specified
#ifndef LOGGING_LEVEL_NONE
#ifndef LOGGING_LEVEL_ALL
#ifndef LOGGING_LEVEL_ERROR
#ifndef LOGGING_LEVEL_WARN
#ifndef LOGGING_LEVEL_INFO
#ifndef LOGGING_LEVEL_DEBUG
#ifndef LOGGING_LEVEL_TRACE
#define LOGGING_LEVEL_INFO
#endif
#endif
#endif
#endif
#endif
#endif
#endif
// mark all the stuff we should see
#ifndef LOGGING_LEVEL_NONE
#ifndef LOGGING_LEVEL_ERROR
#define LOGGING_LEVEL_ERROR
#ifndef LOGGING_LEVEL_WARN
#define LOGGING_LEVEL_WARN
#ifndef LOGGING_LEVEL_INFO
#define LOGGING_LEVEL_INFO
#ifndef LOGGING_LEVEL_DEBUG
#define LOGGING_LEVEL_DEBUG
#ifndef LOGGING_LEVEL_TRACE
#define LOGGING_LEVEL_TRACE
#endif
#endif
#endif
#endif
#endif
#endif
// no logging output
#ifdef LOGGING_LEVEL_NONE
#define LOG_ERROR(x)
#define LOG_WARN(x)
#define LOG_INFO(x)
#define LOG_DEBUG(x)
#define LOG_TRACE(x)
// all logging output
#elif defined(LOGGING_LEVEL_ALL)
#define LOG_ERROR(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogError)
#define LOG_WARN(x)                                                                                  \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogWarn)
#define LOG_INFO(x)                                                                                  \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogInfo)
#define LOG_DEBUG(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogDebug)
#define LOG_TRACE(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogTrace)
// some level and up
#else
#ifdef LOGGING_LEVEL_ERROR
#define LOG_ERROR(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogError)
#define LOGLN_ERROR(x)                                                                               \
  ::valhalla::midgard::logging::GetLogger().Log(std::string(__FILE__) + ": " +                       \
                                                    std::to_string(__LINE__) + ": " + x,             \
                                                ::valhalla::midgard::logging::LogLevel::LogError)
#else
#define LOG_ERROR(x)
#define LOGLN_ERROR(x)
#endif
#ifdef LOGGING_LEVEL_WARN
#define LOG_WARN(x)                                                                                  \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogWarn)
#define LOGLN_WARN(x)                                                                                \
  ::valhalla::midgard::logging::GetLogger().Log(std::string(__FILE__) + ":" +                        \
                                                    std::to_string(__LINE__) + ": " + x,             \
                                                ::valhalla::midgard::logging::LogLevel::LogWarn)
#else
#define LOG_WARN(x)
#define LOGLN_WARN(x)
#endif
#ifdef LOGGING_LEVEL_INFO
#define LOG_INFO(x)                                                                                  \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogInfo)
#define LOGLN_INFO(x)                                                                                \
  ::valhalla::midgard::logging::GetLogger().Log(std::string(__FILE__) + ":" +                        \
                                                    std::to_string(__LINE__) + ": " + x,             \
                                                ::valhalla::midgard::logging::LogLevel::LogInfo)
#else
#define LOG_INFO(x) ;
#define LOGLN_INFO(x) ;
#endif
#ifdef LOGGING_LEVEL_DEBUG
#define LOG_DEBUG(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::LogDebug)
#else
#define LOG_DEBUG(x)
#endif
#ifdef LOGGING_LEVEL_TRACE
#define LOG_TRACE(x)                                                                                 \
  ::valhalla::midgard::logging::GetLogger().Log(std::string(__FILE__) + ":" +                        \
                                                    std::to_string(__LINE__) + ": " + x,             \
                                                ::valhalla::midgard::logging::LogLevel::LogTrace)
#else
#define LOG_TRACE(x)
#endif
#endif

// guarding against redefinitions
#endif
#endif
#endif
#endif
#endif

} // namespace logging

} // namespace midgard
} // namespace valhalla

#endif
