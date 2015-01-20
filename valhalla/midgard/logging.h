#include <string>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <ctime>
#include <cstdlib>

namespace valhalla {
namespace midgard {
namespace logging {

//a factory that can create Loggers (that derive from 'Logger') via function pointers
//this way you could make your own Logger that sends log messages to who knows where
class Logger;
using LoggerCreator = Logger *(*)(const std::string &);
class LoggerFactory : 
    public std::unordered_map<std::string, LoggerCreator> {
 public:
  Logger* produce(const std::string& config) const {
    auto found = find(config.substr(0, config.find_first_of(',')));
    if(found != end()) {
      return found->second(config);
    }
    throw std::runtime_error("Couldn't produce Logger from: " + config);
  }
};

static LoggerFactory& GetFactory() {
  static LoggerFactory factory_singleton{};
  return factory_singleton;
}

//register your custom Loggers here
static bool RegisterLogger(const std::string& name, LoggerCreator function_ptr) {
  auto success = GetFactory().emplace(name, function_ptr);
  return success.second;
}

template <class T>
T atoT(const std::string& input)
{
  std::istringstream stream(input);
  T output;
  stream >> std::ws >> output >> std::ws;
  if(!stream.eof())
    throw std::runtime_error("Couldn't convert string to integral type");
  return output; 
}

//returns formated to: 'year/mo/dy hr:mn:sc.xxxxxx'
std::string TimeStamp() {
  //get the time
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(tp);
  std::tm gmt = {0}; gmtime_r(&tt, &gmt);
  using sec_t = std::chrono::duration<double>;
  std::chrono::duration<double> fractional_seconds = 
    (tp - std::chrono::system_clock::from_time_t(tt)) + std::chrono::seconds(gmt.tm_sec);
  //format the string
  std::string buffer("year/mo/dy hr:mn:sc.xxxxxx");
  sprintf(&buffer.front(), "%04d/%02d/%02d %02d:%02d:%09f.6", gmt.tm_year + 1900, gmt.tm_mon + 1,
    gmt.tm_mday, gmt.tm_hour, gmt.tm_min, fractional_seconds.count());
  return buffer;
}

//the log levels we support
enum class LogLevel : char { TRACE, DEBUG, INFO, WARN, ERROR };
struct EnumHasher { template <typename T> std::size_t operator()(T t) const { return static_cast<std::size_t>(t); } };

//returns standard message format: 'timestamp [LOG LEVEL]: message'
std::string StandardMessage(const std::string& message, const LogLevel level) {
  std::string output;
  output.reserve(message.length() + 64);
  output.append(TimeStamp());
  switch(level) {
    case LogLevel::ERROR:
      output.append(" [ERROR]: ");
      break;
    case LogLevel::WARN:
      output.append(" [WARN]: ");
      break;
    case LogLevel::INFO:
      output.append(" [INFO]: ");
      break;
    case LogLevel::DEBUG:
      output.append(" [DEBUG]: ");
      break;
    case LogLevel::TRACE:
      output.append(" [TRACE]: ");
      break;
  }
  output.append(message);
  output.push_back('\n');
  return output;
}

//Logger base class, not pure virtual so you can use it has a null Logger if you want
class Logger {
 public:
  Logger() = delete;
  Logger(const std::string& config) {};
  virtual ~Logger() {};
  virtual void Log(const std::string&, const LogLevel) {};
 protected:
  std::mutex lock;
};
static bool logger_registered = 
  RegisterLogger("", [](const std::string& config){Logger* l = new Logger(config); return l;});

//Logger that writes to standard out
class StdOutLogger : public Logger {
 public:
  StdOutLogger() = delete;
  StdOutLogger(const std::string& config):Logger(config),levels(GetLogDirectives(config)) {}
  virtual void Log(const std::string& message, const LogLevel level) {
    std::string output;
    output.reserve(message.length() + 64);
    output.append(TimeStamp());
    output.append(levels.find(level)->second);
    output.append(message);
    output.push_back('\n');
    //cout is thread safe, to avoid multiple threads interleaving on one line
    //though, we make sure to only call the << operator once on std::cout
    //otherwise the << operators from different threads could interleave
    std::cout << output;
  }
 protected:
  static std::unordered_map<LogLevel, std::string, EnumHasher> GetLogDirectives(const std::string& config) {
    //if it not colored don't use color
    auto pos = config.find_first_of(',');
    if(pos == std::string::npos || pos == config.length() - 1 || config.substr(pos + 1) != "color")
      return {{LogLevel::ERROR, " [ERROR] "}, {LogLevel::WARN, " [WARN] "}, {LogLevel::INFO, " [INFO] "}, 
        {LogLevel::DEBUG, " [DEBUG] "}, {LogLevel::TRACE, " [TRACE] "} };
    //use color
    return {{LogLevel::ERROR, " \x1b[31;1m[ERROR]\x1b[0m "}, {LogLevel::WARN, " \x1b[33;1m[WARN]\x1b[0m "}, 
      {LogLevel::INFO, " \x1b[32;1m[INFO]\x1b[0m "}, {LogLevel::DEBUG, " \x1b[34;1m[DEBUG]\x1b[0m "}, 
      {LogLevel::TRACE, " \x1b[37;1m[TRACE]\x1b[0m "} };
  }
  const std::unordered_map<LogLevel, std::string, EnumHasher> levels;
};
static bool std_out_logger_registered = 
  RegisterLogger("std_out", [](const std::string& config){Logger* l = new StdOutLogger(config); return l;});

//TODO: add reopen interval
//TODO: add log rolling
//Logger that writes to file
class FileLogger : public Logger {
 public:
  FileLogger() = delete;
  FileLogger(const std::string& config):Logger(config) {
    //grab the file name
    auto pos = config.find_first_of(',');
    if(pos == std::string::npos || pos == config.length() - 1)
      throw std::runtime_error("No output file provided to file Logger");
    file_name = config.substr(++pos);
    
    //if we specify an interval
    reopen_interval = std::chrono::seconds(10);
    pos = file_name.find_first_of(',', pos);
    if(pos != std::string::npos && pos != config.length() - 1)
    {
      auto interval = file_name.substr(pos + 1);
      file_name.resize(pos);
      try {        
        reopen_interval = std::chrono::seconds(atoT<int>(interval));
      }
      catch(...) {
        throw std::runtime_error(interval + " is not a valid reopen interval");
      }
    }
    
    //crack the file open
    ReOpen();
  }
  virtual ~FileLogger() {
    try{ file.close(); }catch(...){}
  }
  virtual void Log(const std::string& message, const LogLevel level) {
    std::string output(StandardMessage(message, level));
    lock.lock();
    file << output;
    lock.unlock();
    ReOpen();    
  }
 protected:
  void ReOpen() {
    //TODO: use CLOCK_MONOTONIC_COARSE
    //check if it should be closed and reopened
    auto now = std::chrono::system_clock::now();
    lock.lock();
    if(now - last_reopen > reopen_interval) {
      last_reopen = now;
      try{ file.close(); }catch(...){}
      try {
        file.open(file_name, std::ofstream::out | std::ofstream::app);
        last_reopen = std::chrono::system_clock::now();
      }
      catch(std::exception& e) {
        try{ file.close(); }catch(...){}
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
static bool file_logger_registered = 
  RegisterLogger("file", [](const std::string& config){Logger* l = new FileLogger(config); return l;});

static Logger& GetLogger(const std::string& config = "std_out,color") {
  static std::unique_ptr<Logger> singleton(GetFactory().produce(config));
  return *singleton;
}

static void Configure(const std::string& config) {
  GetLogger(config);
}

//convenience macros stand out when reading code
#ifndef LOGGING_DEBUG 
#define LOG_TRACE(x)
#define LOG_DEBUG(x)
#else
#define LOG_TRACE(x) ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::TRACE);
#define LOG_DEBUG(x) ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::DEBUG);
#endif
#define LOG_INFO(x)  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::INFO);
#define LOG_WARN(x)  ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::WARN);
#define LOG_ERROR(x) ::valhalla::midgard::logging::GetLogger().Log(x, ::valhalla::midgard::logging::LogLevel::ERROR);

}
}
}

#include <thread>
#include <future>
#include <vector>
#include <functional>

size_t work() {
  std::ostringstream s; s << "hi my name is: " << std::this_thread::get_id();
  
  for(size_t i  = 0; i < 2; ++i) {
    //std::async is pretty uninteresting unless you make things yield
    LOG_ERROR(s.str()); std::this_thread::sleep_for(std::chrono::milliseconds(500));
    LOG_WARN(s.str()); std::this_thread::sleep_for(std::chrono::milliseconds(500));
    LOG_INFO(s.str()); std::this_thread::sleep_for(std::chrono::milliseconds(500));
    LOG_DEBUG(s.str()); std::this_thread::sleep_for(std::chrono::milliseconds(500));
    LOG_TRACE(s.str()); std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return 10;
}

int main(void) {
  //configure logging, if you dont it defaults to standard out logging with colors
  //valhalla::midgard::logging::Configure("file,test.log,1");
  
  //start up some threads
  std::vector<std::future<size_t> > results;
  for(size_t i = 0; i < 4; ++i) {
    results.emplace_back(std::async(std::launch::async, work));
  }
  
  //dont really care about the results but we can pretend
  bool exit_code = 0;
  for(auto& result : results) {
    try {
      size_t count = result.get();
    }
    catch(std::exception& e) {
      std::cout << e.what();
      exit_code++;
    }
  }
  return exit_code;
}
