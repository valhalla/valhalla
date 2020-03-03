#ifndef VALHALLA_MIDGARD_LOGGING_H_
#define VALHALLA_MIDGARD_LOGGING_H_

// This file contains implementation of a highly configurable, thread-safe version of a logger.
// It can be use both in synchronous and asynchronous mode. If you use it in synchronous mode, all log
// operations are blocking, and non-blocking otherwise. Synchronous mode is used by default.
// Three standard log methods are presented: stdout, stderr and text file.
// Logger consists of the main Logger object and two auxiliary Processor and Worker objects. Logger
// is responsible for common operations, such as configuration and preparing messages for logging.
// Processor is responsible for the way of processing messages.
// Now there are two type of processors available: DefaultProcessor (blocking) and AsyncQueueProcessor
// (non-blocking).
// Workers are responsible for endpoint message operations (sending to some standard output, file,
// socket or just ignoring, it depends on worker implementation).
// Three types of workers are now available: StdOutWorker, StdErrWorker and TextFileWorker. Both
// Processor and Worker objects can be implemented by user and used to configure Logger from external
// code.
// Log messages are presented by Message objects (and its derivatives). The main goal of this object
// is to store copy of heteroheneous user data and serialize it right before endpoint operations. This
// is useful for asynchronous operations.
// StringMessage, CallableMessage and FormattedMessage classes are available now. StringMessage is
// used for simple strings. CallableMessage can be used for user defined serialization algorithms.
// FormattedMessage is used for formatted data. {fmt} formatting library
// (https://fmt.dev/latest/index.html) is used to format strings, so its syntax should be used for
// formatting.
// Every Message can be filled with set of Tags. Tags are objects that mark message with special
// string flags. These flags, for example, can be converted than to the leading string sequence,
// such as [DEBUG][ERROR][TIME] if tags were "DEBUG", "ERROR" and "TIME".
// Logger can be configured using configure() method. It has different implementations for different
// cases: configuration by LogginConfig object (for pre-registered workers) and inplace configuration
// using any callable object (both copyable and non-copyable) that has appropriate signature.
// Global method Configure() can be used to configure current Logger object.
// Logger has log() method that can be used to log some message. Global Log message and LOG_ERROR,
// LOG_WARN, LOG_INFO, LOG_DEBUG and LOG_TRACE macros. As well they propose formatting opportunities.
// Any object that has overloaded operator<< can be formatted. Detailed formatting API can be found
// at https://fmt.dev/latest/api.html
//
// Examples:
// Configure logger.
//   GetLogger().configure([](Message& message){ std::cout << message.serialize(); })
// Log simple string.
//   LOG_DEBUG("Some simple test string");
// Log formatting string with its data.
//   LOG_DEBUG("Some test string with formatting {} {} {} {}", "Test string", 123, 1.f,
//             std::string("Test string"));

#if defined(_MSC_VER) && defined(ERROR)
#undef ERROR
#endif

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#define FMT_HEADER_ONLY 1
#include "fmt/format.h"

namespace valhalla {
namespace midgard {

namespace logging {
using namespace std::chrono_literals;

// Logger configuration.
// Is used only to configure logger to use predefined workers with specified settings.
using LoggingConfig = std::unordered_map<std::string, std::string>;

// The Log levels we support.
enum class LogLevel : char { TRACE, DEBUG, INFO, WARN, ERROR };

// Logger tag.
// Is used to set additional properties for a message.
// For example, to specify that log message is related to Valhalla or some external library.
// If we use tags "DEBUG", "VALHALLA" and "CALL" with "meili::MapMatcher::costing()" message,
// it can be started with formatted sequence of tags [DEBUG][VALHALLA][CALL] and final result
// will look like "[DEBUG][VALHALLA][CALL]: meili::MapMatcher::costing()".
class Tag {
public:
  // Create tag form the string value and custom flag.
  explicit Tag(const std::string& value, bool custom = true) : value_(value), custom_(custom) {
  }

  // Return string representation.
  const std::string& str() const {
    return value_;
  }

  // Return if tag is custom.
  bool custom() const {
    return custom_;
  }

protected:
  // String value.
  std::string value_;
  // Custom flag. Is set if tag is not "ERROR", "WARN" or some other special.
  bool custom_;
};

// Container for logger tags.
class Tags {
public:
  Tags() = default;

  // Initialize with set of heterogeneous tags.
  template <class... Types> explicit Tags(Types&&... ids) {
    values_ = std::vector<Tag>{Tag(ids)...};
  }

  // Initialize with string value.
  explicit Tags(const std::string& value) : values_(std::vector<Tag>{Tag(value)}) {
  }

  // Initialize with set of tags.
  explicit Tags(const std::vector<Tag>& tags) : values_(tags) {
  }

  // Concatenate two tag sets.
  Tags& operator+=(const Tags& other) {
    values_.reserve(values_.size() + other.values_.size());
    values_.insert(values_.end(), other.values_.begin(), other.values_.end());
    return *this;
  }

  // Return as an std::string value.
  std::string str() const {
    std::stringstream ss;
    for (const auto& value : values_) {
      ss << '[' << value.str() << ']';
    }
    return ss.str();
  }

  // Return id set of user tags is empty.
  bool empty() const {
    return values_.empty();
  }

  // Return reference to the set of user tags.
  const std::vector<Tag>& tags() const {
    return values_;
  }

private:
  // Tags, represented as string values.
  std::vector<Tag> values_;
};

Tags operator+(Tags lhs, const Tags& rhs);

// Basic message class.
// Provides serialization interface for further use with text logger.
class Message {
public:
  Message() = default;
  // Initialize message with tags.
  explicit Message(const Tags& tags) : tags_(tags) {
  }

  virtual ~Message() = default;

  // Serialize message.
  virtual std::string serialize() const = 0;

  // Return tags related to message.
  const Tags& tags() const {
    return tags_;
  }

protected:
  // Set of tags that are used to set additional properties for a message.
  Tags tags_;
};

// Simple string message.
class StringMessage : public Message {
public:
  // Initialize from string.
  explicit StringMessage(const std::string& message) : message_(message) {
  }

  // Initialize from set of tags and string.
  StringMessage(const Tags& tags, const std::string& message) : Message(tags), message_(message) {
  }

  std::string serialize() const override {
    // Just return containing string.
    return message_;
  }

protected:
  // Text message.
  std::string message_;
};

// Auxiliary class to implement callable serializer.
// Can be used only as a base class for others.
class CallableMessage : public Message {
public:
  std::string serialize() const override {
    // Just return result of callable object.
    return callable_();
  }

protected:
  CallableMessage() = default;
  // Initialize from the set of user tags.
  explicit CallableMessage(const Tags& tags) : Message(tags) {
  }

  std::function<std::string()> callable_;
};

// Custom message.
// Can be used to implement non-standard user inline serializers.
// NOTE: CALLABLE MUST HAVE LIFETIME GREATED OR EQUAL TO LOGGER.
template <class CallableType, class... ParameterTypes> class CustomMessage : public CallableMessage {
public:
  // Create message from callable serializer and parameters.
  CustomMessage(CallableType callable, ParameterTypes&&... parameters) {
    // Creat callable with binded parameters for delayed serialization.
    callable_ = std::bind(callable, std::forward<ParameterTypes>(parameters)...);
  }

  // Create message from user tags, callable serializer and parameters.
  CustomMessage(const Tags& tags, CallableType callable, ParameterTypes&&... parameters)
      : CallableMessage(tags) {
    // Creat callable with binded parameters for delayed serialization.
    callable_ = std::bind(callable, std::forward<ParameterTypes>(parameters)...);
  }
};

// Formatted string message.
// Is useful for formatting messages with sstream serializable parameters.
template <class... ParameterTypes> class FormattedMessage : public CallableMessage {
public:
  // Create message from formatter string and its arguments.
  FormattedMessage(const std::string& message, ParameterTypes&&... parameters) {
    callable_ = [message, parameters...]() { return fmt::format(message, parameters...); };
  }

  // Create message from user tags, formatter string and its arguments.
  FormattedMessage(const Tags& tags, const std::string& message, ParameterTypes&&... parameters)
      : CallableMessage(tags) {
    callable_ = [message, parameters...]() { return fmt::format(message, parameters...); };
  }
};

// Logger configuration.
// Is used only to configure logger to use predefined workers with specified settings.
using LoggingConfig = std::unordered_map<std::string, std::string>;

// Container class for worker producer and its type identifier.
// Is used to register predefined workers by logger.
class Worker {
public:
  using Signature = void(const Message&);

  // Should be used only with further initialization.
  // std::bad_function_call is thrown otherwise.
  Worker() = default;

  // Initialize worker with type and producer.
  explicit Worker(const std::string& type,
                  const std::function<std::function<Signature>(const LoggingConfig&)>& producer)
      : type_(type), producer_(producer) {
  }

  // Initialize object with worker producer of the specified type (non-copyconstructible).
  template <class WorkerType,
            class = std::enable_if_t<!std::is_copy_constructible<std::decay_t<WorkerType>>::value>>
  explicit Worker(const std::string& type) : type_(type) {
    using Type = std::decay_t<WorkerType>;

    // Store non-copyconstructible object into internal shared pointer storage.
    producer_ = [](const LoggingConfig& config) {
      return std::bind([](const std::shared_ptr<Type>& pointer) { return (*pointer)(); },
                       std::make_shared<Type>(config));
    };
  }

  // Initialize object with worker producer of the specified type (copyconstructible).
  template <class WorkerType,
            std::enable_if_t<std::is_copy_constructible<std::decay_t<WorkerType>>::value, int> = 0>
  static Worker create(const std::string& type) {
    using Type = std::decay_t<WorkerType>;

    return Worker(type, [](const LoggingConfig& config) { return Type(config); });
  }

  // Initialize object with worker producer of the specified type (non-copyconstructible).
  template <class WorkerType,
            std::enable_if_t<!std::is_copy_constructible<std::decay_t<WorkerType>>::value, int> = 0>
  static Worker create(const std::string& type) {
    using Type = std::decay_t<WorkerType>;

    // Store non-copyconstructible object into internal shared pointer storage.
    return Worker(type, [](const LoggingConfig& config) {
      return std::bind([](const std::shared_ptr<Type>& pointer,
                          const Message& message) { return (*pointer)(message); },
                       std::make_shared<Type>(config), std::placeholders::_1);
    });
  }

  // Return worker type identifier.
  const auto& type() const {
    return type_;
  }

  // Produce and return worker in form of callable object.
  auto produce(const LoggingConfig& config) const {
    return producer_(config);
  }

private:
  // Worker type identifier.
  std::string type_;
  // Worker producer.
  std::function<std::function<Signature>(const LoggingConfig&)> producer_;
};

template <class Type> struct type { using nested = Type; };

// Basic Processor class. Is used by logger to implement queue processing in synchronous or
// asynchronous way.
class Processor {
public:
  virtual ~Processor() = default;

  virtual void push(std::unique_ptr<Message>&&) = 0;
};

// Default blocking processor. Process messages as soon as acquire them.
// Custom workers are used to write messages to their destination.
class DefaultProcessor : public Processor {
public:
  // Initialize object with specified worker.
  explicit DefaultProcessor(std::function<Worker::Signature> worker)
      : worker_proc_(std::move(worker)) {
  }

  // Process message.
  void push(std::unique_ptr<Message>&& message) override {
    worker_proc_(*message);
  }

private:
  // Worker procedure.
  std::function<Worker::Signature> worker_proc_;
};

// Asynchronous queue processor. Acquire, store and process messages.
// Simple list is used to store messages. Separate thread is used to process messages.
// Custom workers are used to write messages to their destination.
class AsyncQueueProcessor : public Processor {
public:
  // Initialize object with specified worker.
  explicit AsyncQueueProcessor(const std::function<Worker::Signature>& worker)
      : worker_proc_(worker) {
    // Create separate thread.
    worker_thread_ = std::thread([&]() {
      decltype(queue_) queue;
      std::unique_ptr<Message> message;
      while (true) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          state_.wait(lock, [&]() { return terminating_ || !queue_.empty(); });
          if (terminating_) {
            queue_.swap(queue);
            break;
          }

          message = std::move(queue_.front());
          queue_.pop();
        }
        worker_proc_(*message);
      }

      while (!queue.empty()) {
        worker_proc_(*std::move(queue.front()));
        queue.pop();
      }
    });
  }

  ~AsyncQueueProcessor() {
    // Stop worker and wait it finishes its jobs.
    stop();
  }

  // Add message to the queue.
  void push(std::unique_ptr<Message>&& message) override {
    {
      // Lock queue, push element and mark queue as nonempty.
      std::lock_guard<std::mutex> guard(mutex_);
      queue_.emplace(std::move(message));
    }
    // Notify worker.
    state_.notify_one();
  }

  // Clear queue.
  void clear() {
    decltype(queue_) queue;
    {
      // Lock queue, swap queue with clear one and mark it as empty.
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.swap(queue);
    }
  }

private:
  // Terminate worker and wait it finishes.
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (terminating_)
        return;
      terminating_ = true;
    }
    // Notify worker.
    state_.notify_one();
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

private:
  // Execution thread for the worker that writes messages to the specified file.
  std::thread worker_thread_;
  // Queue protection mutex. Is used both by condition variable and for manual locks.
  std::mutex mutex_;
  // Condition variable that is used to notify worker about state changes (elemend pushed or stop
  // request sended).
  std::condition_variable state_;
  // Flag that is used to request worker thread to stop its jobs.
  bool terminating_ = false;
  // Queue that is used to store elements.
  std::queue<std::unique_ptr<Message>> queue_;
  // Worker procedure.
  std::function<Worker::Signature> worker_proc_;
};

// Asyncronous thread-safe logger class.
// Logger is base on AsyncQueueProcessor that includes message queue and custom workers that process
// this queue. Default worker sends messages to the stdout device. Logger can be safely reconfigured
// at any time with LoggingConfig structure (in such case worker should be preregistered), or with any
// callable object with void(const std::string&) signature. In case when some non-copyconstructible,
// moveconstructible object is used as a worker, it is moved into internal storage.
class Logger {
public:
  Logger() = default;
  // Remove default copy/move constructors/assignment operators.
  Logger(const Logger&) = delete;
  Logger(Logger&&) = delete;
  Logger& operator=(const Logger&) = delete;
  Logger& operator=(Logger&&) = delete;

  // Special constructor to initialize logger with predefined workers.
  // TODO: check all types are same to Worker when move to C++17 (use std::enable_if,
  // std::conjunction, std::is_same, std::decay).
  template <class... WorkerTypes> explicit Logger(WorkerTypes&&... workers) {
    // Use dummy list initialization to unpack and register workers.
    bool dummy[] = {false, (registerWorker(std::forward<WorkerTypes>(workers)), false)...};
    // Try to configure using default stdout worker (done for backward compatibility).
    // In case if stdout is not registered configuration will be ignored.
    configure({{"type", "stdout"}, {"color", "true"}});
  }

  // Register worker using its string type identifier.
  void registerWorker(const Worker& worker) {
    // Lock registry (it can be used by configure method simultaneously).
    std::lock_guard<std::mutex> lock(registry_mutex_);
    // Check if worker has already been registered.
    auto it = registry_.find(worker.type());
    if (it != registry_.end())
      throw std::invalid_argument("Worker " + worker.type() + " already registered");
    // Copy worker to the registry.
    registry_.emplace(worker.type(), worker);
  }

  template <class Type1, class Type2>
  using enable_if_not_same_and_copyconstructible =
      std::enable_if_t<!std::is_same<Type1, Type2>::value &&
                           std::is_copy_constructible<std::decay_t<Type1>>::value,
                       int>;

  // Configure logger to use copyconstructible object as a worker.
  template <class WorkerType,
            class ProcessorType = DefaultProcessor,
            enable_if_not_same_and_copyconstructible<std::decay_t<WorkerType>, LoggingConfig> = 0>
  void configure(WorkerType&& worker, type<ProcessorType> = type<DefaultProcessor>()) {
    // Can be used to enqueue message asynchronously, so protect processor.
    std::lock_guard<std::mutex> lock(processor_mutex_);
    // First remove the old processor and wait it finishes all its jobs.
    processor_ = nullptr;
    // Create new one.
    processor_ = std::make_unique<ProcessorType>(std::forward<WorkerType>(worker));
  }

  template <class Type1, class Type2>
  using enable_if_not_same_and_non_copyconstructible =
      std::enable_if_t<!std::is_same<Type1, Type2>::value &&
                           !std::is_copy_constructible<std::decay_t<Type1>>::value,
                       int>;

  // Configure logger to use non-copyconstructible object as a worker.
  template <class WorkerType,
            class ProcessorType = DefaultProcessor,
            enable_if_not_same_and_non_copyconstructible<std::decay_t<WorkerType>, LoggingConfig> = 0>
  void configure(WorkerType&& worker, type<ProcessorType> = type<DefaultProcessor>()) {
    using Type = std::decay_t<WorkerType>;

    // Move non-copyconstructible object to shared pointer storage and bind it to caller lambda.
    configure(std::bind([](std::shared_ptr<Type> pointer,
                           const Message& message) { return (*pointer)(message); },
                        std::make_shared<Type>(std::move(worker)), std::placeholders::_1));
  }

  // Configure logger to use predefined worker with specified parameters.
  template <class ProcessorType = DefaultProcessor>
  void configure(const LoggingConfig& config, type<ProcessorType> = type<DefaultProcessor>()) {
    // Check if configuration has logger type identifier.
    auto it = config.find("type");
    if (it == config.end())
      throw std::invalid_argument("Invalid configuration data: field 'type' is missed");

    // Get type identifier.
    auto type = it->second;

    // Check if type is empty. It's common practice across valhalla.
    // Use of empty type disables logger.
    if (!type.empty()) {
      Worker worker;
      {
        // Lock registry (it can be used by registerWorker method simultaneously).
        std::lock_guard<std::mutex> lock(registry_mutex_);
        // Check if specified worked is presented in registry.
        auto it = registry_.find(type);
        if (it == registry_.end())
          throw std::invalid_argument("Worker of type '" + type + "' is not registered");
        // Get producer of the specified worker.
        worker = it->second;
      }

      // Produce worker and update it.
      {
        // Can be used to enqueue message asynchronously, so protect processor.
        std::lock_guard<std::mutex> lock(processor_mutex_);
        // First remove the old processor and wait it finishes all its jobs.
        processor_ = nullptr;
        // Create new one.
        processor_ = std::make_unique<ProcessorType>(worker.produce(config));
      }
    } else {
      reset();
    }
  }

  // Log simple string message with tags.
  void log(const Tags& tags, const std::string& message) {
    log(std::make_unique<StringMessage>(defaultTags() + tags, message));
  }

  // Log simple string message.
  void log(const std::string& message) {
    log(std::make_unique<StringMessage>(defaultTags(), message));
  }

  // Log formatted string message with tags.
  template <class... ParamTypes>
  void log(const Tags& tags, const std::string& message, ParamTypes&&... params) {
    log(std::make_unique<FormattedMessage<ParamTypes...>>(defaultTags() + tags, message,
                                                          std::forward<ParamTypes>(params)...));
  }

  // Log formatted string message.
  template <class... ParamTypes> void log(const std::string& message, ParamTypes&&... params) {
    log(std::make_unique<FormattedMessage<ParamTypes...>>(defaultTags(), message,
                                                          std::forward<ParamTypes>(params)...));
  }

  template <class CallableType, class... ParamTypes>
  using enable_if_convertible_t = std::enable_if_t<
      std::is_convertible<CallableType, std::function<std::string(ParamTypes...)>>::value>;

  // Log custom data using specified serializer.
  // Serializer can be any callable object that matches std::string(ParamTypes...) signature.
  template <class CallableType,
            class... ParamTypes,
            class = enable_if_convertible_t<CallableType, ParamTypes...>>
  void log(CallableType&& callable, ParamTypes&&... params) {
    log(std::make_unique<CustomMessage<CallableType, ParamTypes...>>(defaultTags(), callable,
                                                                     std::forward<ParamTypes>(
                                                                         params)...));
  }

  // Log custom data using tags and specified serializer.
  // Serializer can be any callable object that matches std::string(ParamTypes...) signature.
  template <class CallableType,
            class... ParamTypes,
            class = enable_if_convertible_t<CallableType, ParamTypes...>>
  void log(const Tags& tags, CallableType&& callable, ParamTypes&&... params) {
    log(std::make_unique<CustomMessage<CallableType, ParamTypes...>>(defaultTags() + tags, callable,
                                                                     std::forward<ParamTypes>(
                                                                         params)...));
  }

  // Reset processor to default (dummy) value.
  void reset() {
    std::lock_guard<std::mutex> lock(processor_mutex_);
    processor_ = std::make_unique<DefaultProcessor>([](const Message&) {});
  }

private:
  // Push message to log.
  void log(std::unique_ptr<Message>&&);

  // Default tags used to implicit log message modification.
  Tags defaultTags() const;

  // Thread-safe processor object.
  // Is used to store text messages asynchronously according to the configured worker.
  std::unique_ptr<Processor> processor_ = std::make_unique<DefaultProcessor>([](const Message&) {});
  // Is used to guard processor storage.
  std::mutex processor_mutex_;

  // Worker registry.
  // Is used to store predefined worker producers and initialize workers using LogginConfig parameter.
  std::unordered_map<std::string, Worker> registry_;
  // Is used to guard registry storage.
  std::mutex registry_mutex_;
};

// Statically get a logger.
Logger& GetLogger();

// statically configure logging
// try something like:
// logging::Configure({ {"type", "std_out"}, {"color", ""} })
// logging::Configure({ {"type", "file"}, {"file_name", "test.log"}, {"reopen_interval", "1"} })
void Configure(const LoggingConfig& config);

// returns level tag
const std::string& LevelTag(valhalla::midgard::logging::LogLevel level);

// statically log manually without the macros below
// statically log manually
template <class... Types> void Log(const logging::LogLevel level, Types&&... parameters) {
  GetLogger().log(Tags(Tag(LevelTag(level), false)), std::forward<Types>(parameters)...);
}

// statically log manually
template <class... Types, class = typename std::enable_if_t<sizeof...(Types) != 0>>
void Log(const std::string& custom_directive, Types&&... parameters) {
  GetLogger().log(Tags(custom_directive), std::forward<Types>(parameters)...);
}

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
#define LOG_ERROR(...)
#define LOG_WARN(...)
#define LOG_INFO(...)
#define LOG_DEBUG(...)
#define LOG_TRACE(...)
// all logging output
#elif defined(LOGGING_LEVEL_ALL)
#define LOG_ERROR(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::ERROR, __VA_ARGS__)
#define LOG_WARN(...)                                                                                \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::WARN, __VA_ARGS__)
#define LOG_INFO(...)                                                                                \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::INFO, __VA_ARGS__)
#define LOG_DEBUG(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::DEBUG, __VA_ARGS__)
#define LOG_TRACE(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::TRACE, __VA_ARGS__)
// some level and up
#else
#ifdef LOGGING_LEVEL_ERROR
#define LOG_ERROR(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::ERROR, __VA_ARGS__)
#define LOGLN_ERROR(...)                                                                             \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::ERROR,                   \
                                    std::string(__FILE__) + ": " + std::to_string(__LINE__) + ": " + \
                                        __VA_ARGS__)

#else
#define LOG_ERROR(...)
#define LOGLN_ERROR(...)
#endif
#ifdef LOGGING_LEVEL_WARN
#define LOG_WARN(...)                                                                                \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::WARN, __VA_ARGS__)
#define LOGLN_WARN(...)                                                                              \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::WARN,                    \
                                    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " +  \
                                        __VA_ARGS__)

#else
#define LOG_WARN(...)
#define LOGLN_WARN(...)
#endif
#ifdef LOGGING_LEVEL_INFO
#define LOG_INFO(...)                                                                                \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::INFO, __VA_ARGS__)
#define LOGLN_INFO(...)                                                                              \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::INFO,                    \
                                    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " +  \
                                        __VA_ARGS__)
#else
#define LOG_INFO(...) ;
#define LOGLN_INFO(...) ;
#endif
#ifdef LOGGING_LEVEL_DEBUG
#define LOG_DEBUG(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::DEBUG, __VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif
#ifdef LOGGING_LEVEL_TRACE
#define LOG_TRACE(...)                                                                               \
  ::valhalla::midgard::logging::Log(::valhalla::midgard::logging::LogLevel::TRACE,                   \
                                    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " +  \
                                        __VA_ARGS__)
#else
#define LOG_TRACE(...)
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
