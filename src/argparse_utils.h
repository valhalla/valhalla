#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <valhalla/baldr/rapidjson_utils.h>
#include <valhalla/config.h>
#include <valhalla/filesystem.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/util.h>

/**
 * Parses common command line arguments across executables. It
 * - alters the config ptree and sets the concurrency config, where it favors the command line arg,
 * then falls back to the config and finally to all threads
 * - sets the logging configuration
 *
 * @param program The executable's name
 * @param opts    The command line options
 * @param result  The parsed result
 * @param config  The config which will be populated here
 * @param log     The logging config node's key. If empty, logging will not be configured.
 * @param use_threads Whether this program multi-threads
 * @param extra_help Optional function pointer to print more stuff to the end of the help message.
 *
 * @returns true if the program should continue, false if we should EXIT_SUCCESS
 * @throws cxxopts::exceptions::exception Thrown if there's no valid configuration
 */
bool parse_common_args(const std::string& program,
                       const cxxopts::Options& opts,
                       const cxxopts::ParseResult& result,
                       boost::property_tree::ptree& conf,
                       const std::string& log,
                       const bool use_threads = false,
                       std::function<void()> extra_help = nullptr) {
  if (result.count("help")) {
    std::cout << opts.help() << "\n";
    if (extra_help) {
      extra_help();
    };
    return false;
  }

  if (result.count("version")) {
    std::cout << std::string(program) << " " << VALHALLA_VERSION << "\n";
    return false;
  }

  // Read the config file
  if (result.count("inline-config")) {
    conf = valhalla::config(result["inline-config"].as<std::string>());
  } else if (result.count("config") &&
             filesystem::is_regular_file(result["config"].as<std::string>())) {
    conf = valhalla::config(result["config"].as<std::string>());
  } else {
    throw cxxopts::exceptions::exception("Configuration is required\n\n" + opts.help() + "\n\n");
  }

  // configure logging
  auto logging_subtree = conf.get_child_optional(log);
  if (!log.empty() && logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  if (use_threads) {
    // override concurrency config if specified as arg
    auto num_threads = std::max(1U, result.count("concurrency")
                                        ? result["concurrency"].as<uint32_t>()
                                        : conf.get<uint32_t>("mjolnir.concurrency",
                                                             std::thread::hardware_concurrency()));
    conf.put<uint32_t>("mjolnir.concurrency", num_threads);

    LOG_INFO("Running " + std::string(program) + " with " + std::to_string(num_threads) +
             " thread(s).");
  }

  return true;
}
