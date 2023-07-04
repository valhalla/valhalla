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
 * @param pt      The config which will be populated here
 * @param log     The logging config node's key
 * @param use_threads Whether this program multi-threads
 *
 * @returns true if the program should continue, false if we should EXIT_SUCCESS
 * @throws cxxopts::OptionException Thrown if there's no valid configuration
 */
bool parse_common_args(const std::string& program,
                       const cxxopts::Options& opts,
                       const cxxopts::ParseResult& result,
                       boost::property_tree::ptree& pt,
                       const std::string& log,
                       const bool use_threads = false) {
  if (result.count("help")) {
    std::cout << opts.help() << "\n";
    return false;
  }

  if (result.count("version")) {
    std::cout << std::string(program) << " " << VALHALLA_VERSION << "\n";
    return false;
  }

  // Read the config file
  if (result.count("inline-config")) {
    std::stringstream ss;
    ss << result["inline-config"].as<std::string>();
    rapidjson::read_json(ss, pt);
  } else if (result.count("config") &&
             filesystem::is_regular_file(result["config"].as<std::string>())) {
    rapidjson::read_json(result["config"].as<std::string>(), pt);
  } else {
    throw cxxopts::OptionException("Configuration is required\n\n" + opts.help() + "\n\n");
  }

  // configure logging
  auto logging_subtree = pt.get_child_optional(log);
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  if (use_threads) {
    // override concurrency config if specified as arg
    auto num_threads = std::max(1U, result.count("concurrency")
                                        ? result["concurrency"].as<uint32_t>()
                                        : pt.get<uint32_t>("mjolnir.concurrency",
                                                           std::thread::hardware_concurrency()));
    pt.put<uint32_t>("mjolnir.concurrency", num_threads);

    LOG_INFO("Running " + std::string(program) + " with " + std::to_string(num_threads) +
             " thread(s).");
  }

  return true;
}
