#include "argparse_utils.h"
#include "mjolnir/add_predicted_speeds.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <filesystem>
#include <iostream>
#include <string>

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;
namespace bpt = boost::property_tree;

int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  std::array<float, vb::kBucketsPerWeek> speeds = {};
  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "get compressed encoded speeds.\n");
    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("s,speeds", "Speeds comma-separated", cxxopts::value<std::string>());

    // clang-format on
    options.parse_positional({"speeds"});
    options.positional_help("Comma-separated speeds (e.g., '0,10,20,30,...')");
    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, nullptr, "mjolnir.logging", true))
      return EXIT_SUCCESS;
    if (!result.count("speeds")) {
      std::cout << "You must provide speeds to get compressed encoded speeds.\n";
      return EXIT_FAILURE;
    }

    // parse speeds to float vector
    std::string speeds_str = result["speeds"].as<std::string>();
    std::stringstream ss(speeds_str);
    std::string speed;
    uint32_t i = 0;

    while (std::getline(ss, speed, ',') && i < vb::kBucketsPerWeek) {
      try {
        speeds[i++] = std::stof(speed);
      } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid speed value: " << speed << ". Please provide valid float values.\n";
        return EXIT_FAILURE;
      } catch (const std::out_of_range& e) {
        std::cerr << "Speed value out of range: " << speed
                  << ". Please provide valid float values.\n";
        return EXIT_FAILURE;
      } catch (const std::exception& e) {
        std::cerr << "Error parsing speed value: " << speed << ". " << e.what() << "\n";
        return EXIT_FAILURE;
      }
    }
    if (speeds.empty() || i != vb::kBucketsPerWeek) {
      std::cerr << "No valid speeds provided. Please provide " << vb::kBucketsPerWeek
                << " speed values.\n";
      return EXIT_FAILURE;
    }
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }
  // Prepare compressed speed buckets
  auto compressed_speed_buckets = vb::compress_speed_buckets(speeds.data());
  // Encode the compressed speeds to a base64 string
  auto encoded_speeds = vb::encode_compressed_speeds(compressed_speed_buckets.data());

  std::cout << encoded_speeds << std::endl;
  return EXIT_SUCCESS;
}