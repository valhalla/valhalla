
// The MIT License (MIT)
//
// Copyright (c) 2024 NNG LLC.
// Attila Kesmarki <attila.kesmarki@nng.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "config.h"
#include "baldr/rapidjson_utils.h"
#include <filesystem>
#include <mutex>

namespace {
struct config_singleton_t {
protected:
  boost::property_tree::ptree config_;

  config_singleton_t() = delete;

  config_singleton_t(const std::string& config_file_or_inline) {
    if (config_file_or_inline.empty()) {
      throw valhalla::ConfigUninitializedException();
    }

    try {
      if (std::filesystem::is_regular_file(config_file_or_inline)) {
        rapidjson::read_json(config_file_or_inline, config_);
      } else {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      }
    } catch (const std::filesystem::filesystem_error& e) {
      if (e.code() == std::errc::filename_too_long) {
        auto inline_config = std::stringstream(config_file_or_inline);
        rapidjson::read_json(inline_config, config_);
      } else {
        throw e;
      }
    }
  }

public:
  config_singleton_t(config_singleton_t const&) = delete;
  void operator=(const config_singleton_t&) = delete;
  friend const boost::property_tree::ptree&
  valhalla::config(const std::string& config_file_or_inline);
};
} // namespace

namespace valhalla {

const boost::property_tree::ptree& config(const std::string& config_file_or_inline) {
  static config_singleton_t instance(config_file_or_inline);
  return instance.config_;
}

}; // namespace valhalla
