#ifndef VALHALLA_BALDR_CURLER_H_
#define VALHALLA_BALDR_CURLER_H_

#if defined(_MSC_VER) && !defined(NOGDI)
#define NOGDI // prevents winsock2.h drag in wingdi.h
#endif

#include <curl/curl.h>

#if defined(_MSC_VER) && defined(GetNameInfo)
#undef GetNameInfo // winsock2.h imports #define GetNameInfo which clashes with
                   // EdgeInfo::GetNameInfo
#endif

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <valhalla/midgard/logging.h>
#include <vector>

namespace {

struct curl_singleton_t {
  curl_singleton_t() {
    curl_global_init(CURL_GLOBAL_DEFAULT);
  }
  ~curl_singleton_t() {
    curl_global_cleanup();
  }
};

static std::shared_ptr<CURL> init_curl() {
  static curl_singleton_t s;
  return std::shared_ptr<CURL>(curl_easy_init(), [](CURL* c) { curl_easy_cleanup(c); });
}

char ALL_ENCODINGS[] = "";

} // namespace

namespace valhalla {
namespace baldr {

struct curler_t {
  struct logged_error_t : public std::runtime_error {
    logged_error_t(const std::string& msg) : std::runtime_error(msg) {
      LOG_ERROR(msg);
    }
  };

  curler_t() : connection(init_curl()) {
    if (connection.get() == nullptr) {
      throw logged_error_t("Failed to created CURL connection");
    }
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error),
                "Failed to set error buffer");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_FOLLOWLOCATION, 1L),
                "Failed to set redirect option ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEFUNCTION, write_callback),
                "Failed to set writer ");
    // this is less secure but we'll worry about that later
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_SSL_VERIFYPEER, 0L),
                "Failed to disable peer verification ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_SSL_VERIFYHOST, 0L),
                "Failed to disable host verification ");
  }

  // TODO: retries?
  std::vector<char>
  operator()(const std::string& url, long& http_code, bool allow_compression = true) {
    // sending content encoding as "" makes curl automatically handle identity, gzip or deflate
    // sending nullptr tells curl to only accept identity (no decoding needed)
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ACCEPT_ENCODING,
                                 allow_compression ? ALL_ENCODINGS : nullptr),
                "Failed to set content encoding header ");
    // set the url
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    // set the location of the result
    std::vector<char> result;
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEDATA, &result),
                "Failed to set write data ");
    // get the url
    assert_curl(curl_easy_perform(connection.get()), "Failed to get URL ");
    // grab the return code
    curl_easy_getinfo(connection.get(), CURLINFO_RESPONSE_CODE, &http_code);
    // hand over the results
    return result;
  }

protected:
  void assert_curl(CURLcode code, const std::string& msg) {
    if (code != CURLE_OK) {
      throw logged_error_t(msg + error);
    }
  };

  static size_t write_callback(char* in, size_t block_size, size_t blocks, std::vector<char>* out) {
    if (!out) {
      return static_cast<size_t>(0);
    }
    out->insert(out->end(), in, in + (block_size * blocks));
    return block_size * blocks;
  }

  std::shared_ptr<CURL> connection;
  char error[CURL_ERROR_SIZE];
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_CURLER_H_
