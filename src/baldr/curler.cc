#include "baldr/curler.h"
#include "midgard/logging.h"
#include "midgard/util.h"

#include <memory>
#include <stdexcept>
#include <string>

#ifdef ENABLE_HTTP

#if defined(_MSC_VER) && !defined(NOGDI)
#define NOGDI // prevents winsock2.h drag in wingdi.h
#endif

#include <curl/curl.h>

#if defined(_MSC_VER) && defined(GetNameInfo)
#undef GetNameInfo // winsock2.h imports #define GetNameInfo which clashes with
                   // EdgeInfo::GetNameInfo
#endif

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

size_t write_callback(char* in, size_t block_size, size_t blocks, std::vector<char>* out) {
  if (!out) {
    return static_cast<size_t>(0);
  }
  out->insert(out->end(), in, in + (block_size * blocks));
  return block_size * blocks;
}

int progress_callback(void* data,
                      curl_off_t /*dltotal*/,
                      curl_off_t /*dlnow*/,
                      curl_off_t /*ultotal*/,
                      curl_off_t /*ulnow*/) {
  auto interrupt = static_cast<const valhalla::baldr::curler_t::interrupt_t*>(data);
  try {
    (*interrupt)();
  } catch (...) { return -1; }

  return 0;
}

} // namespace

namespace valhalla {
namespace baldr {

struct curler_t::pimpl_t {
  pimpl_t(const std::string& user_agent) : connection(init_curl()), user_agent(user_agent) {
    if (connection.get() == nullptr) {
      LOG_ERROR("Failed to created CURL connection");
      throw std::runtime_error("Failed to created CURL connection");
    }
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error),
                "Failed to set error buffer ");
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
  std::vector<char> fetch(const std::string& url,
                          long& http_code,
                          bool gzipped,
                          const curler_t::interrupt_t* interrupt) const {
    if (interrupt) {
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_XFERINFOFUNCTION, progress_callback),
                  "Failed to set custom progress callback ");
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_XFERINFODATA, interrupt),
                  "Failed to set custom progress data");
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_NOPROGRESS, 0L),
                  "Failed to turn the progress callback on ");
    }

    // use gzip compression in any case
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ACCEPT_ENCODING, "gzip"),
                "Failed to set content encoding header ");
    // Curler do uncompressing by default. So if user asks for compressed data,
    // we just disable default uncompressing
    if (gzipped) {
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_HTTP_CONTENT_DECODING, 0L),
                  "Failed to disable decoding ");
    }
    // set the user agent
    if (!user_agent.empty())
      assert_curl(curl_easy_setopt(connection.get(), CURLOPT_USERAGENT, user_agent.c_str()),
                  "Failed to set User-Agent ");
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

  void assert_curl(CURLcode code, const std::string& msg) const {
    if (code != CURLE_OK) {
      std::string what = msg + error;
      LOG_ERROR(what);
      throw std::runtime_error(what);
    }
  }

  std::shared_ptr<CURL> connection;
  char error[CURL_ERROR_SIZE]{};
  std::string user_agent;
};

curler_t::curler_t(const std::string& user_agent) : pimpl(new pimpl_t(user_agent)) {
}

std::vector<char> curler_t::operator()(const std::string& url,
                                       long& http_code,
                                       bool gzipped,
                                       const curler_t::interrupt_t* interrupt) const {
  return pimpl->fetch(url, http_code, gzipped, interrupt);
}

// curler_pool_t

curler_pool_t::curler_pool_t(const size_t pool_size, const std::string& user_agent)
    : size_(pool_size) {
  for (size_t i = 0; i < pool_size; ++i) {
    curlers_.emplace_back(user_agent);
  }
}

curler_t curler_pool_t::acquire() {
  std::unique_lock<std::mutex> guard(curler_pool_lock_);
  while (curlers_.empty()) {
    curler_pool_empty_cond_.wait(guard);
  }

  curler_t curler = std::move(curlers_.back());
  curlers_.pop_back();

  return curler;
}

void curler_pool_t::release(curler_t&& curler) {
  {
    std::lock_guard<std::mutex> guard(curler_pool_lock_);
    curlers_.emplace_back(std::move(curler));
  }
  curler_pool_empty_cond_.notify_one();
}

} // namespace baldr
} // namespace valhalla

#else

// if you dont build with CURL support we always error when you try to use it
namespace valhalla {
namespace baldr {
curler_t::curler_t(const std::string& user_agent) {
}
std::vector<char>
curler_t::operator()(const std::string&, long&, bool gzipped, const curler_t::interrupt_t*) const {
  LOG_ERROR("This version of libvalhalla was not built with CURL support");
  throw std::runtime_error("This version of libvalhalla was not built with CURL support");
}

curler_pool_t::curler_pool_t(const size_t pool_size, const std::string&) : size_(pool_size) {
}

curler_t curler_pool_t::acquire() {
  LOG_ERROR("This version of libvalhalla was not built with CURL support");
  throw std::runtime_error("This version of libvalhalla was not built with CURL support");
  return curler_t("");
}

void curler_pool_t::release(curler_t&&) {
  LOG_ERROR("This version of libvalhalla was not built with CURL support");
  throw std::runtime_error("This version of libvalhalla was not built with CURL support");
}

} // namespace baldr
} // namespace valhalla

#endif
