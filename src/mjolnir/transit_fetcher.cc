#include <iostream>
#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <curl/curl.h>

#include <valhalla/midgard/logging.h>

struct logged_error_t: public std::runtime_error {
  logged_error_t(const std::string& msg):std::runtime_error(msg) {
    LOG_ERROR(msg);
  }
};

size_t write_callback(char *in, size_t size, size_t blocks, std::string *out) {
  if(!out) return static_cast<size_t>(0);
  out->append(in, size * blocks);
  return size * blocks;
}

struct curler_t {
  curler_t():connection(curl_easy_init(), [](CURL* c){curl_easy_cleanup(c);}) {
    if(connection.get() == nullptr)
      throw logged_error_t("Failed to created CURL connection");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_ERRORBUFFER, error), "Failed to set error buffer");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_FOLLOWLOCATION, 1L), "Failed to set redirect option ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEDATA, &result), "Failed to set write data ");
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_WRITEFUNCTION, write_callback), "Failed to set writer ");  }
  const std::string& operator()(const std::string& url) {
    result.clear();
    assert_curl(curl_easy_setopt(connection.get(), CURLOPT_URL, url.c_str()), "Failed to set URL ");
    assert_curl(curl_easy_perform(connection.get()), "Failed to fetch url");
    return result;
  }
protected:
  void assert_curl(CURLcode code, const std::string& msg){
    if(code != CURLE_OK)
      throw logged_error_t(msg + error);
  };
  std::shared_ptr<CURL> connection;
  char error[CURL_ERROR_SIZE];
  std::string result;
};

//TODO: object to keep information from various threads about
//stop pairs who leave a given tile and need filled in in a second pass
struct dangling_pair_t {

};

int main(int argc, char** argv) {
  if(argc < 2) {
    std::cerr << "Usage: " << std::string(argv[0]) << " conf/valhalla.json" << std::endl;
    return 1;
  }

  //config file
  std::string config_file(argv[1]);
  boost::property_tree::ptree config;
  boost::property_tree::read_json(config_file, config);

  //yes we want to curl
  curl_global_init(CURL_GLOBAL_DEFAULT);

  //TODO: go get information about what transit tiles we should be fetching

  //TODO: spawn a bunch of threads to download all the tiles
  //storing the daingling_pair information of each
  curl_global_cleanup();

  //TODO: make a pass of all dangling_pairs to add back the information they
  //are missing

  //TODO: show some summary informant?
  return 0;
}
