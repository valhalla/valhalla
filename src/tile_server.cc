#include "valhalla/tile_server.h"

#include "valhalla/filesystem.h"

#include "baldr/compression_utils.h"

#include <prime_server/http_protocol.hpp>
#include <prime_server/http_util.hpp>
#include <prime_server/prime_server.hpp>

#include <chrono>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <thread>

using namespace prime_server;

namespace {
std::string gzip(std::string& uncompressed) {
  auto deflate_src = [&uncompressed](z_stream& s) {
    s.next_in = static_cast<Byte*>(static_cast<void*>(&uncompressed[0]));
    s.avail_in = static_cast<unsigned int>(uncompressed.size() * sizeof(std::string::value_type));
    return Z_FINISH;
  };

  std::string compressed;
  auto deflate_dst = [&compressed](z_stream& s) {
    // if the whole buffer wasn't used we are done
    auto size = compressed.size();
    if (s.total_out < size)
      compressed.resize(s.total_out);
    // we need more space
    else {
      // set the pointer to the next spot
      compressed.resize(size + 16);
      s.next_out = static_cast<Byte*>(static_cast<void*>(&compressed[0] + size));
      s.avail_out = 16;
    }
  };

  if (!valhalla::baldr::deflate(deflate_src, deflate_dst))
    throw std::logic_error("Can't write gzipped string");

  return compressed;
}

std::string extract_file_path_from_request(const std::string& request_path) {
  // request path format /route-tile/vXXX/%id
  size_t pos = 0;
  for (size_t i = 0; i < 3; ++i) {
    pos = request_path.find('/', pos) + 1;
  }
  return request_path.substr(pos);
}

worker_t::result_t disk_work(const std::list<zmq::message_t>& job,
                             void* request_info,
                             worker_t::interrupt_function_t&,
                             const std::string& tile_source_dir) {
  worker_t::result_t result{false, std::list<std::string>(), ""};
  auto* info = static_cast<http_request_info_t*>(request_info);
  try {
    // parse request
    const auto request =
        http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

    auto encoding_it = request.headers.find("Accept-Encoding");
    auto gz = encoding_it != request.headers.end() && encoding_it->second == "gzip";

    auto path = extract_file_path_from_request(request.path);
    // load the file and gzip it if we have to
    std::string full_path = tile_source_dir + (filesystem::path::preferred_separator + path);
    std::fstream input(full_path, std::ios::in | std::ios::binary);
    if (input) {
      std::string buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
      if (gz) {
        buffer = gzip(buffer);
      }
      http_response_t response(200, "OK", buffer,
                               headers_t{{"Content-Encoding", gz ? "gzip" : "identity"}});
      response.from_info(*info);
      result.messages = {response.to_string()};
    }
  } catch (const std::exception& e) {
    http_response_t response(400, "Bad Request", e.what());
    response.from_info(*static_cast<http_request_info_t*>(request_info));
    result.messages = {response.to_string()};
  }

  // 404 if its not there
  if (result.messages.empty()) {
    http_response_t response(404, "Not Found", "Not Found");
    response.from_info(*info);
    result.messages = {response.to_string()};
  }
  return result;
}

} // namespace

namespace valhalla {
// static
void test_tile_server_t::start(const std::string& tile_source_dir, zmq::context_t& context) {
  // change these to tcp://known.ip.address.with:port if you want to do this across machines
  std::string result_endpoint{"ipc:///tmp/http_test_result_endpoint" + m_url};
  std::string request_interrupt{"ipc:///tmp/http_test_request_interrupt" + m_url};
  std::string proxy_endpoint{"ipc:///tmp/http_test_proxy_endpoint" + m_url};
  // server
  std::thread server(std::bind(&http_server_t::serve,
                               http_server_t(context, "tcp://" + m_url, proxy_endpoint + "_upstream",
                                             result_endpoint, request_interrupt, false)));
  server.detach();

  // load balancer for file serving
  std::thread file_proxy(std::bind(&proxy_t::forward, proxy_t(context, proxy_endpoint + "_upstream",
                                                              proxy_endpoint + "_downstream")));
  file_proxy.detach();

  // file serving thread
  std::thread file_worker(
      std::bind(&worker_t::work,
                worker_t(context, proxy_endpoint + "_downstream", "ipc:///dev/null", result_endpoint,
                         request_interrupt,
                         std::bind(&disk_work, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3, tile_source_dir))));
  file_worker.detach();

  std::this_thread::sleep_for(std::chrono::seconds(1));
}
} // namespace valhalla
