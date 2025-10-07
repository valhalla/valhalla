#include "tile_server.h"
#include "baldr/compression_utils.h"
#include "filesystem_utils.h"
#include "midgard/util.h"

#include <prime_server/http_protocol.hpp>
#include <prime_server/prime_server.hpp>
#include <zlib.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>

using namespace prime_server;

namespace {
class HTTPAuthException : public std::runtime_error {
public:
  explicit HTTPAuthException(const std::string& msg) : std::runtime_error(msg) {
  }
};

std::string decode_basic_auth(const std::string& header) {
  const std::string prefix = "Basic ";
  if (header.rfind(prefix, 0) != 0) {
    throw std::runtime_error("Not a Basic auth header");
  }

  std::string b64 = header.substr(prefix.size());
  std::string decoded = valhalla::midgard::decode64(b64);

  if (decoded.find(':') == std::string::npos) {
    throw std::runtime_error("Invalid Basic auth credentials");
  }

  return decoded;
}

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
                             const std::string& tile_source_dir,
                             const std::string& tar_path,
                             const std::string& server_user_pw) {
  worker_t::result_t result{false, std::list<std::string>(), ""};
  auto* info = static_cast<http_request_info_t*>(request_info);
  try {
    // parse request
    const auto request =
        http_request_t::from_string(static_cast<const char*>(job.front().data()), job.front().size());

    // if there's an authorization header set, it better match the one that was passed in
    if (auto auth_header = request.headers.find("Authorization");
        auth_header != request.headers.end()) {
      const auto req_user_pw = decode_basic_auth(auth_header->second);
      if (req_user_pw != server_user_pw) {
        throw HTTPAuthException("Server's user_pw " + server_user_pw +
                                " doesn't match the request's user_pw " + req_user_pw);
      }
    }

    http_response_t response;
    // is it about plain tiles or a tar?
    if (request.path.rfind("/route-tile", 0) == 0) {
      auto encoding_it = request.headers.find("Accept-Encoding");
      auto gz = encoding_it != request.headers.end() && encoding_it->second == "gzip";

      auto path = extract_file_path_from_request(request.path);
      // load the file and gzip it if we have to
      std::filesystem::path full_path{tile_source_dir};
      full_path.append(path);
      std::ifstream input(full_path, std::ios::binary);
      if (input) {
        std::string buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (gz) {
          buffer = gzip(buffer);
        }
        response = http_response_t{200, "OK", buffer,
                                   headers_t{{"Content-Encoding", gz ? "gzip" : "identity"}}};
      } else {
        response = http_response_t{404, "Not Found", "Not Found"};
      }
    } else if (request.path.rfind("/route-tar", 0) == 0) {
      // for now we only need to support range requests
      auto range_header = request.headers.find("Range");
      if (range_header == request.headers.end()) {
        throw std::runtime_error("No Range header found to GET a tar file");
      }

      // extract start & end bytes from the header
      std::string prefix;
      int start_bytes, end_bytes;
      char dash;
      std::stringstream ss(range_header->second);
      std::getline(ss, prefix, '='); // discard "bytes"
      ss >> start_bytes >> dash >> end_bytes;

      // read the requested byte range from the file
      std::streamsize length = end_bytes - start_bytes + 1;
      std::ifstream file(tar_path, std::ios::binary);
      file.seekg(start_bytes);
      std::string contents(length, '\0');
      file.read(&contents[0], length);
      contents.resize(file.gcount());

      response = http_response_t{206, "OK", contents};
    }

    response.from_info(*info);
    result.messages = {response.to_string()};
  } catch (const HTTPAuthException& e) {
    http_response_t response(401, "Unauthorized", e.what());
    response.from_info(*info);
    result.messages = {response.to_string()};
  } catch (const std::exception& e) {
    http_response_t response(400, "Bad Request", e.what());
    response.from_info(*info);
    result.messages = {response.to_string()};
  }

  return result;
}

} // namespace

namespace valhalla {
// static
void test_tile_server_t::start(const std::string& tile_source_dir,
                               const std::string& tar_path,
                               zmq::context_t& context) {
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
                                   std::placeholders::_3, tile_source_dir, tar_path, m_user_pw))));
  file_worker.detach();

  std::this_thread::sleep_for(std::chrono::seconds(1));
}
} // namespace valhalla
