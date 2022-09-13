#include "cpp-statsd-client/../../tests/StatsdServer.hpp"
#include "test.h"
#include "worker.h"

#include <thread>

using namespace valhalla;

void mock(Statsd::StatsdServer& server, std::vector<std::string>& messages) {
  do {
    // Grab the messages that are waiting
    auto recvd = server.receive();

    // Split the messages on '\n'
    auto start = std::string::npos;
    do {
      // Keep this message
      auto end = recvd.find('\n', ++start);
      messages.emplace_back(recvd.substr(start, end));
      start = end;

      // Bail if we found the special quit message
      if (messages.back().find("DONE") != std::string::npos) {
        messages.pop_back();
        return;
      }
    } while (start != std::string::npos);
  } while (server.errorMessage().empty() && !messages.back().empty());
}

class test_worker_t : public service_worker_t {
public:
  test_worker_t(const boost::property_tree::ptree& config) : service_worker_t(config) {
  }
  valhalla_exception_t fail(unsigned code) {
    valhalla_exception_t e(code);
    if (!e.statsd_key.empty()) {
      Api request;
      serialize_error(e, request);
      service_worker_t::enqueue_statistics(request);
      service_worker_t::cleanup();
    }
    return e;
  }
  void stop_server() {
    Api request;
    auto* stat = request.mutable_info()->add_statistics();
    stat->set_key("DONE");
    stat->set_value(1);
    stat->set_type(count);
    service_worker_t::enqueue_statistics(request);
    service_worker_t::cleanup();
  }
  std::string service_name() const override {
    return "test";
  }
#ifdef HAVE_HTTP
  virtual prime_server::worker_t::result_t
  work(const std::list<zmq::message_t>&, void*, const std::function<void()>&) {
    throw std::runtime_error("We arent testing the work method directly");
  }
#endif
};

TEST(statsd, errors_and_warnings) {
  // start up a mock statsd server
  Statsd::StatsdServer mock_server;
  std::vector<std::string> messages, expected;
  std::thread server(mock, std::ref(mock_server), std::ref(messages));

  // make a testable worker that is configured to hit the mock server
  boost::property_tree::ptree config;
  config.put("statsd.host", "localhost");
  test_worker_t worker(config);

  // try all error and warns then signal quit for the server
  for (unsigned i = 100; i < 600; ++i) {
    auto e = worker.fail(i);
    if (e.statsd_key.empty())
      continue;
    expected.push_back(e.statsd_key);
  }

  // tell the server we are done and wait for it to quit
  worker.stop_server();
  server.join();

  // check that the round trip worked over udp
  // each message the fake server received should contain the statsd_key from the exception
  ASSERT_EQ(messages.size(), expected.size()) << "Wrong number of stats received";
  for (size_t i = 0; i < messages.size(); ++i) {
    ASSERT_NE(messages[i].find(expected[i]), std::string::npos)
        << "Could not find key " << expected[i] << " in stat " << messages[i];
  }
}
