#include "valhalla/midgard/util.h"
#include "test.h"

#include <boost/format.hpp>

using namespace std;
using namespace valhalla::midgard;

namespace {

//need ostream operators for some of these types
std::string to_string(const std::vector<PointLL>& points) {
  std::string out = "{";
  for(const auto& p : points) {
    out += (boost::format("{%1%, %2%}, ") % p.lng() % p.lat()).str();
  }
  out += "}";
  if(out.length() > 2)
    out.erase(out.end() - 3, out.end() - 1);
  return out;
}

void do_pair(const std::vector<PointLL>& points, const std::string& encoded) {
  auto enc_answer = encode(points);
  if(enc_answer != encoded)
    throw std::runtime_error("Simple polyline encoding failed. Got " + enc_answer + " but expected " + encoded);
  auto dec_answer = decode(encoded);
  if(dec_answer != points)
    throw std::runtime_error("Simple polyline decoding failed. Got " + to_string(dec_answer) + " but expected " + to_string(points));
  if(encode(decode(encoded)) != encoded)
    throw std::runtime_error("Nested polyline encoding of a decoding failed");
  if(decode(encode(points)) != points)
    throw std::runtime_error("Nested polyline decoding of an encoding failed");
}

void TestSimple() {
  do_pair({{40.49437, -76.60025}}, "p~orMy`dvF");
}

}

int main() {
  test::suite suite("encode_decode");

  // Test kilometer per degree longitude at a specified latitude
  suite.test(TEST_CASE(TestSimple));

  return suite.tear_down();
}
