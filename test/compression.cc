#include "baldr/compression_utils.h"

#include <string>

#include "test.h"

namespace {

int deflate_src(z_stream& s, std::string& data) {
  s.next_in = static_cast<Byte*>(static_cast<void*>(&data[0]));
  s.avail_in = static_cast<unsigned int>(data.size() * sizeof(std::string::value_type));
  return Z_FINISH;
}

void deflate_dst(z_stream& s, std::string& deflated) {
  // if the whole buffer wasn't used we are done
  auto size = deflated.size();
  if (s.total_out < size)
    deflated.resize(s.total_out);
  // we need more space
  else {
    // set the pointer to the next spot
    deflated.resize(size + 16);
    s.next_out = static_cast<Byte*>(static_cast<void*>(&deflated[0] + size));
    s.avail_out = 16;
  }
}

void inflate_src(z_stream& s, std::string& data) {
  s.next_in = static_cast<Byte*>(static_cast<void*>(&data[0]));
  s.avail_in = static_cast<unsigned int>(data.size() * sizeof(std::string::value_type));
}

int inflate_dst(z_stream& s, std::string& inflated) {
  // if the whole buffer wasn't used we are done
  auto size = inflated.size();
  if (s.total_out < size)
    inflated.resize(s.total_out);
  // we need more space
  else {
    // set the pointer to the next spot
    inflated.resize(size + 16);
    s.next_out = static_cast<Byte*>(static_cast<void*>(&inflated[0] + size));
    s.avail_out = 16;
  }
  return Z_NO_FLUSH;
}

TEST(Compression, roundtrip) {
  // deflate
  std::string message = "message in a gzipped bottle";
  std::string deflated;
  EXPECT_TRUE(
      valhalla::baldr::deflate(std::bind(deflate_src, std::placeholders::_1, std::ref(message)),
                               std::bind(deflate_dst, std::placeholders::_1, std::ref(deflated))))
      << "Can't write gzipped string";

  // inflate
  std::string inflated;
  EXPECT_TRUE(
      valhalla::baldr::inflate(std::bind(inflate_src, std::placeholders::_1, std::ref(deflated)),
                               std::bind(inflate_dst, std::placeholders::_1, std::ref(inflated))))
      << "failed to inflate string";

  // check the data
  EXPECT_EQ(inflated, "message in a gzipped bottle")
      << "decompressed doesn't match string before compression";
}

TEST(Compression, fail_deflate) {
  auto deflate_src_fail = []([[maybe_unused]] z_stream& s) -> int {
    throw std::runtime_error("you cant catch me");
    return Z_FINISH;
  };
  auto deflate_dst_fail = []([[maybe_unused]] z_stream& s) -> void {
    throw std::runtime_error("im the gingerbread man");
  };

  // deflate it
  std::string src = "who cares", dst;

  EXPECT_FALSE(valhalla::baldr::deflate(deflate_src_fail,
                                        std::bind(deflate_dst, std::placeholders::_1, std::ref(dst))))
      << "src should fail";

  EXPECT_FALSE(valhalla::baldr::deflate(std::bind(deflate_src, std::placeholders::_1, std::ref(src)),
                                        deflate_dst_fail))
      << "dst should fail";
}

TEST(Compression, fail_inflate) {
  auto inflate_src_fail = []([[maybe_unused]] z_stream& s) -> void {
    throw std::runtime_error("you cant catch me");
  };
  std::string bad = "this isn't gzipped";
  auto inflate_src_fail2 = [&bad](z_stream& s) -> void {
    s.next_in = static_cast<Byte*>(static_cast<void*>(&bad[0]));
    s.avail_in = static_cast<unsigned int>(bad.size() * sizeof(std::string::value_type));
  };
  auto inflate_src_fail3 = []([[maybe_unused]] z_stream& s) -> void {
    /* Nothing to do, simulates 'cannot inflate' - reproducible if no disk space. */
  };
  auto inflate_dst_fail = []([[maybe_unused]] z_stream& s) -> int {
    throw std::runtime_error("im the gingerbread man");
    return Z_NO_FLUSH;
  };

  // we do need some deflated stuff
  std::string message = "message in a gzipped bottle";
  std::string deflated;

  EXPECT_TRUE(
      valhalla::baldr::deflate(std::bind(deflate_src, std::placeholders::_1, std::ref(message)),
                               std::bind(deflate_dst, std::placeholders::_1, std::ref(deflated))))
      << "Can't write gzipped string";

  std::string inflated;
  EXPECT_FALSE(
      valhalla::baldr::inflate(inflate_src_fail,
                               std::bind(inflate_dst, std::placeholders::_1, std::ref(inflated))))
      << "src should fail";

  EXPECT_FALSE(
      valhalla::baldr::inflate(inflate_src_fail2,
                               std::bind(inflate_dst, std::placeholders::_1, std::ref(inflated))))
      << "src should fail";

  EXPECT_FALSE(
      valhalla::baldr::inflate(std::bind(inflate_src, std::placeholders::_1, std::ref(deflated)),
                               inflate_dst_fail))
      << "dst should fail";

  bool inflate_result{false};
  EXPECT_NO_THROW(
      inflate_result =
          valhalla::baldr::inflate(inflate_src_fail3, std::bind(inflate_dst, std::placeholders::_1,
                                                                std::ref(inflated))));
  EXPECT_FALSE(inflate_result);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
