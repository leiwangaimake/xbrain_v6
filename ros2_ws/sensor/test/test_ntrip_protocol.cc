/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_ntrip_protocol.cc
 * Brief: Offline unit test for the NTRIP request / reply / chunked decoder
 *
 * Description:
 * Requests are compared byte for byte (a caster rejects a malformed GET with the
 * same 401 as a wrong password, which is the worst way to find a typo). Reply
 * classification covers every branch the network thread acts on differently:
 * stream, chunked stream, source table and 401 (long back-off), other errors,
 * a header split across reads, and a header that never ends.
 */

#include "sensor/ntrip_protocol.h"

#include <cstdio>
#include <string>

using sensor::base64_encode;
using sensor::build_ntrip_request;
using sensor::ChunkedDecoder;
using sensor::NtripReplyKind;
using sensor::NtripRequest;
using sensor::parse_ntrip_reply;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static void test_base64_vectors() {
  // RFC 4648 section 10.
  CHECK(base64_encode("") == "");
  CHECK(base64_encode("f") == "Zg==");
  CHECK(base64_encode("fo") == "Zm8=");
  CHECK(base64_encode("foo") == "Zm9v");
  CHECK(base64_encode("foob") == "Zm9vYg==");
  CHECK(base64_encode("fooba") == "Zm9vYmE=");
  CHECK(base64_encode("foobar") == "Zm9vYmFy");
}

static NtripRequest req(int version) {
  return NtripRequest{version, "rtk.example.com", 8002, "AUTO", "qxuser", "secret"};
}

static void test_request_v1() {
  CHECK(build_ntrip_request(req(1)) ==
        "GET /AUTO HTTP/1.0\r\n"
        "User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n"
        "Authorization: Basic cXh1c2VyOnNlY3JldA==\r\n"
        "\r\n");
}

static void test_request_v2() {
  CHECK(build_ntrip_request(req(2)) ==
        "GET /AUTO HTTP/1.1\r\n"
        "Host: rtk.example.com:8002\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n"
        "Authorization: Basic cXh1c2VyOnNlY3JldA==\r\n"
        "Connection: close\r\n"
        "\r\n");
}

static void test_reply_icy() {
  const std::string buf = std::string("ICY 200 OK\r\n") + "\xD3\x00\x13";
  const auto r = parse_ntrip_reply(buf, 4096);
  CHECK(r.kind == NtripReplyKind::kStream);
  CHECK(r.header_len == 12);                 // data starts right after the line
  CHECK(r.status_line == "ICY 200 OK");
}

static void test_reply_http_stream_and_chunked() {
  const std::string plain = "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\nDATA";
  auto r = parse_ntrip_reply(plain, 4096);
  CHECK(r.kind == NtripReplyKind::kStream);
  CHECK(plain.substr(r.header_len) == "DATA");
  const std::string chunked =
      "HTTP/1.1 200 OK\r\nTransfer-Encoding: Chunked\r\nCache-Control: no-store\r\n\r\n";
  r = parse_ntrip_reply(chunked, 4096);
  CHECK(r.kind == NtripReplyKind::kStreamChunked);
  CHECK(r.header_len == chunked.size());
}

static void test_reply_sourcetable_and_401() {
  CHECK(parse_ntrip_reply("SOURCETABLE 200 OK\r\nServer: x\r\n", 4096).kind ==
        NtripReplyKind::kSourceTable);
  CHECK(parse_ntrip_reply("HTTP/1.0 401 Unauthorized\r\n", 4096).kind ==
        NtripReplyKind::kUnauthorized);
  CHECK(parse_ntrip_reply("HTTP/1.1 401\r\n", 4096).kind == NtripReplyKind::kUnauthorized);
  CHECK(parse_ntrip_reply("ICY 401 Unauthorised\r\n", 4096).kind ==
        NtripReplyKind::kUnauthorized);
  CHECK(parse_ntrip_reply("HTTP/1.1 404 Not Found\r\n", 4096).kind == NtripReplyKind::kError);
  CHECK(parse_ntrip_reply("garbage\r\n", 4096).kind == NtripReplyKind::kError);
}

static void test_reply_incomplete_and_too_long() {
  CHECK(parse_ntrip_reply("ICY 20", 4096).kind == NtripReplyKind::kIncomplete);
  CHECK(parse_ntrip_reply("HTTP/1.1 200 OK\r\nServer: x\r\n", 4096).kind ==
        NtripReplyKind::kIncomplete);         // 200 but no blank line yet
  const std::string endless = "HTTP/1.1 200 OK\r\n" + std::string(5000, 'x');
  CHECK(parse_ntrip_reply(endless, 4096).kind == NtripReplyKind::kError);
  CHECK(parse_ntrip_reply(std::string(5000, 'y'), 4096).kind == NtripReplyKind::kError);
}

static void test_chunked_decoder() {
  ChunkedDecoder d;
  std::string out;
  const std::string body = "5\r\nhello\r\n6;ext=1\r\n world\r\n";
  // Byte by byte: chunk boundaries never line up with reads in practice.
  for (char c : body) CHECK(d.feed(&c, 1, &out));
  CHECK(out == "hello world");
  CHECK(d.feed("A\r\n0123456789\r\n", 15, &out));
  CHECK(out == "hello world0123456789");
  CHECK(!d.feed("0\r\n", 3, &out));            // final chunk: the stream is over
  ChunkedDecoder bad;
  std::string o2;
  CHECK(!bad.feed("zz\r\n", 4, &o2));
  ChunkedDecoder bad2;
  CHECK(!bad2.feed("3\r\nabcX", 7, &o2));       // missing CRLF after the data
}

int main() {
  test_base64_vectors();
  test_request_v1();
  test_request_v2();
  test_reply_icy();
  test_reply_http_stream_and_chunked();
  test_reply_sourcetable_and_401();
  test_reply_incomplete_and_too_long();
  test_chunked_decoder();
  if (g_failures == 0) {
    std::printf("ALL NTRIP PROTOCOL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d NTRIP PROTOCOL TEST(S) FAILED\n", g_failures);
  return 1;
}
