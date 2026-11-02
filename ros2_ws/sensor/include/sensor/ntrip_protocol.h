/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ntrip_protocol.h
 * Brief: NTRIP v1/v2 client protocol pieces: request, reply header, chunked body
 *
 * Description:
 * The backup source (a commercial caster, 11 S3.2A) speaks NTRIP. This header
 * holds the protocol as pure functions so every reply shape can be tested
 * without a socket:
 *   - build_ntrip_request(): the GET for v1 (HTTP/1.0) or v2 (HTTP/1.1 with
 *     Ntrip-Version), Basic auth built in memory only (RTG-9: the password never
 *     leaves this string -- callers must not log the request);
 *   - parse_ntrip_reply(): classifies what the caster sent back. A v1 caster
 *     answers "ICY 200 OK" and starts streaming right after that line, possibly
 *     without a blank line, so for ICY the header ends at the first CRLF; HTTP
 *     replies end at the blank line. "SOURCETABLE 200 OK" means the mountpoint
 *     does not exist; 401 is a bad account. Both are told apart from a generic
 *     error because they call for a long back-off rather than a quick retry;
 *   - ChunkedDecoder: v2 casters may send Transfer-Encoding: chunked.
 *
 * Pure: no I/O, no clock.
 */

#ifndef SENSOR__NTRIP_PROTOCOL_H_
#define SENSOR__NTRIP_PROTOCOL_H_

#include <cstddef>
#include <string>

namespace sensor {

struct NtripRequest {
  int version;              // 1 or 2
  std::string host;
  int port;
  std::string mountpoint;
  std::string user;
  std::string password;     // RTG-9: never logged
};

// Standard base64 (RFC 4648) with '=' padding.
std::string base64_encode(const std::string& in);

// The full request bytes, ending with the blank line.
std::string build_ntrip_request(const NtripRequest& r);

enum class NtripReplyKind {
  kIncomplete,      // header not complete yet: read more
  kStream,          // 200: RTCM follows the header
  kStreamChunked,   // 200 with Transfer-Encoding: chunked
  kSourceTable,     // "SOURCETABLE 200 OK": mountpoint unknown
  kUnauthorized,    // 401: account rejected
  kError,           // anything else, or a header longer than max_header
};

struct NtripReply {
  NtripReplyKind kind = NtripReplyKind::kIncomplete;
  std::size_t header_len = 0;   // bytes of buf that belong to the header
  std::string status_line;      // first line without CRLF (safe to log: no credentials)
};

// Classify the reply accumulated so far in buf.
NtripReply parse_ntrip_reply(const std::string& buf, std::size_t max_header);

// Decoder for an HTTP/1.1 chunked body. feed() appends decoded payload to *out
// and returns false on a malformed stream or on the final zero-size chunk (the
// caster ended the stream) -- either way the connection is finished.
class ChunkedDecoder {
 public:
  bool feed(const char* data, std::size_t n, std::string* out);

 private:
  enum class State { kSize, kSizeExt, kSizeLf, kData, kDataCr, kDataLf };
  State state_ = State::kSize;
  std::size_t remaining_ = 0;
  std::size_t size_digits_ = 0;
};

}  // namespace sensor

#endif  // SENSOR__NTRIP_PROTOCOL_H_
