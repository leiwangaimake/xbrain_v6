/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ntrip_protocol.cc
 * Brief: NTRIP request / reply / chunked decoding (see ntrip_protocol.h)
 *
 * Description:
 * Status lines are matched on their status code, not their reason phrase:
 * casters disagree on the wording ("Unauthorized", "Unauthorised", none at
 * all), never on the number.
 */

#include "sensor/ntrip_protocol.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace sensor {

namespace {
const char* kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

bool starts_with(const std::string& s, const char* p) {
  return s.compare(0, std::char_traits<char>::length(p), p) == 0;
}

std::string lower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// The three-digit status code of "PROTO CODE reason", or -1.
int status_code(const std::string& line) {
  const std::size_t sp = line.find(' ');
  if (sp == std::string::npos || line.size() < sp + 4) return -1;
  int code = 0;
  for (std::size_t i = sp + 1; i < sp + 4; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(line[i]))) return -1;
    code = code * 10 + (line[i] - '0');
  }
  return code;
}

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
}  // namespace

std::string base64_encode(const std::string& in) {
  std::string out;
  std::size_t i = 0;
  while (i + 2 < in.size()) {
    const unsigned v = (static_cast<unsigned char>(in[i]) << 16) |
                       (static_cast<unsigned char>(in[i + 1]) << 8) |
                       static_cast<unsigned char>(in[i + 2]);
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
    out += kB64[(v >> 6) & 63];
    out += kB64[v & 63];
    i += 3;
  }
  const std::size_t rest = in.size() - i;
  if (rest == 1) {
    const unsigned v = static_cast<unsigned char>(in[i]) << 16;
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
    out += "==";
  } else if (rest == 2) {
    const unsigned v = (static_cast<unsigned char>(in[i]) << 16) |
                       (static_cast<unsigned char>(in[i + 1]) << 8);
    out += kB64[(v >> 18) & 63];
    out += kB64[(v >> 12) & 63];
    out += kB64[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

std::string build_ntrip_request(const NtripRequest& r) {
  const std::string auth = base64_encode(r.user + ":" + r.password);
  std::string q;
  if (r.version == 2) {
    q = "GET /" + r.mountpoint + " HTTP/1.1\r\n";
    q += "Host: " + r.host + ":" + std::to_string(r.port) + "\r\n";
    q += "Ntrip-Version: Ntrip/2.0\r\n";
    q += "User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n";
    q += "Authorization: Basic " + auth + "\r\n";
    q += "Connection: close\r\n";
  } else {
    q = "GET /" + r.mountpoint + " HTTP/1.0\r\n";
    q += "User-Agent: NTRIP xbrain-rtk-driver/1.0\r\n";
    q += "Authorization: Basic " + auth + "\r\n";
  }
  q += "\r\n";
  return q;
}

NtripReply parse_ntrip_reply(const std::string& buf, std::size_t max_header) {
  NtripReply r;
  const std::size_t eol = buf.find("\r\n");
  if (eol == std::string::npos) {
    r.kind = buf.size() > max_header ? NtripReplyKind::kError : NtripReplyKind::kIncomplete;
    r.status_line = buf.substr(0, std::min<std::size_t>(buf.size(), 80));
    return r;
  }
  r.status_line = buf.substr(0, eol);
  if (starts_with(r.status_line, "SOURCETABLE")) {
    r.kind = NtripReplyKind::kSourceTable;
    r.header_len = eol + 2;
    return r;
  }
  const int code = status_code(r.status_line);
  if (starts_with(r.status_line, "ICY ")) {
    // v1: data follows the status line directly; the header is that one line.
    r.header_len = eol + 2;
    r.kind = (code == 200) ? NtripReplyKind::kStream
           : (code == 401) ? NtripReplyKind::kUnauthorized
                           : NtripReplyKind::kError;
    return r;
  }
  if (!starts_with(r.status_line, "HTTP/")) {
    r.kind = NtripReplyKind::kError;
    r.header_len = eol + 2;
    return r;
  }
  if (code == 401) {
    r.kind = NtripReplyKind::kUnauthorized;
    r.header_len = eol + 2;
    return r;
  }
  if (code != 200) {
    r.kind = NtripReplyKind::kError;
    r.header_len = eol + 2;
    return r;
  }
  const std::size_t end = buf.find("\r\n\r\n");
  if (end == std::string::npos) {
    r.kind = buf.size() > max_header ? NtripReplyKind::kError : NtripReplyKind::kIncomplete;
    return r;
  }
  r.header_len = end + 4;
  const std::string head = lower(buf.substr(0, end));
  const bool chunked = head.find("\r\ntransfer-encoding: chunked") != std::string::npos ||
                       head.find("\r\ntransfer-encoding:chunked") != std::string::npos;
  r.kind = chunked ? NtripReplyKind::kStreamChunked : NtripReplyKind::kStream;
  return r;
}

bool ChunkedDecoder::feed(const char* data, std::size_t n, std::string* out) {
  for (std::size_t i = 0; i < n; ++i) {
    const char c = data[i];
    switch (state_) {
      case State::kSize: {
        const int h = hex_value(c);
        if (h >= 0) {
          if (++size_digits_ > 8) return false;          // absurd chunk size
          remaining_ = remaining_ * 16 + static_cast<std::size_t>(h);
        } else if (c == ';' && size_digits_ > 0) {
          state_ = State::kSizeExt;                       // chunk extension: ignored
        } else if (c == '\r' && size_digits_ > 0) {
          state_ = State::kSizeLf;
        } else {
          return false;
        }
        break;
      }
      case State::kSizeExt:
        if (c == '\r') state_ = State::kSizeLf;
        break;
      case State::kSizeLf:
        if (c != '\n') return false;
        if (remaining_ == 0) return false;                // final chunk: stream over
        state_ = State::kData;
        break;
      case State::kData: {
        const std::size_t take = std::min(remaining_, n - i);
        out->append(data + i, take);
        remaining_ -= take;
        i += take - 1;
        if (remaining_ == 0) state_ = State::kDataCr;
        break;
      }
      case State::kDataCr:
        if (c != '\r') return false;
        state_ = State::kDataLf;
        break;
      case State::kDataLf:
        if (c != '\n') return false;
        state_ = State::kSize;
        size_digits_ = 0;
        break;
    }
  }
  return true;
}

}  // namespace sensor
