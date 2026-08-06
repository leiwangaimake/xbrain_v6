/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: canonical_digest.h
 * Brief: C++17 side of common_digest and FenceSet crc32, byte-equal to Python
 *
 * Description:
 * What this is for. 10 S5.4.4 and 11 S9A.2 both require their fingerprint to
 * come out identical from a Python implementation and a C++ one. This header is
 * the C++ half; xbrain/common/digest/ is the Python half, and
 * tests/common/test_digest_cross_language.py compiles this file and compares the
 * two against one shared set of golden vectors.
 *
 * Why header-only. Its consumers are chassis_relay, quadruped, perception and
 * rtk_driver. CLAUDE.md 5.3 forbids anything in common/ from depending on rclcpp
 * or any ROS type, precisely so those four can use it, and a header with no link
 * step is the cheapest way to keep that true. Nothing here is outside the C++17
 * standard library -- no C++20 features, per CPP-1, and no distribution
 * detection macros, per PB-5, because the humble/jazzy baseline D-45 is still
 * unresolved.
 *
 * What is implemented here and why not a library:
 *   * SHA-256, about eighty lines, from FIPS 180-4
 *   * CRC-32, IEEE 802.3, computed bitwise
 * Both are small and fully specified. Pulling in OpenSSL or zlib to get them
 * would add a link dependency to every consumer, including one on the
 * emergency-stop path, in exchange for code that fits on one screen.
 *
 * Both fingerprints allocate, through std::string. That is fine where they are
 * used: the config digest is computed once at the freeze line and the fence
 * crc32 at stage/commit, never inside the 20 Hz loop and never inside the relay
 * hop that CRL-5 budgets at 200 microseconds.
 *
 * The one trap worth stating up front. The number rendering rules are not
 * decoration. An integral double must print as "1" and not "1.0", keys must sort
 * by UTF-8 BYTES and not by wide-character value, and hard_enforce must be
 * "1"/"0" and not "true"/"false". Each of those is a place where a natural C++
 * spelling and a natural Python spelling differ, and the resulting mismatch
 * shows up on the robot as a consumer rejecting every frame -- never as a failing
 * unit test in either language alone.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_DIGEST_CANONICAL_DIGEST_H_
#define HACHIST_XBRAIN_V6_COMMON_DIGEST_CANONICAL_DIGEST_H_

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hachist {
namespace xbrain {
namespace digest {

// ---------------------------------------------------------------------------
// SHA-256, FIPS 180-4. Straight from the specification with no optimisation:
// this runs once per boot, and a clear implementation is worth more here than a
// fast one, because the only way it can fail is by disagreeing with Python.
// ---------------------------------------------------------------------------

namespace detail {

inline uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

// The first thirty-two bits of the fractional parts of the cube roots of the
// first sixty-four primes. Constant, so a typo here is caught by the very first
// golden vector rather than by inspection.
inline const uint32_t* Sha256K() {
  static const uint32_t k[64] = {
      0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
      0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
      0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
      0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
      0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
      0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
      0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
      0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
      0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
      0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
      0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
      0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
      0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
  return k;
}

}  // namespace detail

/* Hexadecimal SHA-256 of an arbitrary byte string, lower case. */
inline std::string Sha256Hex(const std::string& data) {
  uint32_t h[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                   0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

  // Padding: one 0x80 byte, then zeros, then the length in BITS as a 64-bit
  // big-endian value. Bits, not bytes -- the commonest way to get SHA-256 subtly
  // wrong, and one that still produces a plausible-looking hash.
  std::string msg = data;
  const uint64_t bit_len = static_cast<uint64_t>(data.size()) * 8u;
  msg.push_back(static_cast<char>(0x80));
  while (msg.size() % 64 != 56) msg.push_back(static_cast<char>(0x00));
  for (int i = 7; i >= 0; --i) {
    msg.push_back(static_cast<char>((bit_len >> (i * 8)) & 0xFFu));
  }

  for (size_t off = 0; off < msg.size(); off += 64) {
    const unsigned char* block =
        reinterpret_cast<const unsigned char*>(msg.data()) + off;
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
             (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
             (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
             static_cast<uint32_t>(block[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = detail::Rotr(w[i - 15], 7) ^
                          detail::Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = detail::Rotr(w[i - 2], 17) ^
                          detail::Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
    uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 =
          detail::Rotr(e, 6) ^ detail::Rotr(e, 11) ^ detail::Rotr(e, 25);
      const uint32_t ch = (e & f) ^ ((~e) & g);
      const uint32_t t1 = hh + s1 + ch + detail::Sha256K()[i] + w[i];
      const uint32_t s0 =
          detail::Rotr(a, 2) ^ detail::Rotr(a, 13) ^ detail::Rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = s0 + maj;
      hh = g; g = f; f = e; e = d + t1;
      d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }

  char out[65];
  for (int i = 0; i < 8; ++i) std::snprintf(out + i * 8, 9, "%08x", h[i]);
  return std::string(out, 64);
}

// ---------------------------------------------------------------------------
// CRC-32, IEEE 802.3: init 0xFFFFFFFF, reflected, final xor 0xFFFFFFFF. Written
// bitwise with the reflected polynomial 0xEDB88320. Naming all four properties
// matters -- "CRC-32" on its own identifies at least four incompatible
// checksums, and picking the wrong one produces a number that looks entirely
// reasonable and never matches.
// ---------------------------------------------------------------------------

inline uint32_t Crc32(const std::string& data) {
  uint32_t crc = 0xFFFFFFFFu;
  for (unsigned char byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320u & (~((crc & 1u) - 1u)));
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

/* Eight lower-case hexadecimal characters, the form 11 S9A.2 transmits. */
inline std::string Crc32Hex(const std::string& data) {
  char out[9];
  std::snprintf(out, sizeof(out), "%08x", Crc32(data));
  return std::string(out);
}

// ---------------------------------------------------------------------------
// Canonical JSON, per 10 S5.4.4.
//
// A minimal value type rather than a JSON library, for the same reason the
// hashes are written out here: a dependency on nlohmann or RapidJSON would
// propagate to chassis_relay, and neither gives control over float rendering
// inside containers anyway, which is the one thing this needs.
// ---------------------------------------------------------------------------

class Value;
using ValuePtr = std::shared_ptr<Value>;

class Value {
 public:
  enum class Kind { kNull, kBool, kInt, kDouble, kString, kList, kMap };

  static ValuePtr Null() { return Make(Kind::kNull); }
  static ValuePtr Bool(bool v) { auto p = Make(Kind::kBool); p->b_ = v; return p; }
  static ValuePtr Int(int64_t v) { auto p = Make(Kind::kInt); p->i_ = v; return p; }
  static ValuePtr Double(double v) {
    auto p = Make(Kind::kDouble); p->d_ = v; return p;
  }
  static ValuePtr Str(const std::string& v) {
    auto p = Make(Kind::kString); p->s_ = v; return p;
  }
  static ValuePtr List() { return Make(Kind::kList); }
  static ValuePtr Map() { return Make(Kind::kMap); }

  void Append(const ValuePtr& v) { list_.push_back(v); }
  void Set(const std::string& key, const ValuePtr& v) { map_[key] = v; }

  Kind kind() const { return kind_; }
  bool b() const { return b_; }
  int64_t i() const { return i_; }
  double d() const { return d_; }
  const std::string& s() const { return s_; }
  const std::vector<ValuePtr>& list() const { return list_; }
  const std::map<std::string, ValuePtr>& map() const { return map_; }

 private:
  static ValuePtr Make(Kind k) {
    auto p = std::make_shared<Value>();
    p->kind_ = k;
    return p;
  }

  Kind kind_ = Kind::kNull;
  bool b_ = false;
  int64_t i_ = 0;
  double d_ = 0.0;
  std::string s_;
  std::vector<ValuePtr> list_;
  // std::map orders by operator< on std::string, which compares BYTES -- the
  // ordering 10 S5.4.4 asks for, obtained here for free.
  //
  // Worth being precise about what that does and does not buy. UTF-8 is
  // order-preserving, so byte order and Unicode code point order are the same
  // ordering for every pair of strings; Python's sorted() on str would agree
  // with this map even without encoding first. What it rules out is UTF-16 code
  // unit order, which is what a port keyed on std::wstring under Windows would
  // produce, and where a character above the BMP sorts below the fullwidth Latin
  // block instead of above it. Do not change this to a map keyed on wstring or
  // u16string.
  std::map<std::string, ValuePtr> map_;
};

/* One number, canonically. */
inline std::string FormatDouble(double value) {
  // %.17g is the shortest form that round-trips every IEEE 754 double, so both
  // languages are printing the same bits under the same rule rather than each
  // choosing something readable. It also renders an integral double as "1" with
  // no decimal point, which is what "整数不带小数点" requires.
  //
  // The trailing-zero removal below is the second half of what S5.4.4 asks for,
  // and under %g it is UNREACHABLE: C99 7.21.6.1 already strips trailing zeros
  // from the fractional part unless the # flag is given. Deleting it leaves the
  // whole cross-language suite green, which is how it was found. It is kept for
  // fidelity to the section and so the rule survives a change of format, but no
  // golden vector covers it and no mutation of it can be seen going red. The
  // Python side carries the same note and a test that asserts the premise.
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", value);
  std::string out(buf);
  if (out.find('.') != std::string::npos &&
      out.find('e') == std::string::npos &&
      out.find('E') == std::string::npos) {
    out.erase(out.find_last_not_of('0') + 1);
    if (!out.empty() && out.back() == '.') out.pop_back();
  }
  return out;
}

/* A JSON string with the minimum escaping the rules allow. */
inline std::string Quote(const std::string& text) {
  std::string out = "\"";
  for (unsigned char ch : text) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (ch < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          // Everything else, including every byte of a multi-byte UTF-8
          // sequence, goes out untouched. Escaping non-ASCII to a \u form would
          // additionally require both languages to agree on the case of the hex
          // digits, since an upper-case and a lower-case escape of the same
          // character are different bytes and hash differently.
          out.push_back(static_cast<char>(ch));
        }
    }
  }
  out.push_back('"');
  return out;
}

inline std::string CanonicalJson(const ValuePtr& node) {
  if (!node) return "null";
  switch (node->kind()) {
    case Value::Kind::kNull:
      return "null";
    case Value::Kind::kBool:
      return node->b() ? "true" : "false";
    case Value::Kind::kInt: {
      return std::to_string(node->i());
    }
    case Value::Kind::kDouble:
      return FormatDouble(node->d());
    case Value::Kind::kString:
      return Quote(node->s());
    case Value::Kind::kList: {
      // Order preserved: R-5 makes a list a unit replaced whole, and
      // qos.bindings is first-match-wins, so a reordering changes behaviour and
      // the digest has to notice.
      std::string out = "[";
      bool first = true;
      for (const auto& item : node->list()) {
        if (!first) out += ",";
        first = false;
        out += CanonicalJson(item);
      }
      return out + "]";
    }
    case Value::Kind::kMap: {
      std::string out = "{";
      bool first = true;
      for (const auto& kv : node->map()) {
        if (!first) out += ",";
        first = false;
        out += Quote(kv.first) + ":" + CanonicalJson(kv.second);
      }
      return out + "}";
    }
  }
  return "null";
}

/* 10 S5.4.4: the first sixteen hexadecimal characters of the sha256. */
inline std::string CommonDigest(const ValuePtr& common_subtree) {
  return Sha256Hex(CanonicalJson(common_subtree)).substr(0, 16);
}

// ---------------------------------------------------------------------------
// FenceSet crc32, per 11 S9A.2 block B-3.
//
// A plain struct rather than the Value tree above, because this recipe is not a
// serialisation of an arbitrary object: it names six fields per polygon in a
// fixed order, and a struct makes a missing one a compile error instead of a
// silently absent key.
// ---------------------------------------------------------------------------

struct Vertex {
  double lat = 0.0;
  double lon = 0.0;
};

struct Polygon {
  std::string poly_id;
  std::string role;
  std::string winding;
  bool hard_enforce = false;
  int32_t priority = 0;
  // Only read when role == "speed_limit". A stray value on a polygon of another
  // role is legal input per S9A.2, so letting it into the string would give two
  // identical fences different checksums.
  double speed_limit_mps = 0.0;
  std::vector<Vertex> vertices;
};

struct FenceSet {
  std::string fence_set_id;
  uint32_t rev = 0;
  std::vector<Polygon> polygons;
};

inline std::string CanonicalFenceString(const FenceSet& fence) {
  std::string out = fence.fence_set_id + "|" + std::to_string(fence.rev) + "|";
  for (const Polygon& poly : fence.polygons) {
    out += poly.poly_id + "|" + poly.role + "|" + poly.winding + "|";
    // "1"/"0", not "true"/"false". Streaming a bool would give "1" and
    // formatting it as text would give "true"; the section settles it.
    out += (poly.hard_enforce ? "1" : "0");
    out += "|";
    // Decimal, no leading zeros, sign kept. std::to_string on an int32 gives
    // exactly that.
    out += std::to_string(poly.priority);
    out += "|";
    if (poly.role == "speed_limit") {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.3f", poly.speed_limit_mps);
      out += buf;
    }
    out += "|";
    for (const Vertex& v : poly.vertices) {
      // Fixed eight decimals, in array order, never sorted. About 1.1 mm at
      // WGS84 degree scale, far finer than this system's positioning, and unlike
      // %.17g it renders identically in both languages without anyone reasoning
      // about shortest round-trip forms. Sorting would erase winding, which is
      // what distinguishes inside from outside.
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.8f,%.8f;", v.lat, v.lon);
      out += buf;
    }
  }
  return out;
}

inline std::string FenceCrc32(const FenceSet& fence) {
  return Crc32Hex(CanonicalFenceString(fence));
}

}  // namespace digest
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_DIGEST_CANONICAL_DIGEST_H_
