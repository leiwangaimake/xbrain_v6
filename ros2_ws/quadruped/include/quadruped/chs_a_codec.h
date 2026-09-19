/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_codec.h
 * Brief: CHS-A APDU header + ASDU JSON encoder for channel one (13 S2.2, S5.1)
 *
 * Description:
 * What this file owns. Turning a command into the exact bytes the M20S accepts,
 * and reading the 16-byte header off bytes that arrive. Nothing else: it does
 * not own the socket (tx_owner), does not reassemble a stream (chs_a_framer),
 * and does not interpret report payloads (batch B2).
 *
 * Why the encoder writes into a caller buffer instead of returning a string.
 * The axis command is emitted from the 100 Hz ctrl thread, where QD-7 forbids
 * dynamic allocation; a std::string return value allocates on every tick. So
 * every Encode* takes (buf, cap) and returns the byte count, and the caller
 * keeps one statically sized frame buffer. The same reason rules out a JSON
 * library on this path -- the payloads we SEND are five fixed shapes, and
 * snprintf renders them without allocating.
 *
 * *** The three things that make the chassis answer 0xE002, all invisible in
 * the manual's field tables and all encoded here:
 *   1. the ASDU root object must be wrapped in {"PatrolDevice": {...}};
 *   2. a "Time" field is mandatory, local time, "YYYY-MM-DD HH:MM:SS";
 *   3. the 32-bit Type/Command codes are serialised as DECIMAL integers --
 *      JSON has no hex literal, so 0x00100064 goes on the wire as 1048676.
 * Item 3 is the one that looks like a typo in review. The golden vectors in
 * test/golden/ come off the real wire and would catch a "helpful" change to
 * hex strings immediately.
 *
 * Length field. header[4..5] is the ASDU length ALONE, not including the 16
 * header bytes (13 S2.2 FR-1, confirmed against a captured frame: 109 bytes on
 * the wire, length field 93). Getting this wrong makes the peer wait forever
 * for bytes that already arrived, which presents as "the chassis stopped
 * responding" rather than as a framing bug.
 *
 * Message id pairing. header[6..7] is ours to choose and the chassis echoes it
 * on the matching response, which is the only way to pair a mode-switch ack
 * with the mode switch that caused it. It wraps at 16 bits, deliberately: a
 * monotonically growing counter would have to be reset somewhere, and a reset
 * is indistinguishable from a wrap for the pairing logic, which must tolerate
 * both anyway.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_CODEC_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_CODEC_H_

#include <cstddef>
#include <cstdint>

namespace quadruped {
namespace chs_a {

// ---------------------------------------------------------------------------
// Wire constants (13 S2.2, guide 1.1.5)
// ---------------------------------------------------------------------------

// Sync word, in wire order. Written as four bytes rather than a 32-bit literal
// so it reads the same here as in a hexdump and carries no endianness question.
inline constexpr std::uint8_t kSync0 = 0xEB;
inline constexpr std::uint8_t kSync1 = 0x91;
inline constexpr std::uint8_t kSync2 = 0xEB;
inline constexpr std::uint8_t kSync3 = 0x90;

inline constexpr std::size_t kHeaderBytes = 16;
// The length field is 16 bits, so this is a property of the protocol, not a
// buffer choice we are free to shrink.
inline constexpr std::size_t kMaxAsduBytes = 65535;
inline constexpr std::size_t kMaxFrameBytes = kHeaderBytes + kMaxAsduBytes;

inline constexpr std::uint8_t kAsduFormatJson = 0x01;  // 0x00 would be XML
inline constexpr std::uint8_t kProtoVersion = 0x01;

// ---------------------------------------------------------------------------
// hex32 code book (13 S5.1 / CB-1). These are the ONLY legal codes: CB-2
// forbids the "fall back to a decimal code book" branch, because no such table
// exists -- writing the fallback would be a branch that cannot work.
// ---------------------------------------------------------------------------
struct Code {
  std::uint32_t type;
  std::uint32_t command;
};

// One lamp's setting for C-07. The chassis takes Led as a TWO-element array,
// [0] the head lamp and [1] the tail lamp (vendor guide 1.2.7), so both are
// always sent -- there is no "change only the tail" form of this command, and
// omitting one would leave the chassis to decide what the missing element
// means.
struct LedSetting {
  // 0 solid / 1 fill_flow / 2 move_flow / 3 grad_flow / 4 breath / 5 blink,
  // the vendor's own numbering (guide 1.2.7), which 11 S9.4.1 names.
  int pattern = 0;
  // 0 black / 1 white / 2 green / 3 blue. *** There is NO RED -- 11 S9.4.1
  // states it outright and puts the deterrent flash on our own payload
  // (PAY-02). A caller asking for red is refused upstream rather than mapped
  // onto a nearby colour.
  int color = 0;
  // Duty cycle in SECONDS, used by the chassis only for breath and blink
  // (guide 1.2.7). Sent regardless: the field is not optional in the message,
  // and the chassis ignoring it for the other four patterns is its own rule,
  // not something to encode by leaving the key out.
  int cycle_s = 0;
};

inline constexpr Code kHeartbeat{0x00100064u, 0x00000005u};       // C-01
inline constexpr Code kUsageModeSwitch{0x00100002u, 0x00500002u}; // C-02
inline constexpr Code kMotionStateSwitch{0x00100001u, 0x00200002u};  // C-03
inline constexpr Code kGaitSwitch{0x00100001u, 0x00300002u};      // C-04
inline constexpr Code kNormalizedAxis{0x00100001u, 0x00100002u};  // C-05 decode only
inline constexpr Code kRealAxis{0x00100001u, 0x00110002u};        // C-06 the one we drive with
inline constexpr Code kCustomLight{0x00100005u, 0x00200002u};     // C-07
inline constexpr Code kSdkMode{0x00100005u, 0x00300002u};         // C-08

// Report codes, for dispatch on the receive side. The command is the same for
// all of them; the type is what distinguishes the four documented reports plus
// the fifth one that measurement found (13 S7.1 v1.3: location/nav, recognised
// and dropped, never treated as an unknown frame).
inline constexpr std::uint32_t kReportCommand = 0x00f00000u;
inline constexpr std::uint32_t kTypeBasic = 0x00100064u;
inline constexpr std::uint32_t kTypeMotion = 0x00100001u;
inline constexpr std::uint32_t kTypeDevice = 0x00100002u;
inline constexpr std::uint32_t kTypeFault = 0x0010007fu;
inline constexpr std::uint32_t kTypeLocationNav = 0x00100003u;

// ---------------------------------------------------------------------------
// Header
// ---------------------------------------------------------------------------
struct Header {
  std::uint16_t asdu_len = 0;  // payload bytes only, NOT including these 16
  std::uint16_t msg_id = 0;    // echoed by the chassis on the paired response
  std::uint8_t format = kAsduFormatJson;
  std::uint8_t packet_no = 0;
  std::uint8_t version = kProtoVersion;
};

// Write the 16 header bytes. Returns false only when cap is too small, so the
// caller cannot accidentally emit a truncated header.
bool WriteHeader(const Header& h, std::uint8_t* buf, std::size_t cap);

// Read the 16 header bytes. Returns false when the sync word does not match or
// the buffer is short -- the caller then resyncs (FR-2) rather than trusting a
// length field it just read out of arbitrary bytes.
bool ReadHeader(const std::uint8_t* buf, std::size_t len, Header* out);

// ---------------------------------------------------------------------------
// Encoders. Each renders a COMPLETE frame (header + ASDU) into buf and returns
// the byte count, or 0 on failure (buffer too small, or a value that would
// produce invalid JSON). Zero is unambiguous: a valid frame is never 0 bytes.
//
// `now_wall` is the wall-clock seconds for the mandatory Time field, passed in
// rather than read here so the encoder is deterministic under test. It is a
// PROTOCOL field, never an age or a timeout -- those are monotonic everywhere
// (CLK-C1).
// ---------------------------------------------------------------------------

// C-01. Items is an empty object, and it must be present: a missing Items is
// one of the three 0xE002 causes.
std::size_t EncodeHeartbeat(std::uint8_t* buf, std::size_t cap,
                            std::uint16_t msg_id, std::int64_t now_wall);

// C-02. mode: 0 normal / 1 navigation / 2 assist (guide 1.2.2).
std::size_t EncodeUsageMode(std::uint8_t* buf, std::size_t cap,
                            std::uint16_t msg_id, std::int64_t now_wall,
                            int mode);

// C-03. motion_param values are 13 S5.2; the caller is responsible for not
// sending one this project does not implement (that list is config, not code).
std::size_t EncodeMotionState(std::uint8_t* buf, std::size_t cap,
                              std::uint16_t msg_id, std::int64_t now_wall,
                              int motion_param);

// C-04. ActionParam is deliberately omitted: the guide's examples carry it but
// its value table does not exist anywhere (13 V-45), so sending a guessed value
// is worse than sending none. If the chassis ever rejects the frame for a
// missing ActionParam, that is the moment to add it -- with a measured value.
// C-07, the custom light command (13 S5.1 / vendor guide 1.2.7). Items is
// {"CustomMode": bool, "Led": [head, tail]} and each element is
// {"Type", "Color": [int], "Cycle"}.
//
// *** Color is an ARRAY in the chassis message while 11 S9.4.1 names a single
// `color`. The vendor's own example sends a one-element array, so that is what
// this writes -- the array is the wire shape, not a per-segment list we have
// any way to fill.
std::size_t EncodeCustomLight(std::uint8_t* buf, std::size_t cap,
                              std::uint16_t msg_id, std::int64_t now_wall,
                              bool custom_mode, const LedSetting& head,
                              const LedSetting& tail);

std::size_t EncodeGait(std::uint8_t* buf, std::size_t cap, std::uint16_t msg_id,
                       std::int64_t now_wall, std::uint32_t gait_param);

// C-06, the only command that moves the robot. Units are physical (m/s, rad/s)
// because the chassis is held in navigation mode; in normal mode the SAME
// fields would mean "fraction of maximum", which is why 11 S9.2.1 makes the
// mode a precondition rather than a hint.
//
// Returns 0 if any value is not finite: a JSON NaN is not valid JSON and the
// chassis answers 0xE002, which 13 S7.5 tells the reader to blame on our
// encoder -- correctly, in that case, so it must never reach the wire.
struct AxisCommand {
  double vx = 0.0;       // m/s   forward
  double vy = 0.0;       // m/s   left
  double vz = 0.0;       // m/s   up (special gaits only; zeroed otherwise)
  double roll = 0.0;     // rad/s (special gaits only)
  double pitch = 0.0;    // rad/s (special gaits only)
  double yaw = 0.0;      // rad/s yaw rate
};
std::size_t EncodeRealAxis(std::uint8_t* buf, std::size_t cap,
                           std::uint16_t msg_id, std::int64_t now_wall,
                           const AxisCommand& cmd);

// C-08. Deployment-time only: 11 S9.3.4 keeps it off the general plane.
std::size_t EncodeSdkMode(std::uint8_t* buf, std::size_t cap,
                          std::uint16_t msg_id, std::int64_t now_wall,
                          bool enable, int joint_rate_hz);

// ---------------------------------------------------------------------------
// Minimal decode. Full payload decoding is B2; what B1 needs is the routing
// information plus the generic response code, both of which are three integers
// in a flat position and do not justify a JSON library on the receive path yet.
// ---------------------------------------------------------------------------
struct AsduRouting {
  std::uint32_t type = 0;
  std::uint32_t command = 0;
  // Present only on a generic response (guide 1.5): Items.ErrorCode. Absent on
  // reports, which is why it is optional rather than defaulted to 0 -- 0 means
  // SUCCESS on this protocol, and a missing field read as success is the
  // fail-silent shape 3.1 forbids.
  bool has_error_code = false;
  std::uint32_t error_code = 0;
};

// Extract routing from an ASDU payload. Returns false when the payload is not
// the expected shape, which the caller reports rather than guessing at.
bool ParseAsduRouting(const std::uint8_t* asdu, std::size_t len,
                      AsduRouting* out);

}  // namespace chs_a
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_CODEC_H_
