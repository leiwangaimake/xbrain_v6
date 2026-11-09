/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_frame_sync_sei.cpp
 * Brief: Deterministic wire-format tests for the frame synchronization SEI (HEVC and AVC).
 *
 * Description:
 * What it checks. Each encoded preview frame carries a user_data_unregistered SEI with the source
 * frame_id and capture time. Cases cover the payload round trip, canonicalisation of an invalid
 * capture timestamp, Annex B round trip with emulation-prevention escaping, rejection of malformed
 * Annex B, and the AVC (H.264) carrier required by the MED-2 preview (B8-6).
 */

#include "common/frame_sync_sei.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

bool same_metadata(
    const perception::frame_sync::Metadata& lhs,
    const perception::frame_sync::Metadata& rhs) {
    return lhs.frame_id == rhs.frame_id &&
           lhs.capture_stamp_ns == rhs.capture_stamp_ns &&
           lhs.encoder_pts_ns == rhs.encoder_pts_ns &&
           lhs.capture_stamp_valid == rhs.capture_stamp_valid;
}

// The fixed-size payload round-trips its metadata; a CRC mismatch is rejected and leaves the
// output unchanged.
void test_payload_round_trip() {
    const perception::frame_sync::Metadata expected{
        0x0102030405060708ULL,
        0x1020304050607080ULL,
        0x0000000000000001ULL,
        true};
    const auto payload = perception::frame_sync::build_user_data_payload(expected);
    check(
        payload.size() == perception::frame_sync::USER_DATA_PAYLOAD_SIZE,
        "payload has fixed documented size");

    perception::frame_sync::Metadata parsed;
    std::string error;
    check(
        perception::frame_sync::parse_user_data_payload(
            payload.data(), payload.size(), parsed, &error),
        "valid payload parses: " + error);
    check(same_metadata(parsed, expected), "payload round-trip preserves metadata");

    auto corrupt = payload;
    corrupt[30] ^= 0x40U;
    const auto sentinel = parsed;
    check(
        !perception::frame_sync::parse_user_data_payload(
            corrupt.data(), corrupt.size(), parsed, &error),
        "CRC rejects corrupt payload");
    check(error == "frame-sync CRC32 mismatch", "CRC failure is observable");
    check(same_metadata(parsed, sentinel), "failed parse leaves output unchanged");

    auto wrong_uuid = payload;
    wrong_uuid[0] ^= 0x01U;
    check(
        !perception::frame_sync::parse_user_data_payload(
            wrong_uuid.data(), wrong_uuid.size(), parsed, &error),
        "wrong UUID is rejected");
    check(error == "frame-sync UUID mismatch", "UUID failure is observable");

    check(
        !perception::frame_sync::parse_user_data_payload(
            payload.data(), payload.size() - 1U, parsed, &error),
        "truncated payload is rejected");
}

// A payload without a valid capture timestamp parses with the flag false and the timestamp written
// as zero; frame id and PTS are kept.
void test_invalid_capture_timestamp_is_canonicalized() {
    const perception::frame_sync::Metadata input{
        42U, 999999U, 123456U, false};
    const auto payload = perception::frame_sync::build_user_data_payload(input);
    perception::frame_sync::Metadata parsed;
    std::string error;
    check(
        perception::frame_sync::parse_user_data_payload(
            payload.data(), payload.size(), parsed, &error),
        "payload without capture timestamp parses");
    check(!parsed.capture_stamp_valid, "capture timestamp valid flag remains false");
    check(parsed.capture_stamp_ns == 0U, "invalid capture timestamp is encoded as zero");
    check(parsed.frame_id == input.frame_id, "frame id remains intact");
    check(parsed.encoder_pts_ns == input.encoder_pts_ns, "encoder PTS remains intact");
}

// The Annex-B HEVC prefix SEI (4-byte start code) round-trips, emulation prevention escapes
// embedded start codes, and the SEI is found among other NAL units.
void test_annex_b_round_trip_and_escaping() {
    const perception::frame_sync::Metadata expected{
        0x0000000000000003ULL,
        0x0000010000000000ULL,
        0x0000000000000002ULL,
        true};
    const auto sei = perception::frame_sync::build_prefix_sei_annex_b(expected);
    check(sei.size() > 8U, "Annex-B SEI has header and payload");
    check(
        sei[0] == 0U && sei[1] == 0U && sei[2] == 0U && sei[3] == 1U,
        "Annex-B SEI uses four-byte start code");
    check(((sei[4] >> 1U) & 0x3fU) == 39U, "NAL unit is HEVC prefix SEI");

    bool unsafe_start_code = false;
    for (std::size_t index = 6; index + 2U < sei.size(); ++index) {
        if (sei[index] == 0U && sei[index + 1U] == 0U &&
            sei[index + 2U] <= 2U) {
            unsafe_start_code = true;
            break;
        }
    }
    check(!unsafe_start_code, "RBSP emulation prevention removes embedded start codes");

    const std::vector<std::uint8_t> vps = {
        0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x0c, 0x01};
    const std::vector<std::uint8_t> slice = {
        0x00, 0x00, 0x01, 0x26, 0x01, 0x80};
    std::vector<std::uint8_t> access_unit = vps;
    access_unit.insert(access_unit.end(), sei.begin(), sei.end());
    access_unit.insert(access_unit.end(), slice.begin(), slice.end());

    perception::frame_sync::Metadata parsed;
    std::string error;
    check(
        perception::frame_sync::extract_prefix_sei_annex_b(
            access_unit.data(), access_unit.size(), parsed, &error),
        "frame-sync SEI is found among other NAL units: " + error);
    check(same_metadata(parsed, expected), "Annex-B round-trip preserves metadata");

    check(
        !perception::frame_sync::extract_prefix_sei_annex_b(
            vps.data(), vps.size(), parsed, &error),
        "stream without frame-sync SEI is rejected");
    check(error == "frame-sync SEI not found", "not-found condition is explicit");
}

// A truncated SEI, or input without an Annex-B start code, is rejected.
void test_malformed_annex_b_is_rejected() {
    const perception::frame_sync::Metadata metadata{7U, 8U, 9U, true};
    auto sei = perception::frame_sync::build_prefix_sei_annex_b(metadata);
    check(sei.size() > 20U, "test SEI has enough bytes to truncate");
    sei.resize(sei.size() - 8U);

    perception::frame_sync::Metadata parsed;
    std::string error;
    check(
        !perception::frame_sync::extract_prefix_sei_annex_b(
            sei.data(), sei.size(), parsed, &error),
        "truncated Annex-B SEI is rejected");
    check(
        error == "malformed SEI message" ||
            error == "unexpected frame-sync payload size",
        "truncation produces a diagnostic error");

    const std::vector<std::uint8_t> no_start_code = {0x4e, 0x01, 0x05, 0x00};
    check(
        !perception::frame_sync::extract_prefix_sei_annex_b(
            no_start_code.data(), no_start_code.size(), parsed, &error),
        "non-Annex-B input is rejected");
    check(error == "Annex-B start code not found", "missing start code is explicit");
}

// AVC / H.264 carrier (reply batch 8, B8-6): same payload, SEI nal_unit_type 6
// with a one-byte header, inserted before the first VCL NAL (types 1..5).
void test_avc_carrier() {
    using namespace perception::frame_sync;
    const Metadata expected{0x0000000000000102ULL, 0x0000000100000000ULL, 66666667ULL, true};
    const auto sei = build_sei_annex_b(expected, Codec::Avc);
    check(sei.size() > 8U && sei[0] == 0U && sei[1] == 0U && sei[2] == 0U && sei[3] == 1U,
          "AVC SEI uses a four-byte start code");
    check(sei[4] == 0x06U, "AVC SEI NAL header: forbidden 0, nal_ref_idc 0, type 6");
    bool unsafe = false;
    for (std::size_t i = 5; i + 2U < sei.size(); ++i)
        if (sei[i] == 0U && sei[i + 1U] == 0U && sei[i + 2U] <= 2U) unsafe = true;
    check(!unsafe, "AVC SEI RBSP has emulation prevention");
    // HEVC carrier is unchanged by the refactor
    check(build_sei_annex_b(expected, Codec::Hevc) == build_prefix_sei_annex_b(expected),
          "HEVC builder output unchanged");

    const std::vector<std::uint8_t> aud = {0x00, 0x00, 0x00, 0x01, 0x09, 0xf0};
    const std::vector<std::uint8_t> sps = {0x00, 0x00, 0x00, 0x01, 0x67, 0x64, 0x00, 0x1f};
    const std::vector<std::uint8_t> pps = {0x00, 0x00, 0x00, 0x01, 0x68, 0xee, 0x3c, 0x80};
    const std::vector<std::uint8_t> idr = {0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00};
    std::vector<std::uint8_t> au = aud;
    au.insert(au.end(), sps.begin(), sps.end());
    au.insert(au.end(), pps.begin(), pps.end());
    const std::size_t idr_at = au.size();
    au.insert(au.end(), idr.begin(), idr.end());
    std::size_t at = 0;
    check(first_vcl_nal_offset(au.data(), au.size(), Codec::Avc, at) && at == idr_at,
          "AVC first VCL NAL is the IDR slice, after AUD/SPS/PPS");
    // non-IDR slice (type 1) is VCL too; SEI / SPS / PPS / AUD are not
    const std::vector<std::uint8_t> p_slice = {0x00, 0x00, 0x01, 0x41, 0x9a, 0x00};
    check(first_vcl_nal_offset(p_slice.data(), p_slice.size(), Codec::Avc, at) && at == 0U, "AVC P slice is VCL");
    std::vector<std::uint8_t> no_vcl = aud;
    no_vcl.insert(no_vcl.end(), sps.begin(), sps.end());
    no_vcl.insert(no_vcl.end(), sei.begin(), sei.end());
    check(!first_vcl_nal_offset(no_vcl.data(), no_vcl.size(), Codec::Avc, at), "no VCL NAL -> no insertion point");

    // what the pad probe does: insert at the first VCL NAL, then read it back
    std::vector<std::uint8_t> injected(au.begin(), au.begin() + static_cast<std::ptrdiff_t>(idr_at));
    injected.insert(injected.end(), sei.begin(), sei.end());
    injected.insert(injected.end(), au.begin() + static_cast<std::ptrdiff_t>(idr_at), au.end());
    Metadata parsed;
    std::string error;
    check(extract_sei_annex_b(injected.data(), injected.size(), Codec::Avc, parsed, &error),
          "AVC frame-sync SEI is found in an access unit: " + error);
    check(same_metadata(parsed, expected), "AVC round trip preserves metadata");
    // a reader with the wrong codec does not misread it
    check(!extract_sei_annex_b(injected.data(), injected.size(), Codec::Hevc, parsed, &error),
          "HEVC reader does not accept an AVC SEI");
    // truncated AVC SEI is rejected, not half-read
    auto cut = sei; cut.resize(cut.size() - 8U);
    check(!extract_sei_annex_b(cut.data(), cut.size(), Codec::Avc, parsed, &error), "truncated AVC SEI rejected");
}

}  // namespace

int main() {
    test_payload_round_trip();
    test_invalid_capture_timestamp_is_canonicalized();
    test_annex_b_round_trip_and_escaping();
    test_malformed_annex_b_is_rejected();
    test_avc_carrier();
    if (failures != 0) {
        std::cerr << failures << " frame-sync SEI checks failed\n";
        return 1;
    }
    std::cout << "all frame-sync SEI checks passed\n";
    return 0;
}
