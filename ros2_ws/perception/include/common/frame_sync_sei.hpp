/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: frame_sync_sei.hpp
 * Brief: Versioned per-frame metadata carried in a user_data_unregistered SEI.
 *
 * Description:
 * Carriers: HEVC (prefix SEI, nal_unit_type 39) and AVC/H.264 (SEI, nal_unit_type 6).
 * This module is deliberately independent of GStreamer and FFmpeg.  It lets the
 * encoder and viewer validate the wire format with deterministic unit tests
 * before the metadata is enabled in the live RTSP path.
 * It is enabled now: production builds define PERCEPTION_FRAME_SYNC_SEI and every MED-2
 * preview frame carries frame_id, the capture wall stamp and the encoder PTS
 * (frame_sync_gst.hpp inserts it); the Gemini hard contract refuses a build without it.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace perception::frame_sync {

constexpr std::size_t UUID_SIZE = 16;
constexpr std::size_t USER_DATA_PAYLOAD_SIZE = 52;
constexpr std::uint8_t SCHEMA_VERSION = 1;

// Bitstream syntax of the carrier. The payload (UUID + XFS1 record) is the same
// for both; only the NAL header and the VCL NAL type range differ.
enum class Codec { Hevc, Avc };

struct Metadata {
    std::uint64_t frame_id = 0;
    std::uint64_t capture_stamp_ns = 0;
    std::uint64_t encoder_pts_ns = 0;
    bool capture_stamp_valid = false;
};

/**
 * @brief Fixed UUID that identifies xBrain frame synchronization metadata.
 *
 * UUID: 87b6cd95-0ada-4ed7-a195-3ba1b38615f2
 */
const std::uint8_t* uuid_bytes();

/**
 * @brief Builds the payload used after HEVC SEI payload_type=5.
 *
 * Layout (network byte order):
 *   UUID[16], magic "XFS1"[4], version[1], flags[1], reserved[2],
 *   frame_id[8], capture_stamp_ns[8], encoder_pts_ns[8], crc32[4].
 */
std::vector<std::uint8_t> build_user_data_payload(const Metadata& metadata);

/**
 * @brief Parses and validates one user_data_unregistered payload.
 *
 * Unknown UUID/schema, non-zero reserved bits, truncation and CRC mismatch are
 * rejected.  On failure, @p output is left unchanged.
 */
bool parse_user_data_payload(
    const std::uint8_t* data,
    std::size_t size,
    Metadata& output,
    std::string* error = nullptr);

/**
 * @brief Builds one Annex-B HEVC prefix-SEI NAL unit (nal_unit_type=39).
 */
std::vector<std::uint8_t> build_prefix_sei_annex_b(const Metadata& metadata);

/**
 * @brief Builds one Annex-B SEI NAL unit for @p codec: HEVC prefix SEI (type 39,
 * two-byte header) or AVC SEI (type 6, one-byte header, nal_ref_idc 0).
 */
std::vector<std::uint8_t> build_sei_annex_b(const Metadata& metadata, Codec codec);

/**
 * @brief Offset of the start code of the first VCL NAL unit in an Annex-B access
 * unit (HEVC: nal_unit_type 0..31; AVC: 1..5). The SEI is inserted there, after
 * any parameter sets / AUD, so the parser keeps it with the picture.
 */
bool first_vcl_nal_offset(const std::uint8_t* data, std::size_t size, Codec codec, std::size_t& output);

/**
 * @brief Finds and validates xBrain frame metadata in Annex-B HEVC bytes.
 *
 * Other NAL units and unrelated SEI messages are skipped.  This is an offline
 * validation helper; the FFmpeg viewer will normally parse the side-data
 * payload directly with parse_user_data_payload().
 */
bool extract_prefix_sei_annex_b(
    const std::uint8_t* data,
    std::size_t size,
    Metadata& output,
    std::string* error = nullptr);

/** @brief extract_prefix_sei_annex_b() for either codec. */
bool extract_sei_annex_b(
    const std::uint8_t* data,
    std::size_t size,
    Codec codec,
    Metadata& output,
    std::string* error = nullptr);

}  // namespace perception::frame_sync
