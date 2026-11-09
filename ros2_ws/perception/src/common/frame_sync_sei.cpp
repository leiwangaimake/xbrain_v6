/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: frame_sync_sei.cpp
 * Brief: HEVC user_data_unregistered SEI wire-format implementation.
 *
 * Description:
 * Implements frame_sync_sei.hpp for HEVC and AVC. The payload is binary, so the SEI body gets
 * emulation-prevention bytes (0x03 after two zero bytes before a byte <= 0x03) and the parser
 * removes them; the CRC32 covers the 32 bytes after the UUID. Parsing rejects anything not
 * exactly schema 1, so a corrupt or foreign SEI is never read as frame metadata.
 */

#include "common/frame_sync_sei.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace perception::frame_sync {

namespace {

constexpr std::array<std::uint8_t, UUID_SIZE> FRAME_SYNC_UUID = {
    0x87, 0xb6, 0xcd, 0x95, 0x0a, 0xda, 0x4e, 0xd7,
    0xa1, 0x95, 0x3b, 0xa1, 0xb3, 0x86, 0x15, 0xf2};
constexpr std::array<std::uint8_t, 4> MAGIC = {'X', 'F', 'S', '1'};
constexpr std::uint8_t FLAG_CAPTURE_STAMP_VALID = 0x01;
constexpr std::uint8_t KNOWN_FLAGS = FLAG_CAPTURE_STAMP_VALID;
constexpr std::uint8_t USER_DATA_UNREGISTERED_TYPE = 5;
constexpr std::uint8_t HEVC_PREFIX_SEI_NAL_TYPE = 39;
constexpr std::uint8_t AVC_SEI_NAL_TYPE = 6;
constexpr std::size_t CUSTOM_DATA_SIZE_WITHOUT_CRC = 32;
constexpr std::size_t CRC_SIZE = 4;
static_assert(
    UUID_SIZE + CUSTOM_DATA_SIZE_WITHOUT_CRC + CRC_SIZE ==
        USER_DATA_PAYLOAD_SIZE,
    "frame-sync payload size constants must match the wire schema");

void set_error(std::string* error, const std::string& message) {
    if (error) {
        *error = message;
    }
}

void append_be64(std::vector<std::uint8_t>& output, std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) {
        output.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
}

void append_be32(std::vector<std::uint8_t>& output, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
        output.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
}

std::uint64_t read_be64(const std::uint8_t* data) {
    std::uint64_t value = 0;
    for (std::size_t index = 0; index < 8; ++index) {
        value = (value << 8U) | data[index];
    }
    return value;
}

std::uint32_t read_be32(const std::uint8_t* data) {
    std::uint32_t value = 0;
    for (std::size_t index = 0; index < 4; ++index) {
        value = (value << 8U) | data[index];
    }
    return value;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xffffffffU;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask =
                0U - static_cast<std::uint32_t>(crc & 1U);
            crc = (crc >> 1U) ^ (0xedb88320U & mask);
        }
    }
    return ~crc;
}

void append_sei_variable_length(
    std::vector<std::uint8_t>& output,
    std::size_t value) {
    while (value >= 0xffU) {
        output.push_back(0xffU);
        value -= 0xffU;
    }
    output.push_back(static_cast<std::uint8_t>(value));
}

std::vector<std::uint8_t> escape_rbsp(const std::vector<std::uint8_t>& rbsp) {
    std::vector<std::uint8_t> escaped;
    escaped.reserve(rbsp.size() + rbsp.size() / 32U);
    int consecutive_zeroes = 0;
    for (const std::uint8_t byte : rbsp) {
        if (consecutive_zeroes >= 2 && byte <= 0x03U) {
            escaped.push_back(0x03U);
            consecutive_zeroes = 0;
        }
        escaped.push_back(byte);
        if (byte == 0) {
            ++consecutive_zeroes;
        } else {
            consecutive_zeroes = 0;
        }
    }
    return escaped;
}

bool unescape_rbsp(
    const std::uint8_t* data,
    std::size_t size,
    std::vector<std::uint8_t>& output,
    std::string* error) {
    output.clear();
    output.reserve(size);
    int consecutive_zeroes = 0;
    for (std::size_t index = 0; index < size; ++index) {
        const std::uint8_t byte = data[index];
        if (consecutive_zeroes >= 2 && byte == 0x03U) {
            if (index + 1U >= size || data[index + 1U] > 0x03U) {
                set_error(error, "invalid emulation-prevention byte");
                return false;
            }
            consecutive_zeroes = 0;
            continue;
        }
        output.push_back(byte);
        if (byte == 0) {
            ++consecutive_zeroes;
        } else {
            consecutive_zeroes = 0;
        }
    }
    return true;
}

struct NalRange {
    std::size_t begin = 0;
    std::size_t end = 0;
};

bool find_start_code(
    const std::uint8_t* data,
    std::size_t size,
    std::size_t from,
    std::size_t& offset,
    std::size_t& length) {
    if (!data || from >= size) {
        return false;
    }
    for (std::size_t index = from; index + 3U <= size; ++index) {
        if (data[index] != 0 || data[index + 1U] != 0) {
            continue;
        }
        if (data[index + 2U] == 1U) {
            offset = index;
            length = 3;
            return true;
        }
        if (index + 4U <= size && data[index + 2U] == 0U &&
            data[index + 3U] == 1U) {
            offset = index;
            length = 4;
            return true;
        }
    }
    return false;
}

std::vector<NalRange> annex_b_nal_ranges(
    const std::uint8_t* data,
    std::size_t size) {
    std::vector<NalRange> ranges;
    std::size_t start = 0;
    std::size_t start_length = 0;
    if (!find_start_code(data, size, 0, start, start_length)) {
        return ranges;
    }
    while (true) {
        const std::size_t nal_begin = start + start_length;
        std::size_t next = 0;
        std::size_t next_length = 0;
        if (!find_start_code(data, size, nal_begin, next, next_length)) {
            if (nal_begin < size) {
                ranges.push_back({nal_begin, size});
            }
            break;
        }
        if (nal_begin < next) {
            ranges.push_back({nal_begin, next});
        }
        start = next;
        start_length = next_length;
    }
    return ranges;
}

bool read_sei_variable_length(
    const std::vector<std::uint8_t>& rbsp,
    std::size_t& cursor,
    std::size_t& output) {
    output = 0;
    while (cursor < rbsp.size() && rbsp[cursor] == 0xffU) {
        if (output > std::numeric_limits<std::size_t>::max() - 0xffU) {
            return false;
        }
        output += 0xffU;
        ++cursor;
    }
    if (cursor >= rbsp.size() ||
        output > std::numeric_limits<std::size_t>::max() - rbsp[cursor]) {
        return false;
    }
    output += rbsp[cursor++];
    return true;
}

}  // namespace

const std::uint8_t* uuid_bytes() {
    return FRAME_SYNC_UUID.data();
}

std::vector<std::uint8_t> build_user_data_payload(const Metadata& metadata) {
    std::vector<std::uint8_t> payload;
    payload.reserve(USER_DATA_PAYLOAD_SIZE);
    payload.insert(payload.end(), FRAME_SYNC_UUID.begin(), FRAME_SYNC_UUID.end());
    payload.insert(payload.end(), MAGIC.begin(), MAGIC.end());
    payload.push_back(SCHEMA_VERSION);
    payload.push_back(metadata.capture_stamp_valid ? FLAG_CAPTURE_STAMP_VALID : 0U);
    payload.push_back(0);
    payload.push_back(0);
    append_be64(payload, metadata.frame_id);
    append_be64(payload, metadata.capture_stamp_valid ? metadata.capture_stamp_ns : 0U);
    append_be64(payload, metadata.encoder_pts_ns);
    const std::uint32_t checksum =
        crc32(payload.data() + UUID_SIZE, CUSTOM_DATA_SIZE_WITHOUT_CRC);
    append_be32(payload, checksum);
    return payload;
}

bool parse_user_data_payload(
    const std::uint8_t* data,
    std::size_t size,
    Metadata& output,
    std::string* error) {
    if (!data || size != USER_DATA_PAYLOAD_SIZE) {
        set_error(error, "unexpected frame-sync payload size");
        return false;
    }
    if (!std::equal(FRAME_SYNC_UUID.begin(), FRAME_SYNC_UUID.end(), data)) {
        set_error(error, "frame-sync UUID mismatch");
        return false;
    }
    const std::uint8_t* custom = data + UUID_SIZE;
    if (!std::equal(MAGIC.begin(), MAGIC.end(), custom)) {
        set_error(error, "frame-sync magic mismatch");
        return false;
    }
    if (custom[4] != SCHEMA_VERSION) {
        set_error(error, "unsupported frame-sync schema version");
        return false;
    }
    const std::uint8_t flags = custom[5];
    if ((flags & ~KNOWN_FLAGS) != 0U) {
        set_error(error, "unknown frame-sync flags");
        return false;
    }
    if (custom[6] != 0U || custom[7] != 0U) {
        set_error(error, "frame-sync reserved bytes are non-zero");
        return false;
    }
    const std::uint32_t expected_crc =
        crc32(custom, CUSTOM_DATA_SIZE_WITHOUT_CRC);
    const std::uint32_t encoded_crc =
        read_be32(custom + CUSTOM_DATA_SIZE_WITHOUT_CRC);
    if (encoded_crc != expected_crc) {
        set_error(error, "frame-sync CRC32 mismatch");
        return false;
    }

    Metadata parsed;
    parsed.frame_id = read_be64(custom + 8);
    parsed.capture_stamp_ns = read_be64(custom + 16);
    parsed.encoder_pts_ns = read_be64(custom + 24);
    parsed.capture_stamp_valid = (flags & FLAG_CAPTURE_STAMP_VALID) != 0U;
    if (!parsed.capture_stamp_valid && parsed.capture_stamp_ns != 0U) {
        set_error(error, "invalid frame-sync capture timestamp validity");
        return false;
    }
    output = parsed;
    if (error) {
        error->clear();
    }
    return true;
}

std::vector<std::uint8_t> build_prefix_sei_annex_b(const Metadata& metadata) {
    return build_sei_annex_b(metadata, Codec::Hevc);
}

std::vector<std::uint8_t> build_sei_annex_b(const Metadata& metadata, Codec codec) {
    const std::vector<std::uint8_t> payload = build_user_data_payload(metadata);
    std::vector<std::uint8_t> rbsp;
    rbsp.reserve(payload.size() + 4U);
    append_sei_variable_length(rbsp, USER_DATA_UNREGISTERED_TYPE);
    append_sei_variable_length(rbsp, payload.size());
    rbsp.insert(rbsp.end(), payload.begin(), payload.end());
    rbsp.push_back(0x80U);  // rbsp_trailing_bits
    const std::vector<std::uint8_t> escaped = escape_rbsp(rbsp);

    std::vector<std::uint8_t> annex_b = {0x00, 0x00, 0x00, 0x01};
    if (codec == Codec::Avc) {
        annex_b.push_back(AVC_SEI_NAL_TYPE);  // forbidden 0, nal_ref_idc 0, type 6
    } else {
        annex_b.push_back(static_cast<std::uint8_t>(HEVC_PREFIX_SEI_NAL_TYPE << 1U));
        annex_b.push_back(0x01U);  // nuh_layer_id=0, nuh_temporal_id_plus1=1
    }
    annex_b.insert(annex_b.end(), escaped.begin(), escaped.end());
    return annex_b;
}

bool extract_prefix_sei_annex_b(
    const std::uint8_t* data,
    std::size_t size,
    Metadata& output,
    std::string* error) {
    return extract_sei_annex_b(data, size, Codec::Hevc, output, error);
}

bool first_vcl_nal_offset(const std::uint8_t* data, std::size_t size, Codec codec, std::size_t& output) {
    if (!data) return false;
    const std::size_t header = codec == Codec::Avc ? 1U : 2U;
    for (std::size_t offset = 0; offset + 3U <= size; ++offset) {
        std::size_t start = 0;
        if (offset + 3U <= size && data[offset] == 0U && data[offset + 1U] == 0U && data[offset + 2U] == 1U) {
            start = 3U;
        } else if (offset + 4U <= size && data[offset] == 0U && data[offset + 1U] == 0U &&
                   data[offset + 2U] == 0U && data[offset + 3U] == 1U) {
            start = 4U;
        } else {
            continue;
        }
        const std::size_t nal = offset + start;
        if (nal + header > size) return false;
        const bool vcl = codec == Codec::Avc
            ? ((data[nal] & 0x1fU) >= 1U && (data[nal] & 0x1fU) <= 5U)
            : (((data[nal] >> 1U) & 0x3fU) <= 31U);
        if (vcl) {
            output = offset;
            return true;
        }
        offset = nal;   // the loop's ++offset moves past the header byte
    }
    return false;
}

bool extract_sei_annex_b(
    const std::uint8_t* data,
    std::size_t size,
    Codec codec,
    Metadata& output,
    std::string* error) {
    if (!data || size == 0U) {
        set_error(error, "empty Annex-B input");
        return false;
    }
    const std::vector<NalRange> ranges = annex_b_nal_ranges(data, size);
    if (ranges.empty()) {
        set_error(error, "Annex-B start code not found");
        return false;
    }
    bool saw_malformed_prefix_sei = false;
    std::string last_error;
    for (const NalRange& range : ranges) {
        const std::size_t header = codec == Codec::Avc ? 1U : 2U;
        if (range.end - range.begin < header + 1U) {
            continue;
        }
        const bool sei = codec == Codec::Avc
            ? (data[range.begin] & 0x1fU) == AVC_SEI_NAL_TYPE
            : static_cast<std::uint8_t>((data[range.begin] >> 1U) & 0x3fU) == HEVC_PREFIX_SEI_NAL_TYPE;
        if (!sei) {
            continue;
        }
        std::vector<std::uint8_t> rbsp;
        if (!unescape_rbsp(
                data + range.begin + header,
                range.end - range.begin - header,
                rbsp,
                &last_error)) {
            saw_malformed_prefix_sei = true;
            continue;
        }
        std::size_t cursor = 0;
        while (cursor < rbsp.size()) {
            if (rbsp[cursor] == 0x80U && cursor + 1U == rbsp.size()) {
                break;
            }
            std::size_t payload_type = 0;
            std::size_t payload_size = 0;
            if (!read_sei_variable_length(rbsp, cursor, payload_type) ||
                !read_sei_variable_length(rbsp, cursor, payload_size) ||
                payload_size > rbsp.size() - cursor) {
                saw_malformed_prefix_sei = true;
                last_error = "malformed SEI message";
                break;
            }
            if (payload_type == USER_DATA_UNREGISTERED_TYPE &&
                payload_size >= UUID_SIZE &&
                std::equal(
                    FRAME_SYNC_UUID.begin(),
                    FRAME_SYNC_UUID.end(),
                    rbsp.data() + cursor)) {
                if (parse_user_data_payload(
                        rbsp.data() + cursor,
                        payload_size,
                        output,
                        &last_error)) {
                    if (error) {
                        error->clear();
                    }
                    return true;
                }
                saw_malformed_prefix_sei = true;
            }
            cursor += payload_size;
        }
    }
    set_error(
        error,
        saw_malformed_prefix_sei
            ? (last_error.empty() ? "malformed frame-sync SEI" : last_error)
            : "frame-sync SEI not found");
    return false;
}

}  // namespace perception::frame_sync
