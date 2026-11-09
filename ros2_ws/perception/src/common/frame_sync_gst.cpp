/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: frame_sync_gst.cpp
 * Brief: GStreamer pad-probe injection of HEVC / AVC frame synchronization SEI.
 *
 * Description:
 * Implements frame_sync_gst.hpp. The probe copies the encoded buffer into new memory with the
 * SEI inserted at the first picture NAL and swaps it into the (made-writable) buffer.
 */

#include "common/frame_sync_gst.hpp"

#include <cstring>
#include <utility>
#include <vector>

namespace perception::frame_sync {

GstSeiInjector::GstSeiInjector(std::size_t max_pending, Codec codec)
    : max_pending_(max_pending == 0 ? 1 : max_pending), codec_(codec) {}

bool GstSeiInjector::remember(
    GstClockTime pts,
    const Metadata& metadata) {
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto inserted = pending_.emplace(pts, metadata).second;
    if (!inserted) {
        duplicate_pts_.fetch_add(1);
        return false;
    }
    remembered_.fetch_add(1);
    while (pending_.size() > max_pending_) {
        pending_.erase(pending_.begin());
        expired_.fetch_add(1);
    }
    return true;
}

void GstSeiInjector::forget(GstClockTime pts) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.erase(pts);
}

GstSeiInjector::Stats GstSeiInjector::stats() const {
    Stats result;
    result.remembered = remembered_.load();
    result.injected = injected_.load();
    result.unmatched_buffers = unmatched_buffers_.load();
    result.duplicate_pts = duplicate_pts_.load();
    result.expired = expired_.load();
    result.injection_failures = injection_failures_.load();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        result.pending = pending_.size();
    }
    return result;
}

GstPadProbeReturn GstSeiInjector::pad_probe(
    GstPad*,
    GstPadProbeInfo* info,
    gpointer user_data) {
    if (!user_data) {
        return GST_PAD_PROBE_OK;
    }
    return static_cast<GstSeiInjector*>(user_data)->inject(info);
}

GstPadProbeReturn GstSeiInjector::inject(GstPadProbeInfo* info) {
    if (!info || (GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER) == 0U) {
        return GST_PAD_PROBE_OK;
    }
    GstBuffer* buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buffer || !GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))) {
        return GST_PAD_PROBE_OK;
    }

    // The SEI must appear immediately before the first VCL NAL.  Simply
    // prepending it before VPS/SPS/PPS causes the parser (h265parse/h264parse)
    // to discard the first frame's SEI while it is still establishing the
    // codec configuration.
    GstMapInfo input_map = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buffer, &input_map, GST_MAP_READ)) {
        injection_failures_.fetch_add(1);
        return GST_PAD_PROBE_OK;
    }
    std::size_t insertion_offset = 0;
    if (!first_vcl_nal_offset(input_map.data, input_map.size, codec_, insertion_offset)) {
        gst_buffer_unmap(buffer, &input_map);
        return GST_PAD_PROBE_OK;
    }

    Metadata metadata;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = pending_.find(GST_BUFFER_PTS(buffer));
        if (found == pending_.end()) {
            gst_buffer_unmap(buffer, &input_map);
            unmatched_buffers_.fetch_add(1);
            return GST_PAD_PROBE_OK;
        }
        metadata = found->second;
        pending_.erase(found);
    }

    const std::vector<std::uint8_t> sei = build_sei_annex_b(metadata, codec_);
    GstMemory* replacement_memory =
        gst_allocator_alloc(nullptr, input_map.size + sei.size(), nullptr);
    GstMapInfo output_map = GST_MAP_INFO_INIT;
    if (!replacement_memory ||
        !gst_memory_map(replacement_memory, &output_map, GST_MAP_WRITE) ||
        output_map.size < input_map.size + sei.size()) {
        if (output_map.data) {
            gst_memory_unmap(replacement_memory, &output_map);
        }
        if (replacement_memory) {
            gst_memory_unref(replacement_memory);
        }
        gst_buffer_unmap(buffer, &input_map);
        injection_failures_.fetch_add(1);
        return GST_PAD_PROBE_OK;
    }
    std::memcpy(
        output_map.data,
        input_map.data,
        insertion_offset);
    std::memcpy(
        output_map.data + insertion_offset,
        sei.data(),
        sei.size());
    std::memcpy(
        output_map.data + insertion_offset + sei.size(),
        input_map.data + insertion_offset,
        input_map.size - insertion_offset);
    gst_memory_unmap(replacement_memory, &output_map);
    gst_buffer_unmap(buffer, &input_map);

    GstBuffer* writable = gst_buffer_make_writable(buffer);
    if (!writable) {
        gst_memory_unref(replacement_memory);
        injection_failures_.fetch_add(1);
        return GST_PAD_PROBE_OK;
    }
    GST_PAD_PROBE_INFO_DATA(info) = writable;
    gst_buffer_remove_all_memory(writable);
    gst_buffer_append_memory(writable, replacement_memory);
    injected_.fetch_add(1);
    return GST_PAD_PROBE_OK;
}

}  // namespace perception::frame_sync
