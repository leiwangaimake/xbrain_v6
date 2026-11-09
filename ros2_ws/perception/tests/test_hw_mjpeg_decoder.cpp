/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_hw_mjpeg_decoder.cpp
 * Brief: Target test of the hardware MJPG decode (HwMjpegDecoder) against a libjpeg decode.
 *
 * Description:
 * What it checks (needs a Jetson with nvv4l2decoder). An embedded 4:2:2 JPEG (the sampling of the
 * Gemini colour stream) is decoded by the hardware path and by OpenCV's libjpeg; the two must agree
 * within the limits accepted for the capture (mean <= 1.5 levels, pixels over 8 levels <= 0.1 %),
 * on every one of 30 repeated decodes; empty input, a bad raster size and use before open are
 * refused; the decoder reopens after close.
 * Manual use: test_hw_mjpeg_decoder <dir> also compares the recorded camera frames
 * s<i>_colour.mjpg with the SDK decodes s<i>_bgr.png written by perception_rgbd_sample_dump.
 */

#include "modules/hw_mjpeg_decoder.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace {

int failures = 0;
#define CHECK(cond) \
    do { if (!(cond)) { ++failures; std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #cond); } } while (0)

bool within_limits(const perception::DecodeAgreement& a) {
    return a.mean_abs <= perception::kHwDecodeMeanAbsLimit &&
           a.over8_share <= perception::kHwDecodeOver8Limit;
}

// A 128x64 4:2:2 JPEG (gradients, six colour patches, thin white lines), made on 1.8 with PIL
// 9.0.1 (libjpeg) at quality 90, subsampling 4:2:2; SHA-256 10ec3b8be640625b...; 3443 bytes. OpenCV
// 4.5.4 on the target cannot write 4:2:2, so the bytes are embedded. On 1.8 the hardware decode of
// this image matched libjpeg exactly (2026-10-05); the test asserts the accepted limits only.
const char kTestJpegBase64[] =
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgFBQQEBQoHBwYIDAoMDAsKCwsNDhIQDQ4RDgsL"
    "EBYQERMUFRUVDA8XGBYUGBIUFRT/2wBDAQMEBAUEBQkFBQkUDQsNFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQU"
    "FBQUFBQUFBQUFBQUFBQUFBQUFBT/wAARCABAAIADASEAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcI"
    "CQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRol"
    "JicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ip"
    "qrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAA"
    "AAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLR"
    "ChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaX"
    "mJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEA"
    "PwD8x4NI/wBmr8Ok9PlrJVCsNiDQg0np8tX4dJ6fLW8ah9PhsQX4NI/2avwaR/s10RqH0+GxGxfh0n/ZrQh0np8tbKof"
    "UYbEbF+DSf8AZq/BpPtW8ah9PhsQX4dJ9qvw6T/s1uqh9PhsQaEGkf7NX4dJ/wBmt41D6fDYgvw6T/s1fg0np8tbqofT"
    "4fEbHyhDpPT5avwaT0+Wvz2NQ/zHw2IL8Ok9Plq/BpH+zW6qH1GHxGxoQaR/s1fh0n/ZreNQ+nw2IL8Ok9Plq/BpP+zW"
    "8ah9PhsQX4NJ9qvw6T7Vuqh9PhsQaEOk/wCzV+DSP9mt41D6jDYgvw6T/s1fh0n/AGa3VQ+nw+I2L8Gkf7NaEGkdPlra"
    "NQ+nw2IPlzSPDC3wk3SeVsx/DnOc+/tWsnhCNP8Al4z/ANs//r1+RV82eHqunyXt5/8AAP5j4G8HHxZkOGzxZh7L2vN7"
    "vsua3LOUPi9pG9+W+yte3mPfQFtyoVt+f9nGK9vT9l6NP+Ziz/24/wD2yvJzbip5VSo1fY83tOb7VrcrS/ld738j+ffF"
    "LMX4W528n5frNvtX9n9inPb3/wCe2/S/WysJ+zTGn/MwZ/7cv/tleE+LddXweLMtZ/avtG//AJabNu3b7HP3q+98Mar8"
    "Rc6jlH+73v73x7QnLb3P5Lb9b9NfH4R47fEVPET+r+z9jyfb5r81/wC6rW5fO9zDT4tRp/zB8/8Abz/9hXa/D7xKvjY3"
    "22x+yfZfL/5a7927d/sjH3f1r+keKvCZ8L5NXzf697T2fL7vs+W/NOMd/aStbmvs9rH6nl2euvXjQdO1+t+yv2P0HT/g"
    "mpGn/NQ8/wDcE/8AuirCf8E3o0/5qBn/ALgv/wB0V+DKLXU+jp8QSp/8u/x/4B+fvxh+KK/CAaOW0f8AtX+0PO/5efJ8"
    "vy9n+w2c7/bpXnifthRp/wAydn/uJ/8A2mqi21c9+vxM8DXdFUua1tea26T7M9b+BPxZX4zHWgui/wBk/wBm+R/y9ef5"
    "nmeZ/sLjHl++c+1fWafs9xp/zG8/9un/ANnXxfE3FT4c9j+59p7Tm+1y25eX+673ufW4XiJvC0sT7P476X2s7b26+hOn"
    "wGjT/mM5/wC3X/7OuFg0jp8ta8L8V/6x+2/c+z9ny/a5r83N/dVrW89z7PJc6eYOacOXlt1ve9/Jdj5VsrT7KrcY3Yqz"
    "XxeNd68n6fkj7DwPlz+H2Wv/AK+/+n6hasbT7UzcZ24r68r4rit3w2EX+P8AOJ/mp9KuV+Oase1v/TNAK+GPj5afZV0H"
    "jG7z/wD2nX7p9GZ241pL1/8ATVY/KPC6VqePXf2f5zPI69v/AGaLT7U3iLjO37N/7Vr/AEb8WnbgvHP/AK9/+naZ+/5Z"
    "LkxcH6/kz97KK/go+lP5/wD9uS0+yr4J4xu+3f8AtvXyvWFF3ppno5jLnxMn6fkj67/YAtPtTeO+M7fsH/txX6hV+IeJ"
    "bvLCr/H/AO2H2WBlfLqC7c3/AKUFeRQaT0+Wn4ay5frf/bn/ALefc8O1PZyqfL9T471e0+yrDxjdu/pWbXXiXeq3/Wx+"
    "s+A0ufw5yx/9fv8A0/VOl8G2n2prvjO3Z/7NX1DXxfFTvRwq/wAf5xP80/pTyvx/iI9uX/0xQCvjL9qC0+yr4Z4xu+1f"
    "+0q/c/o0O3G9Ff4v/TNY/KfDKVpYtd+T/wBvPCa+if2RLT7U3ivjO37J/wC1q/0c8XnbgnHv/r3/AOnqZ+/4WXJWjI/d"
    "Kiv4OPrj8Gf+ChFp9lXwBxjd/aH/ALbV8e1yYR3oxf8AW50YiXPUb9PyPtX/AIJuWn2pviHxnb/Z3/t1X6UV+IeJTvVw"
    "y/x/lA+yy2V8JBdr/mwrl4NJ/wBms/DuXL9b/wC3P/bz6/Kans5S+X6nx5q3g1tZEO2b7P5e7+DdnOPcelVE+FUj/wDM"
    "Qx/2w/8Asq9aUFOXNc/GvDv6RS4J4awvDzyv23sef3/bcl+epKfw+yla3Nb4ne19L2Ok8K+BW0A3G64+0+dt/wCWe3bj"
    "Puc9a9OTXZH/AOXXH/bT/wCtXkZnlCzONOPtOXkv0ve9vNdj+ffE7N14ncQ1c+5Pq/Py+5f2lrU4Q+K0L35L/D1t0u50"
    "1OR/+XfH/A//AK1eV/Fr4Et8VxpG3Vv7J+wed/y6+d5m/Z/trjGz361994Z45eHme0865fb8t/dvyXvCcPi9/wDnvt0t"
    "1uvI4WytZDOpL2nPz26Wta/m+5wKfsRSP/zOGP8AuF//AG6vVvgj8AW+ER1gtrH9sf2j5P8Ay6+R5fl+Z/ttnO/2xj3r"
    "+keMPGxcW5HiMm/s/wBl7Xl972vNblnGfw+zje/LbdWvfyP1OjilNppH3On7VEj/APMqY/7iP/2qp0/adkf/AJlfH/cQ"
    "/wDtVfhizlP7H4/8A+up46NTp+J8PftHfsqN+0MPDm3xH/wj39kfaf8Alw+0+d5vlf8ATRNuPK987u2OfG0/4JcyP/zU"
    "nH/cB/8Aums8PmKpU1T5b28/+AepTkqu7Pff2Xf2TW/ZzPiUt4k/4SP+2fs3/Lh9l8nyfN/6avuz5vtjb3zx9Ip5j/wY"
    "/Gvi+I8oXEM6c/aez5b9Oa97ea7H1mBqqlTVPcsJayP2xViDSOny0uHsj/sP2v73n57dLWtfzfc+owtZRd0eSxeNZD/z"
    "VP4gH625/wDk+rcXjKQ/81P8en625/8Ak6t1jl/0E1Pu/wDtz+AKOexf/MyxD/7d/wDu5ci8XyH/AJqX46P1gP8A8m1b"
    "i8VyH/mpHjg/WA//ACbWyxy/6CKn3f8A2571HOYv/mPrv/t3/wC7FyLxRIf+aieNT9YD/wDJlW4vEsh/5qB4yP1hP/yX"
    "Wqxq/wCgip93/wBue7QzWL/5jKr+X/3UtxeIpD/zPvi8/WE//JdXItfkP/M8+LD9Yj/8lVqsYv8An/P7v/tz3qOYxf8A"
    "zEVH8v8A7ctxa5If+Z18Un6xH/5Jq5FrMh/5nHxMfrGf/kmtVjF/z/n93/257tHGxf8Ay9k/l/8AbFuLVZD/AMzd4kP1"
    "jP8A8kVbi1KQ/wDM1eIT9UP/AMfrVYtf8/p/d/8AbHvUcTF/ab/r1LkV/If+Zm14/VD/APH6txXkh/5mPWz9UP8A8erZ"
    "Ypf8/p/d/wDbHu0asX0TLkVzIf8AmP6wfqp/+O1bimkP/Mb1U/VT/wDHa1WJX/P2X3f/AGx7tFxf/LuL/r/CfPkXiqU/"
    "81G8bH6wH/5Mq3F4mlP/ADULxofrCf8A5Mr41Zgv+gqr93/3Q/hWjxDF/wDM1xT/AO3f/vgtxeI5T/zP3jE/WE//ACXV"
    "yLxBKf8Ame/Fx+sJ/wDkqtVmC/6Cqn3f/dD3qOexf/MyxD/7d/8Au5bi12U/8zv4rP1iP/yVVyLWpT/zOfig/WI//JNa"
    "rHr/AKCan3f/AHQ92jnMX/zH13/27/8Adi3Fq8p/5nDxKfrGf/kircWpyn/mbPER+sZ/+SK2WOX/AEE1Pu/+3Pdo5rF/"
    "8xlV/L/7qXItQlP/ADNHiA/VD/8AH6txXsp/5mTXT9UP/wAerVY1f9BFT7v/ALc96jmMX/zEVH8v/ty5FdSn/mYdaP1U"
    "/wDx6rcU8p/5jurn6qf/AI7Wqxq/6CJ/d/8Abnu0cbF/8vZP5f8A2xbiklP/ADGdUP1U/wDxyrkXmn/mK6ifqD/8crVY"
    "xf8AP+f3f/bnvUcTF/ab/r1LcSSn/mJX5+oP/wAXVyKGU/8AL9eH65/+KrVYtf8AP6f3f/bnu0asX0TP/9k=";

std::vector<std::uint8_t> base64_decode(const char* text) {
    std::vector<std::uint8_t> out;
    unsigned value = 0;
    int bits = 0;
    for (const char* p = text; *p; ++p) {
        const char c = *p;
        int v = c >= 'A' && c <= 'Z' ? c - 'A' : c >= 'a' && c <= 'z' ? c - 'a' + 26
              : c >= '0' && c <= '9' ? c - '0' + 52 : c == '+' ? 62 : c == '/' ? 63 : -1;
        if (v < 0) continue;  // '=' padding
        value = (value << 6) | static_cast<unsigned>(v);
        bits += 6;
        if (bits >= 8) { bits -= 8; out.push_back(static_cast<std::uint8_t>((value >> bits) & 0xFFU)); }
    }
    return out;
}

// The embedded 4:2:2 JPEG decodes in hardware within the accepted limits of libjpeg, 30 times.
void test_embedded_jpeg_agrees_with_libjpeg() {
    const auto jpeg = base64_decode(kTestJpegBase64);
    CHECK(jpeg.size() == 3443);
    const cv::Mat reference = cv::imdecode(jpeg, cv::IMREAD_COLOR);
    CHECK(reference.cols == 128 && reference.rows == 64);
    perception::HwMjpegDecoder decoder;
    std::string error;
    CHECK(decoder.open(128, 64, error));
    if (!decoder.is_open()) { std::fprintf(stderr, "open: %s\n", error.c_str()); return; }
    double hw_ms = 0, convert_ms = 0;
    for (int i = 0; i < 30; ++i) {
        cv::Mat bgr;
        perception::HwMjpegDecoder::Timing t;
        CHECK(decoder.decode(jpeg.data(), jpeg.size(), bgr, error, &t));
        const auto a = perception::compare_bgr(bgr, reference);
        CHECK(within_limits(a));
        if (i == 0)
            std::printf("embedded: mean_abs %.3f over8 %.5f max %d\n", a.mean_abs, a.over8_share, a.max_abs);
        hw_ms += t.hw_us / 1000.0 / 30; convert_ms += t.convert_us / 1000.0 / 30;
    }
    std::printf("embedded: hw %.2f ms, convert %.2f ms per frame (wall, 30 decodes)\n", hw_ms, convert_ms);
}

// Empty input, a bad raster and use before open are refused; close and reopen work.
void test_refusals_and_reopen() {
    perception::HwMjpegDecoder decoder;
    std::string error;
    cv::Mat bgr;
    const std::uint8_t byte = 0;
    CHECK(!decoder.decode(&byte, 1, bgr, error));
    CHECK(!decoder.open(1282, 800, error));
    CHECK(!decoder.open(1280, 0, error));
    CHECK(decoder.open(1280, 800, error));
    CHECK(!decoder.decode(nullptr, 0, bgr, error));
    decoder.close();
    CHECK(!decoder.is_open());
    CHECK(decoder.open(1280, 800, error));
}

std::vector<std::uint8_t> read_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

// Manual: recorded camera frames against the SDK decodes of the same bytes.
void compare_recorded(const std::string& dir) {
    perception::HwMjpegDecoder decoder;
    std::string error;
    CHECK(decoder.open(1280, 800, error));
    double worst_mean = 0, worst_over8 = 0, hw_ms = 0, convert_ms = 0;
    int worst_max = 0, n = 0;
    for (int i = 0;; ++i) {
        const auto jpeg = read_file(dir + "/s" + std::to_string(i) + "_colour.mjpg");
        const cv::Mat sdk = cv::imread(dir + "/s" + std::to_string(i) + "_bgr.png", cv::IMREAD_COLOR);
        if (jpeg.empty() || sdk.empty()) break;
        cv::Mat bgr;
        perception::HwMjpegDecoder::Timing t;
        CHECK(decoder.decode(jpeg.data(), jpeg.size(), bgr, error, &t));
        const auto a = perception::compare_bgr(bgr, sdk);
        CHECK(within_limits(a));
        worst_mean = std::max(worst_mean, a.mean_abs); worst_over8 = std::max(worst_over8, a.over8_share);
        worst_max = std::max(worst_max, a.max_abs);
        hw_ms += t.hw_us / 1000.0;
        convert_ms += t.convert_us / 1000.0;
        ++n;
    }
    CHECK(n > 0);
    if (n) std::printf("recorded %d frames vs SDK: worst mean_abs %.3f, worst over8 %.5f, max %d; "
                       "hw %.2f ms, convert %.2f ms per frame\n",
                       n, worst_mean, worst_over8, worst_max, hw_ms / n, convert_ms / n);
}

}  // namespace

int main(int argc, char** argv) {
    test_embedded_jpeg_agrees_with_libjpeg();
    test_refusals_and_reopen();
    if (argc > 1) compare_recorded(argv[1]);
    if (failures) {
        std::fprintf(stderr, "test_hw_mjpeg_decoder: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("test_hw_mjpeg_decoder: PASS\n");
    return EXIT_SUCCESS;
}
