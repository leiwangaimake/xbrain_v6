/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_a19_src_golden.cpp
 * Brief: Golden tests A19-SRC-1/A19-PROF-1b: the production bin walk and the Profile converter together.
 *
 * Description:
 * What it checks. The B8-2 rows run the PRODUCTION traversal (walk_normal_bin) and then the Profile
 * converter, so their source bits and distances come from the real path rather than a second
 * handwritten oracle; the other rows give the converter a hand-built walk result (see the note on
 * row 6 below). A19-PROF-1b itself: an obstacle in the middle of a cell gives d_free
 * <= blind, never 0.85. The empty verified interval is checked for two blind values (B8-3: it is
 * published as the blind_near_m value, not null).
 */

/* A19-SRC-1 / A19-SEM-2 golden rows (19 v1.7 section 14; 11 section 3.1B.1
 * v2.1/v2.2 "src bit definition and examples").
 *
 * Every row runs profile_bin_with_semantic(), the very per-bin function the
 * NormalProfileBuilder uses (S prefix restriction, then wire conversion). The
 * inputs are the per-bin traversal a scene produces under the 19 section 3.4
 * cell rule (cells [blind + k*dr, blind + (k+1)*dr), stop on far >= d_blk);
 * the expected values are the 11 rows.
 *
 * Row 6 (pit edge at 1.5 m) is the exception: its input is NOT what the
 * production walk gives for that scene. The row is hand-built like a positive
 * obstacle standing at 1.5 m (3 cells passed, FREE to 1.34, block 1.5). The
 * production walk puts a negative boundary at the near edge of the cell that
 * holds the pierce or in-pit evidence and ends FREE one cell earlier: 2 cells
 * passed, 1.09 / 1.34 for this scene (walk_normal_bin run on it, 2026-10-13).
 * So row 6 checks the converter only (src bits, h_block and the null pattern
 * of a Negative walk result), not the distances of a pit. A pit as a depth
 * scene through the production chain, distances included, is A19-NEG-1:
 * test_a19_neg1_pit_scene in test_rns_normal_profile.cpp.
 *
 * What is asserted exactly: src bit by bit (the A19-SRC-1 wording), the
 * d_block / h_block values and the null pattern. Two known differences from the
 * printed 11 distances are NOT silently absorbed (notes/QUESTIONS_BATCH8.md):
 *   B8-2  11 prints d_free 6.0 / 2.75; the current implementation anchors cells
 *         at blind_near (19 section 3.4, A19-PROF-1b) and drops a trailing
 *         partial cell, giving 5.84 / 2.84 for blind 0.59. Kept (reply batch 8);
 *         a non-blocking erratum goes to the contract maintainer.
 *   B8-3  (fixed, reply batch 8) a TRUE empty verified interval is emitted as
 *         d_free == blind_near_m, bit-identical to the wire blind value, never
 *         null just because 0.59 is not float32-representable.
 * With --emit <file>: also writes the encoded Profile bodies of the B8-3 group
 * (one JSON line per case) for the upstream receiver check
 * harness/profile_empty_interval_check.py.
 * Pure host test: no SDK, network, clock or allocation checks.
 */
#include "output/rns_contract.hpp"
#include "modules/normal_grid.hpp"
#include "output/rns_normal_profile.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

using namespace perception::rns;
using perception::geometry::BinWalkResult;
using perception::geometry::BoundaryKind;
using perception::geometry::CellEvidence;
using perception::geometry::GridBinEvidence;
using perception::geometry::StopReason;

namespace {
int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

constexpr double kBlind = 0.59, kRange = 6.0, kDr = 0.25;
constexpr std::size_t kCells = 21;   // floor((6.0 - 0.59) / 0.25): no trailing partial cell
const std::vector<CellEvidence> flat_cells(kCells);   // mean height 0: flat ground, slope 0

double far(std::size_t passed) { return kBlind + static_cast<double>(passed) * kDr; }
float down(double v) { auto f = static_cast<float>(v); return static_cast<double>(f) > v ? std::nextafter(f, -INFINITY) : f; }
float up(double v) { auto f = static_cast<float>(v); return static_cast<double>(f) < v ? std::nextafter(f, INFINITY) : f; }

GridBinEvidence seen(std::size_t returns = 100, std::size_t pierce = 0) {
    GridBinEvidence e; e.valid_returns = returns; e.valid_pierce_rays = pierce; e.expected_rays = 120; return e;
}
BinWalkResult free_to(std::size_t passed, bool t_backed, StopReason stop) {
    BinWalkResult r; r.valid = true; r.stop = stop; r.boundary = BoundaryKind::None;
    r.passed_cells = passed; r.free_end_m = far(passed); r.t_backed_prefix = t_backed && passed > 0;
    if (passed >= 2) r.slope_rad = 0.0;
    return r;
}

struct Row {
    const char* scene;
    GridBinEvidence evidence;
    BinWalkResult walk;
    bool fresh_t;
    std::optional<double> semantic;
    // expected wire (11 row), with the two documented B8-2 / B8-3 deviations
    std::optional<float> d_free, d_block, h_block;
    std::uint8_t src;
};

bool run(const Row& row, Bin& out) {
    return profile_bin_with_semantic(row.evidence, row.walk, flat_cells.data(), kCells, kBlind, kDr, kRange,
                                  row.fresh_t, row.semantic, out);
}

void expect_row(const Row& row) {
    Bin out;
    const bool ok = run(row, out);
    CHECK(ok);
    const bool src_ok = out.src == row.src;
    const bool free_ok = out.d_free.has_value() == row.d_free.has_value() && (!row.d_free || *out.d_free == *row.d_free);
    const bool block_ok = out.d_block.has_value() == row.d_block.has_value() && (!row.d_block || *out.d_block == *row.d_block);
    const bool h_ok = out.h_block.has_value() == row.h_block.has_value() && (!row.h_block || *out.h_block == *row.h_block);
    CHECK(src_ok); CHECK(free_ok); CHECK(block_ok); CHECK(h_ok);
    std::printf("  %-44s src=0b%d%d%d%d d_free=%s d_block=%s h_block=%s %s\n", row.scene,
                (out.src >> 3) & 1, (out.src >> 2) & 1, (out.src >> 1) & 1, out.src & 1,
                out.d_free ? std::to_string(*out.d_free).c_str() : "null",
                out.d_block ? std::to_string(*out.d_block).c_str() : "null",
                out.h_block ? std::to_string(*out.h_block).c_str() : "null",
                (ok && src_ok && free_ok && block_ok && h_ok) ? "OK" : "MISMATCH");
}

BinWalkResult box_at3() {   // 3.0 m, 0.4 m box: cells advance while far < 3.0 -> 9 cells, far 2.84
    auto r = free_to(9, true, StopReason::Boundary);
    r.boundary = BoundaryKind::Positive; r.block_m = 3.0; r.block_height_m = 0.4;
    return r;
}
}  // namespace

// ---- B8-2: the PRODUCTION traversal (walk_normal_bin), then the converter ----
// The rows above feed hand-built walk results; these run the real walk over
// uniform full-coverage flat cells, so the cell anchor (blind + k*dr), the
// far-edge stop and the dropped trailing partial cell are exercised, and the
// distance / UNKNOWN band is asserted separately from the src bits.
struct WalkCase {
    const char* name;
    double blind, range, obstacle, height;   // obstacle < 0: none
    double expect_free_end;                  // walk result, metres (double)
    std::size_t expect_passed;
};
void expect_walk(const WalkCase& w) {
    using namespace perception::geometry;
    GridOptions o; o.dr_m = kDr; o.cover_min = 0.9; o.slope_max_rad = 0.26; o.sigma_max_m = 0.05;
    const std::size_t count = full_cell_count(w.blind, w.range, kDr, kMaxRadialCells);
    std::vector<CellEvidence> cells(count);
    for (auto& c : cells) { c.expected = 10; c.ground = 10; c.covered_ground = 10; }
    BinWalkInput in; in.cells = cells.data(); in.cell_count = count; in.blind_m = w.blind; in.range_max_m = w.range;
    in.has_samples = true;
    if (w.obstacle >= 0) { in.positive_m = w.obstacle; in.positive_height_m = w.height; }
    const auto walk = walk_normal_bin(in, o);
    CHECK(walk.valid && walk.free_end_m && walk.passed_cells == w.expect_passed && *walk.free_end_m == w.expect_free_end);
    GridBinEvidence e; e.valid_returns = 400; e.expected_rays = 400;
    Bin b;
    const bool ok = profile_bin_with_semantic(e, walk, cells.data(), count, w.blind, kDr, w.range, false, std::nullopt, b);
    CHECK(ok);
    const float wire_blind = up(w.blind);
    bool pass = ok && b.d_free.has_value() && (b.src & kG) && !(b.src & kT);
    if (pass && walk.passed_cells == 0) pass = *b.d_free == wire_blind;                       // B8-3 empty interval
    if (pass && walk.passed_cells > 0) pass = *b.d_free <= w.expect_free_end && *b.d_free > wire_blind;  // conservative
    if (pass && w.obstacle >= 0) pass = b.d_block && *b.d_block <= w.obstacle && *b.d_free < *b.d_block; // UNKNOWN band kept
    if (pass && w.obstacle < 0) pass = !b.d_block && *b.d_free <= w.range;
    CHECK(pass);
    std::printf("  %-54s cells=%zu passed=%zu free_end=%.17g -> d_free=%.9g d_block=%s %s\n", w.name, count,
                walk.passed_cells, walk.free_end_m ? *walk.free_end_m : -1.0, b.d_free ? *b.d_free : -1.0F,
                b.d_block ? std::to_string(*b.d_block).c_str() : "null", pass ? "OK" : "MISMATCH");
}
void walk_cases() {
    const double one_up = std::nextafter(1.0, 2.0), one_down = std::nextafter(1.0, 0.0);
    const WalkCase cases[] = {
        // A19-PROF-1b itself: obstacle mid-cell -> d_free <= blind, never 0.85
        {"A19-PROF-1b: blind .60 dr .25 obstacle .80", 0.60, 6.0, 0.80, 0.2, 0.60, 0},
        // flat, nothing in range: 21 full cells, the partial [5.84, 6.0) gives no FREE
        {"flat, blind .59, range 6.0 (trailing partial cell)", 0.59, 6.0, -1, 0, 0.59 + 21 * kDr, 21},
        {"0.4 m box at 3.0 m, blind .59", 0.59, 6.0, 3.0, 0.4, 0.59 + 9 * kDr, 9},
        {"far edge == obstacle (blind .5, obstacle 1.0)", 0.5, 6.0, 1.0, 0.3, 0.75, 1},
        {"obstacle one double above a far edge (1.0+ulp)", 0.5, 6.0, one_up, 0.3, 1.0, 2},
        {"obstacle one double below a far edge (1.0-ulp)", 0.5, 6.0, one_down, 0.3, 0.75, 1},
        {"obstacle 1e-8 past a far edge (float-equal)", 0.5, 6.0, 1.0 + 1e-8, 0.3, 1.0, 2},
    };
    for (const auto& c : cases) expect_walk(c);
}

// ---- B8-3: the empty verified interval, for two blind values -------------
struct EmptyCase {
    std::string name;
    Bin bin;
    bool ok = false;
    const char* expect;   // "empty" = d_free == blind_near_m; "null"; "free" = non-empty FREE
};
std::vector<EmptyCase> empty_interval_cases(double blind) {
    std::vector<EmptyCase> out;
    const auto cells = [&]() { return std::vector<CellEvidence>(kCells); };
    const auto run1 = [&](const std::string& name, const GridBinEvidence& e, const BinWalkResult& r,
                          std::optional<double> s, bool ground, const char* expect) {
        EmptyCase c; c.name = name; c.expect = expect;
        const auto cv = cells();
        c.ok = profile_bin_with_semantic(e, r, ground ? cv.data() : nullptr, ground ? kCells : 0,
                                      ground ? std::optional<double>(blind) : std::nullopt, kDr, kRange,
                                      true, s, c.bin);
        out.push_back(c);
    };
    BinWalkResult first = BinWalkResult{}; first.valid = true; first.stop = StopReason::Boundary;
    first.boundary = BoundaryKind::Positive; first.free_end_m = blind; first.block_m = blind + 0.11; first.block_height_m = 0.4;
    run1("first cell stops (obstacle 0.11 m past blind)", seen(300), first, std::nullopt, true, "empty");
    BinWalkResult cover = BinWalkResult{}; cover.valid = true; cover.stop = StopReason::Coverage;
    cover.boundary = BoundaryKind::None; cover.free_end_m = blind;
    run1("glass door with frame points, S 2.5 m", seen(6), cover, 2.5, true, "empty");
    BinWalkResult none = BinWalkResult{}; none.valid = true; none.stop = StopReason::NoSamples;
    run1("S-only, no depth sample (G=0), S 2.5 m", GridBinEvidence{}, none, 2.5, true, "null");
    BinWalkResult collide = first; collide.block_m = blind + 1e-9;
    run1("blind/block collision (block 1e-9 past blind)", seen(300), collide, std::nullopt, true, "null");
    // Only meaningful when blind is not float32-representable (0.59): for 0.5
    // "block at the wire blind" IS the free end, an invalid walk the converter
    // refuses (asserted below), not a representable collision.
    BinWalkResult equal = first; equal.block_m = static_cast<double>(up(blind));
    if (static_cast<double>(up(blind)) > blind)
        run1("block exactly at the wire blind value", seen(300), equal, std::nullopt, true, "null");
    else {
        Bin refused;
        const auto cv = cells();
        CHECK(!profile_bin_with_semantic(seen(300), equal, cv.data(), kCells, blind, kDr, kRange, true,
                                      std::nullopt, refused));
    }
    BinWalkResult nonempty = BinWalkResult{}; nonempty.valid = true; nonempty.stop = StopReason::Boundary;
    nonempty.boundary = BoundaryKind::Positive; nonempty.passed_cells = 1; nonempty.free_end_m = blind + kDr;
    nonempty.block_m = blind + 0.4; nonempty.block_height_m = 0.3; nonempty.t_backed_prefix = true;
    run1("non-empty FREE (one cell passed)", seen(300), nonempty, std::nullopt, true, "free");
    return out;
}
void expect_empty_cases(double blind, std::ofstream* emit) {
    const float wire_blind = up(blind);
    auto cases = empty_interval_cases(blind);
    Profile p;
    p.t_capture_mono_ms = 1000; p.t_publish_mono_ms = 1020; p.t_seg_mono_ms = 990;
    p.z_pass_m = 0.75F; p.range_max_m = 6.0F; p.blind_near_m = wire_blind;
    p.angle_step_rad = 0.25F;
    for (const auto& c : cases) {
        CHECK(c.ok);
        const auto& b = c.bin;
        bool pass = c.ok;
        if (std::string_view(c.expect) == "empty")
            pass = pass && b.d_free && *b.d_free == wire_blind && !(b.src & kT) && !b.slope_deg && (b.src & kG) &&
                   (!b.d_block || *b.d_free < *b.d_block);
        else if (std::string_view(c.expect) == "null")
            pass = pass && !b.d_free && !(b.src & kT) && !b.slope_deg;
        else
            pass = pass && b.d_free && *b.d_free == down(blind + kDr) && *b.d_free > wire_blind;
        CHECK(pass);
        std::printf("  blind %.2f %-46s d_free=%s d_block=%s src=0x%x %s\n", blind, c.name.c_str(),
                    b.d_free ? std::to_string(*b.d_free).c_str() : "null",
                    b.d_block ? std::to_string(*b.d_block).c_str() : "null", b.src, pass ? "OK" : "MISMATCH");
        p.bins.push_back(b);
    }
    // collision rows: the boundary is NOT moved to make room
    CHECK(*cases[3].bin.d_block == down(blind + 1e-9));
    if (cases.size() == 6) CHECK(*cases[4].bin.d_block == wire_blind);
    while (p.bins.size() % 2 == 0) p.bins.push_back(Bin{});   // odd count: symmetric angular domain
    p.angle_min_rad = -0.25F * static_cast<float>((p.bins.size() - 1) / 2);
    std::vector<char> buffer(1 << 16);
    const auto encoded = encode(p, buffer.data(), buffer.size());
    CHECK(encoded);
    if (emit && encoded) {
        *emit << "{\"blind\":" << blind << ",\"expect\":[";
        for (std::size_t i = 0; i < cases.size(); ++i) *emit << (i ? "," : "") << '"' << cases[i].expect << '"';
        *emit << "],\"body\":" << std::string(buffer.data(), encoded.size) << "}\n";
    }
    // No ground reference: never a fabricated empty interval (11 v2.3, A19-NULL-1).
    Bin ng;
    BinWalkResult ngw = BinWalkResult{}; ngw.valid = true; ngw.stop = StopReason::NoGroundReference;
    CHECK(profile_bin_with_semantic(seen(300), ngw, nullptr, 0, std::nullopt, kDr, kRange, true, std::nullopt, ng));
    CHECK(!ng.d_free && (ng.src & kG));
}

int main(int argc, char** argv) {
    std::ofstream emit_file;
    if (argc == 3 && std::string_view(argv[1]) == "--emit") emit_file.open(argv[2]);
    std::printf("A19-SRC-1 golden rows (blind 0.59, range 6.0, dr 0.25)\n");
    // 1 flat ground, segmentation, nothing in range -> 0b0011
    expect_row({"flat, T, nothing in range", seen(), free_to(kCells, true, StopReason::Range), true, std::nullopt,
               down(far(kCells)), std::nullopt, std::nullopt, kT | kG});
    // 2 flat ground, NO segmentation (slow line) -> 0b0010
    expect_row({"flat, no T (slow line)", seen(), free_to(kCells, false, StopReason::Range), false, std::nullopt,
               down(far(kCells)), std::nullopt, std::nullopt, kG});
    // 3 0.4 m box at 3.0 m -> 0b0011, d_block 3.0, h 0.4
    expect_row({"0.4 m box at 3.0 m", seen(), box_at3(), true, std::nullopt,
               down(far(9)), 3.0F, 0.4F, kT | kG});
    // 4 glass door: depth invalid over the sector, frame points give samples,
    //   no geometric boundary; S footprint at 2.5 m -> 0b0110
    expect_row({"glass door, S 2.5 m, frame points", seen(6), free_to(0, false, StopReason::Coverage), true, 2.5,
               up(kBlind) /* == blind_near_m on the wire (B8-3) */, 2.5F, std::nullopt, kG | kS});
    // 5 (v2.2, A19-SEM-2) whole bin invalid + S 2.5 m -> null / 2.5 / null / 0b0100
    {
        BinWalkResult none; none.valid = true; none.stop = StopReason::NoSamples;
        expect_row({"A19-SEM-2: no depth, S 2.5 m", GridBinEvidence{}, none, true, 2.5,
                   std::nullopt, 2.5F, std::nullopt, kS});
    }
    // 6 pit edge at 1.5 m (pierce + in-pit point -0.3 m) -> 0b1011
    //   hand-built walk, not the production one for a pit (that gives 1.09 / 1.34): see the file header
    {
        auto r = free_to(3, true, StopReason::Boundary);   // far 1.34 < 1.5
        r.boundary = BoundaryKind::Negative; r.block_m = 1.5; r.block_height_m = -0.3;
        expect_row({"pit edge 1.5 m, -0.3 m", seen(90, 10), r, true, std::nullopt,
                   down(far(3)), 1.5F, -0.3F, kNegative | kT | kG});
    }
    // 7 whole bin invalid (strong reflection) -> all null, 0b0000
    {
        BinWalkResult none; none.valid = true; none.stop = StopReason::NoSamples;
        expect_row({"whole bin invalid", GridBinEvidence{}, none, true, std::nullopt,
                   std::nullopt, std::nullopt, std::nullopt, 0});
    }

    std::printf("A19-SRC-1 extra: bit1 in a face-close obstacle bin, ground hidden\n");
    // The obstacle fills the first cell: no cell passes, the ground is never
    // seen, but the obstacle points ARE geometry samples -> bit1 = 1.
    // Mutant: BIT_G set only where d_free advances -> bit1 = 0 -> red.
    {
        auto r = free_to(0, false, StopReason::Boundary);
        r.boundary = BoundaryKind::Positive; r.block_m = 0.70; r.block_height_m = 0.5;
        expect_row({"face-close obstacle 0.70 m, ground hidden", seen(400), r, true, std::nullopt,
                   up(kBlind) /* empty verified interval (B8-3) */, 0.70F, 0.5F, kG});
    }
    std::printf("A19-SRC-1 extra: S farther than the geometric boundary\n");
    // S only where d_sem < d_geom. Mutant: BIT_SEM OR-ed unconditionally -> red.
    expect_row({"box 3.0 m, S footprint 4.0 m (farther)", seen(), box_at3(), true, 4.0,
               down(far(9)), 3.0F, 0.4F, kT | kG});
    expect_row({"box 3.0 m, S footprint 3.0 m (equal)", seen(), box_at3(), true, 3.0,
               down(far(9)), 3.0F, 0.4F, kT | kG});
    // ... and the nearer case, for contrast: S tightens and is selected.
    expect_row({"box 3.0 m, S footprint 2.0 m (nearer)", seen(), box_at3(), true, 2.0,
               down(far(5)), 2.0F, std::nullopt, kT | kG | kS});

    std::printf("A19-SEM-2 reverse: an injected no-depth bin never gets FREE\n");
    {
        BinWalkResult none; none.valid = true; none.stop = StopReason::NoSamples;
        Row row{"no depth, S 2.5 m", GridBinEvidence{}, none, true, 2.5, std::nullopt, 2.5F, std::nullopt, kS};
        Bin out; CHECK(run(row, out));
        CHECK(!out.d_free && !out.slope_deg && !(out.src & kT) && !(out.src & kG));
    }
    std::printf("B8-2 production traversal walkNormalBin -> converter (distance and UNKNOWN band)\n");
    walk_cases();
    std::printf("B8-3 empty verified interval (true empty -> d_free == blind_near_m; unknown -> null)\n");
    for (const double blind : {0.5, 0.59}) expect_empty_cases(blind, emit_file.is_open() ? &emit_file : nullptr);
    std::printf("{\"result\":\"%s\",\"checks\":%d,\"failures\":%d,\"scope\":\"A19-SRC-1 rows 1-7 + face-close + S farther/equal/nearer; A19-SEM-2; B8-2 walk; B8-3\"}\n",
                failures ? "FAIL" : "PASS", checks, failures);
    return failures ? 1 : 0;
}
