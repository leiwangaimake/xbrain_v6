/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_semantic_profile.cpp
 * Brief: Unit tests for semantic (S) evidence in the Profile (19 section 3.4A).
 *
 * Description:
 * What it checks. Measured Objects footprints are projected into Profile bins as S evidence. Cases
 * cover success, absent and invalid snapshots, publication guards, the commit instant (a snapshot
 * valid at entry but expired at commit is NOT put), conversion, lifecycle details, the motion
 * envelope, an S owner without a T owner (a projector built on another geometry is refused), the
 * A19-SEM-1 and A19-SEM-2 golden pairs of 19 section 14 (A19-SEM-2: bins without any depth sample
 * under a fresh footprint carry the S block and nothing else), and the publish guard (a snapshot
 * within the guard of its lifetime is Stale at the check, so the frame is published geometry-only
 * instead of being refused when the snapshot expires before the put).
 */

/* Synthetic semantic integration. Reuse the sealed normal-depth fixture;
 * its original main is retained but not invoked. No SDK, network or device. */
#define main normal_profile_fixture_unused_main
// The original main relies on C++'s implicit return 0. It is never invoked
// under this renamed symbol; keep the parent fixture source byte-identical.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#include "test_rns_normal_profile.cpp"
#pragma GCC diagnostic pop
#undef main

namespace {
std::size_t semantic_calls = 0;
struct SemanticFixture {
    Fixture f;
    SemanticOptions options{0, 0, 0, 50000000, 77};
    SemanticProjector semantic{f.geometry, options};
    NormalProfileBuilder builder{f.grid, f.t, semantic};
    SemanticObject object;
    SemanticFrame view;
    std::int64_t now = -1;
    explicit SemanticFixture(SemanticOptions custom = {0, 0, 0, 50000000, 77}) : options(custom) {
        object.track_id = 17; object.class_id = 12; object.class_name = "pit";
        rectangle(.60F, -.06F, .70F, .06F);
    }
    void rectangle(float x0, float y0, float x1, float y1) {
        object.vertices = 4;
        object.footprint[0] = {x0, y0}; object.footprint[1] = {x1, y0};
        object.footprint[2] = {x1, y1}; object.footprint[3] = {x0, y1};
    }
    void advance(bool fresh_t = false) {
        f.advance(true, fresh_t);
        now = f.capture.capture_mono_ns + 1000000;
        view = {f.capture, 77, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    }
    SemanticStatus process(const SemanticFrame* input) {
        const auto before = allocations.load();
        const auto result = semantic.process(f.mailbox, f.capture, now, input).status;
        ++semantic_calls; CHECK(allocations.load() == before); return result;
    }
    NormalProfileStatus build() {
        const auto before = allocations.load();
        const auto status = builder.build(f.mailbox, f.capture).status;
        ++build_calls; CHECK(allocations.load() == before); return status;
    }
    NormalProfilePut publish(std::int64_t at_time = -1) {
        if (at_time < 0) at_time = now;
        f.clock.mono = at_time / 1000000;
        const auto before = allocations.load();
        const auto p = builder.publish(f.mailbox, f.channel, at_time);
        ++publish_calls; CHECK(allocations.load() == before); return p;
    }
    void check() const {
        const auto* p = builder.body(); CHECK(p && p->blind_near_m);
        std::size_t sb = 0;
        for (std::size_t i = 0; i < p->bins.size(); ++i) {
            const auto& b = p->bins[i]; const auto& original = f.t.prefixes()[i];
            Bin old; CHECK(normal_profile_bin(f.grid.bins()[i], original, *f.grid.result().blind_m,
                f.geometry_options.range_max_m, f.t.report().status == TStatus::Fresh, old));
            CHECK((b.src & kG) == (old.src & kG) && b.conf == old.conf);
            if (b.d_free) CHECK(old.d_free && *b.d_free <= *old.d_free);
            if (!old.d_free) CHECK(!b.d_free);
            if (b.src & kS) {
                ++sb; CHECK(b.d_block && !b.h_block && !(b.src & kNegative));
                CHECK(semantic.distances()[i]);
                CHECK(!original.block_m || *semantic.distances()[i] < *original.block_m);
            }
            if (b.d_free && b.d_block) CHECK(*b.d_free < *b.d_block);
            if (!b.d_free || *b.d_free <= *p->blind_near_m) CHECK(!(b.src & kT) && !b.slope_deg);
        }
        CHECK(sb == builder.report().semantic_bins);
        CHECK(p->t_seg_mono_ms.has_value() == (f.t.report().status == TStatus::Fresh));
    }
    void emit(const char* name, bool emit_fixture) {
        check(); const auto p = publish(); CHECK(p.dispatched && p.publish);
        if (emit_fixture) print_fixture(name, f, p);
    }
};

void semantic_success(bool fixtures) {
    SemanticFixture s; s.advance();
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins > 0);
    s.emit("semantic_ground", fixtures);
    CHECK(!s.publish().dispatched && !s.builder.body());
    auto prior = s.view;
    s.advance(true); s.view = prior; // repeated old S is valid for new depth
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.semantic.report().age_at_capture_ns == 10000000);
    CHECK(s.build() == NormalProfileStatus::Ready); s.emit("semantic_t", fixtures);
    s.advance(); s.rectangle(-.1F, -.1F, .1F, .1F);
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.build() == NormalProfileStatus::Ready);
    std::size_t only = 0;
    for (const auto& b : s.builder.body()->bins) {
        CHECK(!b.d_free && b.d_block && *b.d_block == 0 && (b.src & kS));
        only += b.src == kS;
    }
    CHECK(only > 0); s.emit("semantic_near_no_g", fixtures);
}
void semantic_absent(bool fixtures) {
    SemanticFixture s; s.advance(); auto old = s.view;
    CHECK(s.process(nullptr) == SemanticStatus::Missing);
    CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
    s.emit("semantic_missing", fixtures);
    for (int i = 0; i < 6; ++i) s.advance();
    CHECK(s.process(&old) == SemanticStatus::Stale);
    CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
    s.emit("semantic_stale", fixtures);
    s.advance(); s.view.count = 0; s.view.objects = nullptr;
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
    s.check();
    s.advance(); s.object.class_name = "traversable_area";
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.semantic.report().excluded_t_objects == 1);
    CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
}
void semantic_invalid() {
    SemanticFixture s;
    for (int variant = 0; variant < 8; ++variant) {
        s.advance();
        auto v = s.view;
        if (variant == 0) v.stamp.clock_epoch++;
        if (variant == 1) v.base_revision++;
        if (variant == 2) v.material = SemanticMaterial::ImageBox;
        if (variant == 3) v.stamp.capture_mono_ns = s.now + 1;
        if (variant == 4) v.stamp.source.depth_device_us++;
        if (variant == 5) v.count = 513;
        if (variant == 6) v.objects = nullptr;
        if (variant == 7) v.stamp.source.stream_id++;
        const auto status = s.process(&v);
        CHECK(status != SemanticStatus::Fresh && status != SemanticStatus::Missing && status != SemanticStatus::Stale);
        for (const auto& d : s.semantic.distances()) CHECK(!d);
        CHECK(s.build() != NormalProfileStatus::Ready && !s.builder.body());
        const auto before = s.f.sink.calls; CHECK(!s.publish().dispatched && s.f.sink.calls == before);
    }
    s.advance(); std::array<SemanticObject, 2> duplicates{s.object, s.object};
    s.view.objects = duplicates.data(); s.view.count = duplicates.size();
    CHECK(s.process(&s.view) == SemanticStatus::InvalidInput);
    s.advance(); s.object.footprint[2] = s.object.footprint[0];
    CHECK(s.process(&s.view) == SemanticStatus::InvalidInput);
    s.advance(); s.rectangle(.6F, -.06F, .7F, .06F); s.object.class_name = {};
    CHECK(s.process(&s.view) == SemanticStatus::InvalidInput);
}
void semantic_publication_guards() {
    SemanticFixture s; s.advance();
    CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
    CHECK(!s.publish(s.f.capture.capture_mono_ns + 50000001).dispatched && !s.builder.body());
    s.advance(); CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
    // Re-processing the same depth fails closed and advances the content revision.
    const auto revision = s.semantic.report().revision;
    s.process(nullptr); CHECK(s.semantic.report().revision > revision);
    CHECK(!s.publish().dispatched && !s.builder.body());
    s.advance(); CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
    s.f.mailbox.end_stream(); CHECK(!s.publish().dispatched && !s.builder.body());
}
// Reply batch 7, B7-1: the S lifetime must hold at the ACTUAL commit instant,
// not only at the publish entry. Serialization and envelope assembly sit
// between the two, so a caller's earlier "now" must not keep expired evidence
// valid. The channel samples its own clock at put-before and rechecks there.
void semantic_commit_instant() {
    const std::int64_t kMaxAge = 50000000;                 // 50 ms (fixture options)
    // entry valid (40 ms old), commit expired (60 ms old): NOT put
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
        CHECK(s.builder.report().semantic_bins > 0);
        const auto t_obj = s.view.stamp.capture_mono_ns;
        s.f.clock.mono = (t_obj + kMaxAge + 10000000) / 1000000;  // the channel's own clock at commit
        const auto calls = s.f.sink.calls;
        const auto p = s.builder.publish(s.f.mailbox, s.f.channel, t_obj + 40000000);  // an older entry time
        ++publish_calls;
        CHECK(p.dispatched && !p.publish && p.publish.error == PublishError::SnapshotInvalid);
        CHECK(!p.publish.audit.attempted && s.f.sink.calls == calls);  // nothing reached the transport
    }
    // entry valid and commit valid: put
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
        const auto t_obj = s.view.stamp.capture_mono_ns;
        s.f.clock.mono = (t_obj + 40000000) / 1000000;
        const auto calls = s.f.sink.calls;
        const auto p = s.builder.publish(s.f.mailbox, s.f.channel, t_obj + 40000000);
        ++publish_calls;
        CHECK(p.dispatched && p.publish && s.f.sink.calls == calls + 1);
    }
    // exactly at the lifetime boundary (floored ms): still valid, never looser
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh && s.build() == NormalProfileStatus::Ready);
        const auto t_obj = s.view.stamp.capture_mono_ns;
        s.f.clock.mono = t_obj / 1000000 + kMaxAge / 1000000;       // age == max_age in ms
        const auto p = s.builder.publish(s.f.mailbox, s.f.channel, t_obj + 1000000);
        ++publish_calls;
        CHECK(p.dispatched && p.publish);
        SemanticFixture late; late.advance();
        CHECK(late.process(&late.view) == SemanticStatus::Fresh && late.build() == NormalProfileStatus::Ready);
        late.f.clock.mono = late.view.stamp.capture_mono_ns / 1000000 + kMaxAge / 1000000 + 1;
        const auto q = late.builder.publish(late.f.mailbox, late.f.channel, late.view.stamp.capture_mono_ns + 1000000);
        ++publish_calls;
        CHECK(q.dispatched && !q.publish && q.publish.error == PublishError::SnapshotInvalid);
    }
    // a body WITHOUT S bins does not depend on the snapshot: no guard applies
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(nullptr) == SemanticStatus::Missing && s.build() == NormalProfileStatus::Ready);
        CHECK(s.builder.report().semantic_bins == 0);
        s.f.clock.mono = s.f.capture.capture_mono_ns / 1000000 + 1000;   // one second later
        const auto p = s.builder.publish(s.f.mailbox, s.f.channel, s.f.capture.capture_mono_ns + 1000000);
        ++publish_calls;
        CHECK(p.dispatched && p.publish);
    }
}
void semantic_conversion() {
    GridBinEvidence e; e.valid_returns = 10; e.expected_rays = 20;
    BinWalkResult r; r.valid = true; r.boundary = BoundaryKind::Positive; r.block_m = .5;
    Bin b; CHECK(normal_profile_bin(e, r, .1, 6, false, b, true));
    CHECK(b.src == (kG | kS) && b.conf == 127 && !b.h_block && !b.d_free);
    CHECK(!normal_profile_bin(e, r, .1, 6, false, b));
    e = {}; CHECK(normal_profile_bin(e, r, .1, 6, false, b, true));
    CHECK(b.src == kS && b.conf == 0 && !b.d_free);
    r.block_height_m = -.2; CHECK(!normal_profile_bin(e, r, .1, 6, false, b, true));
    CHECK(empty(b));
}
void semantic_lifecycle_details() {
    {
        SemanticFixture s; s.advance();
        s.now = s.f.capture.capture_mono_ns + s.options.max_age_ns;
        CHECK(s.process(&s.view) == SemanticStatus::Fresh);
        CHECK(s.semantic.report().age_at_capture_ns == 0);
        CHECK(s.semantic.report().age_at_check_ns == s.options.max_age_ns);
        CHECK(s.semantic.usable_at(s.now) && !s.semantic.usable_at(s.now + 1));
        CHECK(!s.semantic.usable_at(s.now - 1));
    }
    {
        SemanticFixture s; s.advance();
        s.now = s.f.capture.capture_mono_ns + s.options.max_age_ns + 1;
        CHECK(s.process(&s.view) == SemanticStatus::Stale);
        CHECK(s.semantic.report().objects && s.semantic.report().age_at_capture_ns == 0);
        CHECK(s.semantic.report().age_at_check_ns == s.options.max_age_ns + 1);
        CHECK(s.semantic.usable_at(s.now) && s.semantic.usable_at(s.now + 1));
        CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
    }
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh); auto old = s.view;
        s.advance(); ++old.stamp.capture_mono_ns;
        CHECK(s.process(&old) == SemanticStatus::InvalidIdentity); // same raw source, recaptured timestamp
        CHECK(!s.semantic.report().objects);
    }
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh); auto old = s.view;
        s.advance(); CHECK(s.process(&s.view) == SemanticStatus::Fresh);
        s.advance(); CHECK(s.process(nullptr) == SemanticStatus::Missing);
        s.advance(); CHECK(s.process(&old) == SemanticStatus::InvalidIdentity);
        for (const auto& d : s.semantic.distances()) CHECK(!d);
    }
    {
        SemanticFixture s; s.advance();
        CHECK(s.process(&s.view) == SemanticStatus::Fresh); auto old = s.view;
        s.advance(); s.now += s.options.max_age_ns;
        CHECK(s.process(&s.view) == SemanticStatus::Stale);
        const auto previous_now = s.now;
        s.advance(); s.now = previous_now + 1;
        CHECK(s.process(&old) == SemanticStatus::InvalidIdentity); // reject rollback past observed stale header
    }
    {
        SemanticFixture s; s.advance(); CHECK(s.process(&s.view) == SemanticStatus::Fresh);
        s.advance(); nd::Mailbox other(kGeometryPixels); CHECK(other.begin_stream() == s.f.mailbox.active_stream());
        const auto before = allocations.load();
        CHECK(s.semantic.process(other, s.f.capture, s.now, &s.view).status == SemanticStatus::InactiveOrStale);
        ++semantic_calls; CHECK(allocations.load() == before);
        CHECK(!s.semantic.usable_at(s.now));
    }
    {
        SemanticFixture s; s.advance();
        std::array<SemanticObject, 2> objects{s.object, s.object};
        objects[1].track_id++; objects[1].class_name = std::string_view("\xc0\x80", 2);
        s.view.objects = objects.data(); s.view.count = objects.size();
        CHECK(s.process(&s.view) == SemanticStatus::InvalidInput);
        CHECK(s.semantic.report().projected_objects == 0 && s.semantic.report().touched_bins == 0);
        for (const auto& d : s.semantic.distances()) CHECK(!d);
    }
}
void semantic_motion_envelope() {
    SemanticFixture stationary, moving({1, 0, 0, 50000000, 77}), rotating({0, 1, 0, 50000000, 77});
    stationary.advance(); moving.advance(); rotating.advance();
    const auto stationary_old = stationary.view, moving_old = moving.view, rotating_old = rotating.view;
    stationary.advance(); moving.advance(); rotating.advance();
    CHECK(stationary.process(&stationary_old) == SemanticStatus::Fresh);
    CHECK(moving.process(&moving_old) == SemanticStatus::Fresh);
    CHECK(rotating.process(&rotating_old) == SemanticStatus::Fresh);
    CHECK(stationary.semantic.report().maximum_margin_m == 0);
    CHECK(moving.semantic.report().maximum_margin_m >= .01);
    CHECK(rotating.semantic.report().maximum_margin_m > 0);
    CHECK(moving.semantic.report().age_at_capture_ns == 10000000);
    CHECK(moving.semantic.report().age_at_check_ns == 11000000);
    for (std::size_t b = 0; b < stationary.semantic.distances().size(); ++b) {
        if (!stationary.semantic.distances()[b]) continue;
        CHECK(moving.semantic.distances()[b] && rotating.semantic.distances()[b]);
        CHECK(*moving.semantic.distances()[b] <= *stationary.semantic.distances()[b]);
        CHECK(*rotating.semantic.distances()[b] <= *stationary.semantic.distances()[b]);
    }
    CHECK(moving.build() == NormalProfileStatus::Ready); moving.check();
    CHECK(rotating.build() == NormalProfileStatus::Ready); rotating.check();
}
// ---- S owner without a T owner (19 section 3.4A; service wiring 2e) --------
// The S channel is carried by src bit2 and does not depend on segmentation.
// Same fixture, but the builder owns geometry + S only: t_seg stays null, no
// bin carries kT, and a geometry-only FREE prefix is still shortened by a
// strictly closer S boundary.
struct SemanticNoTFixture {
    Fixture f;
    SemanticOptions options{0, 0, 0, 50000000, 77};
    SemanticProjector semantic{f.geometry, options};
    NormalProfileBuilder builder{f.grid, semantic};
    SemanticObject object;
    SemanticFrame view;
    std::int64_t now = -1;
    SemanticNoTFixture() {
        object.track_id = 21; object.class_id = 3; object.class_name = "chair";
        rectangle(.60F, -.06F, .70F, .06F);
    }
    void rectangle(float x0, float y0, float x1, float y1) {
        object.vertices = 4;
        object.footprint[0] = {x0, y0}; object.footprint[1] = {x1, y0};
        object.footprint[2] = {x1, y1}; object.footprint[3] = {x0, y1};
    }
    // process_t stays true so the fixture's own T owner still runs: the builder
    // must ignore it because it does not own it, not because it is absent.
    void advance() {
        f.advance(true, true);
        now = f.capture.capture_mono_ns + 1000000;
        view = {f.capture, 77, SemanticMaterial::MeasuredBaseFootprint, &object, 1};
    }
    SemanticStatus process(const SemanticFrame* input) {
        const auto before = allocations.load();
        const auto result = semantic.process(f.mailbox, f.capture, now, input).status;
        ++semantic_calls; CHECK(allocations.load() == before); return result;
    }
    NormalProfileStatus build() {
        const auto before = allocations.load();
        const auto status = builder.build(f.mailbox, f.capture).status;
        ++build_calls; CHECK(allocations.load() == before); return status;
    }
};
void semantic_without_t() {
    // wrong owner: an S projector built on another geometry is refused
    {
        Fixture other;
        SemanticOptions o{0, 0, 0, 50000000, 77};
        SemanticProjector foreign{other.geometry, o};
        Fixture here;
        bool threw = false;
        try { NormalProfileBuilder rejected{here.grid, foreign}; }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }
    SemanticNoTFixture s;
    s.advance();
    CHECK(s.process(&s.view) == SemanticStatus::Fresh);
    CHECK(s.build() == NormalProfileStatus::Ready);
    CHECK(s.builder.report().semantic_bins > 0 && s.builder.report().t_bins == 0);
    CHECK(s.builder.report().semantic_status == SemanticStatus::Fresh);
    const auto* p = s.builder.body();
    CHECK(p && p->blind_near_m && !p->t_seg_mono_ms);  // S never invents a T identity
    std::size_t shortened = 0, free_bins = 0;
    for (std::size_t i = 0; i < p->bins.size(); ++i) {
        const auto& b = p->bins[i];
        // the G-only baseline for this bin: the geometry traversal, no T prefix
        Bin base;
        CHECK(normal_profile_bin(s.f.grid.bins()[i], s.f.grid.bins()[i].traversal, *s.f.grid.result().blind_m,
                               s.f.geometry_options.range_max_m, false, base));
        CHECK(!(b.src & kT));  // no T owner: the wire T bit never appears
        CHECK((b.src & kG) == (base.src & kG) && b.conf == base.conf);
        free_bins += b.d_free.has_value();
        if (b.src & kS) {
            CHECK(b.d_block && !b.h_block && !(b.src & kNegative));
            CHECK(s.semantic.distances()[i]);
            if (base.d_free && b.d_free) { CHECK(*b.d_free <= *base.d_free); shortened += *b.d_free < *base.d_free; }
            if (!base.d_free) CHECK(!b.d_free);            // S never creates FREE
        } else {  // untouched by S: bit-identical to the geometry-only bin
            CHECK(b.d_free.has_value() == base.d_free.has_value());
            if (b.d_free) CHECK(*b.d_free == *base.d_free);
            CHECK(b.slope_deg.has_value() == base.slope_deg.has_value());
            if (b.slope_deg) CHECK(*b.slope_deg == *base.slope_deg);
            CHECK(b.d_block.has_value() == base.d_block.has_value());
            if (b.d_block) CHECK(*b.d_block == *base.d_block);
        }
        if (b.d_free && b.d_block) CHECK(*b.d_free < *b.d_block);
    }
    CHECK(free_bins > 0 && shortened > 0);
    // a missing S frame leaves the geometry-only profile intact
    SemanticNoTFixture absent;
    absent.advance();
    CHECK(absent.process(nullptr) == SemanticStatus::Missing);
    CHECK(absent.build() == NormalProfileStatus::Ready && absent.builder.report().semantic_bins == 0);
    for (const auto& b : absent.builder.body()->bins) CHECK(!(b.src & kS) && !(b.src & kT));
    // an S owner that has not processed this depth frame is not bound to the
    // mailbox yet: the build is refused on identity, before any bin work
    SemanticNoTFixture unprocessed;
    unprocessed.advance();
    CHECK(unprocessed.build() == NormalProfileStatus::InvalidIdentity);
    CHECK(unprocessed.builder.body() == nullptr);
    // processed once, then a NEW depth frame without re-processing S: refused
    SemanticNoTFixture skipped;
    skipped.advance();
    CHECK(skipped.process(&skipped.view) == SemanticStatus::Fresh);
    CHECK(skipped.build() == NormalProfileStatus::Ready);
    skipped.advance();
    CHECK(skipped.build() == NormalProfileStatus::InvalidEvidence);
    CHECK(skipped.builder.body() == nullptr);
}
// ---- A19-SEM-1 golden pair (19 section 14) --------------------------------
// The glass-door scene: depth is invalid across a sector (nothing returns), the
// detector still sees the door and its measured footprint lands in that hole.
// Positive: those bins must carry BIT_SEM and the injected d_block.
// Negative: once the snapshot is older than the staleness bound, the injection
// stops - the obstacle must NOT stay pinned after the object leaves.
void a19_sem1() {
    const auto hole_columns = [](Fixture& f, std::size_t from, std::size_t to) {
        for (std::size_t y = 0; y < 400; ++y)
            for (std::size_t x = from; x < to; ++x) f.raster[at(x, y)] = 0;
    };
    // The same scene is built twice: with an S owner (injection on) and without
    // it (injection off). Off is the mutant this golden is paired against.
    SemanticNoTFixture on;
    hole_columns(on.f, 280, 360);
    on.rectangle(.55F, -.08F, .65F, .08F);         // the door, in front of the hole
    on.advance();
    CHECK(on.process(&on.view) == SemanticStatus::Fresh);
    CHECK(on.build() == NormalProfileStatus::Ready);
    const auto* body = on.builder.body();
    CHECK(body);
    std::size_t hole_bins = 0, injected = 0;
    for (std::size_t i = 0; i < body->bins.size(); ++i) {
        const auto& b = body->bins[i];
        const auto& e = on.f.grid.bins()[i];
        const bool blind_to_geometry = e.valid_returns == 0 && e.valid_pierce_rays == 0;
        if (!blind_to_geometry) continue;
        ++hole_bins;
        if (!on.semantic.distances()[i]) continue;   // the footprint does not cover this bearing
        ++injected;
        // src carries the semantic bit, and only it: no geometry, no T, no NEG
        CHECK(b.src == kS);
        CHECK(b.d_block && !b.h_block && !b.d_free && !b.slope_deg);
        // the wire value IS the injected distance (rounded toward the nearer bin edge)
        CHECK(static_cast<double>(*b.d_block) <= *on.semantic.distances()[i]);
        CHECK(*b.d_block > *on.semantic.distances()[i] - 1e-3);
    }
    CHECK(hole_bins > 0 && injected > 0);
    // paired mutant: the very same scene without the S owner is invisible
    SemanticNoTFixture off;
    hole_columns(off.f, 280, 360);
    off.advance();
    NormalProfileBuilder geometry_only{off.f.grid};
    CHECK(geometry_only.build(off.f.mailbox, off.f.capture).status == NormalProfileStatus::Ready);
    const auto* plain = geometry_only.body();
    CHECK(plain);
    std::size_t plain_hole = 0;
    for (std::size_t i = 0; i < plain->bins.size(); ++i) {
        const auto& e = off.f.grid.bins()[i];
        if (e.valid_returns || e.valid_pierce_rays) continue;
        ++plain_hole;
        CHECK(plain->bins[i].src == 0 && !plain->bins[i].d_block);  // the door is invisible
    }
    CHECK(plain_hole > 0);
    // negative half: the object leaves, so no new snapshot is published; the one
    // in the slot ages out and the injection stops (it is not pinned)
    const auto door = on.view;                                     // the last snapshot ever published
    for (int i = 0; i < 6; ++i) on.advance();                      // 60 ms > max_age 50 ms
    CHECK(on.process(&door) == SemanticStatus::Stale);
    CHECK(on.build() == NormalProfileStatus::Ready && on.builder.report().semantic_bins == 0);
    const auto* aged = on.builder.body();
    CHECK(aged);
    std::size_t still_blind = 0;
    for (std::size_t i = 0; i < aged->bins.size(); ++i) {
        const auto& e = on.f.grid.bins()[i];
        if (e.valid_returns || e.valid_pierce_rays) continue;
        ++still_blind;
        CHECK(!(aged->bins[i].src & kS) && !aged->bins[i].d_block);
    }
    CHECK(still_blind > 0);
    for (const auto& d : on.semantic.distances()) CHECK(!d);
}
// ---- A19-SEM-2 golden pair (19 section 14; sample row of 11 section 3.1B.1) ----
// The frameless glass door: whole bins have no depth sample at all (no door frame, so no solid
// point either) and a fresh measured footprint has its near edge 2.5 m straight ahead. Every such
// bin under the footprint must read d_free null / d_block 2.5 / h_block null / src 0b0100: S gives
// the block only, never FREE. The two mutants of the row:
//   forward - inject only where G = 1: these bins lose the block and the door is invisible;
//   reverse - d_free = blind_near_m after injecting: an interval called verified with no sample
//             behind it.
// The expected distance comes from the scene, not from the projector: a sector whose bearing
// nearest to straight ahead is theta sees the near edge x = 2.5 m at 2.5 / cos(theta).
void a19_sem2() {
    const float kNearEdgeM = 2.5F, kHalfWidthM = .08F;
    const auto glass_door = [&](auto& s) {   // the hole of A19-SEM-1 and a footprint 2.5 m ahead
        for (std::size_t y = 0; y < 400; ++y)
            for (std::size_t x = 280; x < 360; ++x) s.f.raster[at(x, y)] = 0;
        s.rectangle(kNearEdgeM, -kHalfWidthM, kNearEdgeM + .1F, kHalfWidthM);
    };
    // Compares every bin of one built frame with the scene.
    const auto expect_rows = [&](const auto& s) {
        const auto* body = s.builder.body();
        // The frame has a ground reference and the footprint is beyond the blind range, so the
        // empty interval that ends at blind_near_m could be expressed: null is not forced.
        CHECK(body && body->blind_near_m && *body->blind_near_m < kNearEdgeM);
        const double step = body->angle_step_rad;
        const double edge_bearing = std::atan2(double{kHalfWidthM}, double{kNearEdgeM});
        std::size_t rows = 0, straight = 0, free_bins = 0;
        for (std::size_t i = 0; i < body->bins.size(); ++i) {
            const auto& b = body->bins[i];
            const auto& e = s.f.grid.bins()[i];
            const bool no_sample = e.valid_returns == 0 && e.valid_pierce_rays == 0;
            // the bearing inside this bin's sector that is nearest to straight ahead
            const double centre = body->angle_min_rad + static_cast<double>(i) * step;
            const double inner = std::max(std::abs(centre) - step / 2, 0.0);
            if (inner > edge_bearing) {              // the footprint is not in this sector
                CHECK(!(b.src & kS));
                if (no_sample) CHECK(empty(b));      // the rest of the hole: all null, src 0
                free_bins += b.d_free.has_value();
                continue;
            }
            ++rows; straight += inner == 0;
            CHECK(no_sample);                        // the scene: the footprint lies inside the hole
            // forward half: the block is there, from S alone (no G, T or NEG) and without a height
            CHECK(b.src == kS);
            CHECK(b.d_block && !b.h_block);
            // reverse half: no FREE, not even the empty interval that ends at blind_near_m
            CHECK(!b.d_free && !b.slope_deg);
            // d_block is the near edge as this sector sees it, rounded toward the robot
            const double nearest = kNearEdgeM / std::cos(inner);
            CHECK(static_cast<double>(*b.d_block) <= nearest);
            CHECK(*b.d_block > nearest - 1e-3);
            CHECK(s.semantic.distances()[i] && static_cast<double>(*b.d_block) <= *s.semantic.distances()[i]);
            // the sample row itself: straight ahead the wire reads 2.5, or the float just below it
            // (the projector rounds its distance down)
            if (inner == 0) CHECK(*b.d_block <= 2.5F && *b.d_block > 2.499F);
        }
        // not vacuous: the row is there, 2.5 m straight ahead is among them, S is nowhere else, and
        // the bins beside the hole still have FREE (null is per bin, the frame is a normal one)
        CHECK(rows > 0 && straight == 1 && free_bins > 0);
        CHECK(s.builder.report().semantic_bins == rows);
    };
    // S owner without a T owner, as in A19-SEM-1
    SemanticNoTFixture plain;
    glass_door(plain);
    plain.advance();
    CHECK(plain.process(&plain.view) == SemanticStatus::Fresh);
    CHECK(plain.build() == NormalProfileStatus::Ready);
    expect_rows(plain);
    // The same scene with a T owner and a fresh mask over the whole image: bit0 still stays 0 in
    // these bins (there is no FREE to back), and the frame passes the wire validator and is put.
    SemanticFixture with_t;
    glass_door(with_t);
    with_t.advance(true);
    CHECK(with_t.process(&with_t.view) == SemanticStatus::Fresh);
    CHECK(with_t.build() == NormalProfileStatus::Ready);
    CHECK(with_t.builder.body()->t_seg_mono_ms && with_t.builder.report().t_bins > 0);
    expect_rows(with_t);
    with_t.check();
    const auto put = with_t.publish();
    CHECK(put.dispatched && put.publish);
}
// 2026-10-10: the publish guard. Lifetime 50 ms, guard 10 ms: a snapshot older than 40 ms at
// the check is Stale (flagged stale_by_guard), its frame is built without S and can be put at
// any later instant. Without the guard the same frame is Fresh at the check and its body is
// refused once the snapshot has crossed the lifetime at the publish entry.
void semantic_publish_guard() {
    const SemanticOptions guarded{0, 0, 0, 50000000, 77, 10000000};
    // exactly at the guard edge: Fresh, and still inside the lifetime 10 ms later
    {
        SemanticFixture s(guarded); s.advance();
        s.now = s.f.capture.capture_mono_ns + 40000000;
        CHECK(s.process(&s.view) == SemanticStatus::Fresh && !s.semantic.report().stale_by_guard);
        CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins > 0);
        const auto p = s.publish(s.now + 10000000);
        CHECK(p.dispatched && p.publish);
    }
    // one nanosecond inside the guard: Stale by guard, no distances, a geometry-only body that
    // is put even after the snapshot's lifetime has passed
    {
        SemanticFixture s(guarded); s.advance();
        s.now = s.f.capture.capture_mono_ns + 40000001;
        CHECK(s.process(&s.view) == SemanticStatus::Stale && s.semantic.report().stale_by_guard);
        CHECK(s.semantic.report().objects && s.semantic.report().age_at_check_ns == 40000001);
        for (const auto& d : s.semantic.distances()) CHECK(!d);
        CHECK(s.semantic.usable_at(s.now) && s.semantic.usable_at(s.now + 1000000000));
        CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins == 0);
        const auto p = s.publish(s.f.capture.capture_mono_ns + 60000000);
        CHECK(p.dispatched && p.publish);
    }
    // past the lifetime: Stale, but not because of the guard
    {
        SemanticFixture s(guarded); s.advance();
        s.now = s.f.capture.capture_mono_ns + 50000001;
        CHECK(s.process(&s.view) == SemanticStatus::Stale && !s.semantic.report().stale_by_guard);
    }
    // the same check instant without a guard: Fresh, and the body is refused at a publish entry
    // past the lifetime (the frame the guard saves)
    {
        SemanticFixture s; s.advance();
        s.now = s.f.capture.capture_mono_ns + 40000001;
        CHECK(s.process(&s.view) == SemanticStatus::Fresh && !s.semantic.report().stale_by_guard);
        CHECK(s.build() == NormalProfileStatus::Ready && s.builder.report().semantic_bins > 0);
        CHECK(!s.publish(s.f.capture.capture_mono_ns + 50000001).dispatched && !s.builder.body());
    }
    // a guard equal to the lifetime keeps only a snapshot of the check instant itself
    {
        SemanticFixture s(SemanticOptions{0, 0, 0, 50000000, 77, 50000000}); s.advance();
        s.now = s.f.capture.capture_mono_ns;
        CHECK(s.process(&s.view) == SemanticStatus::Fresh);
        s.advance(); s.now = s.f.capture.capture_mono_ns + 1;
        CHECK(s.process(&s.view) == SemanticStatus::Stale && s.semantic.report().stale_by_guard);
    }
    // a negative guard, or one longer than the lifetime, is a configuration error
    {
        Fixture f;
        for (const std::int64_t guard : {std::int64_t{-1}, std::int64_t{50000001}}) {
            bool refused = false;
            try { SemanticProjector bad(f.geometry, SemanticOptions{0, 0, 0, 50000000, 77, guard}); }
            catch (const std::invalid_argument&) { refused = true; }
            CHECK(refused);
        }
    }
}
}
int main(int argc, char** argv) {
    const bool fixtures = argc == 2 && std::string_view(argv[1]) == "--fixtures";
    semantic_success(fixtures); semantic_absent(fixtures); semantic_invalid();
    semantic_publication_guards(); semantic_conversion(); semantic_lifecycle_details(); semantic_motion_envelope();
    semantic_without_t(); a19_sem1(); a19_sem2(); semantic_commit_instant(); semantic_publish_guard();
    if (!fixtures) std::cout << "semantic profile 12/12 groups PASS (incl. A19-SEM-1 and A19-SEM-2 golden pairs, "
        "commit-instant lifetime, publish guard); no-allocation semantic=" << semantic_calls
        << " build=" << build_calls << " publish=" << publish_calls << '\n';
}
