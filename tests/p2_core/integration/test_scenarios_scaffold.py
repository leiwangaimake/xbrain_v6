"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scenarios_scaffold.py
Brief: integration tests -- scenarios scaffold

Description:
BIZ-P2-25 -- integration scenario scaffold + placeholder tests.

14 S12.1 lists 8 scenarios that require a FULL P2 runtime with
arbiters + mode SM + BIT + payload_io + Zenoh session all wired
together. Building this end-to-end needs:

  * a running P2 process (BIZ-P2-1 main loop + all threads)
  * a live payload-service (or a full-fidelity fake)
  * Zenoh routers up (or in-process Zenoh)

Until those land, this file provides one scenario per row as a
placeholder pytest.mark.skip test that names its exact preconditions.
The skip reason IS the deliverable at this stage; it turns 'nobody
knows what still blocks this' into 'this specific line will unlock
this specific test'.
"""


import pytest


pytestmark = pytest.mark.no_device


# --- 14 S12.1 scenarios 1..8 ---

@pytest.mark.skip(reason="requires: running P2 + arbiter grant flow + "
                          "fake payload-service /tts + gate publisher wired")
def test_scenario_1_tts_playing_then_d_triggered():
    """§12.1 场景 1: TTS 播报中 D 触发 -> WAIT_ATOMIC 超时后强制抢占;
    喇叭 <= 3s 转警笛; 产生 1 条 warn arbitration/forced_preempt
    含 detail.overdue_ms."""


@pytest.mark.skip(reason="requires: process supervision + arbiter "
                          "source_death path + fault event bus wired")
def test_scenario_2_kill_p4_holding_speaker_releases_within_1s():
    """§12.1 场景 2: kill P4 while it holds speaker -> <=1s auto
    release + fault(source_death) event."""


@pytest.mark.skip(reason="requires: mode SM + arbiter self-held + "
                          "blocked[]/self_held[] event roundtrip")
def test_scenario_3_mode_switch_with_ptz_held_by_cloud_rolls_back():
    """§12.1 场景 3: mode switch while cloud holds PTZ manually ->
    reject + full rollback + blocked[] reports all, self_held[]
    also populated."""


@pytest.mark.skip(reason="requires: mode SM + b_mode_forward wired + "
                          "rt/audio/gate publisher live")
def test_scenario_4_b_to_d_transition_domain2_only_gen_bump_no_gate_flip():
    """§12.1 场景 4: B->D transition. domain 2 gen+1, domains 3/4/5
    gen unchanged. rt/audio/gate mic_open never flips true across
    the switch. + event/info/payload{kind:strobe_gap}."""


@pytest.mark.skip(reason="requires: mode SM + 4 arbiters wired + "
                          "gen counter observation across sequence")
def test_scenario_5_bdbd_rapid_switching_no_forced_preempt():
    """§12.1 场景 5: B->D->B->D. Every transition succeeds; NO
    forced_preempt anywhere; payload_light/ptz gen unchanged
    throughout."""


@pytest.mark.skip(reason="requires: three-stops module + arb_suspend/rearm + "
                          "ForceStrobeState + 4 arbiters live")
def test_scenario_6_soft_estop_domain_behavior_10s():
    """§12.1 场景 6 (ARB-7 / U35): after cmd/estop{stop} run for 10s.
    Assertions (verbatim from spec):
      (1) 域②③④⑤ state/arb/{domain}.suspended stays null
      (2) 域② holder unchanged (D siren keeps sounding)
      (3) 域③ holder unchanged (voice still available)
      (4) 域④ red/blue strobe FORCED ON; mode_driver still holder
      (5) manual light close requests denied E_BUSY throughout
      (6) after re-arm, force_strobe_on clears; 域④ back to mode value.
    Two variants:
      * apply arb_suspend to 域②③④⑤ -> (1)(2)(3)(4) all fail
      * implement force_strobe_on as one-shot instead of max()
        composition -> (5) fails
    """


@pytest.mark.skip(reason="requires: BIZ-P2-10 LAPI write guard + PTZ arbiter + "
                          "boost 阶段代码 (removed 2026-08-05)")
def test_scenario_7_ptz_lapi_writes_never_exceed_two_keys():
    """§12.1 场景 7 (v0.8 revised): under all normal + exception paths
    (Stop lost, PTZ preempted, kill+restart, ptz turns fail), LAPI
    WRITE count for bitrate-family keys == 0."""


@pytest.mark.skip(reason="requires: full P2 lifecycle + BIT + mode SM + "
                          "8b/8d/8e sub-cases each need own fake fixture")
def test_scenario_8_shutdown_reboot_end_to_end():
    """§12.1 场景 8: L3 shutdown flow start-to-finish.
      8b: no confirm_token -> cmd/system 零发布
      8d: token stale -> reject
      8e: token OK -> systemd stop sequence
    """
