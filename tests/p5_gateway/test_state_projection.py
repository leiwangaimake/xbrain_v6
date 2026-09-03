"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_state_projection.py
Brief: 机内状态 -> v2.0 state/* 投影的判据 (B-2)

Description:
本文件的重心是一件事: *** 无源的字段必须是 null, 不许编数.

这是 CLAUDE.md 3.1 的线上版本. 那条铁律讲的是"安全参数写 0.0 冒充已赋值
会让 v_max 静默限死整机"; 投到云端面就是: battery.soc 编一个 0 会让操作员
中止出勤, 编一个 100 会让他派机器人出长任务然后半路没电. 两种都是
fail-silent, 而且是[看起来完全正常]的那一种 -- Qt 上什么异常都没有.

*** 于是本文件的用例分两类, 缺一不可:
  正向  有源时字段确实带上了值(否则一个"全部返回 null"的实现能全绿)
  负向  无源时字段确实是 null(否则一个"什么都编一个"的实现能全绿)
只写其中一类, 另一类的空壳实现就能通过 -- 这正是 11 S14.6 那条"断言只有
负向 -> 空壳实现全绿"的形状.

Boundaries: 纯函数层. 发布与节律在 test_cloud_bridge.py.
"""
from __future__ import annotations

import math

import pytest

pytestmark = pytest.mark.no_device


POSE = {"lat": 31.2301971, "lon": 121.4732683, "alt": 8.4,
        "heading_rad": math.radians(92.5), "heading_valid": True,
        "speed_mps": 0.6, "fix_type": "rtk_fixed", "cov_h_m": 0.03}
CLOCK = {"ts_sync": True, "source": "rtk", "age_ms": 80}


def _robot(**over):
    from xbrain.p5_gateway.outbound.state_projection import robot_payload

    kw = {"robot_state": "running", "task_state": "running", "pose": POSE,
          "clock": CLOCK, "devices": None, "alarm_window_active": True}
    kw.update(over)
    return robot_payload(**kw)


# --- 无源即 null ------------------------------------------------------

def test_battery_and_storage_are_null_not_invented():
    """*** 本文件最重要的一条.

    state/power 今天没有任何机内发布者. 三种做法:
      soc: 0    -> Qt 显示电量耗尽, 操作员中止出勤
      soc: 100  -> Qt 显示满电, 操作员派长任务, 半路没电
      soc: null -> Qt 显示"未接入", 联调当天一眼看出哪一侧没做完
前两种是 fail-silent 且看起来正常. 第三种难看但真实.

    MUTATION: 把 battery 换成 {"soc": 0, ...} -> 这里红.
    """
    d = _robot()

    assert d["battery"] is None, (
        "battery 被编了一个值: %r -- 操作员会据此做出错误决定" % d["battery"])
    assert d["storage"] is None


def test_the_unsourced_list_matches_what_is_actually_null():
    """*** 清单与产出必须一致, 两个方向都要.

    正向: 清单里写的段, 产出里确实是 null.
    反向: 产出里是 null 的段, 清单里必须写着 -- 否则某天一段悄悄变成 null
          (比如上游改了字段名)而没人知道, 表现成 Qt 上那一栏突然空了.

    这条也是为了将来: 源接上以后要删 UNSOURCED_ROBOT_SECTIONS 里的对应项,
    忘了删的话本条会红, 提醒投影还在发 null.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        UNSOURCED_ROBOT_SECTIONS, robot_payload)

    d = _robot()
    actually_null = {k for k, v in d.items() if v is None}

    assert set(UNSOURCED_ROBOT_SECTIONS) == actually_null, (
        "清单 %s 与实际为 null 的段 %s 对不上"
        % (sorted(UNSOURCED_ROBOT_SECTIONS), sorted(actually_null)))
    assert robot_payload is not None


def test_no_pose_means_the_whole_gps_section_is_null():
    """*** NO 不返回一个经纬度为 0 的壳.

    (0, 0) 是几内亚湾上的一个真实坐标, Qt 会把机器人画在那里. 本项目在
    HMI 时区推导上踩过这个具体的坑 -- geo_timezone 必须显式挡掉 (0,0),
    否则它映到 Etc/GMT 而页脚钟显示一个错误时区.
    """
    d = _robot(pose=None)

    assert d["gps"] is None, "无 pose 时 gps 段不是 null: %r" % d["gps"]


def test_an_invalid_heading_is_null_not_a_converted_number():
    """*** heading_valid 为假时不返回换算值.

    一个无效航向换算成度仍然像模像样, Qt 会照着画箭头 -- 而机器人实际
    朝哪没人知道. 画错方向的箭头比不画箭头更糟: 操作员会信它.

    MUTATION: 删掉 _heading_deg 里的 heading_valid 判断 -> 这里红.
    """
    d = _robot(pose=dict(POSE, heading_valid=False))

    assert d["gps"]["heading_deg"] is None, (
        "无效航向被换算成了 %r 度" % d["gps"]["heading_deg"])


# --- 有源即带上 -------------------------------------------------------

def test_a_real_pose_lands_in_the_gps_section():
    """反向. 没有这条, 一个"gps 恒 null"的实现能让上面三条全绿 --
    而那个实现会让 Qt 永远看不到机器人在哪."""
    d = _robot()

    g = d["gps"]
    assert g["fix"] == "rtk_fixed"
    assert g["latitude"] == pytest.approx(31.2301971)
    assert g["longitude"] == pytest.approx(121.4732683)
    assert g["speed_mps"] == pytest.approx(0.6)
    assert g["accuracy_m"] == pytest.approx(0.03)


def test_heading_is_converted_from_radians_to_degrees():
    """机内是弧度, v2.0 要度. 忘了换算的话数值差 57 倍 -- 而 92.5 弧度
    取模后是个合法角度, 看不出错."""
    d = _robot()

    assert d["gps"]["heading_deg"] == pytest.approx(92.5, abs=1e-6)


#: 11 S5.1 的真实 health/summary 形状(2026-09-01 从 ORIN 总线抓的那条裁剪而来).
#: kind 混着 device 与 cap; state 是 11 S5.1 的五值, NO 不是 v2.0 的 status.
_REAL_HEALTH = {
    "allow_motion": False,
    "items": {
        "cam_rgbd": {"kind": "device", "level": "fatal", "state": "unknown"},
        "chassis":  {"kind": "device", "level": "fatal", "state": "fail",
                     "since_mono": 100.0},
        "mic":      {"kind": "device", "level": "degraded", "state": "ok",
                     "since_mono": 118.5},
        "ptz":      {"kind": "device", "level": "degraded", "state": "warn"},
        "battery":  {"kind": "cap", "level": "fatal", "state": "unknown"},
        "clock":    {"kind": "cap", "level": "fatal", "state": "fail"},
    },
}


def test_gps_fix_maps_the_internal_closed_set_to_v2():
    """*** 机内 11 S4.5 五值 -> v2.0 五值, 不同名的两个是重点.

    no_fix / single 在 v2.0 里没有同名项. 不映射的话, RTK 一上电报 no_fix
    (锁星之前必经)就把整条 state/robot 打掉 -- 而 state/robot 同时载着电量/
    定位/运动/时钟, Qt 那侧全瞎. 2026-09-02 联调现场实测: rtk_driver 一启动,
    10 Hz 刷出几百条 ProjectionError.

    *** 这条缺陷只在接了 RTK 的机器上出现. 没有 rtk_driver 时 gps 恒为 null,
    走的是 `if fix else "none"` 那条短路, 永远碰不到映射. 所以它在开发机上
    不可能被发现 -- 判据必须显式喂机内值, 不能等硬件.

    变异体: 表里删掉 no_fix 一行 => 本条红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        INTERNAL_TO_V2_GPS_FIX, GPS_FIXES, to_v2_gps_fix)

    internal = {"no_fix", "single", "dgps", "rtk_float", "rtk_fixed"}
    assert set(INTERNAL_TO_V2_GPS_FIX) == internal, (
        "与 11 S4.5 闭集不一致, 差集 %s"
        % sorted(set(INTERNAL_TO_V2_GPS_FIX) ^ internal))
    assert set(INTERNAL_TO_V2_GPS_FIX.values()) <= set(GPS_FIXES), (
        "映射落到 v2.0 闭集外: %s"
        % sorted(set(INTERNAL_TO_V2_GPS_FIX.values()) - set(GPS_FIXES)))
    # 两个不同名的必须映对.
    assert to_v2_gps_fix("no_fix") == "none"
    assert to_v2_gps_fix("single") == "gps", (
        "单点定位应映 gps(米级普通定位), 实得 %r" % to_v2_gps_fix("single"))


def test_an_off_set_gps_fix_throws():
    """表外必抛, NO 不兜底成 none.

    兜底会把一个真实的定位状态报成"无定位", 操作员据此以为失去定位而中止
    任务 -- 比抛错难查得多(CLAUDE.md 3.5).

    变异体: 改成 .get(value, "none") => 本条红.
    """
    import pytest as _pytest
    from xbrain.p5_gateway.outbound.state_projection import to_v2_gps_fix
    with _pytest.raises(Exception):
        to_v2_gps_fix("gnss_wtf")


def test_devices_come_from_the_real_health_summary_shape():
    """*** 用 11 S5.1 的真实字段名喂, NO 不用想象的形状.

    原实现找 item["status"] / item["label"] / item["age_ms"] -- 这三个键在
    11 S5.1 里一个都不存在(真名是 state / 无 / since_mono). 于是每一项都落进
    unknown 兜底, 而判据当时喂的是与实现同源的假形状, 两边一起错就都不红.
    本条喂真形状: 只要有一个字段名对不上, status 就会退化成 unknown.

    变异体: _devices_from_health 里 item.get("state") 改回 item.get("status")
    => 四项全变 unknown, 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _devices_from_health

    out = {d["id"]: d for d in _devices_from_health(_REAL_HEALTH, now_mono=120.0)}

    assert out["mic"]["status"] == "online", (
        "state=ok 应映 online, 实得 %r -- 字段名或映射表对不上"
        % out["mic"]["status"])
    assert out["chassis"]["status"] == "fault"
    assert out["cam_rgbd"]["status"] == "unknown"


def test_warn_maps_to_degraded_not_online():
    """*** warn -> degraded 是一个判断, 本条把它钉住(用户 2026-09-01 裁决).

    v2.0 S4.2 的 status 没有 warn 档. 映 online 的话, 11 S5.1A 里那些"仅记录"
    级的问题在 Qt 上完全看不见; 映 degraded 是显得比实际差, 而这个方向会被
    操作员发现并追问, 反过来不会 -- 与 17 S10.2 "绑窄立即发现/绑宽永不发现"
    同一条不对称取舍.

    变异体: 映射表里 warn 改成 online => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _devices_from_health

    out = {d["id"]: d for d in _devices_from_health(_REAL_HEALTH, now_mono=120.0)}
    assert out["ptz"]["status"] == "degraded", (
        "state=warn 映成了 %r" % out["ptz"]["status"])


def test_cap_items_are_not_devices():
    """*** v2.0 的 devices[] 是设备清单, cap 是能力, 不能混.

    11 S5.1 的 items 里 kind 分 device/cap. battery/clock/compute 这些 cap 项
    混进设备面板, Qt 上就会出现点不开也修不了的"设备". 不过滤的代价不是
    多几行, 是操作员对着一个不存在的设备排障.

    变异体: 去掉 kind != "device" 的 continue => battery/clock 出现, 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _devices_from_health

    ids = {d["id"] for d in _devices_from_health(_REAL_HEALTH, now_mono=120.0)}
    assert ids == {"cam_rgbd", "chassis", "mic", "ptz"}, (
        "cap 项混进了 devices: %s" % sorted(ids - {"cam_rgbd", "chassis", "mic", "ptz"}))


def test_last_update_ms_is_null_when_since_mono_is_absent():
    """*** 无来源必须是 null, NO 不填 0(CLAUDE.md 3.1 同一失效模式).

    since_mono 在 11 S5.1 是可选字段. 填 0 表示"刚刚更新", 与真的刚更新完全
    一样 -- 操作员据此判断"这个设备状态是新的", 而其实我们根本不知道.

    变异体: age_ms 初值改成 0 => 本条红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import _devices_from_health

    out = {d["id"]: d for d in _devices_from_health(_REAL_HEALTH, now_mono=120.0)}
    assert out["ptz"]["last_update_ms"] is None, (
        "ptz 无 since_mono 却报了 %r" % out["ptz"]["last_update_ms"])
    # 有 since_mono 的照单调钟差换算成毫秒.
    assert out["mic"]["last_update_ms"] == 1500
    assert out["chassis"]["last_update_ms"] == 20000


def test_device_status_closed_set_is_complete_both_ways():
    """*** 双向差集: 11 S5.1 的五个 state 每个都要有落点, 且落点都在 v2.0 闭集内.

    单向检查会漏掉两种缺陷: 只查"落点合法"漏掉少映一个 state(那个 state 到达
    时才抛); 只查"每个 state 有映射"漏掉映到一个 v2.0 不认的值.

    变异体: 表里删掉 warn 一行 => 左边差集非空, 本条红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        HEALTH_STATE_TO_V2_STATUS)

    internal = {"ok", "warn", "degraded", "fail", "unknown"}
    v2_status = {"online", "degraded", "offline", "fault", "unknown"}

    assert set(HEALTH_STATE_TO_V2_STATUS) == internal, (
        "机内 state 闭集(11 S5.1)与映射表不一致, 差集 %s"
        % sorted(set(HEALTH_STATE_TO_V2_STATUS) ^ internal))
    assert set(HEALTH_STATE_TO_V2_STATUS.values()) <= v2_status, (
        "映射落到了 v2.0 S4.2 闭集外的值: %s"
        % sorted(set(HEALTH_STATE_TO_V2_STATUS.values()) - v2_status))
    # *** offline 永远不会被产生 -- 机内没有对应态. 这是事实不是缺口,
    # 断言它免得将来有人"补全"一个没有来源的映射.
    assert "offline" not in set(HEALTH_STATE_TO_V2_STATUS.values()), (
        "offline 被映出来了, 但机内 11 S5.1 五值里没有它的来源")


def test_an_off_set_health_state_throws():
    """*** 表外必抛, NO 不兜底成 unknown(CLAUDE.md 3.5).

    兜底会让 11 S5.1 将来扩容的新 state 静默变成"未知设备" -- Qt 上看到一批
    莫名其妙的 unknown, 而我们这边一行日志都没有.

    变异体: to_v2_device_status 改成 .get(value, "unknown") => 本条红.
    """
    import pytest as _pytest
    from xbrain.p5_gateway.outbound.state_projection import to_v2_device_status

    with _pytest.raises(ValueError):
        to_v2_device_status("brand_new_state")


def test_devices_are_empty_list_not_null_when_none_are_found():
    """*** 空数组与 null 对 Qt 的意思不同.

    空数组说"一个设备都没发现"(这段实现了); null 说"这段我没实现".
    devices 这段是实现了的, 只是今天一个都没有 -- 发 null 会让客户以为
    我们还没做.
    """
    d = _robot(devices=None)

    assert d["devices"] == [], "无设备时发了 %r" % d["devices"]


def test_a_device_carries_all_four_required_fields():
    """v2.0 S4.2 逐字: devices[] 每项必须有 id/name/status/last_update_ms."""
    d = _robot(devices=[{"id": "cam_ptz_vis", "name": "布控球可见光",
                         "status": "online", "last_update_ms": 120}])

    dev = d["devices"][0]
    assert set(dev) == {"id", "name", "status", "last_update_ms"}


# --- 闭集 -------------------------------------------------------------

def test_a_value_outside_the_closed_set_raises():
    """CLAUDE.md 3.5: 闭集外的值必抛, NO 不静默透传.

    v2.0 S1.3 从对面重复了同一条: 接收方不得把未知枚举降级解释为某个已知
    值. 我方发出去一个闭集外的值, 就是逼 Qt 去做那件被禁止的事.
    """
    from xbrain.p5_gateway.outbound.state_projection import ProjectionError

    with pytest.raises(ProjectionError):
        _robot(robot_state="running_fast")
    with pytest.raises(ProjectionError):
        _robot(task_state="pending")
    with pytest.raises(ProjectionError):
        _robot(pose=dict(POSE, fix_type="rtk"))       # 闭集里是 rtk_fixed


def test_clock_ts_sync_is_only_true_when_literally_true():
    """*** 缺省一律 false(11 F-2 逐字), 只有字面 true 才算同步.

    rtk_driver 是唯一有权判定 ClockStatus.sync 的进程(CLK-A1); 消费者不得
    把字符串 "true" 或数字 1 升格成同步. 方向是 fail-safe: 未同步只是让
    带时间窗的规则不命中, 而误判已同步会让它们在错误的时刻命中.
    """
    from xbrain.p5_gateway.outbound.state_projection import robot_payload

    for bad in ("true", 1, "yes", None, {}):
        d = robot_payload(robot_state="idle", task_state="idle", pose=None,
                          clock={"ts_sync": bad}, devices=None,
                          alarm_window_active=False)
        assert d["clock"]["ts_sync"] is False, (
            "ts_sync=%r 被当成了同步" % bad)
    assert _robot()["clock"]["ts_sync"] is True     # 反向


# --- state/mode -------------------------------------------------------

def test_broadcast_without_a_stream_id_raises():
    """v2.0 S4.3: broadcast 时 stream_id 必填.

    没有它 Qt 没法把 state/audio 的帧对上是哪一次喊话 -- 两条 key 各说
    各话, 而按钮选中态要同时参考两条(v2.0 S4.4 逐字).
    """
    from xbrain.p5_gateway.outbound.state_projection import (ProjectionError,
                                                             mode_payload)

    with pytest.raises(ProjectionError):
        mode_payload(voice_mode="broadcast", source="cloud")
    # 反向: 带了就该通过.
    d = mode_payload(voice_mode="broadcast", source="cloud",
                     stream_id="audio-gj001-0001")
    assert d["stream_id"] == "audio-gj001-0001"


def test_mode_payload_carries_all_five_fields_even_when_null():
    """五个字段全部必填, 值可以是 null 但键不能少.

    少一个键的话 Qt 那边是 KeyError 而不是"这项未知" -- v2.0 S1.3 把
    必填字段缺失列为拒绝条件.
    """
    from xbrain.p5_gateway.outbound.state_projection import mode_payload

    d = mode_payload(voice_mode="normal", source="system")
    assert set(d) == {"voice_mode", "source", "stream_id", "entered_ts",
                      "exit_reason"}


# --- state/audio ------------------------------------------------------

def test_playing_is_derived_never_contradicts_speaker_state():
    """*** 与 ack 的 accepted 同一个道理.

    两个字段表达同一件事时它们迟早会不一致, 而一条
    {speaker_state:"fault", playing:true} 会让 Qt 的按钮亮着而喇叭是哑的.
    所以 playing 由 speaker_state 推导, 不单独传.

    MUTATION: 给 audio_payload 加一个 playing 形参 -> 本条的签名检查红.
    """
    import inspect

    from xbrain.p5_gateway.outbound.state_projection import audio_payload

    params = set(inspect.signature(audio_payload).parameters)
    assert "playing" not in params and "recording" not in params, (
        "playing/recording 成了独立形参 -- 它们迟早会与 speaker_state 打架")

    for spk, want in (("playing", True), ("fault", False),
                      ("idle", False), ("buffering", False)):
        d = audio_payload(speaker_state=spk, microphone_state="disabled")
        assert d["playing"] is want, spk
        assert d["recording"] is False


def test_recording_tracks_microphone_state():
    from xbrain.p5_gateway.outbound.state_projection import audio_payload

    d = audio_payload(speaker_state="idle", microphone_state="recording")
    assert d["recording"] is True and d["playing"] is False


# --- state/geo/manifest -----------------------------------------------

def test_geo_id_prefix_must_match_the_type():
    """*** 前缀与 type 是两条线索, 打架时 Qt 按哪条走都可能错.

    v2.0 S4.5 逐字规定 waypoint=w-, path=r-, region=f-. 一条 type 说是
    路点而 id 以 f- 开头的记录, 在 Qt 的两处代码里会被分到两个图层.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        ProjectionError, geo_manifest_payload)

    with pytest.raises(ProjectionError):
        geo_manifest_payload(manifest_rev=1, objects=[
            {"geo_id": "f-north", "type": "waypoint", "name": "x", "rev": 1,
             "latitude": 31.0, "longitude": 121.0}])


def test_a_waypoint_without_coordinates_raises():
    """v2.0 S4.5: waypoint 必须带 WGS84 坐标.

    没有坐标的路点在地图上画不出来. 静默发出去只会让 Qt 空一格, 而
    操作员看到的是"这个点不见了" -- 他会以为点被删了.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        ProjectionError, geo_manifest_payload)

    with pytest.raises(ProjectionError):
        geo_manifest_payload(manifest_rev=1, objects=[
            {"geo_id": "w-north", "type": "waypoint", "name": "北门",
             "rev": 3}])


def test_an_alarm_region_must_carry_enabled():
    """停用一个报警区与删除它是两件事(W4-F 的裁决).

    enabled 缺失时 Qt 没法区分"这个区停用了"和"这个区还在生效" -- 而后者
    会让操作员以为区域仍在保护现场.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        ProjectionError, geo_manifest_payload)

    with pytest.raises(ProjectionError):
        geo_manifest_payload(manifest_rev=1, objects=[
            {"geo_id": "f-equip", "type": "alarm_region", "name": "设备区",
             "rev": 8}])


def test_a_full_manifest_round_trips():
    """反向. 没有这条, 一个"什么都抛"的实现能让上面三条全绿."""
    from xbrain.p5_gateway.outbound.state_projection import (
        geo_manifest_payload)

    d = geo_manifest_payload(manifest_rev=12, objects=[
        {"geo_id": "w-north_gate", "type": "waypoint", "name": "北门",
         "rev": 3, "latitude": 31.2301971, "longitude": 121.4732683,
         "altitude": 8.4},
        {"geo_id": "r-route_north", "type": "recorded_path",
         "name": "北侧巡检路径", "rev": 5},
        {"geo_id": "f-alarm_equipment", "type": "alarm_region",
         "name": "设备区", "rev": 8, "enabled": True}])

    assert d["manifest_rev"] == 12 and d["full"] is True
    assert len(d["objects"]) == 3
    assert d["objects"][0]["latitude"] == pytest.approx(31.2301971)
    assert d["objects"][2]["enabled"] is True
    # recorded_path 不带坐标, 也不该被塞一个 null 坐标进去.
    assert "latitude" not in d["objects"][1]


# --- state/task -------------------------------------------------------

def test_unknown_progress_is_null_never_zero():
    """*** v2.0 S3.2 逐字: 未知时必须为 null, 禁止填 0.

    填 0 的话进度条停在最左边, 与"刚开始"完全一样 -- 操作员会以为任务
    卡住并去中止它. 这与 CLAUDE.md 3.1 那条"0.0 冒充已赋值"是同一个失效
    模式的两个面.

    MUTATION: 把 normalise_progress 的 None 分支改成返回 0.0 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.state_projection import task_item

    # 11 S4.4: progress 整块为 null(路径未展开, EX-4 门控)是常态.
    d = task_item({"task_id": "t-1", "type": "GOTO_KEYPOINT",
                   "state": "running", "progress": None})
    assert d["progress_percent"] is None, (
        "未知进度被填成了 %r" % d["progress_percent"])
    # 反向: 真有进度时必须带上, 否则一个恒 null 的实现能通过.
    # pct 是 S4.4 里的字段名(按里程算, 不按 index 算).
    d2 = task_item({"task_id": "t-1", "type": "GOTO_KEYPOINT",
                    "state": "running", "progress": {"pct": 37.5}})
    assert d2["progress_percent"] == pytest.approx(37.5)


def test_a_task_item_carries_all_eleven_keys():
    """字段全部必填(值可为 null). 少一个键 Qt 那边就是 KeyError."""
    from xbrain.p5_gateway.outbound.state_projection import task_item

    d = task_item({"task_id": "t-1", "task_type": "GOTO_KEYPOINT",
                   "state": "queued"})
    assert set(d) == {"task_id", "task_type", "state", "current_waypoint_id",
                      "completed_count", "total_count", "progress_percent",
                      "route_id", "route_rev", "started_ts", "message"}


# --- D-1: exit_reason 闭集 (审计复审) ---------------------------------

def test_exit_reason_closed_set_in_mode():
    """*** state/mode 的 exit_reason 闭集(v2.0 S4.3, 审计 D-1).

    null 允许(没有退出); 非 null 必须在 8 值里. 之前直接透传, 我方发一个
    闭集外的值 Qt 会当未知枚举(S1.3 禁降级解释).

    MUTATION: mode_payload 的 exit_reason 改回直接透传 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (ProjectionError,
                                                             mode_payload)

    # 合法值 + null 都通过.
    for good in (None, "requested", "target_left_fence", "manual_cloud"):
        d = mode_payload(voice_mode="normal", source="system",
                         exit_reason=good)
        assert d["exit_reason"] == good
    # 闭集外必抛.
    for bad in ("stopped", "cancel", "done", ""):
        with pytest.raises(ProjectionError):
            mode_payload(voice_mode="normal", source="system",
                         exit_reason=bad)


def test_exit_reason_closed_set_in_audio():
    from xbrain.p5_gateway.outbound.state_projection import (ProjectionError,
                                                             audio_payload)

    audio_payload(speaker_state="idle", microphone_state="idle",
                  exit_reason="timeout")        # 合法
    with pytest.raises(ProjectionError):
        audio_payload(speaker_state="idle", microphone_state="idle",
                      exit_reason="bogus")


# --- D-2: event 规整成 v2.0 S5.1 (审计复审) --------------------------

def test_event_payload_puts_sev_and_category_into_data():
    """*** v2.0 S5.1/S5: data.sev/category 必须存在且与 key 逐字一致.

    机内事件的 sev/category 在 key 上, 不在 data 里. 直接透传的话 Qt 按 S5
    找 data.sev 会找不到. event_payload 从 key 取写进 data.

    MUTATION: event_payload 不写 data.sev(留机内的) -> 这里红.
    """
    from xbrain.p5_gateway.outbound.state_projection import event_payload

    # 机内事件: sev/category 不在 data 里(它们在 key 上).
    d = event_payload({"eid": "evt-1", "title": "x", "src": "p3_task"},
                      sev="info", category="task")
    assert d["sev"] == "info" and d["category"] == "task"


def test_event_payload_normalises_alias_field_names():
    """*** S5 逐字"不发布 event_id/severity 别名". 机内用 event_id/src 时归一.

    MUTATION: eid 只取 internal.get("eid")(不认 event_id) -> 这里红.
    """
    from xbrain.p5_gateway.outbound.state_projection import event_payload

    # 机内老字段名 event_id / src.
    d = event_payload({"event_id": "evt-9", "src": "p2_core"},
                      sev="warn", category="comm")
    assert d["eid"] == "evt-9"                   # event_id -> eid
    assert d["source"] == "p2_core"              # src -> source
    assert "event_id" not in d and "severity" not in d


def test_event_payload_fills_the_full_s5_field_set():
    """*** S5.1 的 14 字段一个不少(缺的补 null/空数组).

    Qt 按 S5.1 读这些字段; 缺一个就是 KeyError. media/file_refs 无引用为
    空数组不是 null(S5.1 逐字).
    """
    from xbrain.p5_gateway.outbound.state_projection import event_payload

    d = event_payload({"eid": "evt-1"}, sev="info", category="system")
    assert set(d) == {"eid", "sev", "category", "state", "source", "code",
                      "title", "message", "task_id", "operator", "result",
                      "detail", "media", "file_refs"}
    assert d["media"] == [] and d["file_refs"] == []
    assert d["task_id"] is None and d["operator"] is None


def test_event_sev_and_state_closed_sets():
    """sev(info|warn|error|fatal) 与 state(active|cleared|acknowledged|
    occurred) 闭集(S5.1). 闭集外必抛.
    """
    from xbrain.p5_gateway.outbound.state_projection import (ProjectionError,
                                                             event_payload)

    with pytest.raises(ProjectionError):
        event_payload({"eid": "e"}, sev="alarm", category="task")   # alarm 非 sev
    with pytest.raises(ProjectionError):
        event_payload({"eid": "e", "state": "bogus"}, sev="info",
                      category="task")


def test_event_missing_eid_raises():
    """eid 是可靠事件的幂等 ID(S5.1 必填). 缺了 Qt 没法去重/补发关联."""
    from xbrain.p5_gateway.outbound.state_projection import (ProjectionError,
                                                             event_payload)

    with pytest.raises(ProjectionError):
        event_payload({"title": "no eid"}, sev="info", category="task")


# --- 机内 12 值 -> v2.0 7 值 的映射完备性 ---------------------------------

def test_every_internal_task_state_has_a_v2_mapping():
    """*** 双向完备性的前一半: 机内每个值都必须有落点.

    守的是 2026-09-01 联调预演实测出来的缺陷: 这张表原先根本不存在, 投影拿
    机内值直接对 v2.0 闭集求值, 于是 15 S3.2 的 12 值里有 9 个被拒 --
    pending/ready/done 这些每条任务必经的状态首当其冲, 任务在 ack 之后就从
    Qt 的 state/task 里消失(current 恒 null), 而 ERROR 以 10 Hz 刷屏.

    *** 从 schema_task 读 INTERNAL, NO 不在这里重列一份.
    重列的话 11 S4.4 扩容时两份各自漂移, 而本判据的全部价值就是不让它们漂:
    新增一个机内状态而忘了给映射, 这里立刻红.

    MUTATION: 从 INTERNAL_TO_V2_TASK_STATE 删掉任意一个键 -> 红.
    """
    from xbrain.p3_task.persistence.schema_task import TASK_STATES as INTERNAL
    from xbrain.p5_gateway.outbound.state_projection import (
        INTERNAL_TO_V2_TASK_STATE)

    missing = sorted(set(INTERNAL) - set(INTERNAL_TO_V2_TASK_STATE))
    assert not missing, (
        "机内状态 %s 没有 v2.0 落点 -- 任务进这些状态时 state/task 投影会抛, "
        "Qt 那侧表现为任务凭空消失" % missing)


def test_the_mapping_never_produces_a_value_outside_v2():
    """*** 双向完备性的后一半: 每个落点都必须在 v2.0 闭集内.

    上一条保证"都有映射", 本条保证"映射的目标合法". 少了这条, 一个把
    done 映成 "done"(而不是 "completed")的表能过上一条, 却在 task_item 的
    _closed 里抛 -- 症状与完全没有映射时一模一样.

    MUTATION: 把任意一个值改成 v2.0 闭集外的词(如 "done") -> 红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        INTERNAL_TO_V2_TASK_STATE, TASK_STATES)

    bad = {k: v for k, v in INTERNAL_TO_V2_TASK_STATE.items()
           if v not in TASK_STATES}
    assert not bad, (
        "这些映射目标不在 v2.0 S3.2 闭集 %s 内: %s" % (sorted(TASK_STATES), bad))


def test_the_three_bucket_states_are_all_reachable():
    """*** v2.0 快照的三个桶都必须有机内来源.

    分桶只取 running/queued/paused. 若某个桶没有任何机内状态映过去, 那个桶
    永远是空的 -- 而空桶与"确实没有这类任务"不可区分, 正是 CLAUDE.md 3.2
    形态1(一条永远绿的断言)在数据面的样子: Qt 上看不出区别, 没人会发现.

    MUTATION: 把 suspended 的映射从 paused 改成别的 -> paused 桶失去唯一
    来源 -> 红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        INTERNAL_TO_V2_TASK_STATE)

    produced = set(INTERNAL_TO_V2_TASK_STATE.values())
    for bucket in ("running", "queued", "paused"):
        assert bucket in produced, (
            "v2.0 快照的 %r 桶没有任何机内状态映过来, 它会恒空" % bucket)


def test_terminal_internal_states_never_map_to_paused():
    """*** 终态不许映成 paused.

    needs_review / interrupted / wait_for_power_off 都是 15 S3.2 的终态.
    paused 在 v2.0 语义里是可恢复的暂停 -- 把终态报成 paused, Qt 上会出现
    一个永远恢复不了的暂停任务, 而操作员会一直等它继续.

    这条把映射表里那个判断(三者归 failed)钉住: 换成 completed 仍过本条,
    但那是另一个方向的错(报没核实过的成功), 由映射表的注释记着理由.

    MUTATION: 把 needs_review 映成 paused -> 红.
    """
    from xbrain.p5_gateway.outbound.state_projection import (
        INTERNAL_TO_V2_TASK_STATE)

    for terminal in ("needs_review", "interrupted", "wait_for_power_off",
                     "done", "failed", "cancelled"):
        assert INTERNAL_TO_V2_TASK_STATE[terminal] != "paused", (
            "终态 %r 映成了 paused -- Qt 会显示一个恢复不了的暂停任务"
            % terminal)


def test_an_unknown_internal_state_raises_not_defaults():
    """*** 表外的值抛, NO 不兜底.

    一个"表外就返回 failed"的兜底会让 11 S4.4 将来新增的状态静默变成失败.
    要的是在这里响, 不是在 Qt 上看到一批莫名其妙的失败任务
    (CLAUDE.md 3.5: 闭集外必抛, 不得未知值降级解释).

    MUTATION: 给 to_v2_task_state 加 .get(value, "failed") -> 红.
    """
    import pytest as _pytest
    from xbrain.p5_gateway.outbound.state_projection import to_v2_task_state

    with _pytest.raises(ValueError):
        to_v2_task_state("some_future_state")


def test_an_unknown_event_severity_throws_instead_of_downgrading():
    """*** 告警面最不能做的就是"未知就当 info".

    与 to_v2_task_state 同一取舍(CLAUDE.md 3.5 越界必抛): 一个"表外返回 info"
    的兜底会把将来新增的严重级别静默降成最低级, 而这条链路上跑的是入侵 .
    急停 . 底盘故障 -- 降级解释比抛出去危险得多.

    MUTATION: 把 raise 换成 return "info" -> 本条红.
    """
    import pytest as _pytest

    from xbrain.p5_gateway.outbound.state_projection import (
        ProjectionError, to_v2_event_sev,
    )

    # 正向: 四个机内值都有像样的落点.
    assert to_v2_event_sev("info") == "info"
    assert to_v2_event_sev("warn") == "warn"
    assert to_v2_event_sev("alarm") == "error"
    assert to_v2_event_sev("fault") == "fatal"
    # 反向: 表外必抛, 且说明白是哪个值.
    with _pytest.raises(ProjectionError) as exc:
        to_v2_event_sev("critical")
    assert "critical" in str(exc.value)
    # v2.0 那一侧的词也不该被当成合法输入 -- 入参是[机内]闭集.
    with _pytest.raises(ProjectionError):
        to_v2_event_sev("error")


def test_the_two_severity_closed_sets_are_covered_both_ways():
    """完备性双向差集: 机内四值全部有映射, 且映射值全部落在 v2.0 闭集内.

    MUTATION: 从表里删掉 fault, 或把它映射成 v2.0 没有的词 -> 本条红.
    """
    from xbrain.common.enums import SEVERITY
    from xbrain.p5_gateway.outbound.state_projection import (
        EVENT_SEV, INTERNAL_TO_V2_EVENT_SEV,
    )

    missing = set(SEVERITY) - set(INTERNAL_TO_V2_EVENT_SEV)
    assert not missing, "机内 severity 未映射: %r" % sorted(missing)
    extra = set(INTERNAL_TO_V2_EVENT_SEV) - set(SEVERITY)
    assert not extra, "映射表里有机内闭集外的键: %r" % sorted(extra)
    bad = set(INTERNAL_TO_V2_EVENT_SEV.values()) - set(EVENT_SEV)
    assert not bad, "映射到了 v2.0 闭集外的值: %r" % sorted(bad)
