"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop_pong_stub.py
Brief: dev stub -- answer probe/estop/ping so estop_path leaves "down"

Description:
p5 每秒往 probe/estop/ping 发一次探活, 等 probe/estop/pong. 按 11 S2.2 那一行,
pong 的发布者是 chassis_relay(CR-3, 转发自 rt/safety/probe/pong) -- 一个 C++
进程, 目前既没在跑也没编出来(ros2_ws 下只有 sensor). 没有应答 =>
EstopProbe 判 estop_path=down => 甲方 Qt 的急停按钮置灰 => 云端急停这条路
在联调里根本走不到.

本 stub 只做一件事: 收到 ping 就按同一个 seq 回一条 pong. 与 scripts/dev/
chassis_stub.py 同性质 -- 让缺席的下位机在开发环境里有个应答方.

*** 这不是"把 estop_path 改成恒 ok".
两者的差别是整个 EstopProbe 存在的理由: estop_probe.py 的注释逐字记着,
MVP 曾把 estop_path 硬编码成 "ok", "armed the button even with nothing
behind it". 硬编码会让链路[真断]时界面照样显示正常; 而本 stub 一停,
estop_path 立刻回到 down -- 判据仍然是活的, 只是被应答方喂着.

*** 严禁进生产.
它伪装成 chassis_relay 的一半(只应答探活, 不做任何急停动作). 真机上跑着它
的后果是: 急停链路显示 ok, 而按下急停不会解除底盘域1 -- 正是本 stub 想要
避免的那种谎报, 只是换了个地方产生. 部署前确认本进程不在.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import time

import zenoh

#: 与 p5 用同一个信封编码器. stub 手拼一份的话, 两边会在某个字段上分叉,
#: 而分叉的表现是 p5 静默收不到 -- 那正是本 stub 要消除的现象.
from xbrain.common.envelope import Envelope, encode, read_local_boot_id

_logger = logging.getLogger("xbrain.dev.estop_pong")

PING_TOPIC = "probe/estop/ping"
PONG_TOPIC = "probe/estop/pong"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    #: 信封的 rid(11 S3.0 必填). 缺省取 XBRAIN_ROBOT_ID -- p5 的 ping 用的是
    #: 同一个来源, 两边不一致的话 pong 看起来像是别的机器人发的.
    ap.add_argument("--rid", default="")
    args = ap.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    cfg = zenoh.Config()
    cfg.insert_json5("connect/endpoints", '["%s"]' % args.endpoint)
    cfg.insert_json5("scouting/multicast/enabled", "false")
    session = zenoh.open(cfg)
    pong_pub = session.declare_publisher(PONG_TOPIC)
    seen = {"n": 0}
    #: 本 stub 自己的信封计数(替身版的 RT-C3.e 重建). 与回显的 data.seq
    #: 分开自增, 见 on_ping.
    env_seq = {"n": 0}
    rid = args.rid or os.environ.get("XBRAIN_ROBOT_ID", "")
    boot = read_local_boot_id()

    def on_ping(sample) -> None:
        # *** 回显的是 data.seq, NO 不是信封 seq(11 S8.5, 2026-09-27 裁决).
        # 真身 chassis_relay 按 RT-C3.e 必须用自己的计数改写信封 seq, 所以
        # 端到端关联号只能在 data 里活下来. 本 stub 若照信封回, 它就会在
        # [没有 relay 的开发机上]恰好跑通, 一上真链路就全对不上 --
        # 一个只在替身在场时成立的判据, 正是本 stub 最不该制造的东西.
        try:
            d = json.loads(bytes(sample.payload).decode("utf-8"))
        except Exception:      # noqa: BLE001
            return
        body = d.get("data")
        if not isinstance(body, dict):
            # 裸报文(2026-09-27 之前 p5 的形态). 不猜, 记一条就走 --
            # 猜的话本 stub 会替一个违约的发布者把链路撑成 ok.
            _logger.warning("estop pong stub: ping has no S3.0 data object, "
                            "cannot correlate; not answering")
            return
        seq = body.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool):
            return
        env_seq["n"] += 1
        pong_pub.put(json.dumps(encode(Envelope(
            v=1, rid=rid,
            ts=time.time(),          # WALL-CLOCK-OK(align): S3.0 envelope ts, alignment only
            mono=time.monotonic() if boot else None,
            boot=boot or None,
            # 自己的计数, 与回显的 data.seq 是两个数 -- 这正是真链路上
            # relay 干的事, stub 照做才能暴露"拿信封 seq 匹配"的错.
            seq=env_seq["n"], src="estop_pong_stub", ts_sync=False,
            data={"type": "pong", "seq": seq,
                  "t_mono_ms": int(time.monotonic() * 1000)},
        ))).encode("utf-8"))
        seen["n"] += 1
        if seen["n"] % 60 == 1:
            _logger.info("estop pong stub alive; answered %d pings",
                          seen["n"])

    sub = session.declare_subscriber(PING_TOPIC, on_ping)
    _logger.warning("DEV STUB: answering %s -> %s. NOT for production "
                    "(it fakes the probe, not the actual estop).",
                    PING_TOPIC, PONG_TOPIC)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        del sub
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
