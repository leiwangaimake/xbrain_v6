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
import time

import zenoh

_logger = logging.getLogger("xbrain.dev.estop_pong")

PING_TOPIC = "probe/estop/ping"
PONG_TOPIC = "probe/estop/pong"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
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

    def on_ping(sample) -> None:
        # seq 必须原样回 -- p5 的 _on_estop_pong 按 seq 匹配, 一条对不上号的
        # pong 会被 EstopProbe 忽略(晚到的旧 pong 不得掩盖当前的中断).
        try:
            d = json.loads(bytes(sample.payload).decode("utf-8"))
        except Exception:      # noqa: BLE001
            return
        seq = d.get("seq")
        if not isinstance(seq, int):
            return
        pong_pub.put(json.dumps(
            {"type": "pong", "seq": seq,
             "t_mono_ms": int(time.monotonic() * 1000),
             "src": "estop_pong_stub"}).encode("utf-8"))
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
