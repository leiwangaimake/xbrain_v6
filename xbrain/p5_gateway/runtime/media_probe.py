"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: media_probe.py
Brief: state/media 的可达性探测 -- 对相机 RTSP 端口做 TCP connect

Description:
v2.0 S4.6 的 endpoints[].state(up/down)要有来源. 这里给的是[我方视角]的
可达性: 机器人到相机这一段 TCP 554 能不能握手. 它不代表 Qt 一定拉得通 --
中间还有 NAT 与 MED-C2 源地址白名单(11 state/media 行逐字).

*** 只握手, NO 不发 RTSP 请求, NO 不认证.
两台相机都有连续认证失败锁账户的策略(docs/PTZ 实测报告 S8). 一个 5 秒一次
的探测若带认证, 密码一旦改错就会在一分钟内把账户锁死 -- 而锁死的表现是
"画面拉不到", 不是"探测失败".

* 探测在后台线程里跑, 10 Hz 的出站循环只读结果(is_up), 不被 1 s 的 connect
  超时拖住. 时间用单调钟(CLK-C1), 注入以便无设备单测.
* 从没探成功过 = down. NO 不默认 up: 开机那几秒 Qt 看到 up 去拉流, 拉不到会
  以为相机坏了.
"""
from __future__ import annotations  # 推迟注解求值

import logging  # 只在 up/down 翻转时记一条
import socket  # 只用 create_connection 做三次握手
import threading  # 后台探测线程 + 结果锁
import time  # time.monotonic, CLK-C1 单调钟
from typing import Callable, Dict, Iterable, Optional  # 纯类型注解

_logger = logging.getLogger("xbrain.p5.media_probe")  # 本模块日志名


def _tcp_connect(host: str, port: int, timeout: float) -> bool:
    """握手成功即返回 True; 失败由调用方按 OSError 处理(不在这里吞)."""
    with socket.create_connection((host, port), timeout=timeout):  # 只握手, 不发任何字节
        return True  # 连上即可, with 退出时立刻关闭


class MediaProbe:
    """targets 是 "host:port" 字符串(即 media 配置里的 probe 字段)."""

    def __init__(self, targets: Iterable[str], *, period_s: float,
                 down_after_s: float,
                 now_mono: Optional[Callable[[], float]] = None,
                 connect: Optional[Callable[[str, int, float], bool]] = None,
                 timeout_s: float = 1.0) -> None:
        self._targets = sorted(set(targets))  # 去重且顺序固定, 日志可比对
        self._period_s = period_s  # 两次探测的间隔
        self._down_after_s = down_after_s  # 距最后一次成功超过它即判 down
        self._now = now_mono or time.monotonic  # 单调钟, 单测可注入假钟
        self._connect = connect or _tcp_connect  # 单测注入假 connect, 不碰网络
        self._timeout_s = timeout_s  # 单次握手超时, 不拖住出站循环(在后台线程)
        self._last_ok: Dict[str, float] = {}  # target -> 最后一次成功的单调时刻
        self._was_up: Dict[str, bool] = {}  # target -> 上次记日志时的状态
        self._lock = threading.Lock()  # _last_ok 被探测线程写, 出站循环读
        self._stop = threading.Event()  # stop() 置位后线程在下一次 wait 退出

    def probe_once(self) -> None:
        """对每个 target 握手一次, 成功就刷新时间戳, 状态翻转时记日志."""
        for t in self._targets:  # 逐个探, 一个失败不影响其他
            host, _, port = t.rpartition(":")  # 从右切, host 里不会有冒号
            try:
                ok = bool(self._connect(host, int(port), self._timeout_s))  # 握手结果
            except (OSError, ValueError):  # 不可达 / 超时 / 端口不是数字
                ok = False  # 一律算不可达, NO 不让异常打断探测线程
            if ok:  # 只有成功才刷新, 失败不清零(窗内仍算 up)
                with self._lock:  # 与 is_up 的读互斥
                    self._last_ok[t] = self._now()  # 记成功时刻
            up = self.is_up(t)  # 按窗口重新判一次
            if self._was_up.get(t) != up:  # 只在翻转时记
                # 只在翻转时记, 5 s 一拍的周期日志会把真正的变化淹掉.
                _logger.info("p5 media probe %s -> %s", t, "up" if up else "down")
                self._was_up[t] = up  # 记住本次状态

    def is_up(self, target: str) -> bool:
        """窗口内有过一次成功即 up; 从没成功过 / 未配置的 target 都是 down."""
        with self._lock:  # 与 probe_once 的写互斥
            last = self._last_ok.get(target)  # 没探成功过为 None
        return last is not None and self._now() - last <= self._down_after_s  # 左闭: 恰好等于窗口仍算 up

    def start(self) -> None:
        """起后台线程; daemon, 进程退出不等它."""
        threading.Thread(target=self._run, name="p5-media-probe",
                         daemon=True).start()  # 1 s 握手超时不能拖住 10 Hz 出站循环

    def stop(self) -> None:
        """让后台线程在当前等待结束后退出."""
        self._stop.set()  # _run 的 wait 立即返回

    def _run(self) -> None:
        """探测循环: 一拍出错只记日志, 下一拍照常探."""
        while not self._stop.is_set():  # stop() 之前一直探
            try:
                self.probe_once()  # 一轮全部 target
            except Exception:                    # noqa: BLE001 -- keep probing
                _logger.exception("p5 media probe tick failed")  # 记下, 不让线程死掉
            self._stop.wait(self._period_s)  # 可被 stop() 打断的睡眠
