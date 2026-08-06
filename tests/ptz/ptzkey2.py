#!/usr/bin/env python3
"""
布控球方向键控制 v2 ---- 速度固定最快, 转动时保持画面质量.

  ← -> ↑ ↓   X/Y 轴转动 (速度固定 1.0), 按住持续转, 松开即停
  + -       变焦 拉近 / 拉远
  f         立刻重新对焦一次
  空格      停止
  q         退出

  --no-boost   不临时提高码率 (默认会提, 见下)
  --selftest   非交互自检

相比 v1 的三处改动, 每一处都是实测定位的:

1. 手感迟钝的真凶是 v1 的 STOP_AFTER=0.30s **短于终端键盘自动重复的首次延迟
   (约 500ms)**.按住键时: 动 0.3s -> 超时 Stop -> 重复键才到 -> 再启动, 于是
   转-停-转顿挫.v2 用自适应保持时间: 首次按下给 HOLD_FIRST(0.85s) 跨过首延迟;
   一旦收到连续重复键(说明真被按住)就切到 HOLD_REPEAT(0.18s), 松手能快速停.
   两头都不牺牲.
   (注: 每条 ONVIF 命令实测 58ms, 换成持久连接只降到 55ms ---- 这个开销在相机
    侧不在网络侧, 优化空间很小.Session 仍保留, 无害.)

2. "一转就花" 实测是编码器带宽不足, 不是对焦问题:
       6144kbps / I间隔50 -> 摇摄时清晰度仅为静止的 31%, 码率封顶在 6.4Mbps
      16384kbps / I间隔25 -> 提升到 51%
   2560x1440@25fps 快速摇摄产生大量帧间残差, CBR 封顶就宏块化, 且 I 间隔 50
   (=2秒) 意味着要等两秒才恢复.v2 默认在运行期间把码率提到 16384,I 间隔缩到
   25, 退出时恢复原值.剩下的 49% 差距是 1/100s 快门在最高速下的运动模糊, 属物理
   限制, 除非降速或提高快门(会更暗).

3. 对焦: ShieldTrigger.MovePTZ 原为 0 = 不屏蔽, 云台一动就重新对焦; 且 v1 退出时
   只调 ONVIF AutoFocusMode=AUTO, 而它**不会映射回 LAPI 的 FocusMode**, 相机会
   停在手动对焦上 -> 转到不同距离的景物必虚.v2 启动时用 LAPI 写 FocusMode=2 且
   MovePTZ=1, 退出时恢复 MovePTZ 原值并确保 FocusMode 仍为自动.
"""
import json
import os
import select
import subprocess
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onvif as O  # noqa: E402

HOST = "192.168.66.13"
AUTH = "admin:Admin123."
TOK = "media_profile1"
PTZ_PATH = "/onvif/ptz"

SPEED = 1.0
HOLD_FIRST = 0.85        # 需 > 终端自动重复首延迟, 否则出现转-停-转顿挫
HOLD_REPEAT = 0.18       # 进入自动重复后的保持时间, 越小松手停得越快
REPEAT_ARMED = 2

BOOST_BITRATE = 16384    # kbps; 机型上限
BOOST_IINTERVAL = 25

sess = O.Session(f"{HOST}:80")
cur = None
deadline = 0.0
repeats = 0
_shield_orig = None
_enc_orig = None


def lapi(path, method="GET", body=None):
    cmd = ["curl", "-s", "--digest", "-u", AUTH, "--max-time", "10",
           "-X", method, f"http://{HOST}/LAPI/V1.0/{path}"]
    if body is not None:
        cmd += ["-H", "Content-Type: application/json", "-d", json.dumps(body)]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20).stdout
        r = json.loads(out)["Response"]
        return r.get("ResponseCode"), r.get("Data")
    except Exception:                                          # noqa: BLE001
        return None, None


# ---------- 对焦 ----------
def focus_prepare():
    global _shield_orig
    code, cfg = lapi("Channels/0/Image/Focus")
    if code != 0 or not isinstance(cfg, dict):
        print("  ! 读不到对焦配置, 跳过")
        return
    _shield_orig = cfg.get("ShieldTrigger", {}).get("MovePTZ")
    was = cfg.get("FocusMode")
    cfg["FocusMode"] = 2
    cfg.setdefault("ShieldTrigger", {})["MovePTZ"] = 1
    code, _ = lapi("Channels/0/Image/Focus", "PUT", cfg)
    print(f"  对焦: FocusMode {was}->2(自动)  MovePTZ {_shield_orig}->1(运动中不重对焦)"
          f"  {'OK' if code == 0 else '失败'}")


def focus_restore():
    code, cfg = lapi("Channels/0/Image/Focus")
    if code != 0 or not isinstance(cfg, dict):
        return
    cfg["FocusMode"] = 2          # 绝不把相机留在手动对焦
    if _shield_orig is not None:
        cfg.setdefault("ShieldTrigger", {})["MovePTZ"] = _shield_orig
    lapi("Channels/0/Image/Focus", "PUT", cfg)


def refocus():
    code, cfg = lapi("Channels/0/Image/Focus")
    if code != 0 or not isinstance(cfg, dict):
        return
    cfg["FocusMode"] = 0
    lapi("Channels/0/Image/Focus", "PUT", cfg)
    time.sleep(0.3)
    cfg["FocusMode"] = 2
    lapi("Channels/0/Image/Focus", "PUT", cfg)


# ---------- 码率 ----------
def boost_prepare():
    global _enc_orig
    code, cfg = lapi("Channels/0/Media/VideoEncode")
    if code != 0 or not isinstance(cfg, dict):
        print("  ! 读不到编码配置, 跳过码率提升")
        return
    _enc_orig = json.loads(json.dumps(cfg))
    g = cfg["VideoEncoderCfg"][0]["VideoStreamCfg"]
    ob, oi = g["BitRate"], g["IInterval"]
    g["BitRate"], g["IInterval"] = BOOST_BITRATE, BOOST_IINTERVAL
    code, _ = lapi("Channels/0/Media/VideoEncode", "PUT", cfg)
    print(f"  码率: {ob}->{BOOST_BITRATE}kbps  I间隔: {oi}->{BOOST_IINTERVAL}"
          f"  {'OK' if code == 0 else '失败'}  (退出时恢复)")


def boost_restore():
    if _enc_orig is not None:
        lapi("Channels/0/Media/VideoEncode", "PUT", _enc_orig)


# ---------- PTZ ----------
def _move(pan, tilt, zoom):
    sess.call(PTZ_PATH,
              f'<ContinuousMove xmlns="{O.NS["tptz"]}">'
              f"<ProfileToken>{TOK}</ProfileToken>"
              f'<Velocity xmlns:tt="{O.NS["tt"]}">'
              f'<tt:PanTilt x="{pan}" y="{tilt}"/><tt:Zoom x="{zoom}"/>'
              f"</Velocity></ContinuousMove>")


def stop():
    global cur, repeats
    if cur is not None:
        sess.call(PTZ_PATH, f'<Stop xmlns="{O.NS["tptz"]}">'
                            f"<ProfileToken>{TOK}</ProfileToken>"
                            f"<PanTilt>true</PanTilt><Zoom>true</Zoom></Stop>")
        cur, repeats = None, 0


def hold(name, pan, tilt, zoom=0.0):
    """同方向重复按只延长保持时间, 不重发命令 ---- 重发会让云台一顿一顿."""
    global cur, deadline, repeats
    if cur == name:
        repeats += 1
        deadline = time.monotonic() + (HOLD_REPEAT if repeats >= REPEAT_ARMED
                                       else HOLD_FIRST)
        return False
    if cur is not None:
        stop()
    _move(pan * SPEED, tilt * SPEED, zoom * SPEED)
    cur, repeats = name, 0
    deadline = time.monotonic() + HOLD_FIRST
    return True


def read_key(timeout):
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    if not select.select([sys.stdin], [], [], 0.03)[0]:
        return "ESC"
    if sys.stdin.read(1) != "[":
        return "ESC"
    if not select.select([sys.stdin], [], [], 0.03)[0]:
        return "ESC"
    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(
        sys.stdin.read(1), "?")


KEYS = {"LEFT": ("← X轴左转", "LEFT", -1, 0, 0),
        "RIGHT": ("→ X轴右转", "RIGHT", 1, 0, 0),
        "UP": ("↑ Y轴上仰", "UP", 0, 1, 0),
        "DOWN": ("↓ Y轴下俯", "DOWN", 0, -1, 0),
        "+": ("变焦 拉近", "ZIN", 0, 0, 1),
        "=": ("变焦 拉近", "ZIN", 0, 0, 1),
        "-": ("变焦 拉远", "ZOUT", 0, 0, -1),
        "_": ("变焦 拉远", "ZOUT", 0, 0, -1)}


def setup(boost):
    print(f"\n布控球方向键控制 v2   速度固定 {SPEED}(最快)")
    print("─" * 52)
    print("  ← → ↑ ↓  转动(按住持续)    + -  变焦")
    print("  f  重新对焦   空格 停止   q 退出")
    print("─" * 52)
    focus_prepare()
    if boost:
        boost_prepare()


def teardown(boost):
    stop()
    focus_restore()
    if boost:
        boost_restore()
    sess.close()


def main(boost):
    setup(boost)
    print()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            k = read_key(0.02)
            if k is None:
                if cur is not None and time.monotonic() >= deadline:
                    stop()
                    print("\r  [停止]                        ", end="", flush=True)
                continue
            if k == "q":
                break
            if k == " ":
                stop()
                print("\r  [停止]                        ", end="", flush=True)
            elif k == "f":
                print("\r  重新对焦 ...                   ", end="", flush=True)
                refocus()
            elif k in KEYS:
                label, name, pan, tilt, zoom = KEYS[k]
                if hold(name, pan, tilt, zoom):
                    print(f"\r  {label}                      ", end="", flush=True)
    finally:
        teardown(boost)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print("\n已停止, 对焦与编码配置已恢复。")


def selftest(boost):
    """
    非交互自检.

    位移度量踩过两个坑, 这里都修掉了:
      - 裸平均帧差: 天黑后传感器噪声本身就把地板顶到 15 以上, 与真实位移信号
        (37) 同量级, 判据失效.解法: 先 8x8 池化再差分, i.i.d. 噪声按 √64=8 倍
        衰减, 真实位移几乎无损.
      - 相位相关未加窗: 大位移时边界不连续产生的 (0,0) 峰会盖过真实峰, 输出假的
        0px.解法: 加汉宁窗.它给出的是像素位移, 物理意义最直接.
    """
    import numpy as np
    from PIL import Image
    rtsp = f"rtsp://admin:Admin123.@{HOST}:554/media/video1"

    def grab():
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error",
                        "-rtsp_transport", "tcp", "-i", rtsp,
                        "-frames:v", "1", "-y", "/tmp/st2.png"],
                       capture_output=True, timeout=60)
        return np.asarray(Image.open("/tmp/st2.png").convert("L"), dtype=np.float64)

    def pool(a, k=8):
        h, w = a.shape[0] // k * k, a.shape[1] // k * k
        return a[:h, :w].reshape(h // k, k, w // k, k).mean(axis=(1, 3))

    def pdiff(a, b):
        return float(np.abs(pool(a) - pool(b)).mean())

    def pshift(a, b):
        """加窗相位相关, 返回像素位移量."""
        h, w = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
        win = np.hanning(h)[:, None] * np.hanning(w)[None, :]
        A = np.fft.fft2((a[:h, :w] - a[:h, :w].mean()) * win)
        B = np.fft.fft2((b[:h, :w] - b[:h, :w].mean()) * win)
        R = A * np.conj(B)
        R /= np.maximum(np.abs(R), 1e-9)
        c = np.fft.ifft2(R).real
        dy, dx = np.unravel_index(np.argmax(c), c.shape)
        if dy > h // 2:
            dy -= h
        if dx > w // 2:
            dx -= w
        return float(np.hypot(dx, dy))

    def sharp(a):
        lap = (-4 * a[1:-1, 1:-1] + a[:-2, 1:-1] + a[2:, 1:-1]
               + a[1:-1, :-2] + a[1:-1, 2:])
        return float(lap.var())

    setup(boost)
    lat = []
    for _ in range(6):
        t0 = time.monotonic()
        _move(0, 0, 0)
        lat.append((time.monotonic() - t0) * 1000)
    print(f"\n  命令延迟中位数 {sorted(lat)[3]:.0f} ms")

    # 用三帧静止样本建立地板, 单次采样容易偏
    s1, s2, s3 = grab(), (time.sleep(1.5), grab())[1], (time.sleep(1.5), grab())[1]
    floor = max(pdiff(s1, s2), pdiff(s2, s3))
    floor_shift = max(pshift(s1, s2), pshift(s2, s3))
    print(f"  静止基线: 池化帧差 {floor:.2f}   位移 {floor_shift:.0f}px   "
          f"清晰度 {sharp(s1):.0f}")
    print("  (俯仰比水平慢, 故俯仰用 4s、水平用 2.5s)")
    if floor > 10:
        print("  ! 静止基线偏高: 暗光下 AGC 增益震荡造成全帧亮度波动, 池化压不掉。\n"
              "    此时自动位移判定不可靠, 请以肉眼观察为准 —— 本自检只报数据。")
    print()

    allok = True
    for key in ("LEFT", "RIGHT", "UP", "DOWN"):
        label, name, pan, tilt, zoom = KEYS[key]
        dur = 2.5 if key in ("LEFT", "RIGHT") else 4.0
        before = grab()
        hold(name, pan, tilt, zoom)
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            hold(name, pan, tilt, zoom)     # 模拟按住(自动重复)
            time.sleep(0.05)
        stop()
        time.sleep(1.0)
        mid = grab()
        time.sleep(3.0)
        after = grab()
        d = pdiff(before, after)
        good = d > floor          # 弱判据: 只要高于静止基线就认为有变化
        allok &= good
        print(f"  {label:12} {dur}s  池化帧差 {d:6.2f} (基线 {floor:.2f}, "
              f"{d/floor:.1f}x)   清晰度 停下 {sharp(mid):6.0f} -> 稳定 "
              f"{sharp(after):6.0f}")

    teardown(boost)
    print(f"\n四个方向{'均有画面变化' if allok else '有方向的变化未超过基线'}。"
          "\n注意: 自动位移判定在暗光下不可靠(见上), 请肉眼确认转动方向与手感。")
    print("清晰度\"停下 -> 稳定\"的上升说明运动结束后重新合焦, 属正常。")
    return 0


if __name__ == "__main__":
    _boost = "--no-boost" not in sys.argv
    sys.exit(selftest(_boost) if "--selftest" in sys.argv else main(_boost))
