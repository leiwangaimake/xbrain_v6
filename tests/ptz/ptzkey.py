#!/usr/bin/env python3
"""
布控球方向键控制 ---- 键盘直接操控 X/Y 轴云台.

  ← ->     X 轴 左转 / 右转 (pan)
  ↑ ↓     Y 轴 上仰 / 下俯 (tilt)
  按住即持续转, 松开自动停

  + -     变焦 拉近 / 拉远
  [ ]     聚焦 近 / 远 (自动切 MANUAL)
  f       自动对焦 (切回 AUTO)
  1-9     调速 (1 最慢 9 最快, 默认 5)
  空格    立即停止
  q       退出 (退出前必定发 Stop)

为什么走 ONVIF: 该机 LAPI 944 个端点里没有连续点动接口, 绝对定位在 PELCO-D
外置云台下是空操作(返回成功但不动), 位置回读恒为假值 (180,0).ONVIF
ContinuousMove 是唯一可用的点动原语.认证必须是 WS-Security PasswordDigest
且不能叠加 HTTP 认证 ---- 详见 onvif.py 注释.

终端收不到"按键释放"事件, 所以用: 收到方向键就开始转并刷新截止时间,
超过 STOP_AFTER 没有新按键就发 Stop.键盘自动重复会不断刷新, 于是"按住"手感自然.
"""
import os
import select
import sys
import termios
import time
import tty

# onvif.py 与本文件同目录 (/usr/local/lib/ptz/), 不要依赖 /tmp ---- 它会被清空
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import onvif as O  # noqa: E402

HOST = "192.168.66.13"
PTZ = f"http://{HOST}:80/onvif/ptz"
IMG = f"http://{HOST}:80/onvif/imaging"
TOK = "media_profile1"
VS = "video_source"

STOP_AFTER = 0.30        # 秒; 大于键盘自动重复间隔, 小于则会抖动
speed = 0.5
cur = None               # 当前运动方向, None = 已停
deadline = 0.0
manual_focus = False


def send(pan=0.0, tilt=0.0, zoom=0.0):
    O.ptz_continuous(PTZ, TOK, pan=pan, tilt=tilt, zoom=zoom)


def stop():
    global cur
    if cur is not None:
        O.ptz_stop(PTZ, TOK)
        cur = None


def move(name, pan, tilt, zoom=0.0):
    """方向未变则只刷新截止时间, 避免重复下发把云台打顿."""
    global cur, deadline
    deadline = time.monotonic() + STOP_AFTER
    if cur != name:
        send(pan * speed, tilt * speed, zoom * speed)
        cur = name
        return True
    return False


def focus(direction):
    global manual_focus
    if not manual_focus:
        O.set_autofocus(IMG, VS, "MANUAL")
        manual_focus = True
    O.focus_move(IMG, VS, direction * 0.7)
    time.sleep(0.35)
    O.focus_stop(IMG, VS)


def autofocus():
    global manual_focus
    O.set_autofocus(IMG, VS, "AUTO")
    manual_focus = False


def read_key(timeout):
    """返回按键标识; 方向键是 ESC [ A/B/C/D 三字节序列."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return "\x1b"
    if sys.stdin.read(1) != "[":
        return "\x1b"
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return "\x1b"
    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(
        sys.stdin.read(1))


BANNER = """
布控球方向键控制  (可见光 192.168.66.13 驱动外置 PELCO-D 云台)
────────────────────────────────────────────────────────────
  ← →  X 轴左右转      ↑ ↓  Y 轴上下转      按住持续转
  + -  变焦            [ ]  聚焦近/远       f  自动对焦
  1-9  调速            空格 停止            q  退出
────────────────────────────────────────────────────────────"""


def main():
    global speed
    print(BANNER)
    print(f"当前速度 {speed:.1f}\n")
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            k = read_key(0.05)

            if k is None:
                # 没有新按键: 到期就停
                if cur is not None and time.monotonic() >= deadline:
                    stop()
                    print("\r  [停止]                    ", end="", flush=True)
                continue

            if k == "q":
                break
            if k == " ":
                stop()
                print("\r  [停止]                    ", end="", flush=True)
            elif k == "LEFT":
                if move("LEFT", -1, 0):
                    print(f"\r  ← X轴左转 speed={speed:.1f}   ", end="", flush=True)
            elif k == "RIGHT":
                if move("RIGHT", 1, 0):
                    print(f"\r  → X轴右转 speed={speed:.1f}   ", end="", flush=True)
            elif k == "UP":
                if move("UP", 0, 1):
                    print(f"\r  ↑ Y轴上仰 speed={speed:.1f}   ", end="", flush=True)
            elif k == "DOWN":
                if move("DOWN", 0, -1):
                    print(f"\r  ↓ Y轴下俯 speed={speed:.1f}   ", end="", flush=True)
            elif k in "+=":
                if move("ZIN", 0, 0, 1):
                    print("\r  变焦 拉近                 ", end="", flush=True)
            elif k in "-_":
                if move("ZOUT", 0, 0, -1):
                    print("\r  变焦 拉远                 ", end="", flush=True)
            elif k == "[":
                print("\r  聚焦 近                   ", end="", flush=True)
                focus(-1)
            elif k == "]":
                print("\r  聚焦 远                   ", end="", flush=True)
                focus(1)
            elif k == "f":
                print("\r  自动对焦 (需十几秒收敛)      ", end="", flush=True)
                autofocus()
            elif k.isdigit() and k != "0":
                speed = int(k) / 9
                print(f"\r  速度 -> {speed:.2f}            ", end="", flush=True)
    finally:
        stop()
        if manual_focus:
            autofocus()
            print("\n  已恢复自动对焦")
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print("\n已停止并退出。")


def selftest():
    """非交互自检: 走与按键完全相同的 move()/stop() 代码路径, 用帧差客观验证."""
    import subprocess

    import numpy as np
    from PIL import Image

    rtsp = f"rtsp://admin:Admin123.@{HOST}:554/media/video1"

    def grab():
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error",
                        "-rtsp_transport", "tcp", "-i", rtsp,
                        "-frames:v", "1", "-y", "/tmp/st.png"],
                       capture_output=True, timeout=60)
        return np.asarray(Image.open("/tmp/st.png").convert("L"), dtype=np.float64)

    print("自检: 每个方向走 1.2s 再反向等量回退\n")
    a = grab()
    time.sleep(2)
    floor = float(np.abs(a - grab()).mean())
    print(f"  噪声地板 = {floor:.2f}, 判据 = {floor*3:.2f}\n")

    seq = [("← X轴左转", "LEFT", -1, 0), ("→ X轴右转(回退)", "RIGHT", 1, 0),
           ("↑ Y轴上仰", "UP", 0, 1), ("↓ Y轴下俯(回退)", "DOWN", 0, -1)]
    allok = True
    for label, name, pan, tilt in seq:
        before = grab()
        move(name, pan, tilt)
        t_end = time.monotonic() + 1.2
        while time.monotonic() < t_end:      # 模拟按住: 不断刷新截止时间
            move(name, pan, tilt)
            time.sleep(0.1)
        time.sleep(STOP_AFTER + 0.05)
        stop()
        time.sleep(2.5)
        d = float(np.abs(before - grab()).mean())
        ok = d > floor * 3
        allok &= ok
        print(f"  {label:18} 帧差 {d:7.2f}  {'通过' if ok else '未检出位移'}")

    print(f"\n自检{'全部通过' if allok else '有项目未通过'}")
    return 0 if allok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
