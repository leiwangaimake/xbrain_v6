#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: gen_overview_diagrams.py
# Brief: Render 10 S2.1 system context + S2.2 full block diagram as PNG
#
# Description:
# Faithful graphical rendering of the two text diagrams in docs/10 S2 --
# S2.1 system context (hub-and-spoke) and S2.2 full system block diagram
# (layer stack). Labels are taken verbatim from the doc. Arrow directions in
# the context diagram follow the PHYSICAL flow (the doc's ASCII uses two
# different arrow frames per column; here every edge points the real way).
# Block-diagram layers are tinted by the S2.3 four-domain split.
# Editor notes at the bottom flag elements the doc diagram still shows but
# which have since been superseded (behavior_proxy/Nav2, 12 S4.6 / 99 U90),
# so the PNG is faithful to the doc without silently presenting stale truth.
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, FancyArrowPatch
from matplotlib.font_manager import FontProperties

# Noto Sans CJK by file path -- most reliable way to guarantee Han glyphs
# regardless of fontconfig face resolution (face0 of the ttc covers all the
# Han used here).
_REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
_BLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
F = lambda s: FontProperties(fname=_REG, size=s)
FB = lambda s: FontProperties(fname=_BLD, size=s)

OUT = os.path.dirname(os.path.abspath(__file__))

# Domain palette (10 S2.3): failure-impact tiers, lightest = least critical.
C_SAFE = "#f4c2c2"   # 安全兜底域  quadruped Tier1
C_RT = "#ffd9a8"     # 实时控制域  P1 / perception / RT Session
C_POL = "#bcd8f0"    # 策略域      P2 / P3
C_VAL = "#c3e6c3"    # 增值域      P4 / P5 / AI Runtime
C_EXT = "#e6e6e6"    # 外部 / 通信 / 硬件 / 横切 (neutral)
C_EDGE = "#555555"
C_STALE = "#cc3333"  # superseded element flag


def box(ax, x, y, w, h, title, cells=None, face=C_EXT, ts=12, cs=10,
        rounded=True, lw=1.4, edge=C_EDGE, title_bold=True, star=False):
    """A layer box: bold title line, optional row of sub-cells under it."""
    style = "round,pad=0.02,rounding_size=0.08" if rounded else "square,pad=0"
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=style,
                 fc=face, ec=edge, lw=lw, zorder=2))
    tfont = FB(ts) if title_bold else F(ts)
    tx = x + 0.12
    ttl = ("[*] " if star else "") + title
    ax.text(tx, y + h - 0.16, ttl, fontproperties=tfont, va="top", ha="left",
            zorder=3)
    if not cells:
        return
    # sub-cells as a single row across the lower part of the box
    n = len(cells)
    pad = 0.10
    inner_x = x + pad
    inner_w = w - 2 * pad
    cw = inner_w / n
    cy = y + 0.12
    ch = h - 0.62
    for i, c in enumerate(cells):
        cx = inner_x + i * cw
        dashed = isinstance(c, tuple)      # (text, "stale") -> dashed red flag
        txt = c[0] if dashed else c
        ax.add_patch(Rectangle((cx + 0.04, cy), cw - 0.08, ch,
                     fc="white", ec=(C_STALE if dashed else "#888888"),
                     lw=(1.6 if dashed else 0.9),
                     ls=("--" if dashed else "-"), zorder=3))
        ax.text(cx + cw / 2, cy + ch / 2, txt, fontproperties=F(cs),
                va="center", ha="center", zorder=4, wrap=True,
                color=(C_STALE if dashed else "black"))


def arrow(ax, p0, p1, color=C_EDGE, lw=1.6, style="-|>", ms=14):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=ms,
                 color=color, lw=lw, zorder=1,
                 shrinkA=2, shrinkB=2))


# ============================================================ diagram 1
def context_diagram():
    fig, ax = plt.subplots(figsize=(15.5, 10.5))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 72)
    ax.axis("off")
    ax.text(50, 70, "XBRAIN_V6 系统上下文  (10 S2.1)", fontproperties=FB(19),
            ha="center", va="center")

    # center hub
    hub = FancyBboxPatch((38, 33), 24, 9, boxstyle="round,pad=0.1,rounding_size=0.6",
                         fc="#34495e", ec="#1b2a38", lw=2, zorder=5)
    ax.add_patch(hub)
    ax.text(50, 39.2, "XBRAIN_V6", fontproperties=FB(20), color="white",
            ha="center", va="center", zorder=6)
    ax.text(50, 35.4, "智能上装", fontproperties=F(14), color="white",
            ha="center", va="center", zorder=6)

    def actor(x, y, w, h, title, sub=""):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle="round,pad=0.08,rounding_size=0.4",
                     fc="#eef3f7", ec=C_EDGE, lw=1.5, zorder=5))
        ax.text(x + w / 2, y + h - (1.6 if sub else h / 2), title,
                fontproperties=FB(13), ha="center",
                va=("top" if sub else "center"), zorder=6)
        if sub:
            ax.text(x + w / 2, y + 0.9, sub, fontproperties=F(9.5),
                    ha="center", va="bottom", color="#555", zorder=6)

    def flows(x, y, lines, ha="left"):
        for i, (d, t) in enumerate(lines):
            col = "#1a6b1a" if d == "in" else "#9a3b1a"
            ax.text(x, y - i * 2.0, ("-> " if d == "in" else "<- ") + t,
                    fontproperties=F(9.5), ha=ha, va="top", color=col)

    # --- top-left: 营区值班人员
    actor(3, 55, 23, 9, "营区值班人员", "（现场）")
    arrow(ax, (18, 55), (40, 42.5), color="#6b8ba4")
    flows(3, 53.5, [("in", "语音 A"), ("out", "喊话"),
                    ("out", "警笛 / 爆闪"), ("out", "照明")])

    # --- top-right: 营区环境
    actor(74, 55, 23, 9, "营区环境", "")
    arrow(ax, (82, 55), (60, 42.5), color="#6b8ba4")
    flows(74, 53.5, [("in", "RGBD 深度"), ("in", "LiDAR 点云"),
                     ("in", "RTK 定位航向"), ("out", "威慑 / 驱离")])

    # --- left: M20S 本体
    actor(2, 30, 23, 9, "M20S 本体", "（厂商黑盒）")
    arrow(ax, (25, 35), (38, 37.5), color="#6b8ba4")
    flows(2, 28.0, [("out", "速度指令"), ("out", "姿态 / 模态命令"),
                    ("in", "本体状态"), ("in", "故障上报"), ("in", "里程计")])

    # --- right: 充电桩
    actor(75, 30, 23, 9, "充电桩", "（路径上 · 已标定）")
    arrow(ax, (75, 35), (62, 37.5), color="#6b8ba4")
    flows(98, 28.0, [("out", "自主回桩导航"), ("in", "充电状态")], ha="right")

    # --- bottom: 加密模块 -> 云端
    enc = FancyBboxPatch((28, 14), 44, 8,
                         boxstyle="round,pad=0.1,rounding_size=0.4",
                         fc="#fff3cd", ec="#b8860b", lw=2, ls="--", zorder=5)
    ax.add_patch(enc)
    ax.text(50, 20.2, "客户加密通讯模块   [*] 外部实体",
            fontproperties=FB(13), ha="center", va="center", zorder=6)
    ax.text(50, 17.0, "甲方自有 · 我方不实现 · 我方不配置 · 视为营区官方加密信道（U23）",
            fontproperties=F(9.5), ha="center", va="center", color="#7a5c00", zorder=6)
    ax.text(50, 15.1, "[*] 通往云端的全部流量（控制面 + 媒体面 + 数据面）唯一出口",
            fontproperties=F(9), ha="center", va="center", color="#9a3b1a", zorder=6)
    # XBRAIN <-> encryption (bidirectional)
    arrow(ax, (50, 33), (50, 22.2), color="#b8860b", style="<|-|>", lw=1.8)

    # cloud
    ax.add_patch(FancyBboxPatch((33, 3.2), 34, 7,
                 boxstyle="round,pad=0.1,rounding_size=0.5",
                 fc="#d6e4f0", ec="#2a5a8a", lw=1.8, zorder=5))
    ax.text(50, 7.6, "云端后台平台", fontproperties=FB(14), ha="center",
            va="center", zorder=6)
    arrow(ax, (50, 14), (50, 10.4), color="#2a5a8a", style="<|-|>", lw=1.8)
    # cloud flows beside the link
    flows(69, 13.8, [("in", "任务 JSON"), ("in", "语音 C"),
                     ("in", "云端喊话 B"), ("in", "PTZ 控制"),
                     ("out", "状态 / 事件"), ("out", "视频 / 作业数据")])

    # legend for arrow colors
    ax.text(3, 10.5, "-> 流入 XBRAIN（输入）", fontproperties=F(10),
            color="#1a6b1a", ha="left")
    ax.text(3, 8.3, "<- 流出 XBRAIN（输出）", fontproperties=F(10),
            color="#9a3b1a", ha="left")
    ax.text(3, 5.5,
            "注：原文 §2.1 的 ASCII 箭头按列采用两种参照系；本图统一按物理流向绘制。",
            fontproperties=F(8.5), color="#666", ha="left")

    fig.tight_layout()
    p = os.path.join(OUT, "10_S2.1_system_context.png")
    fig.savefig(p, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return p


# ============================================================ diagram 2
def block_diagram():
    # 12 layers top->bottom (10 S2.2), each: (title, [cells], face, star)
    layers = [
        ("外部终端", ["云端计算机", "HMI 浏览器", "微信 Bot（预留）"], C_EXT, False),
        ("通信面", ["控制面 Zenoh", "媒体面 RTSP / 音频", "数据面 FTP（限速）"], C_EXT, False),
        ("P5 gateway　对外网关",
         ["事件汇聚", "断网补发", "遥测上报", "HMI 后端", "FTP 交付"], C_VAL, False),
        ("P4 agent　语言与工具",
         ["ASR 后处理", "语音旁路", "意图路由", "Prompt 装配", "AI 网关"], C_VAL, False),
        ("AI Runtime　独立周边服务（仅 P4 经网关调用）",
         ["ai_asr  CPU ONNX", "LLM  Qwen2.5-3B", "(机上无 TTS 服务)"], C_VAL, False),
        ("P3 task　任务与持久化",
         ["任务调度", "任务状态机", "充电选桩", "SQLite"], C_POL, False),
        ("P2 core　模式与仲裁",
         ["模式状态机", "七域仲裁", "可疑判定", "载荷 / PTZ", "健康度"], C_POL, False),
        ("P1 motion　实时域　唯一速度出口",
         ["RNS 反应式导航", "目标导向行为", "路径跟踪",
          "运动仲裁", "速度门", "围栏裁剪 / 限幅"], C_RT, True),
        ("本地实时 Session　localhost-only / 不经云端桥 / 隔离队头阻塞",
         [], C_RT, True),
        ("ROS 2 域　C++",
         [("perception\n融合 / YOLO / MOT", None),  # handled specially below
          ("behavior_proxy / Nav2\nspin / backup / wait", "stale"),
          ("quadruped [Tier1]\n协议 / 超时停车", "safe")], None, False),
        ("硬件层",
         ["Gemini 336L", "LiDAR 单话题", "RTK 20Hz", "USB MIC",
          "三合一载荷", "PTZ 4K 云台", "M20S 底盘 + 充电桩"], C_EXT, False),
        ("横切关注点",
         ["配置管理（三层）", "开机自检 / 健康度", "时间同步（RTK 授时）"], C_EXT, False),
    ]

    W = 17.0
    LH = 1.25            # layer height
    GAP = 0.55           # gap between layers
    x0 = 0.6
    n = len(layers)
    total_h = n * LH + (n - 1) * GAP
    fig_h = total_h + 3.2
    fig, ax = plt.subplots(figsize=(15.5, fig_h))
    ax.set_xlim(0, W + 1.2)
    ax.set_ylim(0, fig_h)
    ax.axis("off")
    ax.text((W + 1.2) / 2, fig_h - 0.5, "XBRAIN_V6 完整系统框图  (10 S2.2)",
            fontproperties=FB(19), ha="center", va="center")

    top = fig_h - 1.4
    ypos = {}
    for idx, (title, cells, face, star) in enumerate(layers):
        y = top - idx * (LH + GAP) - LH
        ypos[title] = (y, LH)
        if title.startswith("ROS 2"):
            # special layer: three differently-tinted cells
            ax.add_patch(FancyBboxPatch((x0, y), W, LH,
                         boxstyle="round,pad=0.02,rounding_size=0.06",
                         fc="white", ec=C_EDGE, lw=1.4, zorder=2))
            ax.text(x0 + 0.12, y + LH - 0.16, "ROS 2 域　C++",
                    fontproperties=FB(12), va="top", ha="left", zorder=3)
            trip = [("perception\n融合 / YOLO / MOT", C_RT, "#888", "-"),
                    ("behavior_proxy / Nav2\nspin / backup / wait", "white",
                     C_STALE, "--"),
                    ("quadruped [Tier1]\n协议 / 超时停车", C_SAFE, "#888", "-")]
            pad = 0.10
            cw = (W - 2 * pad) / 3
            cy = y + 0.10
            ch = LH - 0.58
            for i, (txt, fc, ec, ls) in enumerate(trip):
                cx = x0 + pad + i * cw
                ax.add_patch(Rectangle((cx + 0.05, cy), cw - 0.10, ch,
                             fc=fc, ec=ec, lw=(1.7 if ls == "--" else 1.0),
                             ls=ls, zorder=3))
                ax.text(cx + cw / 2, cy + ch / 2, txt, fontproperties=F(9.5),
                        va="center", ha="center", zorder=4,
                        color=(C_STALE if ls == "--" else "black"))
        else:
            box(ax, x0, y, W, LH, title, cells=cells, face=face, ts=12,
                cs=9.5, star=star)

    # inter-layer arrows between P1 / Session / ROS2 (10 S2.2 shows these)
    def mid(title):
        y, h = ypos[title]
        return y
    p1_y = mid("P1 motion　实时域　唯一速度出口")
    ses_t = "本地实时 Session　localhost-only / 不经云端桥 / 隔离队头阻塞"
    ses_y = mid(ses_t)
    ros_y = mid("ROS 2 域　C++")
    # P1 <-> Session
    ya = p1_y
    yb = ses_y + LH
    ax.annotate("", xy=(5.5, yb), xytext=(5.5, ya),
                arrowprops=dict(arrowstyle="-|>", color="#9a3b1a", lw=1.8))
    ax.text(5.1, (ya + yb) / 2, "速度指令", fontproperties=F(9),
            ha="right", va="center", color="#9a3b1a", rotation=90)
    ax.annotate("", xy=(12, ya), xytext=(12, yb),
                arrowprops=dict(arrowstyle="-|>", color="#1a6b1a", lw=1.8))
    ax.text(12.4, (ya + yb) / 2, "感知 / RTK / Nav2 cmd_vel",
            fontproperties=F(8.5), ha="left", va="center", color="#1a6b1a",
            rotation=90)
    # Session <-> ROS2
    ya2 = ses_y
    yb2 = ros_y + LH
    ax.annotate("", xy=(5.5, yb2), xytext=(5.5, ya2),
                arrowprops=dict(arrowstyle="-|>", color="#9a3b1a", lw=1.8))
    ax.text(5.1, (ya2 + yb2) / 2, "速度指令", fontproperties=F(9),
            ha="right", va="center", color="#9a3b1a", rotation=90)
    ax.annotate("", xy=(12, ya2), xytext=(12, yb2),
                arrowprops=dict(arrowstyle="-|>", color="#1a6b1a", lw=1.8))
    ax.text(12.4, (ya2 + yb2) / 2, "感知数据", fontproperties=F(9),
            ha="left", va="center", color="#1a6b1a", rotation=90)

    # domain legend
    lx = x0
    ly = 1.55
    for i, (c, name) in enumerate([
            (C_SAFE, "安全兜底域"), (C_RT, "实时控制域"),
            (C_POL, "策略域"), (C_VAL, "增值域"),
            (C_EXT, "外部/通信/硬件/横切")]):
        ax.add_patch(Rectangle((lx + i * 3.4, ly), 0.4, 0.3, fc=c, ec="#888",
                     lw=0.8))
        ax.text(lx + i * 3.4 + 0.5, ly + 0.15, name, fontproperties=F(9),
                va="center", ha="left")

    # editor note on staleness
    ax.text(x0, 0.85,
            "编者注（非原文）：本图忠实复现 10 S2.2（2026-07-30 原图）。"
            "虚线红框  behavior_proxy / Nav2  已于 2026-09-30 作废",
            fontproperties=F(9), color=C_STALE, ha="left", va="center")
    ax.text(x0, 0.45,
            "（12 S4.6 整节作废 · 99 U90 ②；其职责已分别由 §6A 旋转许可 / "
            "rns.backup / rns.dynamic.wait 覆盖）。[*] = 唯一速度出口链路。",
            fontproperties=F(9), color=C_STALE, ha="left", va="center")

    p = os.path.join(OUT, "10_S2.2_system_block.png")
    fig.savefig(p, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return p


if __name__ == "__main__":
    p1 = context_diagram()
    p2 = block_diagram()
    print("wrote:", p1)
    print("wrote:", p2)
