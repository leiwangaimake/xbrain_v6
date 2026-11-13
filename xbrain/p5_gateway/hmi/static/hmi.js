/* XBRAIN_V6 HMI client. Vanilla JS (17 S6.6 HMI-06: keep the frontend light,
   no framework). Two fetches drive everything: /api/hmi/ui_config once on load
   (17 S6.10.2 U1..U6 presentation params) and /api/hmi/snapshot on a timer
   (17 S6.8 A..F data). Where a snapshot group reports available:false the layer
   renders a "no data" state -- the frontend NEVER invents a robot position, an
   RTK badge, or a progress fraction (17 S6.10.4 / 3.1 / 3.2). */
(() => {
  "use strict";
  const SVGNS = "http://www.w3.org/2000/svg";
  const $ = (id) => document.getElementById(id);
  const view = { zoom: 1, pan: { x: 0, y: 0 } };   // pan in ENU metres
  let cfg = null;                       // ui_config, set on load
  let zoomBounds = { min: 0.65, max: 2.8, wheel: 0.12 };
  let baseVB = [-100, -65, 240, 145];   // ENU-metre base viewBox window (U1)
  let lastHeadingCompass = null;        // last VALID compass heading (deg); null until first fix
  // Continuous (unwrapped) dial angle in degrees + the last raw compass value,
  // so the dial accumulates the SHORTEST step each update. Without this, a
  // heading that jitters across the 0/360 compass seam (e.g. hovering near due
  // east) makes the CSS `transition: transform` animate a ~360deg spin the long
  // way every frame -- the "dial spins wildly" bug.
  let dialAngle = null;
  let lastCompass = 0;
  // Footer clock timezone: ui_config.timezone (= common.timezone) is only the
  // load-time DEFAULT; the live zone follows the GPS fix via snapshot.timezone
  // (derived server-side from pose lat/lon, 17 S6.10.2 v1.3) and overrides it in
  // renderSnapshot. null -> browser zone. The clock ticks LOCALLY every second
  // (not off the 2 Hz snapshot) so the seconds advance smoothly; siteTz only
  // changes when the robot crosses into a new timezone band.
  let siteTz = null;
  let clockSync = null;
  let clockTimer = null;

  // -- ENU projection: snapshot geometry may be {lat,lon} or [e_m,n_m]. Result is
  //    SVG frame coords: x = east, y = -north (SVG y is DOWN, so negating north
  //    puts NORTH UP -- matching the compass dial, user 2026-08-18). Both the
  //    array [e_m,n_m] path and the lat/lon path negate north the SAME way so the
  //    geo geometry and the lat/lon robot/events share one orientation. Absent
  //    origin -> lat/lon points are skipped (cannot place), never plotted at 0,0.
  function toXY(pt, origin) {
    if (Array.isArray(pt)) return [pt[0], -pt[1]];    // [e_m, n_m] -> north up
    if (pt && pt.lat != null && pt.lon != null) {
      if (!origin) return null;                       // no origin -> cannot place
      const R = 111320, c = Math.cos((origin.lat || 0) * Math.PI / 180);
      return [(pt.lon - origin.lon) * R * c, -(pt.lat - origin.lat) * R];
    }
    return null;
  }
  // The INVERSE of toXY, for W2 goto: a tap on the map -> {lat, lon}.
  // Deliberately the exact algebraic inverse (same R, same cos(origin.lat),
  // same north negation) rather than a second approximation -- if the two
  // drifted, the pin the operator sees and the point the robot drives to
  // would differ, and nothing on screen would show it.
  // Returns null without an origin: better no goto than a goto to 0,0.
  function fromXY(x, y, origin) {
    if (!origin || origin.lat == null || origin.lon == null) return null;
    const R = 111320, c = Math.cos((origin.lat || 0) * Math.PI / 180);
    if (!c) return null;                              // at the pole, degenerate
    return { lat: origin.lat - y / R, lon: origin.lon + x / (R * c) };
  }
  const ptsAttr = (arr, origin) => arr.map((p) => toXY(p, origin))
    .filter(Boolean).map((xy) => xy.join(",")).join(" ");

  // req4d (2026-08-18): geometry length in METRES for the label. segLen/perimM
  // operate on PROJECTED [x,y] points (SVG-frame metres from toXY), so euclidean
  // distance == metres for BOTH ENU fences and WGS84 {lat,lon} routes -- callers
  // must toXY the points first (v1.5 PLAN A: raw route points are lat/lon degrees,
  // not metres). Route = open-path sum; fence = closed perimeter (add the closing
  // edge). Returns "" for degenerate input so the label just shows the name.
  function segLen(pts) {
    let d = 0;
    for (let i = 1; i < (pts || []).length; i++)
      d += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    return d;
  }
  function perimM(pts) {
    if (!pts || pts.length < 2) return 0;
    const n = pts.length;
    return segLen(pts) + Math.hypot(pts[0][0] - pts[n - 1][0], pts[0][1] - pts[n - 1][1]);
  }
  const lenTag = (m) => (m > 0 ? ` (${Math.round(m)} m)` : "");

  function lineStroke(el, line) {
    if (!line) return;
    el.setAttribute("stroke", line.color);
    // req(2026-08-18): double the config width; non-scaling-stroke (CSS) keeps it
    // a constant screen px regardless of zoom. The per-fence stroke COLOUR here is
    // overridden by the per-role CSS class (.keep-in/.alarm-region/.forbid-region/
    // .speed-region) so forbid renders dark red and speed bright blue, not the
    // config colour.
    const w = (line.width_px || 1.4) * 2;
    el.setAttribute("stroke-width", w);
    // 99 U183: a dash scales with a thick line (a 10.5 px fence drawn "4 3"
    // reads as dots); lines up to ~2.7 px keep the old 4 / 3 pattern
    if (line.style === "dashed")
      el.setAttribute("stroke-dasharray", Math.max(4, w * 1.5) + " " + Math.max(3, w));
    else if (line.style === "dotted") el.setAttribute("stroke-dasharray", "1 2");
    else el.removeAttribute("stroke-dasharray");     // solid
  }

  // Map a fence to its display type. The contract role (11 S9A.2: allow/forbid/
  // speed_limit/warning) wins; the name fallback below is a defence for a
  // role-less payload only. Default active (keep-in) when neither says. NOTE:
  // "warning" is the alarm role; "zone" is its retired name (11 S9A.1A rename,
  // 2026-08-18) -- still accepted defensively.
  function fenceType(fen) {
    const r = fen.role;
    if (r === "allow") return "active";
    if (r === "forbid") return "forbid";
    if (r === "speed_limit") return "speed";
    if (r === "warning" || r === "zone") return "alarm";
    const n = fen.name || "";
    if (n.indexOf("禁入") >= 0 || n.indexOf("禁止") >= 0) return "forbid";
    if (n.indexOf("限速") >= 0) return "speed";
    if (n.indexOf("报警") >= 0) return "alarm";
    if (n.indexOf("活动") >= 0) return "active";
    return "active";
  }

  // -- apply ui_config: CSS vars (U2/U3/U5), grid pattern metres (U1), zoom (U4) --
  function applyUiConfig(c) {
    cfg = c;
    // Load-time DEFAULT footer-clock zone (= common.timezone; null -> browser
    // zone). The live zone arrives with the first snapshot (snapshot.timezone,
    // GPS-derived) and overrides this in renderSnapshot. Start the local 1 Hz
    // ticker once ui_config is in.
    siteTz = c.timezone || null;
    startClock();
    const root = document.documentElement.style;
    const f = c.font || {};
    for (const [k, v] of Object.entries(f)) {
      if (!v) continue;
      root.setProperty(`--font-${k}-family`, v.family);
      root.setProperty(`--font-${k}-size`, (v.size_px || 12) + "px");
    }
    const lay = c.layout || {};
    if (lay.plan_panel) {
      root.setProperty("--plan-width", (lay.plan_panel.width_px || 260) + "px");
      root.setProperty("--plan-max-h", (lay.plan_panel.max_height_px || 420) + "px");
    }
    if (lay.status_bar) root.setProperty("--status-h", (lay.status_bar.height_px || 34) + "px");
    // legend colours (17 S6.10.2A): fence by type + route recorded/realtime.
    const fc = c.fence || {};
    if (fc.active) root.setProperty("--fence-active-color", fc.active.line.color);
    if (fc.alarm) root.setProperty("--fence-alarm-color", fc.alarm.line.color);
    if (fc.disabled) root.setProperty("--fence-disabled-color", fc.disabled.line.color);
    if (c.route && c.route.recorded) root.setProperty("--route-recorded-color", c.route.recorded.line.color);
    if (c.route && c.route.realtime) root.setProperty("--route-realtime-color", c.route.realtime.line.color);

    // U1: grid metres. The viewBox (ENU metres) is the BASE window; applyView()
    // pans/zooms it by rewriting the viewBox attribute (NOT a CSS transform on
    // the element), so the SVG element always fills the viewport. The grid rect
    // spans a huge extent and its pattern is userSpaceOnUse (anchored at the ENU
    // origin), so the井字格 fills the screen at ANY pan/zoom and never shows an
    // edge -- an infinite grid the operator cannot drag or zoom off.
    const map = c.map || {};
    baseVB = (map.viewbox || "-100 -65 240 145").split(/\s+/).map(Number);
    // req(2026-08-18): the 1m minor grid is dropped (majorGridRect fill=none),
    // and the cell size is ADAPTIVE + yaml-configurable (map.grid.cell_options_m)
    // -- the actual cell metres + scaleText are chosen per zoom by
    // updateGridAndDots() so a huge (~5km) map never shows a pinprick sieve.
    $("majorGridRect").setAttribute("fill", "none");     // no 1m minor grid
    const G = 100000;                     // metres; far beyond any mouse pan/zoom
    for (const [k, v] of [["x", -G], ["y", -G], ["width", 2 * G], ["height", 2 * G]])
      $("gridBg").setAttribute(k, v);
    applyView();                          // sets viewBox + adaptive grid + dots

    const z = map.zoom || {};
    zoomBounds = { min: z.min || 0.65, max: z.max || 2.8, wheel: z.wheel_step || 0.12 };
  }
  function setPattern(id, size, d) {
    const p = $(id);
    p.setAttribute("width", size); p.setAttribute("height", size);
    p.querySelector("path").setAttribute("d", d);
  }

  // Grid (U1). The cell list is map.grid.cell_options_m (yaml); user
  // 2026-11-10 limits it to 1..10 m. GRID_TARGET_PX = aim each cell at least
  // this many screen px. Below GRID_MIN_PX even the LARGEST allowed cell would
  // turn the map into a grey sieve, so the grid is hidden and the scale says so
  // rather than being drawn as a smear. DOT_PX = constant screen radius of the
  // geo dots so they never shrink to nothing on a wide view.
  // ~~GRID_CELLS_DEFAULT = [1, 2, 5, 10, 20, ..., 5000]~~ superseded 2026-11-10.
  const GRID_CELLS_DEFAULT = [1, 2, 5, 10];
  const GRID_TARGET_PX = 42;
  const GRID_MIN_PX = 8;
  const GRID_LINE_PX_FALLBACK = 2;
  const DOT_PX = 3.5;

  // <map-marks>
  // Map marks, user 2026-11-11. Pure (no DOM): the tests run this block under node.
  // Values come from ui_config (configs/p5_gateway.yaml hmi.web.waypoint / robot,
  // 17 S6.10.2 U6 / U9); the constants below are presentation fallbacks only (UI
  // values, not safety values -- 17 S6.10.2 allows defaults for these).
  //  * keypoint (位置点): a true circle filled bright white with a bright red ring
  //    and NO gap between them. Old mark: r 3.5 px plus a 1 px semi-transparent
  //    white stroke centred on the edge -> outer radius 4.0 px, and the translucent
  //    ring read as a hollow. The user asked for 1.5x (outer 6 px) and then, the
  //    same day, twice that: outer 12.0 px = r 11.0 + a 2 px ring centred on the
  //    edge (1 px in, 1 px out). An SVG stroke is painted ON the circle's edge, so
  //    fill and ring touch with no gap, and both are drawn opaque (.keypoint-dot).
  //  * robot: the arrow is bright red; 3x the old half-length 9 px (27 px), then
  //    halved (13.5 px), then a third larger, all the same day -> 18 px on screen,
  //    constant over zoom like before.
  //  * 2026-11-12 (user): every disc twice as large again (outer 12 -> 24 px) with
  //    the red ring twice as thick (2 -> 4 px): r 22 + 4 / 2 = 24; the robot arrow
  //    twice as large (18 -> 36 px). ~~11.0 / 2.0 / 18~~ superseded.
  const KEYPOINT_DOT_PX_FALLBACK = 22.0;    // circle radius, screen px (ring centre)
  const KEYPOINT_RING_PX_FALLBACK = 4.0;    // ring width, screen px
  const MARK_RED_FALLBACK = "#ff1f1f";      // bright red
  const ROBOT_ARROW_PX_FALLBACK = 36;       // arrow half-length, screen px (old 9)
  const DOCK_FILL_FALLBACK = "#ffd400";     // charging point disc: yellow
  const HOME_FILL_FALLBACK = "#22c55e";     // return-home disc: green
  const DISC_OUTLINE_FALLBACK = "#1a1a1a";  // 1 px dark edge on yellow / green
  const DISC_OUTLINE_PX_FALLBACK = 1.0;
  const GRID_LINE_COLOR_FALLBACK = "#2a2a2a";  // grid line, half the old #555555
  // 2026-11-12 (user): a waypoint name sits to the RIGHT of its disc, centred on
  // it vertically and right against it; the gap between the disc's outer edge
  // and the first character, in screen px.
  const KEYPOINT_LABEL_GAP_PX_FALLBACK = 4;

  // The marks' look from ui_config. A missing or non-positive size falls back
  // (a zero radius would hide every keypoint without a word).
  function markStyle(c) {
    const w = (c && c.waypoint) || {}, r = (c && c.robot) || {};
    const pos = (v, fb) => (typeof v === "number" && v > 0 ? v : fb);
    return { kpRadiusPx: pos(w.radius_px, KEYPOINT_DOT_PX_FALLBACK),
             kpRingPx: pos(w.ring_px, KEYPOINT_RING_PX_FALLBACK),
             kpFill: w.fill_color || "#ffffff",
             kpRing: w.ring_color || MARK_RED_FALLBACK,
             dockFill: w.dock_fill_color || DOCK_FILL_FALLBACK,
             homeFill: w.home_fill_color || HOME_FILL_FALLBACK,
             discOutline: w.disc_outline_color || DISC_OUTLINE_FALLBACK,
             discOutlinePx: pos(w.disc_outline_px, DISC_OUTLINE_PX_FALLBACK),
             numColor: w.number_color || "#000000",
             labelGapPx: pos(w.label_gap_px, KEYPOINT_LABEL_GAP_PX_FALLBACK),
             robotPx: pos(r.size_px, ROBOT_ARROW_PX_FALLBACK),
             robotColor: r.color || MARK_RED_FALLBACK };
  }

  // 2026-11-12 (user): every waypoint disc is the same size (outer radius =
  // radius_px + ring_px / 2; 12 px, doubled the same day to 24 px); a charging point (type dock) is a yellow
  // disc carrying the number of its name ("1号充电站" -> 1), the return-home
  // point (type home) a green disc, any other point the white disc with the red
  // ring. Yellow and green get a 1 px dark outline inside the same outer radius,
  // so a disc on the yellow trail line stays visible.
  //   kind: "dock" | "home" | "poi"; num: the digits to print ("" = none).
  // The number is taken ONLY from "<N>号" at the start of the name, Chinese
  // numerals converted ("二号充电站" -> 2). A name without one gets an empty
  // disc -- never an invented number that could disagree with the label.
  const CN_DIGIT = { "零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
                     "五": 5, "六": 6, "七": 7, "八": 8, "九": 9 };
  const CN_UNIT = { "十": 10, "百": 100, "千": 1000 };
  function cnToInt(s) {
    let total = 0, cur = 0;
    for (const ch of s) {
      if (ch in CN_DIGIT) cur = CN_DIGIT[ch];
      else if (ch in CN_UNIT) { total += (cur || 1) * CN_UNIT[ch]; cur = 0; }
      else return null;
    }
    return total + cur;
  }
  function waypointMark(w) {
    const t = w && w.type;
    const kind = t === "dock" ? "dock" : t === "home" ? "home" : "poi";
    let num = "";
    if (kind === "dock") {
      const m = /^([0-9]+|[零一二两三四五六七八九十百千]+)号/.exec((w && w.name) || "");
      if (m) {
        const n = /^[0-9]+$/.test(m[1]) ? parseInt(m[1], 10) : cnToInt(m[1]);
        if (n != null) num = String(n);
      }
    }
    return { kind, num };
  }
  // Tap radius for a keypoint in screen px: the whole drawn disc (its outer
  // radius), never less than snap_px. 2026-11-12: the disc grew to 24 px while
  // snap_px stayed 14, so a tap on the outer ring missed the keypoint.
  function keypointHitPx(snapPx, c) {
    const ms = markStyle(c);
    return Math.max(snapPx, ms.kpRadiusPx + ms.kpRingPx / 2);
  }

  // The number's font size in screen px, scaled with the disc (user 2026-11-12:
  // a larger disc, a larger number). At the first disc size (outer radius 12 px)
  // two digits fit at 14 px and three or more at 10 px; the same proportions
  // hold for any outer radius, so the doubled 24 px disc gets 28 / 20 px.
  function numFontPx(num, outerPx) {
    return Math.round((num.length <= 2 ? 14 : 10) * outerPx / 12);
  }

  // The arrow's corners in map units: (x, y) its centre, a the rotation (0 =
  // pointing north, i.e. screen up), s the half-length in map units.
  function robotArrowPoints(x, y, a, s) {
    return [[0, -s], [-s * 0.7, s], [0, s * 0.4], [s * 0.7, s]].map(([px, py]) => [
      px * Math.cos(a) - py * Math.sin(a) + x,
      px * Math.sin(a) + py * Math.cos(a) + y]);
  }
  // Where a waypoint name starts (2026-11-12, user): just right of the disc's
  // outer edge, on the disc's own horizontal centre line (the text is drawn
  // with dominant-baseline central). Screen px are turned into map metres with
  // the current mpp, so the name stays tight to the disc at every zoom.
  function keypointLabelAnchor(cx, cy, outerPx, gapPx, mpp) {
    return [cx + (outerPx + gapPx) * mpp, cy];
  }
  // A fence's look (99 U183, user 2026-11-13). type is fenceType's answer
  // (active | forbid | alarm | speed), state the catalogue state (11 S7.8.2).
  // A fence that is not switched on is a grey dashed outline with
  // "(未启用)" after its name, whatever its type; a switched-on one keeps its
  // type's colour, solid. Every fence line is 1.5x the recorded route's width
  // (configs/p5_gateway.yaml hmi.web.fence.*.line.width_px 5.25 -> 10.5 px).
  // A fence with no state (an old producer) is drawn as switched on.
  const FENCE_DISABLED_FALLBACK = { style: "dashed", color: "#9a9a9a", width_px: 5.25 };
  const FENCE_LINE_FALLBACK = { style: "solid", color: "#e0b000", width_px: 5.25 };
  function fenceLook(type, state, fenceCfg) {
    const fc = fenceCfg || {};
    const layer = type === "active" ? "keepInLayer" : "alarmLayer";
    if (state != null && state !== "active")
      return { cls: "fence-disabled", layer, suffix: "(未启用)",
               line: (fc.disabled && fc.disabled.line) || FENCE_DISABLED_FALLBACK };
    const cls = { active: "keep-in", alarm: "alarm-region",
                  forbid: "forbid-region", speed: "speed-region" }[type] || "keep-in";
    const line = ((fc[type] || fc.active || {}).line) || FENCE_LINE_FALLBACK;
    return { cls, layer, suffix: "", line };
  }
  // </map-marks>

  // <event-text>
  // HMI event list wording (user design 2026-11-12, 17 S6.6 event drawer).
  // Producers title events in English for logs and record.db; the drawer shows
  // Chinese, chosen by cat + detail.type (or detail.kind). An unmapped event
  // shows its original title -- never dropped, never guessed (recommendation 6).
  // Pure functions, cut out and run under node by test_event_drawer_js.py.
  const EV_CAT_CN = {
    charging: "充电", rtk: "定位", comm: "通信", motion: "运动", task: "任务",
    arbitration: "仲裁", payload: "载荷", ptz: "云台", health: "健康", system: "系统",
    geo: "地图", perception: "感知", bit: "自检", chassis: "底盘", teach: "录制",
    fence: "围栏",
  };
  // ~~intrusion / audio / voice (and a planned mode_change)~~ removed 2026-11-12
  // (user): no process publishes events of these categories (grep of every
  // event/ publish in xbrain/), so their wording could never be shown. A new
  // producer adds its row here together with its first event.
  const EV_SEV_CN = { fault: "故障", alarm: "告警", warn: "警告", info: "信息" };
  // health/summary item names (p2 health/items.py ITEMS) and payload devices.
  const EV_DEVICE_CN = {
    rtk: "定位", heading: "航向", chassis: "底盘", clock: "授时", compute: "算力",
    battery: "电池", cam_rgbd: "深度相机", lidar: "雷达", gpu: "GPU", dla: "DLA",
    storage: "存储", ptz: "云台", mic: "麦克风", payload_speaker: "扬声器",
    payload_siren: "警笛", payload_strobe: "爆闪灯", payload_light: "灯光",
    speech_preset: "话术", network: "网络", ptz_home: "云台归零", speaker: "扬声器",
    payload: "载荷",
  };
  const EV_HEALTH_STATE_CN = { ok: "正常", warn: "警告", degraded: "降级", fail: "故障", unknown: "未知" };
  const EV_TASK_CN = {
    accepted: "已接受", started: "已开始", completed: "已完成", failed: "已失败",
    cancelled: "已取消", suspended: "已挂起", resumed: "已恢复", paused: "已暂停",
  };
  const EV_GEO_CN = {
    "geo.created": "已新建", "geo.deleted": "已删除", "geo.renamed": "已改名",
    "geo.updated": "已更新", "geo.route_deleted": "路径已删除",
  };
  // events whose producer gives no detail.type: keyed on the bare title
  const EV_TITLE_CN = {
    estop: "急停", estop_rearm: "急停解除", rotation_blocked: "原地旋转被拦(周围空间不足)",
    rotation_clearance_unconfigured: "旋转许可未标定", nav_failed: "导航失败",
    health_factor_degraded: "健康度降级, 已限速", broadcast_fault: "广播故障",
    "ptz deadman stop": "云台遥控松手, 已停止", grant: "仲裁授予", release: "仲裁释放",
    preempt: "仲裁抢占", suspend: "仲裁挂起", rearm: "仲裁重新武装",
  };
  const EV_ARB_CN = { grant: "占用", release: "释放", preempt: "抢占" };

  function evDevice(name) { return EV_DEVICE_CN[name] || name || "设备"; }
  function evSoc(d) { return typeof d.soc_pct === "number" ? "(" + d.soc_pct + "%)" : ""; }

  // One entry -> {cat, text}: the category tag and the Chinese line.
  function eventText(ev) {
    const d = (ev && ev.detail) || {};
    const sub = d.type || d.kind || null;
    const title = (ev && ev.title) || "";
    const cat = EV_CAT_CN[ev && ev.cat] || (ev && ev.cat) || "";
    const word = (t) => ({ cat: cat, text: t });
    const first = title.split(" ")[0];
    const idIn = (re) => { const m = title.match(re); return m ? m[1] : ""; };
    switch (sub) {
      case "low_battery": return word("电量低" + evSoc(d) + ", 正在返回" + (d.target_name || "充电站"));
      case "low_battery_no_target": return word("电量低" + evSoc(d) + ", 没有可用的充电站");
      case "low_battery_unreachable": return word("电量低" + evSoc(d) + ", 剩余航程到不了充电站");
      case "low_battery_stop": return word("电量严重不足" + evSoc(d) + ", 已停车趴下");
      case "battery_charging": return word("低电后充电中" + evSoc(d));
      case "charge_interrupted": return word("未充满即中断充电" + evSoc(d));
      case "low_battery_cleared":
        return word(d.cause === "charge_full" ? "已充满, 低电告警解除"
          : d.cause === "battery_swap" ? "已换电, 低电告警解除" : "低电告警解除");
      case "return_target_missing": return word("没有定义返航目标(充电站或返航终点站)");
      case "rtk_lost": return word("定位丢失, 机器人已停车");
      case "heading_recovered": return word("定位恢复");
      case "cloud_up": return word("云端连接恢复");
      case "cloud_down": return word("云端连接断开");
      case "cloud_degraded": return word("云端连接不稳定");
      case "cloud_lost": return word("云端连接丢失");
      case "health.item_changed": {
        // 99 U166: a speaker fail says why -- online but cannot play (alarm)
        // or offline (fault) -- since the state alone reads 故障 for both.
        const why = (r) => typeof r !== "string" ? ""
          : r.indexOf("playback failed") === 0 ? "(播放失败)"
          : r.indexOf("link down") === 0 ? "(离线)" : "";
        return word(evDevice(d.device) + " " + (EV_HEALTH_STATE_CN[d.from] || d.from || "?") +
                    why(d.from_reason) + " -> " + (EV_HEALTH_STATE_CN[d.to] || d.to || "?") +
                    why(d.reason));
      }
      case "device_offline": return word(evDevice(first) + " 离线");
      case "device_online": return word(evDevice(first) + " 上线");
      case "hmi.session_closed": return word("HMI 会话关闭");
      case "unclean_shutdown": return word("上次为非正常关机");
      case "teach.time_warning": return word("录制即将到达时长上限");
      case "teach.expired": return word("录制已超时结束");
      case "zone_enter": return word("进入报警区域" + (d.zone_name ? " " + d.zone_name : ""));
      case "zone_exit": return word("离开报警区域" + (d.zone_name ? " " + d.zone_name : ""));
      case "perception.reason_changed": return word("感知状态变化");
      case "config_digest": return word("感知配置摘要");
      case "config_digest_mismatch": return word("配置摘要不一致");
      case "config_digest_ok": return word("配置摘要一致");
      case "config_manifest_unreadable": return word("配置清单不可读");
      default: break;
    }
    if (ev && ev.cat === "task" && EV_TASK_CN[sub]) {
      return word("任务 " + (d.task_id || idIn(/^task (\S+)/)) + " " + EV_TASK_CN[sub]);
    }
    if (ev && ev.cat === "geo" && EV_GEO_CN[sub]) {
      return word("地图对象 " + idIn(/^geo (\S+)/) + " " + EV_GEO_CN[sub]);
    }
    if (sub && EV_TITLE_CN[sub]) return word(EV_TITLE_CN[sub]);
    const arb = title.match(/^arbitration (grant|release|preempt) on (\S+)/);
    if (arb) return word(evDevice(arb[2]) + " " + EV_ARB_CN[arb[1]]);
    if (EV_TITLE_CN[title]) return word(EV_TITLE_CN[title]);
    return word(title || "(无标题)");
  }

  // Continuous alarms (recommendation 5): one pinned "持续中" record per family
  // while its newest start is not followed by an end. repeatS: the low-battery
  // alarm repeats every 2 s while it lasts, so an episode silent for longer
  // than repeatS has stopped even without low_battery_cleared (a reboot ended
  // it -- found live 2026-11-12: a 12:36 episode stayed "持续中" after the 12:52
  // reboot). rtk / comm are one-shot transitions: start / end pairing only.
  const EV_FAMILIES = {
    charging: { cat: "charging", name: "低电量", repeatS: 30,
                starts: ["low_battery", "low_battery_no_target", "low_battery_unreachable",
                         "low_battery_stop", "battery_charging", "charge_interrupted",
                         "return_target_missing"],
                ends: ["low_battery_cleared"] },
    rtk: { cat: "rtk", name: "定位丢失", starts: ["rtk_lost"], ends: ["heading_recovered"] },
    comm: { cat: "comm", name: "云端断链", starts: ["cloud_lost", "cloud_down", "cloud_degraded"],
            ends: ["cloud_up"] },
  };

  function eventFamily(ev) {
    const d = (ev && ev.detail) || {};
    const sub = d.type || d.kind;
    for (const k of Object.keys(EV_FAMILIES)) {
      const f = EV_FAMILIES[k];
      if (!ev || ev.cat !== f.cat) continue;
      if (f.starts.indexOf(sub) >= 0) return { fam: k, role: "start" };
      if (f.ends.indexOf(sub) >= 0) return { fam: k, role: "end" };
    }
    return null;
  }

  // items -> [{fam, name, since, latest}], each family whose last word was a start
  // (and, for a repeating family, recent enough at nowS -- P5's clock).
  // since = the first occurrence of this episode (merged entries keep first_ts).
  function activeFamilies(items, nowS) {
    const st = {};
    const sorted = (items || []).slice().sort((a, b) => (a.ts || 0) - (b.ts || 0));
    for (const it of sorted) {
      const f = eventFamily(it); if (!f) continue;
      if (f.role === "end") { delete st[f.fam]; continue; }
      const start = it.first_ts != null ? it.first_ts : it.ts;
      const cur = st[f.fam];
      st[f.fam] = { fam: f.fam, name: EV_FAMILIES[f.fam].name, latest: it,
                    since: cur ? Math.min(cur.since, start) : start };
    }
    return Object.keys(st).map((k) => st[k]).filter((a) => {
      const r = EV_FAMILIES[a.fam].repeatS;
      return !(r && typeof nowS === "number" && nowS - (a.latest.ts || 0) > r);
    }).sort((a, b) => (b.latest.ts || 0) - (a.latest.ts || 0));
  }

  // Recommendation 5, second half: when a continuous alarm ends, its entries
  // become normal records carrying the end time. items -> an array aligned with
  // items: null for a non-family entry, else {state: "live" | "ended", endTs}.
  // The episode ends at its end event (rtk heading_recovered, comm cloud_up,
  // low_battery_cleared); a repeating family (low battery) that fell silent past
  // repeatS ended at its last occurrence. An end event itself gets null -- it is
  // the record of the ending, not part of the episode.
  function familyStatus(items, nowS) {
    const out = (items || []).map(() => null);
    const order = (items || []).map((it, i) => i)
      .sort((a, b) => ((items[a].ts || 0) - (items[b].ts || 0)));
    const open = {};                         // fam -> [index, ...] of the open episode
    for (const i of order) {
      const f = eventFamily(items[i]); if (!f) continue;
      if (f.role === "start") { (open[f.fam] = open[f.fam] || []).push(i); continue; }
      for (const j of open[f.fam] || []) out[j] = { state: "ended", endTs: items[i].ts };
      delete open[f.fam];
    }
    for (const fam of Object.keys(open)) {
      const idx = open[fam];
      const lastTs = Math.max.apply(null, idx.map((j) => items[j].ts || 0));
      const r = EV_FAMILIES[fam].repeatS;
      const silent = r && typeof nowS === "number" && nowS - lastTs > r;
      for (const j of idx) out[j] = silent ? { state: "ended", endTs: lastTs } : { state: "live", endTs: null };
    }
    return out;
  }

  // Alarm / fault entries of the last windowS, newest first, at most max, minus
  // the families already pinned as "持续中".
  function pinnedAlarms(items, nowS, active, windowS, max) {
    const fams = (active || []).map((a) => a.fam);
    return (items || []).filter((it) => {
      if (it.sev !== "alarm" && it.sev !== "fault") return false;
      if (typeof it.ts !== "number" || nowS - it.ts > windowS) return false;
      const f = eventFamily(it);
      return !(f && fams.indexOf(f.fam) >= 0);
    }).sort((a, b) => b.ts - a.ts).slice(0, max);
  }

  // Drawer width while dragging an edge (wireEventDrawerResize): centred, so the
  // width changes by twice the drag; never under the default, never past maxW.
  const EV_MIN_W = 560;                    // = .event-drawer default width
  function evDrawerWidth(startW, startX, x, side, maxW) {
    const dx = side === "right" ? x - startX : startX - x;
    return Math.max(EV_MIN_W, Math.min(maxW, startW + 2 * dx));
  }

  // chip filter: "alarm" = alarm + fault, "warn", "info", "all"
  function eventPasses(ev, filter) {
    if (filter === "all") return true;
    if (filter === "alarm") return ev.sev === "alarm" || ev.sev === "fault";
    return ev.sev === filter;
  }
  // </event-text>

  // Chosen every view change (applyView) from the CURRENT metres-per-screen-px.
  // applyView keeps the viewBox at the element's aspect, so the scale is uniform
  // (x==y) and mpp is a single value.
  function updateGridAndDots() {
    const svg = $("mapSvg");
    const cw = svg.clientWidth || 1;
    const vb = svg.getAttribute("viewBox");
    const vbW = vb ? parseFloat(vb.split(/\s+/)[2]) : baseVB[2];
    const mpp = vbW / cw;                  // metres per screen px
    const grid = (cfg && cfg.map && cfg.map.grid) || {};
    const cells = grid.cell_options_m || GRID_CELLS_DEFAULT;
    // smallest configured cell whose on-screen size >= target; else the largest.
    let cell = cells[cells.length - 1];
    for (const cm of cells) { if (cm / mpp >= GRID_TARGET_PX) { cell = cm; break; } }
    const hidden = cell / mpp < GRID_MIN_PX;
    setPattern("majorGrid", cell, `M ${cell} 0 L 0 0 0 ${cell}`);
    $("majorGridRect").setAttribute("width", cell);
    $("majorGridRect").setAttribute("height", cell);
    // Fixed SCREEN line width (user 2026-11-10: never thicker or thinner with
    // zoom). The pattern draws each line on its tile's edge, so half the stroke
    // is clipped -- hence 2x. Inline style, because the .map-grid-major class
    // rule would beat a presentation attribute.
    const linePx = grid.line_px || GRID_LINE_PX_FALLBACK;
    $("majorGridPath").style.strokeWidth = (2 * linePx * mpp).toFixed(4);
    // 2026-11-11 (user): grid lines one step darker, colour from the yaml
    $("majorGridPath").style.stroke = grid.line_color || GRID_LINE_COLOR_FALLBACK;
    $("gridBg").style.display = hidden ? "none" : "";
    $("scaleText").textContent = hidden ? "网格 已隐藏" : `网格 ${cell}m`;
    // non-scaling geo dots: constant screen radius regardless of zoom.
    const rr = (DOT_PX * mpp).toFixed(3);
    svg.querySelectorAll("circle.geo-dot").forEach((c) => c.setAttribute("r", rr));
    const ms = markStyle(cfg);
    const kr = (ms.kpRadiusPx * mpp).toFixed(3);  // keypoints: own size
    svg.querySelectorAll("circle.keypoint-dot").forEach((c) => c.setAttribute("r", kr));
    // yellow / green discs: same outer radius, the 1 px edge drawn inside it
    const outer = ms.kpRadiusPx + ms.kpRingPx / 2;
    const dr = ((outer - ms.discOutlinePx / 2) * mpp).toFixed(3);
    svg.querySelectorAll("circle.keypoint-disc").forEach((c) => c.setAttribute("r", dr));
    svg.querySelectorAll("text.keypoint-num").forEach((t) => {
      t.style.fontSize = (numFontPx(t.textContent, outer) * mpp).toFixed(3) + "px"; });
    // waypoint names: right of the disc, tight (keypointLabelAnchor)
    svg.querySelectorAll("text.keypoint-label").forEach((t) => {
      const a = keypointLabelAnchor(parseFloat(t.dataset.cx), parseFloat(t.dataset.cy),
                                    outer, ms.labelGapPx, mpp);
      t.setAttribute("x", a[0]); t.setAttribute("y", a[1]); });
    sizeGotoPins();                        // W2 target dots: constant screen size
  }

  // req1 (2026-08-18): keep geo NAMES from overlapping. Greedy de-confliction in
  // SCREEN space: each label keeps its anchor if free, else is nudged (down / up /
  // side) to the first collision-free slot; if none fits (map too dense at this
  // zoom) it is hidden and reappears when zoom-in spreads the features. Runs at
  // the end of renderGeo (labels are fresh at their anchors each snapshot) and is
  // re-evaluated as the operator zooms (2 Hz re-render).
  function estWidthPx(txt, fontPx) {
    let u = 0;                             // CJK ~1em, latin/space ~0.52em
    for (let i = 0; i < txt.length; i++) u += (txt.charCodeAt(i) > 255 ? 1.0 : 0.52);
    return u * fontPx + 4;
  }
  // The number inside a charging-point disc is part of the mark, not a label:
  // nudging or hiding it would separate it from its disc (2026-11-12).
  const LABEL_LAYERS = "#keepInLayer text, #alarmLayer text, #recordedRouteLayer text, " +
                       "#realtimeTrajectoryLayer text, #keypointLayer text:not(.keypoint-num)";
  function declutterLabels() {
    const svg = $("mapSvg");
    const cw = svg.clientWidth || 1;
    const vb = (svg.getAttribute("viewBox") || "").split(/\s+/).map(Number);
    if (vb.length < 4) return;
    const mpp = vb[2] / cw;                // metres per screen px (uniform)
    const fontPx = parseFloat(getComputedStyle(document.documentElement)
      .getPropertyValue("--font-plan-size")) || 24;
    const hpx = fontPx * 1.05;
    const placed = [];
    const hit = (a, b) => !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1);
    // candidate offsets (columns of screen px): stay, then step down/up/side.
    const cand = [[0, 0], [0, 1], [0, -1], [0, 2], [0, -2], [1, 0], [-1, 0],
                  [1, 1], [-1, 1], [1, -1], [-1, -1], [0, 3], [0, -3]];
    for (const t of svg.querySelectorAll(LABEL_LAYERS)) {
      t.style.display = "";
      const ax = parseFloat(t.getAttribute("x")), ay = parseFloat(t.getAttribute("y"));
      const sx = (ax - vb[0]) / mpp, sy = (ay - vb[1]) / mpp;   // anchor in screen px
      const wpx = estWidthPx(t.textContent || "", fontPx);
      let done = false;
      for (const [ox, oy] of cand) {
        const dx = ox * (wpx * 0.5 + 4), dy = oy * (hpx + 2);
        // a waypoint name is drawn centred on its anchor (dominant-baseline
        // central, 2026-11-12); every other label sits on its baseline
        const central = t.classList.contains("keypoint-label");
        const box = central
          ? { x1: sx + dx - 1, y1: sy + dy - hpx / 2, x2: sx + dx + wpx, y2: sy + dy + hpx / 2 }
          : { x1: sx + dx - 1, y1: sy + dy - hpx, x2: sx + dx + wpx, y2: sy + dy + 2 };
        if (!placed.some((p) => hit(box, p))) {
          if (dx || dy) { t.setAttribute("x", ax + dx * mpp); t.setAttribute("y", ay + dy * mpp); }
          placed.push(box);
          done = true;
          break;
        }
      }
      if (!done) t.style.display = "none";   // no room at this zoom -> hide
    }
  }

  // -- render one snapshot -------------------------------------------------- */
  function renderSnapshot(snap) {
    // Footer-clock zone follows the GPS fix (17 S6.10.2 v1.3). snapshot.timezone
    // is derived server-side (pose lat/lon -> IANA), or the common.timezone
    // fallback when there is no fix -- always a usable string, so just adopt it.
    // The 1 Hz ticker reads siteTz each tick, so the clock switches within a
    // second of crossing a band. Guard on truthiness so an older backend that
    // omits the field cannot clobber the ui_config default with undefined.
    if (snap.timezone) siteTz = snap.timezone;
    const origin = (snap.geo && snap.geo.enu_origin) || null;
    lastOrigin = origin;                  // W2 needs it to place a target
    lastGeo = snap.geo || {};             // W2 snaps a tap to these points
    // patrolling = a plan is running -> the yellow realtime trajectory shows;
    // once no plan runs (patrol complete) it hides unless show_after_complete.
    const plans = (snap.plan && snap.plan.plans) || [];
    const patrolling = plans.some((p) => p.state === "running");
    renderGeo(snap.geo || {}, origin, patrolling);
    renderRobot(snap.pose || {}, origin);
    renderEvents(snap.events || {}, snap.server_ts);
    // The task panel is NOT rendered from the snapshot -- it pulls current +
    // history from GET /api/tasks (17 S6.8.4, refreshTasks below). snap.plan
    // (state/task) still drives the `patrolling` trajectory flag above.
    renderStatus(snap.status || {}, snap.pose || {}, origin, snap.clock || {});
    renderChassisLock((snap.status || {}).chassis_lock || {});   // 99 U159
    renderBattery(snap.battery || {});                            // 99 U160
    renderHeading(snap.pose || {});
    renderTeach(snap.teach || {}, origin, snap.pose || {});
    renderGotoPins();                     // W2 target dot(s), fixed screen size
    followGoto(plans);                    // done / failed -> dot off, banner
    // Task state changed -> pull the panel now rather than on the 4 s poll, so
    // the state flow on the cards moves with the robot (user 2026-11-10).
    const sig = plans.map((p) => p.task_id + ":" + p.state).join("|");
    if (sig !== lastPlanSig) { lastPlanSig = sig; refreshTasks(); }
  }
  let lastPlanSig = null;

  // -- 录制指示 (11 S12A.5) ------------------------------------------------
  //
  // 只读: HMI 开不了也停不了录制 (S12.1.1 白名单 5 类不含 teach), 这里只把 P3
  // 正在做的事画出来.
  //
  // * 为什么要在前端累积: state/teach [故意不带点序列] -- S12A.5 逐字说 2000
  // 个点会把 1 Hz 状态话题撑爆, 只给 stats + last_point. 所以轨迹是按
  // last_point.seq 逐帧攒出来的, 是[近似]: 采点 1 Hz, 推送 1 Hz, 丢一帧就少一
  // 个点, 折线会抄近路. 图例上标了"近似"就是这个意思.
  // * 保存后由 geo.db 的权威几何 (state/geo/objects) 接管 -- 所以会话一结束就把
  // 这条近似轨迹清掉, NO 不留在图上冒充已保存的路径.
  let teachTrail = [];        // [[x, y], ...] 已攒到的点 (frame 坐标)
  let teachSeq = 0;           // 见过的最大 last_point.seq
  let teachSession = null;    // 当前会话 id, 变了就重来

  // U10 flow effect (17 S6.10.2, user 2026-11-12): a short bright segment runs
  // along the recording polyline from its start to the robot, one trip per
  // period_s, looping. It lives in its own layer (#teachFlowLayer) and is moved
  // by requestAnimationFrame, NOT redrawn by renderTeach: renderTeach clears
  // #teachLayer on every snapshot (2 Hz), which would restart a CSS animation
  // twice a second. Lengths are SCREEN px (converted with the current metres-
  // per-pixel), so the segment looks the same at every zoom.
  // Defaults mirror configs/p5_gateway.yaml hmi.web.teach.flow (UI values may
  // default, 17 S6.10.2); the yaml wins once P5 serves it.
  let teachFlowPts = [];       // the polyline the flow runs on (frame coords)
  let teachFlowRunning = false;  // a rAF loop is scheduled
  function teachFlowCfg() {
    const f = ((cfg.teach || {}).flow) || {};
    return { color: f.color || "#ffffff", width: f.width_px || 4,
             length: f.length_px || 60, period: f.period_s || 1.4 };
  }
  // <teach-flow>
  // The bright segment for one frame, PURE (cut out and run under node by
  // tests/p5_gateway/hmi/test_teach_flow_js.py): pts = the polyline in frame
  // metres, mpp = metres per screen pixel, lengthPx / periodS from the config,
  // nowMs = the animation clock. The head runs 0 -> total + seg, so the
  // segment enters at the start and leaves at the robot instead of popping in
  // or out mid-line. Returns [] when there is nothing to draw.
  function teachFlowSegment(pts, mpp, lengthPx, periodS, nowMs) {
    if (!pts || pts.length < 2 || !(mpp > 0) || !(periodS > 0)) return [];
    let total = 0;                           // polyline length, map metres
    for (let i = 1; i < pts.length; i++) total += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
    const seg = lengthPx * mpp;              // the bright segment, map metres
    const head = ((nowMs / 1000) % periodS) / periodS * (total + seg);
    const a0 = Math.max(0, head - seg), a1 = Math.min(total, head);
    const out = [];
    let acc = 0;
    // a1 > a0 is also what returns [] for a zero-length line (all points on
    // one spot): total = 0 clamps a1 to 0, so no edge is walked.
    for (let i = 1; i < pts.length && a1 > a0; i++) {
      const p = pts[i - 1], q = pts[i];
      const sl = Math.hypot(q[0] - p[0], q[1] - p[1]);
      const s0 = acc, s1 = acc + sl;
      if (sl > 0 && s1 >= a0 && s0 <= a1) {  // this edge carries part of the segment
        const f0 = Math.max(0, (a0 - s0) / sl), f1 = Math.min(1, (a1 - s0) / sl);
        if (!out.length) out.push([p[0] + (q[0] - p[0]) * f0, p[1] + (q[1] - p[1]) * f0]);
        out.push([p[0] + (q[0] - p[0]) * f1, p[1] + (q[1] - p[1]) * f1]);
      }
      acc = s1;
    }
    return out.length >= 2 ? out : [];
  }
  // </teach-flow>
  function teachFlowTick(now) {
    const el = $("teachFlow");
    if (teachFlowPts.length < 2) {           // recording ended / paused
      el.setAttribute("points", ""); teachFlowRunning = false; return;
    }
    const fc = teachFlowCfg();
    const svg = $("mapSvg"), cw = svg.clientWidth || 1;
    const mpp = (parseFloat((svg.getAttribute("viewBox") || "").split(/\s+/)[2]) || baseVB[2]) / cw;
    const out = teachFlowSegment(teachFlowPts, mpp, fc.length, fc.period, now);
    el.setAttribute("points", out.map((p) => p[0] + "," + p[1]).join(" "));
    el.style.stroke = fc.color;
    el.style.strokeWidth = fc.width + "px";
    el.style.filter = "drop-shadow(0 0 4px " + fc.color + ")";  // the glow
    requestAnimationFrame(teachFlowTick);
  }

  const TEACH_STATE_CN = {
    recording: "录制中", paused: "已暂停", finalizing: "待命名",
    arming: "准备中", closed: "已结束", idle: "无录制",
  };

  function renderTeach(teach, origin, pose) {
    const badge = $("teachBadge");
    clear("teachLayer");
    const live = teach.available &&
      ["arming", "recording", "paused", "finalizing"].indexOf(teach.state) >= 0;
    if (!live) {
      // 会话结束 / 没有会话 / 这个后端还没有 teach 组: 收起指示并丢掉近似轨迹.
      teachTrail = []; teachSeq = 0; teachSession = null;
      badge.hidden = true;
      return;
    }
    if (teach.session_id !== teachSession) {
      // 新会话 (或 P3 重启后重新编号): 上一条的点绝不能续到这一条上.
      teachTrail = []; teachSeq = 0; teachSession = teach.session_id;
    }
    const lp = teach.last_point;
    if (lp && typeof lp.seq === "number" && lp.seq > teachSeq) {
      const xy = toXY(lp, origin);
      // seq 是单调的, 所以只按它去重就够 -- 不比坐标 (原地打的 mark 点坐标可能
      // 与上一个几乎相同, 按坐标去重会把 F05 手动打的点吃掉).
      if (xy) { teachTrail.push(xy); teachSeq = lp.seq; }
    }
    // 2026-11-12 (用户): 录制中的轨迹只画折线, 粗细与已录制路径相同 (同一个
    // 配置 route.recorded.line.width_px, lineStroke 按 x2 画成屏幕 7 px, CSS
    // non-scaling-stroke 保证缩放不变粗); 颜色仍是录制粉 (CSS --teach-color
    // 覆盖 lineStroke 写的绿色). 折线末端接到机器人当前位置, 让线一直连到红色
    // 三角 -- 采点 1 Hz, 不接的话线头总落后机器人一截.
    // ~~每个采样点画一个半径 1 m 的圆点, 当前点再画一个 r=4 m 的圆圈~~ -- 被替代
    // (2026-11-12): 两者都按地图米制画, 放大后圆点连成一条很粗的粉带, 圆圈变成
    // 盖住机器人的大粉圈 (用户截图).
    const livePts = teachTrail.slice();
    const rxy = (pose && pose.available && pose.lat != null)
      ? toXY({ lat: pose.lat, lon: pose.lon }, origin) : null;
    if (rxy && livePts.length) livePts.push(rxy);  // the line reaches the triangle
    if (livePts.length >= 2) {
      const pts = livePts.map((p) => p[0] + "," + p[1]).join(" ");
      $("teachLayer").appendChild(
        polyline("teach-trail", pts, ((cfg.route || {}).recorded || {}).line));
    }
    // U10 flow (user 2026-11-12, chose "流光" from a demo): only while
    // actually recording -- paused / finalizing keep the still pink line.
    teachFlowPts = (teach.state === "recording" && livePts.length >= 2) ? livePts : [];
    if (teachFlowPts.length && !teachFlowRunning) {
      teachFlowRunning = true;
      requestAnimationFrame(teachFlowTick);
    }
    badge.hidden = false;
    $("teachDot").className = "teach-dot " + teach.state;
    $("teachState").textContent = TEACH_STATE_CN[teach.state] || teach.state;
    $("teachKind").textContent = teach.kind === "fence" ? "围栏" : "路径";
    $("teachName").textContent = teach.name_hint || "";
    // 点数用 P3 的 stats, 不用 teachTrail.length -- 后者是丢帧后的近似, 前者是
    // 真正入库的点数, 两者不一致时该信 P3 的.
    $("teachPoints").textContent = (teach.point_count || 0) + " 点";
    $("teachLength").textContent = fmtLen(teach.length_m || 0);
    $("teachElapsed").textContent = fmtDur(teach.elapsed_s || 0);
    const warns = (teach.warn || []).slice();
    if (teach.dropped_by_quality) warns.push("丢弃 " + teach.dropped_by_quality);
    const w = $("teachWarn");
    w.hidden = warns.length === 0;
    w.textContent = warns.join(" / ");
  }

  function fmtLen(m) {
    return m >= 1000 ? (m / 1000).toFixed(2) + " km" : m.toFixed(1) + " m";
  }
  function fmtDur(sec) {
    const s = Math.max(0, Math.round(sec));
    return s < 60 ? s + "s" : Math.floor(s / 60) + "m" + (s % 60) + "s";
  }

  function clear(id) { const g = $(id); while (g.firstChild) g.removeChild(g.firstChild); }
  function poly(cls, pts, cfgLine) {
    const el = document.createElementNS(SVGNS, "polygon");
    el.setAttribute("class", cls); el.setAttribute("points", pts);
    lineStroke(el, cfgLine); return el;
  }
  function polyline(cls, pts, cfgLine) {
    const el = document.createElementNS(SVGNS, "polyline");
    el.setAttribute("class", cls); el.setAttribute("points", pts);
    lineStroke(el, cfgLine); return el;
  }
  function label(x, y, cls, text) {
    const t = document.createElementNS(SVGNS, "text");
    t.setAttribute("x", x); t.setAttribute("y", y); t.setAttribute("class", cls);
    t.textContent = text; return t;
  }

  // req(2026-08-18): the mock data spans ~5km, far beyond the config 240m
  // default view. Auto-fit the map to the geo BOUNDS once, on the first snapshot
  // that carries the fences (they define the true extent -- the active-area
  // lake), so the operator sees the whole site on load. toXY maps every point
  // (ENU array or WGS84 {lat,lon}) into the SVG frame (north up), so the fit
  // bounds are already in frame coords. Zoom-in is widened
  // so the 5m grid is legible when zoomed in. Fits ONCE; afterwards the operator
  // pans/zooms freely.
  let geoFitted = false;
  function fitGeoToData(geo, origin) {
    if (geoFitted) return;
    const f = geo.fences || {};
    if (!(f.available && (f.items || []).length)) return;   // wait for fences
    const xs = [], ys = [];
    // v1.5 PLAN A: routes/keypoints are WGS84 {lat,lon} and need the origin to
    // project; ENU fence arrays ignore it. Passing origin lets all three define
    // the fit extent (fences alone already bound it -- the active-area lake).
    const add = (p) => { const xy = toXY(p, origin); if (xy) { xs.push(xy[0]); ys.push(xy[1]); } };
    for (const it of f.items) (it.vertices || []).forEach(add);
    const r = geo.routes || {}; if (r.available) for (const it of r.items) (it.points || []).forEach(add);
    const w = geo.waypoints || {}; if (w.available) for (const it of w.items) add(it.geom || it);
    if (xs.length < 3) return;
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    let vbW = (maxX - minX) * 1.12 + 40, vbH = (maxY - minY) * 1.12 + 40;
    // Match the viewport aspect so preserveAspectRatio="none" does NOT distort
    // the true shape: a 5km x 1km lake stays 5:1 and dots stay round (uniform
    // scale). Otherwise the box would be stretched to fill the viewport.
    const svg = $("mapSvg");
    const ar = (svg.clientWidth || 4) / (svg.clientHeight || 3);
    if (vbW / vbH > ar) vbH = vbW / ar; else vbW = vbH * ar;
    baseVB = [cx - vbW / 2, cy - vbH / 2, vbW, vbH];
    view.zoom = 1; view.pan = { x: 0, y: 0 };
    // widen zoom-in so ~200m (5m grid legible) is reachable from the ~5km fit.
    zoomBounds = { min: 0.5, max: Math.max(zoomBounds.max, Math.ceil((maxX - minX) / 200)),
                   wheel: zoomBounds.wheel };
    geoFitted = true;
    applyView();
  }

  function renderGeo(geo, origin, patrolling) {
    fitGeoToData(geo, origin);
    clear("keepInLayer"); clear("alarmLayer");
    clear("recordedRouteLayer"); clear("realtimeTrajectoryLayer"); clear("keypointLayer");
    const fences = geo.fences || {};
    if (fences.available) for (const fen of fences.items) {
      const pts = ptsAttr(fen.vertices || [], origin); if (!pts) continue;
      const type = fenceType(fen);            // active | forbid | alarm | speed
      // Per-role polygon class carries the stroke colour (CSS var) + faint fill
      // tint: active blue / warning(alarm) bright red / forbid dark red /
      // speed_limit bright blue. Fill comes from the class, not an inline attr.
      // 99 U183: a fence not switched on is grey dashed (fenceLook).
      const look = fenceLook(type, fen.state, cfg.fence);
      const layer = look.layer;
      $(layer).appendChild(poly(look.cls, pts, look.line));
      // Fence name label -- unified bright white (CSS .keep-in-label), NOT the
      // line colour (user 2026-08-18). No per-layer inline fill.
      if (fen.name && fen.vertices && fen.vertices[0]) {
        const xy = toXY(fen.vertices[0], origin);
        // req4d: name + fence perimeter in metres, e.g. "东油库报警区 (1627 m)".
        // Vertices are WGS84 {lat,lon} now (v1.5) -- measure on the PROJECTED ring
        // so the perimeter is metres, not degrees.
        const fproj = (fen.vertices || []).map((p) => toXY(p, origin)).filter(Boolean);
        if (xy) $(layer).appendChild(label(xy[0], xy[1] - 2, "keep-in-label",
          fen.name + look.suffix + lenTag(perimM(fproj))));
      }
    }
    const routes = geo.routes || {};
    if (routes.available) for (const r of routes.items) {
      const pts = ptsAttr(r.points || [], origin); if (!pts) continue;
      const realtime = r.kind === "realtime";
      if (realtime) {
        // 17 S6.10.2A: the yellow realtime trajectory shows while patrolling;
        // after patrol completes it hides unless show_after_complete is true.
        const rt = (cfg.route || {}).realtime || {};
        if (!patrolling && rt.show_after_complete === false) continue;
      }
      const rline = realtime ? cfg.route.realtime.line : cfg.route.recorded.line;
      const rlayer = realtime ? "realtimeTrajectoryLayer" : "recordedRouteLayer";
      $(rlayer).appendChild(
        polyline(realtime ? "realtime-trajectory" : "recorded-route", pts, rline));
      // 2026-11-12 (user): a route is the polyline alone, as thick as the old
      // point dots were wide (2 x DOT_PX = 7 screen px; lineStroke doubles the
      // configured width, so route.recorded.line.width_px is 3.5).
      // ~~a round dot at EVERY RTK point on the route (req1, 2026-08-18)~~
      // superseded. A tap still snaps to the nearest route point (pickGotoTarget,
      // snap_px); recorded points are about 0.55 m apart, so a tap on the line
      // lands within snap_px of one at any normal zoom.
      // Route name label at its first vertex -- unified bright white (CSS).
      if (r.name && r.points && r.points[0]) {
        const rxy = toXY(r.points[0], origin);
        // req4d: name + route length in metres, e.g. "环湖主巡逻线 (4561 m)".
        // Measure on the PROJECTED points (WGS84 {lat,lon} -> SVG metres) so the
        // length is metres, not degrees (v1.5 PLAN A).
        const rproj = (r.points || []).map((p) => toXY(p, origin)).filter(Boolean);
        if (rxy) $(rlayer).appendChild(label(rxy[0], rxy[1] - 2, "route-label",
          r.name + lenTag(segLen(rproj))));
      }
    }
    const wps = geo.waypoints || {};
    if (wps.available) for (const w of wps.items) {
      const xy = toXY(w.geom || w, origin); if (!xy) continue;
      // req1: keypoint = a round dot (constant screen size) + its name in bright
      // white (CSS .keypoint-label). The old ".keypoint text" selector never
      // matched a <text class="keypoint">, hence the dedicated class.
      // 2026-11-11 (user): white dot, red ring, one look for every keypoint --
      // ~~fill = cfg.waypoint.recorded / unrecorded colour~~ superseded.
      const ms = markStyle(cfg), wm = waypointMark(w);
      if (wm.kind === "poi") {
        const kd = dot(xy[0], xy[1], null, "keypoint-dot");
        kd.style.fill = ms.kpFill; kd.style.stroke = ms.kpRing;   // hmi.web.waypoint
        kd.style.strokeWidth = ms.kpRingPx + "px";  // non-scaling (hmi.css)
        $("keypointLayer").appendChild(kd);
      } else {
        // 2026-11-12: charging point yellow (+ its number), return-home green
        const kd = dot(xy[0], xy[1], null, "keypoint-disc");
        kd.style.fill = wm.kind === "dock" ? ms.dockFill : ms.homeFill;
        kd.style.stroke = ms.discOutline;
        kd.style.strokeWidth = ms.discOutlinePx + "px";
        $("keypointLayer").appendChild(kd);
        if (wm.num) {
          const nt = label(xy[0], xy[1], "keypoint-num", wm.num);
          nt.style.fill = ms.numColor;
          $("keypointLayer").appendChild(nt);
        }
      }
      if (w.name) {
        // 2026-11-12 (user): the name right of the disc, centred, tight to it.
        // The disc centre is kept on the element; updateGridAndDots places the
        // text from it on every view change (the gap is screen px, so its map
        // length changes with zoom). ~~2 m above the point, left-aligned~~
        // superseded: 2 map metres grew and shrank with zoom.
        const lt = label(xy[0], xy[1], "keypoint-label", w.name);
        lt.dataset.cx = xy[0]; lt.dataset.cy = xy[1];
        $("keypointLayer").appendChild(lt);
      }
    }
    updateGridAndDots();     // size the dots just drawn to a constant screen radius
    declutterLabels();       // req1: nudge/hide names so they never overlap
  }

  // req1 (2026-08-18): a geo dot -- a round <circle class="geo-dot"> whose radius
  // is set to a constant screen size by updateGridAndDots (placeholder r here).
  // Used for both route RTK points and keypoints so they never vanish on a big map.
  function dot(x, y, color, cls) {
    const c = document.createElementNS(SVGNS, "circle");
    c.setAttribute("cx", x); c.setAttribute("cy", y);
    c.setAttribute("r", 1);
    // cls lets a caller take the colour from CSS instead of an inline fill
    // (the keypoint discs do). ~~the teach trail does~~ -- it draws no dots
    // since 2026-11-12, only a polyline.
    c.setAttribute("class", cls || "geo-dot");
    if (!cls) c.setAttribute("fill", color || "#3aa0ff");
    return c;
  }


  function renderRobot(pose, origin) {
    clear("robotLayer");
    // No fix -> draw nothing. Never plot the robot at (0,0) (17 S6.10.4).
    if (!pose.available || pose.lat == null) return;
    const xy = toXY({ lat: pose.lat, lon: pose.lon }, origin); if (!xy) return;
    // Arrow base points north (up) at a=0. ENU heading is east=0, CCW+; on the
    // north-up screen (y=-north) that maps to rotation a = pi/2 - heading. Heading
    // invalid -> a=0 (points north as a neutral default). NOTE: the valid-heading
    // direction is untested here (heading is LOSS in this env) -- verify with real
    // RTK heading. s in ENU metres, so the arrow is small on a wide (5km) fit.
    const svg = $("mapSvg"), cw = svg.clientWidth || 1;
    const mpp = (parseFloat((svg.getAttribute("viewBox") || "").split(/\s+/)[2]) || baseVB[2]) / cw;
    // 2026-11-11 (user): 3x the old size (hmi.web.robot.size_px, was 9) and red.
    const ms = markStyle(cfg);
    const a = pose.heading_valid ? (Math.PI / 2 - (pose.heading_rad || 0)) : 0;
    const s = ms.robotPx * mpp;
    const el = document.createElementNS(SVGNS, "polygon");
    // simple arrow along heading
    const pts = robotArrowPoints(xy[0], xy[1], a, s)
      .map(([rx, ry]) => `${rx},${ry}`).join(" ");
    el.setAttribute("points", pts); el.setAttribute("class", "robot-arrow");
    el.style.fill = ms.robotColor;                // hmi.web.robot.color
    $("robotLayer").appendChild(el);
  }

  // -- event drawer (user design 2026-11-12, 17 S6.6) -----------------------
  // Bottom-centre drawer in the history-panel style: collapsed it is one bar
  // (alarm count + the newest line), expanded about half the map high with
  // chips 告警/故障 / 警告 / 信息 / 全部, the "持续中" families and recent
  // alarms / faults pinned on top. Collapses after 10 s without mouse movement
  // or on a click of its title. The standing red event dots on the map are gone
  // (recommendation 8): an entry with a place flashes it when clicked.
  const EV_PIN_WINDOW_S = 600;     // alarms / faults of the last 10 min stay pinned
  const EV_PIN_MAX = 5;            // at most this many pinned alarm / fault rows
  const EV_IDLE_MS = 10000;        // same idle collapse as the history panel
  const EV_FLASH_MS = 3000;        // a clicked place blinks this long
  let evItems = [];                // the snapshot's events.items, ring order
  let evAvailable = false;
  let evExpanded = false;
  let evFilter = "all";
  let evIdleTimer = null;
  let evSig = null;                // skip a re-render when nothing changed
  let evRowRefs = [];              // row index -> entry, for the click handler
  let evServerTs = null;           // snapshot.server_ts: P5's clock, the events' clock

  // HH:MM:SS in siteTz; MM-DD in front when it is not today there.
  function fmtEvTime(ts) {
    if (typeof ts !== "number") return "--";
    const opts = { month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit",
                   second: "2-digit", hour12: false };
    const o = siteTz ? Object.assign({ timeZone: siteTz }, opts) : opts;
    const parts = (d) => {
      const g = {};
      try { for (const p of new Intl.DateTimeFormat("zh-CN", o).formatToParts(d)) g[p.type] = p.value; }
      catch (e) { return null; }
      return g;
    };
    const g = parts(new Date(ts * 1000)), n = parts(new Date());
    if (!g) return "--";
    const hms = `${g.hour}:${g.minute}:${g.second}`;
    return (n && n.month === g.month && n.day === g.day) ? hms : `${g.month}-${g.day} ${hms}`;
  }

  function evRowHTML(ev, idx, extraCls, timeHTML, tagHTML) {
    const t = eventText(ev);
    const cnt = (ev.count || 1) > 1 ? `<span class="ev-count">${ev.count} 次</span>` : "";
    const cls = `ev-row sev-${esc(ev.sev || "info")}${ev.pos ? " has-pos" : ""}${extraCls || ""}`;
    return `<div class="${cls}" data-ev="${idx}"${ev.pos ? ' title="点击在地图上闪烁位置"' : ""}>` +
      `<span class="ev-time">${timeHTML}</span><span class="ev-cat">${esc(t.cat)}</span>` +
      `<span class="ev-text">${esc(t.text)}</span>${tagHTML || ""}${cnt}</div>`;
  }
  // A continuous-alarm entry: "first - end" plus 已结束, or 持续中 while live.
  function evFamilyTime(ev, st) {
    const start = typeof ev.first_ts === "number" ? ev.first_ts : ev.ts;
    if (st.state === "live") return { time: `${fmtEvTime(start)} 起`, tag: `<b class="ev-state live">持续中</b>` };
    return { time: `${fmtEvTime(start)} - ${fmtEvTime(st.endTs)}`, tag: `<b class="ev-state ended">已结束</b>` };
  }
  function evTimeRange(ev) {
    const many = (ev.count || 1) > 1 && typeof ev.first_ts === "number" && ev.first_ts !== ev.ts;
    return many ? `${fmtEvTime(ev.first_ts)} - ${fmtEvTime(ev.ts)}` : fmtEvTime(ev.ts);
  }

  function renderEvents(events, serverTs) {
    if (typeof serverTs === "number") evServerTs = serverTs;
    evAvailable = !!events.available;
    evItems = evAvailable ? (events.items || []) : [];
    drawEventDrawer(false);
  }

  function drawEventDrawer(force) {
    const box = $("eventDrawer"); if (!box) return;
    const last = evItems[evItems.length - 1];
    // An older backend has no server_ts: the browser clock is the fallback.
    const nowS = evServerTs != null ? evServerTs : Date.now() / 1000;
    const sig = [evAvailable, evExpanded, evFilter, evItems.length,
                 last ? last.eid + ":" + last.count : "", Math.floor(nowS / 10)].join("|");
    if (!force && sig === evSig) return;
    evSig = sig;
    box.classList.toggle("expanded", evExpanded);
    const active = activeFamilies(evItems, nowS);
    const pins = pinnedAlarms(evItems, nowS, active, EV_PIN_WINDOW_S, EV_PIN_MAX);
    const nAlarm = active.length + pins.length;
    evRowRefs = [];
    const ref = (ev) => { evRowRefs.push(ev); return evRowRefs.length - 1; };
    // collapsed bar: always there, it is also the title of the expanded drawer
    const newest = evItems.slice().sort((a, b) => (b.ts || 0) - (a.ts || 0))[0];
    $("eventBarCount").textContent = nAlarm ? `告警 ${nAlarm}` : "无告警";
    $("eventBarCount").classList.toggle("has-alarm", nAlarm > 0);
    $("eventBarLatest").textContent = !evAvailable ? "事件源不可用"
      : newest ? `${fmtEvTime(newest.ts)}  ${eventText(newest).text}` : "暂无事件";
    if (!evExpanded) { $("eventPinned").innerHTML = ""; $("eventList").innerHTML = ""; return; }
    document.querySelectorAll("#eventChips .ev-chip").forEach((c) =>
      c.classList.toggle("on", c.dataset.filter === evFilter));
    $("eventPinned").innerHTML =
      active.map((a) => evRowHTML(a.latest, ref(a.latest), " ev-live",
        `<b class="ev-live-tag">持续中</b> 自 ${fmtEvTime(a.since)}`)).join("") +
      pins.map((ev) => evRowHTML(ev, ref(ev), " ev-pin", evTimeRange(ev))).join("");
    const keep = $("eventList").scrollTop;      // new rows must not jump the reader
    const fst = familyStatus(evItems, nowS);
    const rows = evItems.map((ev, i) => [ev, fst[i]]).filter(([ev]) => eventPasses(ev, evFilter))
      .sort((a, b) => (b[0].ts || 0) - (a[0].ts || 0));
    $("eventList").innerHTML = rows.length
      ? rows.map(([ev, st]) => {
          if (!st) return evRowHTML(ev, ref(ev), "", evTimeRange(ev));
          const ft = evFamilyTime(ev, st);
          return evRowHTML(ev, ref(ev), "", ft.time, ft.tag);
        }).join("")
      : `<div class="task-empty">${evAvailable ? "暂无事件" : "事件源不可用"}</div>`;
    $("eventList").scrollTop = keep;
  }

  // Restart the idle countdown only -- no re-render, so the list keeps its
  // scroll position while the operator reads it.
  function armEventIdle() {
    if (evIdleTimer) { clearTimeout(evIdleTimer); evIdleTimer = null; }
    if (evExpanded) evIdleTimer = setTimeout(() => setEventDrawer(false), EV_IDLE_MS);
  }
  function setEventDrawer(open) {
    const changed = evExpanded !== open;
    evExpanded = open;
    armEventIdle();
    if (changed) drawEventDrawer(true);
  }

  // Blink the entry's place; pan there first when it is off screen.
  function flashEventPos(ev) {
    const xy = ev && ev.pos ? toXY(ev.pos, lastOrigin) : null;
    if (!xy) return;
    const svg = $("mapSvg");
    let vb = (svg.getAttribute("viewBox") || "").split(/\s+/).map(Number);
    if (xy[0] < vb[0] || xy[0] > vb[0] + vb[2] || xy[1] < vb[1] || xy[1] > vb[1] + vb[3]) {
      view.pan = { x: xy[0] - (baseVB[0] + baseVB[2] / 2), y: xy[1] - (baseVB[1] + baseVB[3] / 2) };
      applyView();
      vb = (svg.getAttribute("viewBox") || "").split(/\s+/).map(Number);
    }
    const mpp = (vb[2] || baseVB[2]) / (svg.clientWidth || 1);
    clear("eventLayer");
    const c = document.createElementNS(SVGNS, "circle");
    c.setAttribute("cx", xy[0]); c.setAttribute("cy", xy[1]);
    c.setAttribute("r", (14 * mpp).toFixed(3));
    c.setAttribute("class", "ev-flash");
    c.style.strokeWidth = (3 * mpp).toFixed(3);
    $("eventLayer").appendChild(c);
    setTimeout(() => { if (c.parentNode) c.parentNode.removeChild(c); }, EV_FLASH_MS);
  }

  function wireEventDrawer() {
    const box = $("eventDrawer"); if (!box) return;
    $("eventBar").addEventListener("click", () => setEventDrawer(!evExpanded));
    $("eventChips").addEventListener("click", (e) => {
      const chip = e.target.closest(".ev-chip"); if (!chip) return;
      evFilter = chip.dataset.filter; armEventIdle();
      $("eventList").scrollTop = 0; drawEventDrawer(true);
    });
    box.addEventListener("click", (e) => {
      const row = e.target.closest(".ev-row"); if (!row) return;
      flashEventPos(evRowRefs[Number(row.dataset.ev)]);
      armEventIdle();                             // a click is activity too
    });
    // Same guards as the task panel: the wheel scrolls the list, not the map,
    // and a press here never starts a map pan.
    box.addEventListener("wheel", (e) => {
      const list = e.target.closest(".ev-scroll");
      if (list) list.scrollTop += e.deltaY;
      e.preventDefault(); e.stopPropagation();
    }, { passive: false });
    box.addEventListener("mousedown", (e) => e.stopPropagation());
    document.addEventListener("mousemove", () => { if (evExpanded) armEventIdle(); });
    wireEventDrawerResize(box);
  }

  // Left / right edge drag (user 2026-11-12, like the history panel's edges).
  // The drawer stays centred, so the width grows by twice the drag and the
  // dragged edge follows the cursor. The default width is the minimum (the
  // history panel's rule); the width is remembered in localStorage.
  const EV_WIDTH_KEY = "xbrain_hmi_event_width";
  function wireEventDrawerResize(box) {
    try {
      const w = parseInt(localStorage.getItem(EV_WIDTH_KEY) || "0", 10);
      if (w >= EV_MIN_W) box.style.width = w + "px";
    } catch (e) { /* absent / blocked -> CSS default */ }
    for (const side of ["left", "right"]) {
      const h = document.createElement("div");
      h.className = "rz-handle rz-" + side;
      h.addEventListener("mousedown", (e) => {
        e.preventDefault(); e.stopPropagation();       // no text select, no map pan
        const startX = e.clientX, startW = box.offsetWidth;
        const maxW = (box.parentElement || document.body).clientWidth - 24;
        const onMove = (ev) => {
          box.style.width = evDrawerWidth(startW, startX, ev.clientX, side, maxW) + "px";
        };
        const onUp = () => {
          document.removeEventListener("mousemove", onMove);
          document.removeEventListener("mouseup", onUp);
          document.body.style.userSelect = "";
          try { localStorage.setItem(EV_WIDTH_KEY, String(box.offsetWidth)); }
          catch (err) { /* not remembered, no throw */ }
        };
        document.body.style.userSelect = "none";
        document.addEventListener("mousemove", onMove);
        document.addEventListener("mouseup", onUp);
      });
      box.appendChild(h);
    }
  }

  // -- task panel (17 S6.8.4): current + history, five fields each ----------
  // Data comes from GET /api/tasks (P5 -> P3 query/tasks queryable), NOT the
  // snapshot -- current and history are the same TaskCard shape, one from each
  // scope. Field 4 (巡逻点/targets) is empty until the keypoint layer (F06) is
  // built, so those rows do not render yet; the other four fields are live.

  // state -> {中文徽标, css 类}. 12-value closed set (11 S4.4); an unknown value
  // still shows (never blank) but that means the closed set drifted.
  function taskBadge(state) {
    const M = {
      running: ["执行中", "running"],
      pending: ["待执行", "pending"], ready: ["待执行", "pending"],
      scheduled: ["待执行", "pending"], blocked: ["待执行", "pending"],
      suspended: ["已挂起", "suspended"], needs_review: ["待处理", "suspended"],
      wait_for_power_off: ["待关机", "suspended"],
      done: ["已完成", "done"], failed: ["失败", "failed"],
      cancelled: ["已取消", "cancelled"], interrupted: ["已中断", "failed"],
    };
    const m = M[state] || [state || "--", "pending"];
    return { label: m[0], cls: m[1] };
  }

  // 字段2 下发时间: created_at is UTC ISO ('...Z'); render it in the GPS-derived
  // siteTz (17 S6.10.2 v1.3) as YY-MM-DD HH:MM:SS. null/unparseable -> '--'
  // (a system-minted task has no created_at; never fabricate a time).
  function fmtDispatchTime(iso) {
    if (!iso) return "--";
    const d = new Date(iso);
    if (isNaN(d.getTime())) return "--";
    const opts = { year: "2-digit", month: "2-digit", day: "2-digit",
                   hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false };
    const o = siteTz ? Object.assign({ timeZone: siteTz }, opts) : opts;
    const g = {};
    try { for (const p of new Intl.DateTimeFormat("zh-CN", o).formatToParts(d)) g[p.type] = p.value; }
    catch (e) { return "--"; }
    return `${g.year}-${g.month}-${g.day} ${g.hour}:${g.minute}:${g.second}`;
  }

  // W7 controls: every current card, and in history only a SUSPENDED card (99
  // U156 files suspended tasks under history; it can still be resumed or
  // cancelled). Any other history card's task has ended, so an action on it
  // would come back E_TASK_STATE.
  function taskActionsHTML(card) {
    const acts = taskActions(card.state);
    if (!acts.length || !card.task_id) return "";
    return `<div class="task-actions">` + acts.map((a) =>
      `<button type="button" class="task-act task-act-${a}" ` +
      `data-task-action="${a}" data-task-id="${esc(card.task_id)}">` +
      `${ACTION_LABEL[a]}</button>`).join("") + `</div>`;
  }

  // The state FLOW of a task as three steps (user 2026-11-10: the panel must
  // show the transitions of an HMI-tapped task as they happen): 已下发 ->
  // 执行中 -> how it ended. The step the task is on is lit; finished steps are
  // ticked. Mapped from the 12-value closed set (11 S4.4) -- every value lands
  // somewhere, an unknown one shows raw on the first step rather than vanishing.
  // Asserted by tests/p5_gateway/hmi/test_task_flow_js.py (block cut out by the
  // markers and run under node), so it must stay self-contained.
  // <task-flow>
  const FLOW_BAD_END = { failed: "失败", interrupted: "已中断",
                         needs_review: "待处理", wait_for_power_off: "待关机" };
  function taskFlow(state) {
    const queued = ["pending", "ready", "scheduled", "blocked"];
    if (queued.indexOf(state) >= 0) return { at: 0, end: "完成", endCls: "" };
    if (state === "running") return { at: 1, end: "完成", endCls: "" };
    if (state === "suspended") return { at: 1, mid: "已暂停", end: "完成", endCls: "" };
    if (state === "done") return { at: 2, end: "已完成", endCls: "ok" };
    if (state === "cancelled") return { at: 2, end: "已取消", endCls: "muted" };
    if (FLOW_BAD_END[state]) return { at: 2, end: FLOW_BAD_END[state], endCls: "bad" };
    return { at: 0, first: state || "--", end: "完成", endCls: "" };
  }
  // </task-flow>
  function taskFlowHTML(state) {
    const f = taskFlow(state);
    const labels = [f.first || "已下发", f.mid || "执行中", f.end];
    const parts = labels.map((txt, i) => {
      let cls = "flow-step";
      if (i < f.at) cls += " passed";
      if (i === f.at) cls += " current" + (i === 2 && f.endCls ? " " + f.endCls : "");
      return `<span class="${cls}">${esc(txt)}</span>`;
    });
    return `<div class="task-flow">` + parts.join(`<i class="flow-arrow"></i>`) + `</div>`;
  }

  function taskCardHTML(card, isCurrent) {
    const b = taskBadge(card.state);
    // 字段3 内容 = command_text (语音/文本原文); 系统自发任务(返航/充电)无原文 ->
    // 用类型名兜底, 不留空白.
    const content = card.command_text || (card.task_type ? "(" + card.task_type + ")" : "--");
    const pct = card.progress ? card.progress.percent : null;
    // 字段5: percent 为 null(路由未展开)时显 '--', NO 不伪造 0/100 (同后端).
    const pctText = (pct == null) ? "--" : Math.round(pct) + "%";
    const pctW = (pct == null) ? 0 : Math.max(0, Math.min(100, pct));
    // 字段4 巡逻点: 走过=finished(绿点绿字) / 当前=current / 未到=pending(灰).
    // targets 现恒空(待关键点层 F06), 空则整段不渲染.
    const targets = (card.targets || []).map((t) => {
      const cls = t.status === "finished" ? "finished"
        : (t.status === "current" ? "current" : "pending");
      return `<li class="${cls}"><i></i><span>${esc(t.name || t.waypoint_id || "")}</span></li>`;
    }).join("");
    // An HMI map-tap goto is tagged, so the operator finds the task they just
    // sent among the voice / cloud ones (P5 writes "HMI 点击前往 ..." as its
    // command_text; P3 maps the hmi channel to source "local", so the text is
    // the only marker the card has).
    const hmiTag = (card.command_text || "").indexOf("HMI ") === 0
      ? `<span class="task-tag-hmi">HMI</span>` : "";
    return (
      `<div class="task-card-head"><strong class="task-id">${esc(card.task_id || "")}</strong>` +
      hmiTag +
      `<span class="task-state ${b.cls}">${esc(b.label)}</span></div>` +
      `<div class="task-content">${esc(content)}</div>` +
      `<div class="task-time"><span>下发</span><time>${esc(fmtDispatchTime(card.created_at))}</time></div>` +
      taskFlowHTML(card.state) +
      (targets ? `<div class="task-targets-title">巡逻点</div><ol class="task-targets">${targets}</ol>` : "") +
      // Actions: every current card, plus a SUSPENDED card in history -- since
      // 99 U156 a suspended task is listed under history, and it is the one
      // history card a person still has to act on (继续 or 取消任务).
      ((isCurrent || card.state === "suspended") ? taskActionsHTML(card) : "") +
      `<div class="task-progress"><span>已巡逻 <strong>${pctText}</strong></span>` +
      `<div class="progress-bar"><div class="progress-fill" style="width:${pctW}%"></div></div></div>`
    );
  }

  // task_ids the operator has expanded in the history stack. Preserved across the
  // periodic refresh so a poll never collapses a card the operator is reading.
  const expandedHistory = new Set();

  function renderTaskList(elId, cards, isHistory) {
    const box = $(elId);
    if (!cards || !cards.length) {
      box.innerHTML = `<div class="task-empty">${isHistory ? "暂无历史任务" : "暂无当前任务"}</div>`;
      return;
    }
    box.innerHTML = "";
    for (const card of cards) {
      const art = document.createElement("article");
      art.className = "task-card";
      art.dataset.taskId = card.task_id || "";
      // The goto whose pink dot is on the map gets the same accent on its card.
      if (gotoTrack && card.task_id === gotoTrack.taskId) art.classList.add("goto-tracked");
      // History cards default collapsed (only id/content/time); a card the
      // operator opened stays expanded. Current cards are always full.
      // A suspended task waits for a person, so in history it is never folded
      // away ("awaiting"): its buttons stay in sight. The rest fold as before.
      if (isHistory && card.state === "suspended") art.classList.add("awaiting");
      else if (isHistory)
        art.classList.add(expandedHistory.has(card.task_id) ? "expanded" : "collapsed");
      art.innerHTML = taskCardHTML(card, !isHistory);
      wireTaskActions(art);                // no-op on a card without buttons
      box.appendChild(art);
    }
  }

  async function fetchTaskScope(scope, elId, isHistory) {
    // Each scope fetched INDEPENDENTLY: a failure (or slow reply) on one panel
    // must never blank or block the other -- one 404/500 should not swallow both
    // renders. A transient miss leaves that panel's last view up; next poll retries.
    try {
      const d = await getJSON(`/api/tasks?scope=${scope}&limit=${isHistory ? 100 : 50}`);
      renderTaskList(elId, d.tasks, isHistory);
    } catch (e) { /* transient; next refresh retries */ }
  }
  function refreshTasks() {
    // history limit is generous -- retention keeps terminal rows to 30 days
    // (15 S8) and the stack scrolls.
    fetchTaskScope("current", "currentTaskList", false);
    fetchTaskScope("history", "historyTaskList", true);
  }

  // History folding (iPhone-lock-screen style, user 2026-08-17): click a collapsed
  // card to expand it to the full five fields; it collapses again on a re-click OR
  // after 10 s with no mouse movement.
  let historyIdleTimer = null;
  function collapseAllHistory() {
    expandedHistory.clear();
    $("historyTaskList").querySelectorAll(".task-card.expanded").forEach((el) => {
      el.classList.remove("expanded"); el.classList.add("collapsed");
    });
  }
  function armHistoryIdle() {
    if (historyIdleTimer) clearTimeout(historyIdleTimer);
    // Only arm while something is open, so an idle panel never runs a timer.
    if (expandedHistory.size) historyIdleTimer = setTimeout(collapseAllHistory, 10000);
  }
  function wireHistoryFolding() {
    // Delegation on the persistent container, so it survives each re-render.
    $("historyTaskList").addEventListener("click", (e) => {
      // A button press (继续 / 取消任务 on a suspended card) is not a fold toggle,
      // and an awaiting card does not fold at all.
      if (e.target.closest("button")) return;
      const card = e.target.closest(".task-card"); if (!card) return;
      if (card.classList.contains("awaiting")) return;
      const id = card.dataset.taskId;
      if (card.classList.contains("expanded")) {          // re-click -> collapse
        card.classList.remove("expanded"); card.classList.add("collapsed");
        expandedHistory.delete(id);
      } else {                                             // expand to full
        card.classList.remove("collapsed"); card.classList.add("expanded");
        expandedHistory.add(id);
      }
      armHistoryIdle();
    });
    // Any mouse movement resets the idle countdown while a card is open.
    document.addEventListener("mousemove", () => { if (expandedHistory.size) armHistoryIdle(); });
  }

  // Mouse-wheel over the task panel scrolls the list UNDER THE CURSOR (当前 or
  // 历史), not the map. Needed because the map viewport's wheel=zoom handler is
  // an ancestor and preventDefaults every wheel -- so it eats the panel's native
  // scroll. Here we scroll the hovered list ourselves and stopPropagation so the
  // zoom handler never fires while the cursor is over the panel (user 2026-08-18).
  function wirePanelScroll() {
    const panel = document.querySelector(".task-panel");
    if (!panel) return;
    panel.addEventListener("wheel", (e) => {
      const list = e.target.closest(".task-list");   // current or history list
      if (list) list.scrollTop += e.deltaY;           // wheel down -> scroll down
      e.preventDefault();                             // never zoom the map here
      e.stopPropagation();                            // stop before the vp handler
    }, { passive: false });
    // Interacting with the panel (resize grip, scrollbar, card) must NOT start a
    // map pan -- the viewport's mousedown-drag handler is an ancestor. Stop the
    // mousedown here; clicks (folding) still fire, only the pan is suppressed.
    panel.addEventListener("mousedown", (e) => e.stopPropagation());
  }

  // Each sub-panel resizes from its LEFT edge / LEFT-BOTTOM corner / BOTTOM edge:
  // the panel is pinned to the HMI's right edge, so it grows/shrinks LEFTWARD like
  // a window (user 2026-08-18). The DEFAULT size is the MINIMUM -- it can be
  // widened/heightened and shrunk back, but never below the default. The dragged
  // size is saved to localStorage and restored on load (survives refresh + restart).
  function wirePanelResize() {
    // * WIDTH is SHARED by both panels (they stack on the HMI's right edge, so a
    // mismatched width misaligns them, user 2026-08-18): resizing width on EITHER
    // panel syncs BOTH. HEIGHT is per-panel. Both persist and restore on load.
    const MINW = 350;                                 // = default width (= minimum)
    const cur = document.querySelector(".current-group");
    const his = document.querySelector(".history-group");
    const both = [cur, his].filter(Boolean);
    if (!both.length) return;
    // Restore the shared width onto BOTH panels.
    try {
      const w = parseInt(localStorage.getItem("xbrain_hmi_task_width") || "0", 10);
      if (w >= MINW) for (const g of both) g.style.width = w + "px";
    } catch (e) { /* absent/corrupt -> CSS default */ }
    // Per-panel: restore height + attach the three drag handles.
    const PANELS = [[cur, "xbrain_hmi_task_h_current", 180],
                    [his, "xbrain_hmi_task_h_history", 240]];
    for (const [g, hKey, minH] of PANELS) {
      if (!g) continue;
      try {
        const h = parseInt(localStorage.getItem(hKey) || "0", 10);
        if (h >= minH) g.style.height = h + "px";
      } catch (e) { /* CSS default */ }
      for (const hcls of ["rz-left", "rz-bottom", "rz-corner"]) {
        const handle = document.createElement("div");
        handle.className = "rz-handle " + hcls;
        handle.addEventListener("mousedown",
          (e) => startPanelResize(e, g, hcls, both, MINW, minH, hKey));
        g.appendChild(handle);
      }
    }
  }

  function startPanelResize(e, el, hcls, both, minW, minH, hKey) {
    e.preventDefault(); e.stopPropagation();          // no text-select, no map pan
    const startX = e.clientX, startY = e.clientY;
    const startW = el.offsetWidth, startH = el.offsetHeight;
    const doW = hcls === "rz-left" || hcls === "rz-corner";
    const doH = hcls === "rz-bottom" || hcls === "rz-corner";
    const maxW = window.innerWidth * 0.78, maxH = window.innerHeight * 0.8;
    function onMove(ev) {
      // WIDTH shared -> apply to BOTH panels so they stay aligned. Right edge
      // pinned: dragging the LEFT edge leftward widens; min = default (no narrower).
      if (doW) {
        const w = Math.max(minW, Math.min(maxW, startW + (startX - ev.clientX)));
        for (const g of both) g.style.width = w + "px";
      }
      // HEIGHT per-panel: only the dragged one. Drag DOWN heightens.
      if (doH) el.style.height =
        Math.max(minH, Math.min(maxH, startH + (ev.clientY - startY))) + "px";
    }
    function onUp() {
      document.removeEventListener("mousemove", onMove);
      document.removeEventListener("mouseup", onUp);
      document.body.style.userSelect = "";
      try {
        if (doW) localStorage.setItem("xbrain_hmi_task_width", String(el.offsetWidth));
        if (doH) localStorage.setItem(hKey, String(el.offsetHeight));
      } catch (e) { /* storage blocked -> not remembered, no throw */ }
    }
    document.body.style.userSelect = "none";          // no selection while dragging
    document.addEventListener("mousemove", onMove);
    document.addEventListener("mouseup", onUp);
  }

  // RTK/heading status text. fix_type comes from rt/gnss/fix (not published yet),
  // so until then surface the heading status that IS flowing: 双天线(L1)/航迹(L2)/
  // 无(L3). Never fabricate -- an unavailable pose reads "无定位/航向".
  // -- footer status labels (user 2026-08-16): uppercase EN convergence tokens,
  //    coloured green(ok)/red(bad), no dots. Each returns {text, cls}.
  // fix convergence: SINGLE/DGPS/FLOAT/FIXED = ok(green); no fix / LOSS = bad(red).
  function fixStat(pose) {
    const FIX = { rtk_fixed: "FIXED", rtk_float: "FLOAT", dgps: "DGPS",
                  single: "SINGLE", no_fix: "LOSS" };
    if (!pose.available || !pose.fix_type || pose.fix_type === "no_fix")
      return { text: "定位 LOSS", cls: "bad" };
    return { text: "定位 " + (FIX[pose.fix_type] || pose.fix_type), cls: "ok" };
  }
  // heading convergence -- FOUR states, in the resolver's degradation order
  // (11 S3.3 L1->L2->L3):
  //     L1 双天线INT   dual-antenna, baseline FIXED (integer)   green
  //     L1 双天线FLOAT dual-antenna, baseline FLOAT             green
  //     L2 航迹COG     dual-antenna lost, course-over-ground    green
  //     L3 LOSS        ALL of the above unavailable             red
  // ***CRITICAL semantics (user 2026-08-16): LOSS = NO USABLE HEADING from ANY
  // source -- shown iff dual-antenna (INT/FLOAT) AND COG are ALL invalid.
  // Losing the dual-antenna fix does NOT go straight to LOSS: while the robot
  // moves, heading falls back to COG (still valid, still green). heading_valid
  // (11 H-1) is THE criterion -- true whenever ANY of INT/FLOAT/COG yields a
  // value, false only when none does: L3 (no source admitted) OR the transient
  // L2-blind (in COG mode but too slow to resolve a course, resolver sets
  // heading_valid=false). So the single `!heading_valid` test below is exactly
  // "all sources exhausted", never "dual-antenna alone lost". Do NOT gate LOSS
  // on source/level/baseline_valid -- a green painted while heading_valid=false
  // would be a fabricated heading (3.1/3.2).
  function hdgStat(pose) {
    // L3: no heading at all (no dual-antenna AND no COG). Only here is it LOSS.
    if (!pose.available || !pose.heading_valid) return { text: "航向 LOSS", cls: "bad" };
    // L1: dual-antenna. INT when the baseline is integer-fixed, else FLOAT.
    if (pose.heading_source === "dual_antenna")
      return { text: "航向 双天线" + (pose.baseline_valid ? "INT" : "FLOAT"), cls: "ok" };
    // L2: course-over-ground fallback (dual-antenna gone, robot moving).
    if (pose.heading_source === "cog") return { text: "航向 航迹COG", cls: "ok" };
    // valid=true with a source outside the closed set {dual_antenna,cog} is a
    // producer contract break (11 S3.3 source<->level is 1:1) -- surface it as
    // LOSS rather than paint a fake green, but this branch should never fire.
    return { text: "航向 LOSS", cls: "bad" };
  }
  function setStat(id, s) {
    const el = $(id); if (el) { el.textContent = s.text; el.className = "stat " + s.cls; }
  }

  function renderStatus(status, pose, origin, clock) {
    $("modeText").textContent = status.mode ? `模式: ${status.mode}` : "模式: --";
    // pose-derived readouts: null until perception/rtk exist (17 S6.10.4).
    $("coordGps").textContent = pose.available && pose.lat != null
      ? `${pose.lat.toFixed(6)}°N   ${pose.lon.toFixed(6)}°E` : "无定位";
    // ENU line = customer block: E / N (lat/lon vs origin) + 航向 + 速度. toXY
    // negates N for SVG screen coords, so flip it back for the northward reading.
    // Each field falls back to "--" until pose exists (W4 GATED), never a fake 0.
    let e = "--", n = "--";
    if (pose.available && pose.lat != null && origin) {
      const xy = toXY({ lat: pose.lat, lon: pose.lon }, origin);
      if (xy) { e = xy[0].toFixed(1); n = (-xy[1]).toFixed(1); }
    }
    const hdg = (pose.available && pose.heading_valid && pose.heading_rad != null)
      ? (pose.heading_rad * 180 / Math.PI).toFixed(1) : "--";
    const spd = (pose.available && pose.speed_mps != null)
      ? pose.speed_mps.toFixed(1) : "--";
    $("coordEnu").textContent =
      `E ${e}m   N ${n}m　航向 ${hdg}°　速度 ${spd}m/s`;
    // Footer status (2026-08-16): fix / heading / sync as coloured text, no dots.
    setStat("fixText", fixStat(pose));
    setStat("hdgText", hdgStat(pose));
    // sync (18-C G47): 授时同步 green / 授时未同步 red. Absent source -> unsynced.
    const synced = !!(clock && clock.available && clock.sync);
    setStat("syncText", synced ? { text: "授时同步", cls: "ok" }
                                : { text: "授时未同步", cls: "bad" });
    renderClock();
    $("precText").textContent = pose.cov_h_m != null ? `${pose.cov_h_m.toFixed(2)}m` : "--";
    // ESTOP button + footer link state: the server's estop_path is only half of
    // the decision -- the other half is whether anything is arriving at all,
    // which the server cannot report (see estopView). Both go through
    // updateEstopUi so the snapshot path and the watchdog tick draw the same.
    lastEstopPath = status.estop_path;
    updateEstopUi();
  }

  // -- ESTOP availability (17 S6.3 / 11 S12.3 / NAV-64) ---------------------
  //
  // *** Why the frontend decides "dead" itself: P5 that has died cannot send
  // "I died". Before this, the button followed the last estop_path it had
  // seen; when P5 went away the WS closed, the REST fallback poll failed
  // silently, and the button kept the last "ok" -- armed and pulsing with
  // nothing behind it. 17 S6.3 calls this the most important row of its table.
  //
  // *** Any downlink frame counts as the heartbeat, not only a `link` message:
  // this build sends no `link` frame (state_snapshot / state_delta / ack only),
  // and the WS loop sends an empty state_delta every tick precisely so a quiet
  // but live link is never mistaken for a dead one. A REST snapshot that
  // arrives (the fallback when the socket is down) counts too: it proves P5
  // is answering and carries a fresh estop_path.
  //
  // estopView is a pure function so its four outcomes are asserted by
  // tests/p5_gateway/hmi/test_estop_view_js.py, which cuts the block between
  // the markers below out of this file and runs it under node.
  // <estop-view>
  // 11 S12.3 "client 判链路死: 2 s 无下行" -- a contract constant, not a tuning
  // knob, and the frontend has no config path for it.
  const LINK_DEAD_MS = 2000;
  function estopView(path, downlinkAgeMs) {
    // No downlink yet, or none for 2 s: greyed, whatever estop_path last said.
    if (downlinkAgeMs == null || !(downlinkAgeMs <= LINK_DEAD_MS)) {
      return { enabled: false, armed: false, btnCls: "",
               notice: "与机器人链路中断 -- 急停不可用, 请使用硬件急停", noticeCls: "bad",
               link: { text: "链路中断", cls: "bad" } };
    }
    if (path === "ok") {
      return { enabled: true, armed: true, btnCls: "",
               notice: "", noticeCls: "",
               link: { text: "急停链路正常", cls: "ok" } };
    }
    // degraded = pongs still come back, only slowly: the stop still gets
    // through, so the button STAYS PRESSABLE with a warning (17 S6.3 table).
    // Greying it here would take the button away exactly while it works.
    if (path === "degraded") {
      return { enabled: true, armed: false, btnCls: "degraded",
               notice: "链路延迟高", noticeCls: "warn",
               link: { text: "急停链路延迟高", cls: "warn" } };
    }
    // down, or anything outside the closed set (null before the first probe
    // result): greyed and pointing at the hardware stop (NAV-64 / 11 S12.3
    // "HMI 必须把这句话显示出来, 而不是只置灰").
    return { enabled: false, armed: false, btnCls: "",
             notice: "急停不可用, 请使用硬件急停", noticeCls: "bad",
             link: { text: "急停链路断", cls: "bad" } };
  }
  // </estop-view>

  // Mono time (performance.now) of the last downlink that was handled. Not
  // Date: a wall-clock step would fake or hide a 2 s gap (11 CLK-C1).
  let lastDownlinkMs = null;
  let lastEstopPath = null;
  // A press whose POST failed. Shown for ESTOP_FAIL_SHOW_MS over whatever the
  // link state says, because "the stop did not go out" is the one thing the
  // operator must hear first.
  let estopSendFailedMs = null;
  const ESTOP_FAIL_SHOW_MS = 5000;

  function markDownlink() { lastDownlinkMs = performance.now(); }

  function updateEstopUi() {
    const now = performance.now();
    const age = lastDownlinkMs == null ? null : now - lastDownlinkMs;
    const v = estopView(lastEstopPath, age);
    const btn = $("estopBtn");
    btn.disabled = !v.enabled;
    btn.classList.toggle("armed", v.armed);
    btn.classList.toggle("degraded", v.btnCls === "degraded");
    btn.title = v.enabled ? "紧急停止" : v.notice;
    // No text beside the button any more (user 2026-11-11): it blinked and
    // pushed the chassis-lock button and battery gauges sideways. Every
    // sentence -- link lost, estop path down / slow, a press that did not go
    // out -- now goes to the one amber banner (linkBannerView).
    const sendFailed = estopSendFailedMs != null &&
                       now - estopSendFailedMs < ESTOP_FAIL_SHOW_MS;
    setStat("linkText", v.link);
    updateLinkUi(age, v, sendFailed);
  }

  // -- link-loss banner + control lock (11 S12.3 reconnect flow, step 1) ------
  //
  // 11 S12.3: when the link drops, keep the last frame but grey every control
  // and put "link lost" across the top -- never keep rendering as if all were
  // well, or the operator believes they are still in control. Two degrees:
  //   * socket down, REST snapshots still arriving: the picture is live but
  //     commands (task / goto) have no path -> locked;
  //   * nothing arriving for 2 s (LINK_DEAD_MS, same judgement as the ESTOP
  //     view): the picture itself is frozen -> say so.
  // The ESTOP button is NOT governed here: estopView owns it, and it stays
  // pressable over the REST fallback while REST still answers.
  // Driven from updateEstopUi, so it ticks every 250 ms with no data arriving.
  function controlsOffline() {
    const wsUp = !!(liveWS && liveWS.readyState === 1);
    const age = lastDownlinkMs == null ? null : performance.now() - lastDownlinkMs;
    return !wsUp || !(age != null && age <= LINK_DEAD_MS);
  }
  // The single amber banner: ONE sentence, chosen by priority, covering both
  // the link and the ESTOP path (user 2026-11-11 -- there used to be a second,
  // blinking line beside the ESTOP button saying part of the same thing).
  // Pure, so its outcomes are asserted under node (test_link_banner_view_js.py).
  // Priority, highest first: a stop that did not go out; never connected;
  // nothing for 2 s (picture frozen); socket down while REST still answers;
  // ESTOP path not confirmed / down; ESTOP path slow. Every case in which the
  // web ESTOP cannot work says "请使用硬件急停" (NAV-64 / 11 S12.3: the
  // sentence must be SHOWN, not only implied by a grey button).
  // <link-banner-view>
  function linkBannerView(s) {
    const hw = "请使用硬件急停";
    if (s.sendFailed) return { show: true, text: "急停指令未发出 -- 请立即" + hw };
    if (!s.everConnected) {
      return { show: true, text: "正在连接机器人 ... 网页急停暂不可用, " + hw };
    }
    if (!s.fresh) {
      return { show: true, text: "与机器人链路中断 -- 画面停在最后一帧, " +
               "网页急停与操作按钮均不可用, " + hw };
    }
    if (!s.wsUp) {
      return { show: true, text: "实时链路已断开, 正在重连 -- 任务/导航按钮暂停使用, " +
               (s.estopEnabled ? "网页急停仍可用" : "网页急停不可用, " + hw) };
    }
    if (s.estopPath == null) {
      return { show: true, text: "正在确认急停链路 -- 网页急停暂不可用, " + hw };
    }
    if (s.estopPath === "degraded") {
      return { show: true, text: "急停链路延迟高 -- 网页急停仍可用, 但响应可能变慢" };
    }
    if (s.estopPath !== "ok") {
      return { show: true, text: "急停链路不通 -- 网页急停不可用, " + hw };
    }
    return { show: false, text: "" };
  }
  // </link-banner-view>
  function updateLinkUi(age, ev, sendFailed) {
    const wsUp = !!(liveWS && liveWS.readyState === 1);
    const fresh = age != null && age <= LINK_DEAD_MS;
    const offline = !wsUp || !fresh;
    document.body.classList.toggle("controls-offline", offline);
    const b = $("linkBanner");
    if (!b) return;
    const v = linkBannerView({ sendFailed: !!sendFailed, everConnected: lastDownlinkMs != null,
                               fresh: fresh, wsUp: wsUp, estopPath: lastEstopPath,
                               estopEnabled: !!(ev && ev.enabled) });
    b.textContent = v.text;
    b.hidden = !v.show;
  }
  function setDot(el, cls) { el.className = "dot" + (cls ? " " + cls : ""); }
  function esc(s) { const d = document.createElement("div"); d.textContent = s; return d.innerHTML; }

  // -- footer clock: local time in the GPS-derived zone (17 S6.10.2 v1.3) ------
  // The displayed time comes from the browser wall clock formatted to siteTz,
  // which follows the robot's position (snapshot.timezone), falling back to
  // common.timezone with no fix. It is a convenience readout, NOT an
  // authoritative timestamp: the
  // AUTHORITATIVE answer is the voice G24 reply, which reads the robot's own
  // (NTP-synced) clock. Whether the shown time is trustworthy is conveyed by the
  // separate coloured syncText span (授时同步 green / 授时未同步 red, 18 S9.5).
  function fmtClock() {
    const opts = { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false };
    // A bad tz name would throw in the Intl ctor; fall back to the browser zone
    // so the clock still ticks (a DISPLAY value never breaks the page).
    if (siteTz) { try { new Intl.DateTimeFormat("zh-CN", Object.assign({ timeZone: siteTz }, opts)); }
                  catch (e) { siteTz = null; } }
    const o = siteTz ? Object.assign({ timeZone: siteTz }, opts) : opts;
    return new Intl.DateTimeFormat("zh-CN", o).format(new Date());
  }
  function renderClock() {
    // Only ticks the local time now; the sync state moved to the coloured
    // syncText span (授时同步/未同步), no clock dot (2026-08-16 footer redo).
    const el = $("clockText"); if (el) el.textContent = fmtClock();
  }
  function startClock() { renderClock(); if (!clockTimer) clockTimer = setInterval(renderClock, 1000); }

  // -- heading dial (机器人本体航向, 17 S6.10 item 6) ----------------------- */
  // A rotating compass CARD: the card turns so the current heading sits at the
  // top under the fixed yellow pointer (user 2026-08-14: rotate the dial, not the
  // pointer). Driven ONLY by pose.heading_rad -- the ROBOT BODY heading, a real
  // state/pose field already shown as text in .coord. It is NOT a gimbal angle:
  // the 17 S6.6/PTZ-C2 ban on a "罗盘/朝向指针" is gimbal-specific (PTZ readback
  // is a fake constant (180,0)); body heading is real, so this dial is in bounds.
  function buildDialFace() {
    const face = $("dialFace");
    if (!face || face.childNodes.length) return;   // static content, build once
    const NS = SVGNS, rad = (a) => a * Math.PI / 180;
    const at = (r, a) => [(r * Math.sin(rad(a))).toFixed(2), (-r * Math.cos(rad(a))).toFixed(2)];
    const ring = document.createElementNS(NS, "circle");
    ring.setAttribute("r", 90); ring.setAttribute("class", "dial-ring");
    face.appendChild(ring);
    for (let a = 0; a < 360; a += 10) {             // ticks: major every 30, minor every 10
      const major = a % 30 === 0, p1 = at(90, a), p2 = at(major ? 76 : 83, a);
      const ln = document.createElementNS(NS, "line");
      ln.setAttribute("x1", p1[0]); ln.setAttribute("y1", p1[1]);
      ln.setAttribute("x2", p2[0]); ln.setAttribute("y2", p2[1]);
      ln.setAttribute("class", major ? "dial-tick major" : "dial-tick");
      face.appendChild(ln);
    }
    const CARD = { 0: "N", 90: "E", 180: "S", 270: "W" };
    for (let a = 0; a < 360; a += 30) {             // labels rotated TANGENTIALLY (斜排):
      const [x, y] = at(61, a);                      // each reads upright only when at the top,
      const t = document.createElementNS(NS, "text");  // so the whole-card rotation reads right.
      t.setAttribute("x", x); t.setAttribute("y", y);
      t.setAttribute("transform", `rotate(${a} ${x} ${y})`);
      t.setAttribute("class", CARD[a] !== undefined ? "dial-label cardinal" : "dial-label");
      t.textContent = CARD[a] !== undefined ? CARD[a] : String(a / 10);   // number = heading/10
      face.appendChild(t);
    }
    for (const a of [0, 90, 180, 270]) {            // yellow cardinal index triangles (on the card)
      const tri = document.createElementNS(NS, "polygon");
      tri.setAttribute("points", "0,-74 -7,-90 7,-90");
      tri.setAttribute("class", "dial-cardinal-tri");
      tri.setAttribute("transform", `rotate(${a})`);
      face.appendChild(tri);
    }
  }
  // Three states (user 2026-08-14). Source is RTK dual-antenna heading, a physical
  // true value: heading_valid is true when EITHER the FIXED dual-antenna RTK
  // heading OR the COG fallback is available -- folded upstream. Per 11 H-1 the
  // HMI reads heading_valid ONLY, never deriving validity from source/level.
  //   1. startup / never had a fix   -> point North (dial at 0).
  //   2. heading valid               -> rotate the CARD in real time (not pointer).
  //   3. BOTH headings lost after a fix -> FREEZE at the last valid tick (stop
  //      rotating, hold), NOT reset to North.
  function renderHeading(pose) {
    const face = $("dialFace"), dial = $("headingDial");
    if (!face || !dial) return;
    if (pose.available && pose.heading_valid && pose.heading_rad != null) {
      // ENU (east=0, CCW+) -> compass (north=0, CW+): compass = (90 - enu) mod 360.
      const enu = pose.heading_rad * 180 / Math.PI;
      const compass = ((90 - enu) % 360 + 360) % 360;
      // Unwrap: accumulate the SHORTEST signed step into a continuous angle, so
      // the CSS transition always animates the short way -- a jitter across the
      // 0/360 seam (compass ~0) no longer triggers a full-circle spin.
      if (dialAngle === null) {
        dialAngle = compass;
      } else {
        const step = ((compass - lastCompass + 540) % 360) - 180;   // (-180, 180]
        dialAngle += step;
      }
      lastCompass = compass;
      lastHeadingCompass = compass;                       // remember for freeze-on-loss
      face.style.transform = `rotate(${-dialAngle}deg)`;  // CARD rotates -> heading to top
      dial.classList.remove("no-heading");
    } else if (lastHeadingCompass !== null) {
      // Both sources lost -> FREEZE at the last valid heading: leave the
      // continuous-angle transform exactly where it is (re-asserting a wrapped
      // value here would itself cause a jump), just keep the dial visible.
      dial.classList.remove("no-heading");
    } else {
      // Startup default: no heading ever -> point North. Slightly dimmed
      // (.no-heading) to signal "no fix yet", never a fabricated heading (3.2).
      face.style.transform = "rotate(0deg)";
      dial.classList.add("no-heading");
    }
  }

  // -- zoom / pan (U4 wheel) ------------------------------------------------ */
  function applyView() {
    // Pan/zoom by rewriting the SVG viewBox (NOT a CSS transform on the element).
    // The element always fills the viewport, so the huge grid rect always covers
    // it -> the井字格 is infinite. zoom scales the window around its centre; pan
    // shifts the window in ENU metres.
    //
    // *** UNIFORM scale, always (user 2026-11-10: the target dot was an ellipse).
    // The svg has preserveAspectRatio="none", so a viewBox whose aspect differs
    // from the element's stretches x and y by different factors -- every circle
    // becomes an ellipse and every route shape is squashed. Only fitGeoToData
    // matched the aspect, and it runs only when fences exist; the default
    // 240 x 145 m window on a wide screen was distorted. So the window HEIGHT is
    // derived from its width and the element's own aspect on every view change
    // (resize included); the base height is used only before layout.
    const [bx, by, bw, bh] = baseVB;
    const svg = $("mapSvg");
    const ew = svg.clientWidth, eh = svg.clientHeight;
    const w = bw / view.zoom;
    const h = (ew > 0 && eh > 0) ? w * eh / ew : bh / view.zoom;
    const cx = bx + bw / 2 + view.pan.x, cy = by + bh / 2 + view.pan.y;
    svg.setAttribute("viewBox", `${cx - w / 2} ${cy - h / 2} ${w} ${h}`);
    // req4 (2026-08-18): map labels match the TASK-CONTENT font size, never
    // scaling with zoom. Task content uses the plan-panel font = config
    // font.plan.size_px (applyUiConfig writes it to --font-plan-size, e.g. 24px),
    // NOT the CSS 12px default -- targeting 12 made map labels look half-size.
    // An SVG font-size is in USER UNITS (ENU m), mapped to screen by clientHeight
    // / viewBoxHeight; so planPx screen == planPx * h / clientHeight user units.
    // Recomputed on every view change (zoom / pan / resize) so labels stay a
    // constant screen size. Guard the pre-layout call (ch==0 -> leave the CSS
    // fallback until the next applyView measures a real height).
    const ch = svg.clientHeight || svg.getBoundingClientRect().height;
    if (ch > 0) {
      const planPx = parseFloat(getComputedStyle(document.documentElement)
        .getPropertyValue("--font-plan-size")) || 12;
      document.documentElement.style.setProperty(
        "--map-label-fs", (planPx * h / ch).toFixed(3) + "px");
    }
    $("zoomText").textContent = `${view.zoom.toFixed(1)}x`;
    updateGridAndDots();                  // adaptive grid cell + non-scaling dots
  }
  function clampZoom(z) { return Math.max(zoomBounds.min, Math.min(zoomBounds.max, z)); }
  //: One button press zooms by this factor (the wheel uses 1 + map.zoom.wheel_step).
  const ZOOM_BUTTON_FACTOR = 1.25;
  function wireInteraction() {
    // Zoom steps are MULTIPLICATIVE (user 2026-11-10: the 1 m grid was out of
    // reach). With an additive +0.2 / +0.12 step, the range now needed to reach
    // a 1 m grid (zoom max raised in the yaml) took ~100 wheel notches; a factor
    // per step covers it in a few dozen and feels the same at every zoom level.
    $("zoomIn").onclick = () => { view.zoom = clampZoom(view.zoom * ZOOM_BUTTON_FACTOR); applyView(); };
    $("zoomOut").onclick = () => { view.zoom = clampZoom(view.zoom / ZOOM_BUTTON_FACTOR); applyView(); };
    $("zoomReset").onclick = () => { view.zoom = 1; view.pan = { x: 0, y: 0 }; applyView(); };
    const vp = $("viewport");
    vp.addEventListener("wheel", (e) => {          // U4: wheel up=in, down=out
      e.preventDefault();
      const f = 1 + zoomBounds.wheel;               // wheel_step is now a ratio
      view.zoom = clampZoom(e.deltaY < 0 ? view.zoom * f : view.zoom / f);
      applyView();
    }, { passive: false });
    let drag = null;
    vp.addEventListener("mousedown", (e) => {
      drag = { x: e.clientX, y: e.clientY, px: view.pan.x, py: view.pan.y };
      vp.classList.add("dragging");
    });
    window.addEventListener("mousemove", (e) => {
      if (!drag) return;
      // pixel delta -> ENU metre delta via the current viewBox scale (uniform
      // since applyView matches the element aspect). Drag right -> the window
      // moves left (grab-pan).
      const r = vp.getBoundingClientRect();
      const vbNow = ($("mapSvg").getAttribute("viewBox") || "").split(/\s+/).map(Number);
      const mppx = (vbNow[2] || baseVB[2] / view.zoom) / r.width;
      const mppy = (vbNow[3] || baseVB[3] / view.zoom) / r.height;
      view.pan.x = drag.px - (e.clientX - drag.x) * mppx;
      view.pan.y = drag.py - (e.clientY - drag.y) * mppy;
      if (Math.hypot(e.clientX - drag.x, e.clientY - drag.y) > DRAG_TAP_PX) drag.moved = true;
      applyView();
    });
    window.addEventListener("mouseup", () => {
      // A pan ends in a click event too; remember it so onMapClick does not
      // treat the end of a drag as a goto pick.
      lastPanMoved = !!(drag && drag.moved);
      drag = null; vp.classList.remove("dragging");
    });
    // ESTOP: W1 (17 S6.4). Enabled only while estopView allows it (NAV-64).
    //
    // *** Primary path: the WebSocket ({type:"estop"}), which P5 forwards at
    // gate G0 before any parsing or rate limit. Fallback: POST /api/estop, used
    // only while the socket is down -- the page keeps polling REST then, and a
    // working HTTP path to stop the robot beats none (17 S6.4 records this).
    //
    // A stop that was not confirmed is SAID, not swallowed: no ack within
    // ESTOP_ACK_WAIT_MS, a refusal, or a failed POST all show "estop not sent,
    // use the hardware stop". The probe only reports the path to the
    // quadruped, so a press that never left the browser would otherwise look
    // exactly like one that worked.
    $("estopBtn").onclick = async () => {
      if ($("estopBtn").disabled) return;
      if (liveWS && liveWS.readyState === 1) {
        reqSeq += 1;
        const reqId = "e" + Date.now().toString(36) + "-" + reqSeq;
        try {
          liveWS.send(JSON.stringify({ type: "estop", action: "stop", req_id: reqId }));
        } catch (_) { estopFailed(); return; }
        estopPending.set(reqId, setTimeout(() => {
          estopPending.delete(reqId);
          estopFailed();
        }, ESTOP_ACK_WAIT_MS));
        return;
      }
      try {
        const r = await fetch("/api/estop", { method: "POST" });
        if (!r.ok) throw new Error("estop " + r.status);
      } catch (_) {
        estopFailed();
      }
    };
  }

  // 11 S12.1.4: the client judges an ack lost after 3 s, deliberately longer
  // than P5's own 2 s so that "nothing came back" can only mean the link died.
  const ESTOP_ACK_WAIT_MS = 3000;
  const estopPending = new Map();        // req_id -> timeout handle
  function estopFailed() { estopSendFailedMs = performance.now(); updateEstopUi(); }
  function onEstopAck(m) {
    const t = estopPending.get(m.req_id);
    if (t === undefined) return;         // late, or not ours
    clearTimeout(t);
    estopPending.delete(m.req_id);
    // Anything but accepted means the publish did not happen (P5 answers
    // error/E_INTERNAL when it raised) -- tell the operator now.
    if (m.result !== "accepted") estopFailed();
  }

  async function getJSON(url) { const r = await fetch(url); if (!r.ok) throw new Error(url + " " + r.status); return r.json(); }
  // W6: the last full snapshot the client holds. A keyframe (state_snapshot or
  // a REST poll) replaces it wholesale; a state_delta merges changed groups in.
  let currentSnap = null;
  function applyFull(snap) { currentSnap = snap; renderSnapshot(snap); }
  function applyDelta(delta) {
    // No base yet -> wait for the next keyframe (do not render a partial view).
    if (!currentSnap) return;
    const keys = Object.keys(delta);
    if (!keys.length) return;                        // empty delta = keepalive only
    // Group-level merge: each key (geo/pose/plan/status/events) is replaced whole.
    for (const k of keys) currentSnap[k] = delta[k];
    renderSnapshot(currentSnap);
  }
  async function tick() {
    // markDownlink BEFORE applyFull so the snapshot's estop_path is drawn
    // against a fresh age; a failed fetch marks nothing, which is how the
    // watchdog learns P5 is gone.
    try { const snap = await getJSON("/api/hmi/snapshot"); markDownlink(); applyFull(snap); }
    catch (e) { /* transient; next tick retries */ }
  }
  // -- W7 task control uplink (11 S12.1.1) ---------------------------------
  // The browser's first WRITE path. Everything before this was read-only: the
  // socket carried snapshots down and nothing up.
  //
  // Held here (not per call) because an ack arrives on the SAME socket, later,
  // matched by req_id -- so the pending map has to outlive the call that sent
  // the frame. A frame whose ack never comes stays pending and its button stays
  // disabled, which is the honest display: "sent, no answer" is a different
  // state from "refused", and an operator who cannot tell them apart will press
  // it again.
  let liveWS = null;
  let reqSeq = 0;
  const pendingReqs = new Map();               // req_id -> {btn, label}

  // *** Feedback is INLINE, never alert() / confirm().
  //
  // A native dialog blocks the page's single thread: while it is open the map
  // stops redrawing, incoming WS frames queue, and the pose readout freezes --
  // on a screen whose entire job is showing where a moving robot is. An
  // unattended refusal dialog would freeze it indefinitely. Found the hard way:
  // an alert() left the page unresponsive for the full 300 s of a probe.
  function taskNotice(taskId, text, cls) {
    const card = document.querySelector(
      `.task-card[data-task-id="${CSS.escape(taskId)}"]`);
    if (!card) return;
    let box = card.querySelector(".task-notice");
    if (!box) {
      box = document.createElement("div");
      box.className = "task-notice";
      card.appendChild(box);
    }
    box.textContent = text;
    box.classList.toggle("bad", cls === "bad");
  }

  function sendUplink(frame, btn) {
    if (controlsOffline()) {
      // No live link = no control. Reported, never queued: a queued pause
      // that fired on reconnect would act on a task the operator stopped
      // looking at minutes ago. A task frame answers on its card; goto has
      // no card, so it answers on the banner.
      if (frame.type === "task") taskNotice(frame.task_id, "连接已断开, 任务控制不可用", "bad");
      else alertBanner("连接已断开, 操作不可用");
      return null;
    }
    reqSeq += 1;
    // req_id is per connection, and P5 prefixes it with "h-" to make the cmd_id.
    const reqId = "w" + Date.now().toString(36) + "-" + reqSeq;
    frame.req_id = reqId;
    if (btn) { btn.disabled = true; pendingReqs.set(reqId, btn); }
    liveWS.send(JSON.stringify(frame));
    return reqId;                        // the goto flow matches its ack by it
  }

  function onUplinkAck(m) {
    if (m.req_type === "estop") { onEstopAck(m); return; }
    const btn = pendingReqs.get(m.req_id);
    // W2 has its own flow (the target dot, the echo, the tracking).
    if (m.req_type === "chassis_enable") {
      pendingReqs.delete(m.req_id);
      onChassisEnableAck(m);
      return;
    }
    if (m.req_type === "goto") {
      pendingReqs.delete(m.req_id);
      if (btn) btn.disabled = false;
      onGotoAck(m);
      return;
    }
    pendingReqs.delete(m.req_id);
    if (btn) btn.disabled = false;
    if (m.result === "accepted") { refreshTasks(); return; }
    // A refusal is SHOWN on the card it belongs to. The codes reaching here are
    // real outcomes the operator can act on -- E_TASK_STATE means the task
    // already moved on (it finished, or someone else paused it), not that the
    // click was wrong.
    const taskId = (m.detail && m.detail.task_id) || "";
    const reason = (m.detail && (m.detail.reason_text || m.detail.reason)) || "";
    taskNotice(taskId, "操作被拒绝 " + (m.code || "") + (reason ? ": " + reason : ""),
               "bad");
    refreshTasks();
  }

  //: The W7 actions offered per task state. Taken from P3's transition graph
  //: (11 S4.4), not guessed: pause is legal only from `running` (it maps onto
  //: the graph's `suspend` arrow), resume only from `suspended`, and cancel
  //: from EVERY live state including the queued ones.
  //:
  //: *** The queued states matter. A task submitted by mistake sits in
  //: pending/ready and is the single most likely thing an operator wants gone
  //: -- offering it no button at all (the first version of this table) leaves
  //: them with the e-stop, which S12.1.1 says outright is not what it is for.
  //:
  //: An action the state forbids is not drawn, because P3 would answer
  //: E_TASK_STATE and the operator would read a refusal as a broken button.
  // <task-actions>
  // node-tested by test_task_actions_js.py, which also pins QUEUED to P3's
  // clear_queue states (99 U174: queued tasks are cancelled by a button here).
  const QUEUED = ["pending", "scheduled", "ready", "blocked"];
  function taskActions(state) {
    if (state === "running") return ["pause", "cancel"];
    if (state === "suspended") return ["resume", "cancel"];
    if (QUEUED.indexOf(state) >= 0) return ["cancel"];
    return [];                       // terminal: done / failed / cancelled
  }
  // </task-actions>

  const ACTION_LABEL = { pause: "暂停", resume: "继续", cancel: "取消任务" };

  //: How long an armed cancel stays armed. Long enough to read the progress it
  //: shows, short enough that a stray first click cannot be completed by an
  //: unrelated one minutes later.
  const CANCEL_ARM_MS = 5000;

  function armCancel(btn, card) {
    btn.dataset.armed = "1";
    btn.classList.add("armed");
    const pct = (card.querySelector(".task-progress strong") || {}).textContent || "";
    btn.textContent = "再点确认 (已完成 " + (pct || "--") + ")";
    btn._armTimer = setTimeout(() => disarmCancel(btn), CANCEL_ARM_MS);
  }

  function disarmCancel(btn) {
    clearTimeout(btn._armTimer);
    btn.dataset.armed = "0";
    btn.classList.remove("armed");
    btn.textContent = ACTION_LABEL.cancel;
  }

  function wireTaskActions(root) {
    root.querySelectorAll("button[data-task-action]").forEach((btn) => {
      const card = btn.closest(".task-card");
      btn.addEventListener("click", () => {
        const action = btn.dataset.taskAction;
        const taskId = btn.dataset.taskId;
        const frame = { type: "task", action: action, task_id: taskId };
        if (action === "cancel") {
          // L2 (18 B07). Two deliberate clicks on the SAME button, not a modal:
          // the first arms it and shows what will be lost, the second sends.
          // S12.1.1 requires the dialog to name the task and its progress, and
          // the armed label carries both -- a native confirm() could show the
          // text but would freeze the map to do it.
          //
          // confirm.level is an AUDIT credential, not a permission check: 11
          // S12.1.1 is explicit that an unauthenticated browser (U23) can fill
          // the field in itself. What keeps the dangerous operations out is the
          // whitelist, not this.
          if (btn.dataset.armed !== "1") {
            armCancel(btn, card);
            return;
          }
          disarmCancel(btn);
          frame.confirm = { level: "L2" };
        }
        sendUplink(frame, btn);
      });
    });
  }

  // -- W2 goto by tapping the map (11 S12.1.1; user rulings 2026-11-10) --------
  // The operator taps a target; P5 turns it into a cmd/task submit of a goto
  // task (NOT a BehaviorCommand -- that would bypass P3's fence pre-check and
  // the U07a breakpoint ledger).
  //
  // *** Only two kinds of target are legal (user 2026-11-10): a point ON A ROUTE
  // (one of its recorded points, the dots drawn on the line) or a KEYPOINT. A
  // tap SNAPS to the nearest such point within goto.snap_px screen pixels,
  // keypoints first; anything else is refused on the spot with a message. A
  // keypoint is sent as waypoint_id (its own arrival radius applies), a route
  // point as its exact lat/lon (P5 adds the configured radius). P5 re-checks
  // the same rule, so the browser is not the only guard.
  //
  // Two clicks, like cancel: the first picks and shows the target, the second
  // sends. The target stays on the map -- a pink/white breathing dot of a FIXED
  // screen size (goto.pin_diameter_px) -- until the task is done or has failed.
  // A failure is announced in the top banner with the target's coordinates and
  // a close button.
  let lastOrigin = null;                  // set each render; needed to invert
  let lastGeo = {};                       // set each render; snap candidates
  let gotoCandidate = null;               // picked, not yet sent
  let gotoPendingReq = null;              // {reqId, target} while the ack is out
  let gotoTrack = null;                   // sent and accepted: {..target, taskId}
  let gotoHistoryAt = 0;                  // last history lookup (throttle)
  const GOTO_TRACK_KEY = "xbrain_hmi_goto_track";
  // Presentation fallbacks if ui_config carries no goto group (17 S6.10.2: UI
  // values, not safety values; the yaml is where they are tuned).
  // 2026-11-12 (user): the blinking target disc is the size of a waypoint disc
  // (outer diameter 2 x 24 px). ~~20~~ superseded.
  const GOTO_PIN_PX_FALLBACK = 48;
  const GOTO_SNAP_PX_FALLBACK = 14;
  const GOTO_HISTORY_EVERY_MS = 2000;
  const BANNER_INFO_MS = 6000;

  function gotoCfg() {
    const g = (cfg && cfg.goto) || {};
    return { pinPx: g.pin_diameter_px || GOTO_PIN_PX_FALLBACK,
             snapPx: g.snap_px || GOTO_SNAP_PX_FALLBACK };
  }
  const fmtLL = (t) => t.lat.toFixed(6) + "°N " + t.lon.toFixed(6) + "°E";
  function targetLabel(t) {
    return t.kind === "keypoint" ? "关键点「" + (t.name || t.waypoint_id) + "」"
                                 : "路径「" + (t.name || "?") + "」上的点";
  }

  // Restore an in-flight goto after a page reload, so the dot and the failure
  // banner survive a refresh. Storage blocked -> simply not remembered.
  try {
    const saved = JSON.parse(localStorage.getItem(GOTO_TRACK_KEY) || "null");
    if (saved && saved.taskId && saved.lat != null) gotoTrack = saved;
  } catch (e) { /* no storage */ }
  function saveTrack() {
    try {
      if (gotoTrack) localStorage.setItem(GOTO_TRACK_KEY, JSON.stringify(gotoTrack));
      else localStorage.removeItem(GOTO_TRACK_KEY);
    } catch (e) { /* no storage */ }
  }

  // Nearest legal target to a tap, in SCREEN pixels (the snap radius is about
  // where the finger/mouse landed on screen, whatever the zoom). Keypoints win
  // over route points within the radius: they are the named, deliberate ones.
  function pickGotoTarget(clientX, clientY) {
    const svg = $("mapSvg");
    const vb = (svg.getAttribute("viewBox") || "").split(/\s+/).map(Number);
    if (vb.length !== 4) return null;
    const r = svg.getBoundingClientRect();
    const cx = clientX - r.left, cy = clientY - r.top;
    const toScreen = (xy) => [(xy[0] - vb[0]) / vb[2] * r.width,
                              (xy[1] - vb[1]) / vb[3] * r.height];
    const snap = gotoCfg().snapPx;
    // A keypoint is hit anywhere on its drawn disc (keypointHitPx), a route
    // point within snap_px -- the disc grew past snap_px on 2026-11-12, and a
    // tap on its outer ring must not answer "导航位置不正确".
    let lim = keypointHitPx(snap, cfg);
    let best = null;
    const consider = (pt, mk) => {
      const xy = toXY(pt, lastOrigin); if (!xy) return;
      const s = toScreen(xy), d = Math.hypot(s[0] - cx, s[1] - cy);
      if (d <= lim && (!best || d < best.d)) best = { d, t: mk() };
    };
    const wps = lastGeo.waypoints || {};
    if (wps.available) for (const w of wps.items) {
      const g = w.geom || w;
      if (!w.geo_id || g.lat == null) continue;
      consider(g, () => ({ kind: "keypoint", waypoint_id: w.geo_id, name: w.name,
                           lat: g.lat, lon: g.lon }));
    }
    if (best) return best.t;                         // keypoint first
    lim = snap;                                      // route points: snap_px
    const routes = lastGeo.routes || {};
    if (routes.available) for (const rt of routes.items) {
      if (rt.kind === "realtime") continue;          // the live trail is not a route
      for (const p of (rt.points || [])) {
        if (p.lat == null) continue;
        consider(p, () => ({ kind: "route", name: rt.name, lat: p.lat, lon: p.lon }));
      }
    }
    return best ? best.t : null;
  }

  //: Movement beyond this between mousedown and mouseup makes it a pan, not a tap.
  const DRAG_TAP_PX = 4;
  let lastPanMoved = false;

  function onMapClick(ev) {
    if (lastPanMoved) { lastPanMoved = false; return; }   // end of a pan
    // Link down: no pick at all. The link banner already says why.
    if (controlsOffline()) return;
    if (!lastOrigin) {
      // No enu_origin -> the map has no georeference, nothing can be placed.
      showBanner("warn", "地图还没有定位基准, 暂时无法选择导航目标");
      return;
    }
    const t = pickGotoTarget(ev.clientX, ev.clientY);
    if (!t) {
      gotoCandidate = null;
      renderGotoPins();
      showBanner("warn", "导航位置不正确: 只能选择路径折线上的点 "
                 + "或地图上带名称的关键点, 请点在它们上面。");
      return;
    }
    gotoCandidate = t;
    renderGotoPins();
    showBanner("confirm", "前往 " + targetLabel(t) + "  " + fmtLL(t)
               + "  -- 点击确认前往, 机器人将出发");
  }

  function sendGoto() {
    if (!gotoCandidate) return;
    const t = gotoCandidate;
    const frame = t.kind === "keypoint"
      ? { type: "goto", waypoint_id: t.waypoint_id }
      : { type: "goto", lat: t.lat, lon: t.lon };
    const reqId = sendUplink(frame, $("gotoSend"));
    if (!reqId) return;                    // offline: sendUplink said so
    gotoPendingReq = { reqId, target: t };
    showBanner("info", "正在下发: 前往 " + targetLabel(t) + " ...");
  }

  function cancelGoto() {
    gotoCandidate = null;
    renderGotoPins();
    hideBanner();
  }

  // P5/P3 reason codes -> what the operator reads. Unknown codes are shown raw.
  // <goto-reason>
  // Why a goto was refused, in the operator's words (pure; node-tested by
  // test_goto_reason_js.py). Keyed on detail.reason from P3 / P5.
  const GOTO_REASON_CN = {
    // 99 U173 (user 2026-11-12): an HMI goto never interrupts the running task
    // and is not queued; P3 answers E_BUSY {reason: task_running}. Wording from
    // the user's ruling; the button is named as it is on screen (确认前往).
    task_running: "机器人正在执行任务, 点击导航不会打断它. 如需立即前往, 请先软件急停, 再点目标位置并点「确认前往」",
    positioning_failed: "定位失效 (单点解或无解), 暂时不能导航",
    target_outside_allow_fence: "目标点在活动区域围栏之外",
    target_inside_forbid_zone: "目标点在禁止区域之内",
    keypoint_not_found: "该关键点已不存在",
    goto_keypoint_unknown: "该关键点已不存在",
    goto_target_not_on_route: "目标不在路径上, 也不是关键点",
    goto_targets_unavailable: "地图数据还未就绪, 请稍后再试",
    goto_arrive_radius_missing: "到点半径未配置, 无法前往路径上的点",
    teach_session_active: "正在录制, 录制期间不接受新任务",
    rate_limited: "操作过于频繁, 请稍后再试",
  };
  function reasonText(m) {
    const d = m.detail || {};
    // P5's own 2 s E_TIMEOUT (11 S12.1.4) carries detail.item, not a reason.
    if (!d.reason && d.item === "downstream_ack") return "机器人未回应, 请稍后再试";
    return GOTO_REASON_CN[d.reason] || d.reason_text || d.reason || m.code || "";
  }
  // </goto-reason>

  function onGotoAck(m) {
    if (!gotoPendingReq || m.req_id !== gotoPendingReq.reqId) return;
    const t = gotoPendingReq.target;
    gotoPendingReq = null;
    gotoCandidate = null;
    if (m.result === "accepted" || m.result === "duplicate") {
      const d = m.detail || {};
      const taskId = d.task_id || (d.applied && d.applied.task_id) || "";
      gotoTrack = Object.assign({}, t, { taskId });
      saveTrack();
      renderGotoPins();
      // CMD-30: echo what the robot accepted, not just "sent".
      showBanner("info", "已下发: 任务 " + (taskId || "--") + "  前往 " + targetLabel(t)
                 + "  " + fmtLL(t));
      refreshTasks();
      return;
    }
    renderGotoPins();
    showBanner("fail", "导航未下发: " + reasonText(m) + "  (目标 " + fmtLL(t) + ")");
  }

  // Follow the tracked goto: on the plan list (state/task) it is alive; once it
  // leaves the list, the history says how it ended. done -> the dot goes;
  // failed / interrupted / needs_review -> the dot goes and the failure banner
  // stays until closed; cancelled -> the dot goes with a closable notice.
  function followGoto(plans) {
    if (!gotoTrack || !gotoTrack.taskId) return;
    if (plans.some((p) => p.task_id === gotoTrack.taskId)) return;
    const now = performance.now();
    if (now - gotoHistoryAt < GOTO_HISTORY_EVERY_MS) return;
    gotoHistoryAt = now;
    const tracked = gotoTrack;
    getJSON("/api/tasks?scope=history&limit=50").then((d) => {
      if (gotoTrack !== tracked) return;     // replaced meanwhile
      const card = (d.tasks || []).find((c) => c.task_id === tracked.taskId);
      if (!card) return;                     // not terminal yet; next tick retries
      gotoTrack = null;
      saveTrack();
      renderGotoPins();
      refreshTasks();
      if (card.state === "done") {
        showBanner("info", "已到达目标点: " + targetLabel(tracked) + "  " + fmtLL(tracked));
      } else if (card.state === "cancelled") {
        showBanner("fail", "导航任务已取消  目标点 " + fmtLL(tracked));
      } else {
        showBanner("fail", "导航任务失败  目标点 " + fmtLL(tracked)
                   + "  (任务 " + tracked.taskId + ")");
      }
    }).catch(() => { /* transient; next tick retries */ });
  }

  // The candidate and the tracked target, as fixed-screen-size dots on their
  // own layer (renderGeo never clears it). Radius in user units = screen px x
  // metres-per-px, re-applied by updateGridAndDots on every zoom/pan.
  function renderGotoPins() {
    const layer = $("gotoLayer"); if (!layer) return;
    clear("gotoLayer");
    for (const t of [gotoTrack, gotoCandidate]) {
      if (!t) continue;
      const xy = toXY({ lat: t.lat, lon: t.lon }, lastOrigin); if (!xy) continue;
      const c = document.createElementNS(SVGNS, "circle");
      c.setAttribute("cx", xy[0]); c.setAttribute("cy", xy[1]);
      c.setAttribute("class", "goto-pin");
      layer.appendChild(c);
    }
    sizeGotoPins();
  }
  function sizeGotoPins() {
    const svg = $("mapSvg");
    const vb = (svg.getAttribute("viewBox") || "").split(/\s+/).map(Number);
    const mpp = (vb[2] || baseVB[2]) / (svg.clientWidth || 1);
    const rr = (gotoCfg().pinPx / 2 * mpp).toFixed(3);
    svg.querySelectorAll("circle.goto-pin").forEach((c) => c.setAttribute("r", rr));
  }

  // -- the top banner: confirm / info / warn / fail -------------------------
  // Not a modal and never alert(): a native dialog blocks the page thread and
  // freezes the map while the robot moves (see taskNotice). confirm keeps its
  // two buttons; warn has one acknowledgement button; fail has a close cross
  // and stays until closed; info hides itself after BANNER_INFO_MS.
  let bannerTimer = null;
  function showBanner(kind, text) {
    const b = $("gotoBanner"); if (!b) return;
    clearTimeout(bannerTimer);
    b.className = "goto-banner kind-" + kind;
    b.hidden = false;
    b.querySelector(".goto-text").textContent = text;
    $("gotoSend").hidden = kind !== "confirm";
    $("gotoCancel").hidden = kind !== "confirm";
    $("gotoAck").hidden = kind !== "warn";
    $("gotoClose").hidden = kind !== "fail";
    if (kind === "info") bannerTimer = setTimeout(hideBanner, BANNER_INFO_MS);
  }
  function hideBanner() {
    clearTimeout(bannerTimer);
    const b = $("gotoBanner"); if (b) b.hidden = true;
  }
  // -- chassis lock button (99 U159) ------------------------------------
  // Colour and label are the state/robot READ-BACK, never the click: deep red
  // 底盘已锁定 while hes_lock or timeout_lock is set, green 底盘已解锁 when both
  // are clear, grey 底盘状态未知 when P5 has no fresh state/robot. So an unlock
  // from anywhere else (commx 「解锁」, the cloud) turns the button green on the
  // next snapshot, and an accepted unlock that did NOT clear the lock (the
  // hardware stop still pressed, 11 S9.3.4 E-2) leaves it red -- BOOT-L3:
  // only the read-back says the lock is gone.
  //
  // Clicks: red -> the first click ARMS (再点确认解锁, CHASSIS_ARM_MS) and the
  // second sends W9 with confirm.level L2 (S9.3.4 E-1: enable is a loosening
  // command). Green -> nothing: the contract has no lock command (a lock only
  // comes from the hardware stop or a command timeout, S9.12.2).
  // <chassis-lock-view>
  function chassisLockView(cl) {
    if (!cl || !cl.available || typeof cl.locked !== "boolean") {
      return { cls: "unknown", text: "底盘状态未知", enabled: false,
               title: "未收到底盘状态, 无法判断是否锁定" };
    }
    if (cl.locked) {
      return { cls: "locked", text: "底盘已锁定", enabled: true,
               title: cl.hes_lock ? "硬件急停锁定: 先松开硬件急停, 再点击解锁"
                                  : "底盘已锁定: 点击后再点一次确认解锁" };
    }
    return { cls: "unlocked", text: "底盘已解锁", enabled: true,
             title: "底盘已解锁 (系统没有上锁指令, 锁定只由硬件急停或指令超时产生)" };
  }
  // </chassis-lock-view>
  const CHASSIS_ARM_MS = 5000;           // armed window, same as task cancel
  const CHASSIS_ACK_SETTLE_MS = 3000;    // accepted -> read-back must follow
  let chassisLock = {};                  // last chassis_lock group
  let chassisArmed = false, chassisArmTimer = null;
  let chassisSentAt = null;              // performance.now() of an accepted unlock
  function renderChassisLock(cl) {
    chassisLock = cl || {};
    const v = chassisLockView(chassisLock);
    const btn = $("chassisLockBtn"); if (!btn) return;
    if (v.cls !== "locked") disarmChassis();
    btn.className = "chassis-lock-btn " + v.cls + (chassisArmed ? " confirm" : "");
    btn.disabled = !v.enabled;
    btn.title = v.title;
    btn.textContent = chassisArmed ? "再点确认解锁" : v.text;
    if (chassisSentAt != null) {
      if (v.cls === "unlocked") chassisSentAt = null;          // read-back cleared
      else if (performance.now() - chassisSentAt > CHASSIS_ACK_SETTLE_MS) {
        chassisSentAt = null;
        showBanner("fail", chassisLock.hes_lock
          ? "解锁未生效: 硬件急停仍处于锁定, 请先松开硬件急停再解锁"
          : "解锁未生效: 底盘仍为锁定状态");
      }
    }
  }
  function disarmChassis() {
    chassisArmed = false;
    if (chassisArmTimer) { clearTimeout(chassisArmTimer); chassisArmTimer = null; }
  }
  function onChassisLockClick() {
    const v = chassisLockView(chassisLock);
    if (v.cls !== "locked") return;      // green / grey: no action exists
    if (!chassisArmed) {
      chassisArmed = true;
      chassisArmTimer = setTimeout(() => { disarmChassis(); renderChassisLock(chassisLock); },
                                   CHASSIS_ARM_MS);
      renderChassisLock(chassisLock);
      return;
    }
    disarmChassis();
    renderChassisLock(chassisLock);
    sendUplink({ type: "chassis_enable", confirm: { level: "L2" } }, $("chassisLockBtn"));
  }
  function onChassisEnableAck(m) {
    const btn = $("chassisLockBtn"); if (btn) btn.disabled = false;
    if (m.result === "accepted" || m.result === "duplicate") {
      // Accepted is not unlocked (BOOT-L3): wait for the read-back.
      chassisSentAt = performance.now();
      return;
    }
    const d = m.detail || {};
    showBanner("fail", "解锁被拒绝 " + (m.code || "") +
               ((d.reason_text || d.reason) ? ": " + (d.reason_text || d.reason) : ""));
  }

  // -- battery gauges (99 U160) ------------------------------------------
  // Slot 0 = A, slot 1 = B (user ruling), from snapshot.battery, which P5
  // builds from state/power.list[] and greys out when state/power is stale.
  // Colour rule (user 2026-11-11): >= 50 green, 20-49 yellow, < 20 red. Inside
  // the bar a present pack ALTERNATES percentage (BATTERY_PCT_MS) and
  // temperature (BATTERY_TEMP_MS) -- user 2026-11-11 -- with the fill keeping
  // the level colour in both phases. A pack with no temperature reading keeps
  // the percentage through the temperature phase rather than blank the bar.
  // An empty slot always says 未插入 (the chassis reports it as 0% / -273 C,
  // which P5 already dropped); unknown always says 未知. The temperature gets
  // no colour: the contract defines no battery temperature limit.
  // <battery-view>
  const BATTERY_PCT_MS = 5000;
  const BATTERY_TEMP_MS = 2000;
  function batteryCellView(pack, phaseMs) {
    const p = pack || {};
    if (p.state === "absent") {
      return { cls: "absent", text: "未插入", width: 0 };
    }
    if (p.state !== "present" || typeof p.level_pct !== "number") {
      return { cls: "unknown", text: "未知", width: 0 };
    }
    const lv = Math.max(0, Math.min(100, p.level_pct));
    const cls = lv >= 50 ? "ok" : (lv >= 20 ? "warn" : "low");
    const t = ((phaseMs || 0) % (BATTERY_PCT_MS + BATTERY_TEMP_MS) +
               (BATTERY_PCT_MS + BATTERY_TEMP_MS)) % (BATTERY_PCT_MS + BATTERY_TEMP_MS);
    const showTemp = t >= BATTERY_PCT_MS && typeof p.temp_c === "number";
    const text = showTemp ? p.temp_c.toFixed(1) + "℃"
                          : (p.charging ? "⚡" : "") + Math.round(lv) + "%";
    return { cls: cls, text: text, width: lv };
  }
  // </battery-view>
  let lastBattery = {};                  // last snapshot.battery
  const BAT_BODY_W = 121;                // body fill width in SVG units
  function renderBattery(bat) {
    if (bat !== undefined) lastBattery = bat || {};
    const packs = (lastBattery && lastBattery.packs) || [];
    const phase = performance.now();
    for (const [id, label] of [["batteryA", "A"], ["batteryB", "B"]]) {
      const el = $(id); if (!el) continue;
      const pack = packs.find((x) => x && x.label === label) || {};
      const v = batteryCellView(pack, phase);
      el.className = "battery-cell " + v.cls;
      // The level fills the body only; the nub is always solid (user
      // 2026-11-11, like the reference icon). Geometry = the SVG in
      // index.html: body fill x 7.5..128.5.
      el.querySelector(".bat-fill").setAttribute("width",
        (v.width / 100 * BAT_BODY_W).toFixed(2));
      el.querySelector(".battery-text").textContent = v.text;
      const fmt = (x, d, u) => (typeof x === "number" ? x.toFixed(d) + u : "--");
      el.title = "电池 " + label + (v.cls === "absent" ? ": 槽位无电池"
        : v.cls === "unknown" ? ": 未收到电量数据"
        : ": " + fmt(pack.level_pct, 0, "%") + "  " + fmt(pack.temp_c, 1, "℃") +
          "  " + fmt(pack.voltage_v, 1, "V") + (pack.charging ? "  充电中" : ""));
    }
  }

  // Offline refusals keep this name.
  function alertBanner(text, withConfirm) { showBanner(withConfirm ? "confirm" : "info", text); }

  // W6: WS server push is primary; REST poll is the fallback when WS is down.
  let pollTimer = null;
  // The task panel (17 S6.8.4) has its OWN poll -- it pulls /api/tasks (P3
  // queryable), which is NOT in the WS snapshot, so it runs regardless of WS.
  let taskTimer = null;
  function startPoll() { if (!pollTimer) { tick(); pollTimer = setInterval(tick, 1000); } }
  function stopPoll() { if (pollTimer) { clearInterval(pollTimer); pollTimer = null; } }
  function connectWS() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    let ws;
    try { ws = new WebSocket(`${proto}://${location.host}/ws`); }
    catch (e) { startPoll(); return; }               // no WS -> poll
    ws.onopen = () => { liveWS = ws; stopPoll(); refreshTasks(); };  // push takes over; re-pull tasks (reconnect full-pull)
    ws.onmessage = (e) => {
      try {
        const m = JSON.parse(e.data);
        // Heartbeat: only a frame that parsed counts -- a stream of garbage is
        // not evidence that P5 is healthy.
        markDownlink();
        if (m.kind === "state_snapshot") applyFull(m.data);      // keyframe
        else if (m.kind === "state_delta") applyDelta(m.data);   // W6 delta merge
        else if (m.kind === "ack") onUplinkAck(m);               // W7 uplink answer
      } catch (_) { /* ignore a malformed frame */ }
    };
    ws.onclose = () => {
      liveWS = null;
      // Every in-flight request dies with the socket: its ack can never arrive,
      // so re-enable the buttons rather than leave them disabled forever.
      pendingReqs.forEach((btn) => { btn.disabled = false; });
      pendingReqs.clear();
      startPoll(); setTimeout(connectWS, 3000);      // fall back + reconnect
    };
    ws.onerror = () => { try { ws.close(); } catch (_) {} };
  }
  async function init() {
    // The ESTOP watchdog runs from the first instant and independently of any
    // data: it is what greys the button when NOTHING arrives, so it cannot be
    // driven by an arrival. 250 ms keeps the grey-out within ~2.25 s of the
    // last frame. Started before the ui_config fetch, which can itself fail.
    updateEstopUi();
    setInterval(updateEstopUi, 250);
    wireInteraction(); applyView(); buildDialFace(); wireHistoryFolding();
    const clb = $("chassisLockBtn");
    if (clb) clb.addEventListener("click", onChassisLockClick);
    // The battery text alternates on a clock of its own: WS deltas only carry
    // the battery group when its numbers change, which can be minutes apart.
    setInterval(() => renderBattery(), 250);
    $("mapSvg").addEventListener("click", onMapClick);
    const gs = $("gotoSend");
    if (gs) gs.addEventListener("click", sendGoto);
    const gc = $("gotoCancel");
    if (gc) gc.addEventListener("click", cancelGoto);
    // warn (知道了) and fail (×) only dismiss the banner; a failed target's dot
    // is already gone by then (followGoto / onGotoAck).
    for (const id of ["gotoAck", "gotoClose"]) {
      const el = $(id); if (el) el.addEventListener("click", hideBanner);
    }
    window.addEventListener("resize", applyView);   // req3: rescale map labels on resize
    wirePanelScroll(); wirePanelResize(); wireEventDrawer();
    try { applyUiConfig(await getJSON("/api/hmi/ui_config")); }
    catch (e) { console.error("ui_config load failed", e); return; }
    await tick();                                     // instant first paint (REST)
    refreshTasks();                                   // 17 S6.8.4 task panel (own poll)
    if (!taskTimer) taskTimer = setInterval(refreshTasks, 4000);
    connectWS();                                      // then live server push (W6)
  }
  init();
})();
