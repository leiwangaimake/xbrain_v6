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

  // -- ENU projection: snapshot geometry may be {lat,lon} or [e_m,n_m]. When
  //    lat/lon + enu_origin are present, project to local ENU metres (the SVG
  //    frame). Equirectangular approx is enough at site scale. Absent origin ->
  //    lat/lon points are skipped (cannot place them), never plotted at 0,0. --
  function toXY(pt, origin) {
    if (Array.isArray(pt)) return pt;                 // already [e_m, n_m]
    if (pt && pt.lat != null && pt.lon != null) {
      if (!origin) return null;                       // no origin -> cannot place
      const R = 111320, c = Math.cos((origin.lat || 0) * Math.PI / 180);
      return [(pt.lon - origin.lon) * R * c, -(pt.lat - origin.lat) * R];
    }
    return null;
  }
  const ptsAttr = (arr, origin) => arr.map((p) => toXY(p, origin))
    .filter(Boolean).map((xy) => xy.join(",")).join(" ");

  function lineStroke(el, line) {
    if (!line) return;
    el.setAttribute("stroke", line.color);
    el.setAttribute("stroke-width", line.width_px || 1.4);
    if (line.style === "dashed") el.setAttribute("stroke-dasharray", "4 3");
    else if (line.style === "dotted") el.setAttribute("stroke-dasharray", "1 2");
    else el.removeAttribute("stroke-dasharray");     // solid
  }

  // Map a fence to its display type. The contract role (17 S6.8: allow/forbid/
  // zone) wins; if role is absent -- P3's fence table stores a free-text
  // zone_label, not the role enum -- fall back to the name ("活动"->active,
  // "禁入"->forbid, "报警"->alarm). Default active (keep-in) when neither says.
  function fenceType(fen) {
    const r = fen.role;
    if (r === "allow") return "active";
    if (r === "forbid") return "forbid";
    if (r === "zone") return "alarm";
    const n = fen.name || fen.zone_label || "";
    if (n.indexOf("禁入") >= 0) return "forbid";
    if (n.indexOf("报警") >= 0) return "alarm";
    if (n.indexOf("活动") >= 0) return "active";
    return "active";
  }

  // -- apply ui_config: CSS vars (U2/U3/U5), grid pattern metres (U1), zoom (U4) --
  function applyUiConfig(c) {
    cfg = c;
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
    if (c.route && c.route.recorded) root.setProperty("--route-recorded-color", c.route.recorded.line.color);
    if (c.route && c.route.realtime) root.setProperty("--route-realtime-color", c.route.realtime.line.color);

    // U1: grid metres. The viewBox (ENU metres) is the BASE window; applyView()
    // pans/zooms it by rewriting the viewBox attribute (NOT a CSS transform on
    // the element), so the SVG element always fills the viewport. The grid rect
    // spans a huge extent and its pattern is userSpaceOnUse (anchored at the ENU
    // origin), so the井字格 fills the screen at ANY pan/zoom and never shows an
    // edge -- an infinite grid the operator cannot drag or zoom off.
    const map = c.map || {};
    const grid = map.grid || { minor_m: 1, major_m: 5 };
    baseVB = (map.viewbox || "-100 -65 240 145").split(/\s+/).map(Number);
    setPattern("minorGrid", grid.minor_m, `M ${grid.minor_m} 0 L 0 0 0 ${grid.minor_m}`);
    setPattern("majorGrid", grid.major_m, `M ${grid.major_m} 0 L 0 0 0 ${grid.major_m}`);
    $("majorGridRect").setAttribute("width", grid.major_m);
    $("majorGridRect").setAttribute("height", grid.major_m);
    const G = 100000;                     // metres; far beyond any mouse pan/zoom
    for (const [k, v] of [["x", -G], ["y", -G], ["width", 2 * G], ["height", 2 * G]])
      $("gridBg").setAttribute(k, v);
    $("scaleText").textContent = `网格 ${grid.minor_m}m / ${grid.major_m}m`;
    applyView();                          // set the initial viewBox from base + view

    const z = map.zoom || {};
    zoomBounds = { min: z.min || 0.65, max: z.max || 2.8, wheel: z.wheel_step || 0.12 };
  }
  function setPattern(id, size, d) {
    const p = $(id);
    p.setAttribute("width", size); p.setAttribute("height", size);
    p.querySelector("path").setAttribute("d", d);
  }

  // -- render one snapshot -------------------------------------------------- */
  function renderSnapshot(snap) {
    const origin = (snap.geo && snap.geo.enu_origin) || null;
    // patrolling = a plan is running -> the yellow realtime trajectory shows;
    // once no plan runs (patrol complete) it hides unless show_after_complete.
    const plans = (snap.plan && snap.plan.plans) || [];
    const patrolling = plans.some((p) => p.state === "running");
    renderGeo(snap.geo || {}, origin, patrolling);
    renderRobot(snap.pose || {}, origin);
    renderEvents(snap.events || {}, origin);
    renderPlan(snap.plan || {});
    renderStatus(snap.status || {}, snap.pose || {}, origin);
    renderHeading(snap.pose || {});
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

  function renderGeo(geo, origin, patrolling) {
    clear("keepInLayer"); clear("alarmLayer");
    clear("recordedRouteLayer"); clear("realtimeTrajectoryLayer"); clear("keypointLayer");
    const fences = geo.fences || {};
    if (fences.available) for (const fen of fences.items) {
      const pts = ptsAttr(fen.vertices || [], origin); if (!pts) continue;
      const type = fenceType(fen);            // active | forbid | alarm
      const line = ((cfg.fence || {})[type] || (cfg.fence || {}).active || {}).line
                   || { style: "solid", color: "#e0b000", width_px: 1.6 };
      const active = type === "active";
      const layer = active ? "keepInLayer" : "alarmLayer";
      const el = poly(active ? "keep-in" : "alarm-region", pts, line);
      // fill: faint tint of the same colour so the polygon body reads as its type
      // (active -> blue, 17 S6.10.2A; alarm/forbid -> red).
      el.setAttribute("fill", active ? "rgba(47,136,255,.06)" : "rgba(255,32,32,.07)");
      $(layer).appendChild(el);
      // U5: fence naming -- show the name (zone_label) in the line colour.
      if (fen.name && fen.vertices && fen.vertices[0]) {
        const xy = toXY(fen.vertices[0], origin);
        if (xy) {
          const t = label(xy[0], xy[1] - 2, "keep-in-label", fen.name);
          t.setAttribute("fill", line.color);
          $(layer).appendChild(t);
        }
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
      $(realtime ? "realtimeTrajectoryLayer" : "recordedRouteLayer").appendChild(
        polyline(realtime ? "realtime-trajectory" : "recorded-route", pts,
          realtime ? cfg.route.realtime.line : cfg.route.recorded.line));
    }
    const wps = geo.waypoints || {};
    if (wps.available) for (const w of wps.items) {
      const xy = toXY(w.geom || w, origin); if (!xy) continue;
      const mk = (cfg.waypoint || {})[w.recorded ? "recorded" : "unrecorded"] || {};
      $("keypointLayer").appendChild(marker(xy[0], xy[1], mk));
      if (w.name) $("keypointLayer").appendChild(label(xy[0] + mk.size + 1, xy[1], "keypoint", w.name));
    }
  }

  // U6: waypoint marker shape (circle/square/triangle/diamond) + colour + size.
  function marker(x, y, mk) {
    const s = mk.size || 3, col = mk.color || "#3aa0ff", shape = mk.shape || "circle";
    let el;
    if (shape === "square") {
      el = document.createElementNS(SVGNS, "rect");
      el.setAttribute("x", x - s); el.setAttribute("y", y - s);
      el.setAttribute("width", 2 * s); el.setAttribute("height", 2 * s);
    } else if (shape === "triangle") {
      el = document.createElementNS(SVGNS, "polygon");
      el.setAttribute("points", `${x},${y - s} ${x - s},${y + s} ${x + s},${y + s}`);
    } else if (shape === "diamond") {
      el = document.createElementNS(SVGNS, "polygon");
      el.setAttribute("points", `${x},${y - s} ${x + s},${y} ${x},${y + s} ${x - s},${y}`);
    } else {
      el = document.createElementNS(SVGNS, "circle");
      el.setAttribute("cx", x); el.setAttribute("cy", y); el.setAttribute("r", s);
    }
    el.setAttribute("fill", col);
    return el;
  }

  function renderRobot(pose, origin) {
    clear("robotLayer");
    // No fix -> draw nothing. Never plot the robot at (0,0) (17 S6.10.4).
    if (!pose.available || pose.lat == null) return;
    const xy = toXY({ lat: pose.lat, lon: pose.lon }, origin); if (!xy) return;
    const a = pose.heading_valid ? (pose.heading_rad || 0) : 0, s = 3;
    const el = document.createElementNS(SVGNS, "polygon");
    // simple arrow along heading
    const pts = [[0, -s], [-s * 0.7, s], [0, s * 0.4], [s * 0.7, s]]
      .map(([px, py]) => {
        const rx = px * Math.cos(a) - py * Math.sin(a) + xy[0];
        const ry = px * Math.sin(a) + py * Math.cos(a) + xy[1];
        return `${rx},${ry}`;
      }).join(" ");
    el.setAttribute("points", pts); el.setAttribute("class", "robot-arrow");
    $("robotLayer").appendChild(el);
  }

  function renderEvents(events, origin) {
    clear("eventLayer");
    if (!events.available) return;
    for (const ev of events.items) {
      if (!ev.pos) continue;                          // located-nowhere -> no dot
      const xy = toXY(ev.pos, origin); if (!xy) continue;
      const dot = document.createElementNS(SVGNS, "circle");
      dot.setAttribute("cx", xy[0]); dot.setAttribute("cy", xy[1]);
      dot.setAttribute("r", 2.4); dot.setAttribute("class", "alarm-dot");
      $("eventLayer").appendChild(dot);
    }
  }

  function renderPlan(plan) {
    const box = $("planList"); box.innerHTML = "";
    if (!plan.available || !plan.plans || !plan.plans.length) {
      box.innerHTML = '<div class="plan-empty">暂无计划</div>'; return;
    }
    for (const p of plan.plans) {
      const running = p.state === "running";
      const total = p.progress && p.progress.total;
      // 17 S6.10.4: show a fraction ONLY when total is known, never fabricate.
      const frac = total != null ? `${p.progress.done} / ${total}` : "--";
      const targets = (p.targets || []).map((t, i) =>
        `<li class="${t.done ? "finished" : "current"}"><i>${t.done ? "✓" : i + 1}</i><span>${esc(t.name || "")}</span></li>`).join("");
      const el = document.createElement("article");
      el.className = "robot-plan";
      el.innerHTML =
        `<div class="robot-plan-header"><strong>${esc(p.task_id || "")}</strong>` +
        `<span class="plan-state ${running ? "running" : "completed"}">${running ? "执行中" : (p.state || "")}</span></div>` +
        `<div class="robot-plan-name">${esc(p.name || "未命名计划")}</div>` +
        `<div class="dispatch-time"><span>下发时间</span> <time>${esc(p.dispatch_ts || "--")}</time></div>` +
        (targets ? `<div class="target-title">巡检目标点</div><ol class="target-list">${targets}</ol>` : "") +
        `<div class="robot-plan-result"><span>已巡检 <strong>${frac}</strong></span></div>`;
      box.appendChild(el);
    }
  }

  function renderStatus(status, pose, origin) {
    $("modeText").textContent = status.mode ? `模式: ${status.mode}` : "模式: --";
    // pose-derived readouts: null until perception/rtk exist (17 S6.10.4).
    $("coordGps").textContent = pose.available && pose.lat != null
      ? `${pose.lat.toFixed(6)}°N · ${pose.lon.toFixed(6)}°E` : "无定位";
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
      `E ${e}m · N ${n}m　航向 ${hdg}°　速度 ${spd}m/s`;
    setDot($("rtkDot"), pose.fix_type ? "ok" : "");
    $("rtkText").textContent = pose.fix_type || "无 RTK";
    $("precText").textContent = pose.cov_h_m != null ? `${pose.cov_h_m.toFixed(2)}m` : "--";
    // link + ESTOP arming (NAV-64): estop_path ok -> button enabled + pulse.
    const ok = status.estop_path === "ok";
    setDot($("linkDot"), ok ? "ok" : "bad");
    $("linkText").textContent = ok ? "链路正常" : "急停链路断";
    const btn = $("estopBtn");
    btn.disabled = !ok;
    btn.classList.toggle("armed", ok);
    btn.title = ok ? "紧急停止" : "急停不可用(链路断)";
  }
  function setDot(el, cls) { el.className = "dot" + (cls ? " " + cls : ""); }
  function esc(s) { const d = document.createElement("div"); d.textContent = s; return d.innerHTML; }

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
      lastHeadingCompass = compass;                       // remember for freeze-on-loss
      face.style.transform = `rotate(${-compass}deg)`;   // CARD rotates -> heading to top
      dial.classList.remove("no-heading");
    } else if (lastHeadingCompass !== null) {
      // Both sources lost -> hold the last valid heading (freeze). The transform
      // is already there; re-assert it (in case anything reset it) and stop.
      face.style.transform = `rotate(${-lastHeadingCompass}deg)`;
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
    const [bx, by, bw, bh] = baseVB;
    const w = bw / view.zoom, h = bh / view.zoom;
    const cx = bx + bw / 2 + view.pan.x, cy = by + bh / 2 + view.pan.y;
    $("mapSvg").setAttribute("viewBox", `${cx - w / 2} ${cy - h / 2} ${w} ${h}`);
    $("zoomText").textContent = `${view.zoom.toFixed(1)}x`;
  }
  function clampZoom(z) { return Math.max(zoomBounds.min, Math.min(zoomBounds.max, z)); }
  function wireInteraction() {
    $("zoomIn").onclick = () => { view.zoom = clampZoom(view.zoom + .2); applyView(); };
    $("zoomOut").onclick = () => { view.zoom = clampZoom(view.zoom - .2); applyView(); };
    $("zoomReset").onclick = () => { view.zoom = 1; view.pan = { x: 0, y: 0 }; applyView(); };
    const vp = $("viewport");
    vp.addEventListener("wheel", (e) => {          // U4: wheel up=in, down=out
      e.preventDefault();
      view.zoom = clampZoom(view.zoom + (e.deltaY < 0 ? zoomBounds.wheel : -zoomBounds.wheel));
      applyView();
    }, { passive: false });
    let drag = null;
    vp.addEventListener("mousedown", (e) => {
      drag = { x: e.clientX, y: e.clientY, px: view.pan.x, py: view.pan.y };
      vp.classList.add("dragging");
    });
    window.addEventListener("mousemove", (e) => {
      if (!drag) return;
      // pixel delta -> ENU metre delta via the current viewBox scale (non-uniform,
      // preserveAspectRatio=none). Drag right -> the window moves left (grab-pan).
      const r = vp.getBoundingClientRect();
      const mppx = (baseVB[2] / view.zoom) / r.width;
      const mppy = (baseVB[3] / view.zoom) / r.height;
      view.pan.x = drag.px - (e.clientX - drag.x) * mppx;
      view.pan.y = drag.py - (e.clientY - drag.y) * mppy;
      applyView();
    });
    window.addEventListener("mouseup", () => { drag = null; vp.classList.remove("dragging"); });
    // ESTOP: W1 (17 S6.4). Button is only enabled when estop_path ok (NAV-64).
    $("estopBtn").onclick = async () => {
      if ($("estopBtn").disabled) return;
      try { await fetch("/api/estop", { method: "POST" }); } catch (_) { /* link probe shows the failure */ }
    };
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
    try { applyFull(await getJSON("/api/hmi/snapshot")); }
    catch (e) { /* transient; next tick retries */ }
  }
  // W6: WS server push is primary; REST poll is the fallback when WS is down.
  let pollTimer = null;
  function startPoll() { if (!pollTimer) { tick(); pollTimer = setInterval(tick, 1000); } }
  function stopPoll() { if (pollTimer) { clearInterval(pollTimer); pollTimer = null; } }
  function connectWS() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    let ws;
    try { ws = new WebSocket(`${proto}://${location.host}/ws`); }
    catch (e) { startPoll(); return; }               // no WS -> poll
    ws.onopen = () => stopPoll();                     // push takes over from poll
    ws.onmessage = (e) => {
      try {
        const m = JSON.parse(e.data);
        if (m.kind === "state_snapshot") applyFull(m.data);      // keyframe
        else if (m.kind === "state_delta") applyDelta(m.data);   // W6 delta merge
      } catch (_) { /* ignore a malformed frame */ }
    };
    ws.onclose = () => { startPoll(); setTimeout(connectWS, 3000); };  // fall back + reconnect
    ws.onerror = () => { try { ws.close(); } catch (_) {} };
  }
  async function init() {
    wireInteraction(); applyView(); buildDialFace();
    try { applyUiConfig(await getJSON("/api/hmi/ui_config")); }
    catch (e) { console.error("ui_config load failed", e); return; }
    await tick();                                     // instant first paint (REST)
    connectWS();                                      // then live server push (W6)
  }
  init();
})();
