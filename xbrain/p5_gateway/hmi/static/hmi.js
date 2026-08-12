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
  const view = { zoom: 1, pan: { x: 0, y: 0 } };
  let cfg = null;                       // ui_config, set on load
  let zoomBounds = { min: 0.65, max: 2.8, wheel: 0.12 };

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
    if (c.fence && c.fence.line) root.setProperty("--fence-color", c.fence.line.color);
    if (c.route && c.route.recorded) root.setProperty("--route-recorded-color", c.route.recorded.line.color);
    if (c.route && c.route.realtime) root.setProperty("--route-realtime-color", c.route.realtime.line.color);
    if (c.alarm_region && c.alarm_region.line) root.setProperty("--alarm-color", c.alarm_region.line.color);

    // U1: grid metres. viewBox is in ENU metres, so a pattern of `minor_m`
    // metres draws one cell per metre when minor_m = 1 (the default).
    const map = c.map || {};
    const grid = map.grid || { minor_m: 1, major_m: 5 };
    const vb = map.viewbox || "-100 -65 240 145";
    const [vx, vy, vw, vh] = vb.split(/\s+/).map(Number);
    $("mapSvg").setAttribute("viewBox", vb);
    setPattern("minorGrid", grid.minor_m, `M ${grid.minor_m} 0 L 0 0 0 ${grid.minor_m}`);
    setPattern("majorGrid", grid.major_m, `M ${grid.major_m} 0 L 0 0 0 ${grid.major_m}`);
    $("majorGridRect").setAttribute("width", grid.major_m);
    $("majorGridRect").setAttribute("height", grid.major_m);
    for (const attr of [["x", vx], ["y", vy], ["width", vw], ["height", vh]])
      $("gridBg").setAttribute(attr[0], attr[1]);
    $("scaleText").textContent = `网格 ${grid.minor_m}m / ${grid.major_m}m`;

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
    renderGeo(snap.geo || {}, origin);
    renderRobot(snap.pose || {}, origin);
    renderEvents(snap.events || {}, origin);
    renderPlan(snap.plan || {});
    renderStatus(snap.status || {}, snap.pose || {});
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

  function renderGeo(geo, origin) {
    clear("keepInLayer"); clear("alarmLayer");
    clear("recordedRouteLayer"); clear("realtimeTrajectoryLayer"); clear("keypointLayer");
    const fences = geo.fences || {};
    if (fences.available) for (const fen of fences.items) {
      const pts = ptsAttr(fen.vertices || [], origin); if (!pts) continue;
      const zone = fen.role === "zone";
      const layer = zone ? "alarmLayer" : "keepInLayer";
      $(layer).appendChild(poly(zone ? "alarm-region" : "keep-in", pts,
        zone ? (cfg.alarm_region || {}).line : cfg.fence.line));
      if (fen.name && fen.vertices && fen.vertices[0]) {
        const xy = toXY(fen.vertices[0], origin);
        if (xy) $(layer).appendChild(label(xy[0], xy[1] - 2,
          zone ? "alarm-label" : "keep-in-label", fen.name));
      }
    }
    const routes = geo.routes || {};
    if (routes.available) for (const r of routes.items) {
      const pts = ptsAttr(r.points || [], origin); if (!pts) continue;
      const realtime = r.kind === "realtime";
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

  function renderStatus(status, pose) {
    $("modeText").textContent = status.mode ? `模式: ${status.mode}` : "模式: --";
    // pose-derived readouts: null until perception/rtk exist (17 S6.10.4).
    $("coordGps").textContent = pose.available && pose.lat != null
      ? `${pose.lat.toFixed(6)}°N · ${pose.lon.toFixed(6)}°E` : "无定位";
    $("coordEnu").textContent = pose.available && pose.speed_mps != null
      ? `航向 ${pose.heading_valid ? (pose.heading_rad * 180 / Math.PI).toFixed(1) : "--"}°  速度 ${pose.speed_mps.toFixed(1)}m/s` : "--";
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

  // -- zoom / pan (U4 wheel) ------------------------------------------------ */
  function applyView() {
    $("mapSvg").style.transform = `translate(${view.pan.x}px,${view.pan.y}px) scale(${view.zoom})`;
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
      drag = { x: e.clientX - view.pan.x, y: e.clientY - view.pan.y };
      vp.classList.add("dragging");
    });
    window.addEventListener("mousemove", (e) => {
      if (!drag) return; view.pan = { x: e.clientX - drag.x, y: e.clientY - drag.y }; applyView();
    });
    window.addEventListener("mouseup", () => { drag = null; vp.classList.remove("dragging"); });
    // ESTOP: W1 (17 S6.4). Button is only enabled when estop_path ok (NAV-64).
    $("estopBtn").onclick = async () => {
      if ($("estopBtn").disabled) return;
      try { await fetch("/api/estop", { method: "POST" }); } catch (_) { /* link probe shows the failure */ }
    };
  }

  async function getJSON(url) { const r = await fetch(url); if (!r.ok) throw new Error(url + " " + r.status); return r.json(); }
  async function tick() {
    try { renderSnapshot(await getJSON("/api/hmi/snapshot")); }
    catch (e) { /* transient; next tick retries */ }
  }
  async function init() {
    wireInteraction(); applyView();
    try { applyUiConfig(await getJSON("/api/hmi/ui_config")); }
    catch (e) { console.error("ui_config load failed", e); return; }
    await tick();
    setInterval(tick, 1000);                          // 1 Hz REST poll (WS later)
  }
  init();
})();
