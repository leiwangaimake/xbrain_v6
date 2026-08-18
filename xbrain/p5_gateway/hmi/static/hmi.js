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
  const ptsAttr = (arr, origin) => arr.map((p) => toXY(p, origin))
    .filter(Boolean).map((xy) => xy.join(",")).join(" ");

  // req4d (2026-08-18): geometry length in METRES for the label. Points are ENU
  // metres ([e_m, n_m], 11 S7.10A GO-1) so euclidean distance == metres. Route =
  // open-path sum; fence = closed perimeter (add the closing edge). Returns "" for
  // degenerate input so the label just shows the name.
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
    el.setAttribute("stroke-width", (line.width_px || 1.4) * 2);
    if (line.style === "dashed") el.setAttribute("stroke-dasharray", "4 3");
    else if (line.style === "dotted") el.setAttribute("stroke-dasharray", "1 2");
    else el.removeAttribute("stroke-dasharray");     // solid
  }

  // Map a fence to its display type. The contract role (11 S9A.1A: allow/forbid/
  // speed_limit/warning) wins; if role is absent -- P3's fence table stores a
  // free-text zone_label, not the role enum -- fall back to the name ("活动"->
  // active, "禁入"->forbid, "报警"->alarm). Default active (keep-in) when neither
  // says. NOTE: "warning" is the alarm role; "zone" is its old name (11 S9A.1A
  // rename, 2026-08-18) -- accept both while the runtime still emits the old one.
  function fenceType(fen) {
    const r = fen.role;
    if (r === "allow") return "active";
    if (r === "forbid") return "forbid";
    if (r === "speed_limit") return "speed";
    if (r === "warning" || r === "zone") return "alarm";
    const n = fen.name || fen.zone_label || "";
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

  // req(2026-08-18): fallback cell sizes if yaml has none. The real list is
  // map.grid.cell_options_m (yaml-configurable). GRID_TARGET_PX = aim each cell
  // this many screen px so a huge map never shows a pinprick sieve; DOT_PX =
  // constant screen radius for the geo dots (so they never shrink to nothing on
  // a ~5km view).
  const GRID_CELLS_DEFAULT = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
  const GRID_TARGET_PX = 42;
  const DOT_PX = 3.5;

  // Chosen every view change (applyView) from the CURRENT metres-per-screen-px.
  // The map uses preserveAspectRatio="none" but fitGeoToData matched the viewport
  // aspect, so the scale is uniform (x==y) and mpp is a single value.
  function updateGridAndDots() {
    const svg = $("mapSvg");
    const cw = svg.clientWidth || 1;
    const vb = svg.getAttribute("viewBox");
    const vbW = vb ? parseFloat(vb.split(/\s+/)[2]) : baseVB[2];
    const mpp = vbW / cw;                  // metres per screen px
    const cells = (cfg && cfg.map && cfg.map.grid && cfg.map.grid.cell_options_m)
                  || GRID_CELLS_DEFAULT;
    // smallest configured cell whose on-screen size >= target; else the largest.
    let cell = cells[cells.length - 1];
    for (const cm of cells) { if (cm / mpp >= GRID_TARGET_PX) { cell = cm; break; } }
    setPattern("majorGrid", cell, `M ${cell} 0 L 0 0 0 ${cell}`);
    $("majorGridRect").setAttribute("width", cell);
    $("majorGridRect").setAttribute("height", cell);
    $("scaleText").textContent = `网格 ${cell}m`;
    // non-scaling geo dots: constant screen radius regardless of zoom.
    const rr = (DOT_PX * mpp).toFixed(3);
    svg.querySelectorAll("circle.geo-dot").forEach((c) => c.setAttribute("r", rr));
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
  const LABEL_LAYERS = "#keepInLayer text, #alarmLayer text, #recordedRouteLayer text, " +
                       "#realtimeTrajectoryLayer text, #keypointLayer text";
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
        const box = { x1: sx + dx - 1, y1: sy + dy - hpx, x2: sx + dx + wpx, y2: sy + dy + 2 };
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
    // patrolling = a plan is running -> the yellow realtime trajectory shows;
    // once no plan runs (patrol complete) it hides unless show_after_complete.
    const plans = (snap.plan && snap.plan.plans) || [];
    const patrolling = plans.some((p) => p.state === "running");
    renderGeo(snap.geo || {}, origin, patrolling);
    renderRobot(snap.pose || {}, origin);
    renderEvents(snap.events || {}, origin);
    // The task panel is NOT rendered from the snapshot -- it pulls current +
    // history from GET /api/tasks (17 S6.8.4, refreshTasks below). snap.plan
    // (state/task) still drives the `patrolling` trajectory flag above.
    renderStatus(snap.status || {}, snap.pose || {}, origin, snap.clock || {});
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

  // req(2026-08-18): the mock data spans ~5km, far beyond the config 240m
  // default view. Auto-fit the map to the geo BOUNDS once, on the first snapshot
  // that carries the fences (they define the true extent -- the active-area
  // lake), so the operator sees the whole site on load. toXY maps [e_m,n_m] to
  // [e, -n] (north up), so the fit bounds are already in SVG frame. Zoom-in is widened
  // so the 5m grid is legible when zoomed in. Fits ONCE; afterwards the operator
  // pans/zooms freely.
  let geoFitted = false;
  function fitGeoToData(geo) {
    if (geoFitted) return;
    const f = geo.fences || {};
    if (!(f.available && (f.items || []).length)) return;   // wait for fences
    const xs = [], ys = [];
    const add = (p) => { const xy = toXY(p, null); if (xy) { xs.push(xy[0]); ys.push(xy[1]); } };
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
    fitGeoToData(geo);
    clear("keepInLayer"); clear("alarmLayer");
    clear("recordedRouteLayer"); clear("realtimeTrajectoryLayer"); clear("keypointLayer");
    const fences = geo.fences || {};
    if (fences.available) for (const fen of fences.items) {
      const pts = ptsAttr(fen.vertices || [], origin); if (!pts) continue;
      const type = fenceType(fen);            // active | forbid | alarm | speed
      const line = ((cfg.fence || {})[type] || (cfg.fence || {}).active || {}).line
                   || { style: "solid", color: "#e0b000", width_px: 1.6 };
      // Per-role polygon class carries the stroke colour (CSS var) + faint fill
      // tint: active blue / warning(alarm) bright red / forbid dark red /
      // speed_limit bright blue. Fill comes from the class, not an inline attr.
      const cls = { active: "keep-in", alarm: "alarm-region",
                    forbid: "forbid-region", speed: "speed-region" }[type] || "keep-in";
      const layer = type === "active" ? "keepInLayer" : "alarmLayer";
      $(layer).appendChild(poly(cls, pts, line));
      // Fence name label -- unified bright white (CSS .keep-in-label), NOT the
      // line colour (user 2026-08-18). No per-layer inline fill.
      if (fen.name && fen.vertices && fen.vertices[0]) {
        const xy = toXY(fen.vertices[0], origin);
        // req4d: name + fence perimeter in metres, e.g. "东油库报警区 (1627 m)".
        if (xy) $(layer).appendChild(label(xy[0], xy[1] - 2, "keep-in-label",
          fen.name + lenTag(perimM(fen.vertices))));
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
      // req1 (2026-08-18): a round dot at EVERY RTK point on the route, coloured
      // the route line. Constant screen size (updateGridAndDots), so visible even
      // on the ~5km fit view. The points stay un-labelled (route points have no
      // names); only keypoints carry a name.
      for (const p of (r.points || [])) {
        const pxy = toXY(p, origin);
        if (pxy) $(rlayer).appendChild(dot(pxy[0], pxy[1], rline.color));
      }
      // Route name label at its first vertex -- unified bright white (CSS).
      if (r.name && r.points && r.points[0]) {
        const rxy = toXY(r.points[0], origin);
        // req4d: name + route length in metres, e.g. "环湖主巡逻线 (4561 m)".
        if (rxy) $(rlayer).appendChild(label(rxy[0], rxy[1] - 2, "route-label",
          r.name + lenTag(segLen(r.points))));
      }
    }
    const wps = geo.waypoints || {};
    if (wps.available) for (const w of wps.items) {
      const xy = toXY(w.geom || w, origin); if (!xy) continue;
      const mk = (cfg.waypoint || {})[w.recorded ? "recorded" : "unrecorded"] || {};
      // req1: keypoint = a round dot (constant screen size) + its name in bright
      // white (CSS .keypoint-label). The old ".keypoint text" selector never
      // matched a <text class="keypoint">, hence the dedicated class.
      $("keypointLayer").appendChild(dot(xy[0], xy[1], mk.color));
      if (w.name) {
        $("keypointLayer").appendChild(
          label(xy[0], xy[1] - 2, "keypoint-label", w.name));
      }
    }
    updateGridAndDots();     // size the dots just drawn to a constant screen radius
    declutterLabels();       // req1: nudge/hide names so they never overlap
  }

  // req1 (2026-08-18): a geo dot -- a round <circle class="geo-dot"> whose radius
  // is set to a constant screen size by updateGridAndDots (placeholder r here).
  // Used for both route RTK points and keypoints so they never vanish on a big map.
  function dot(x, y, color) {
    const c = document.createElementNS(SVGNS, "circle");
    c.setAttribute("cx", x); c.setAttribute("cy", y);
    c.setAttribute("r", 1);
    c.setAttribute("class", "geo-dot");
    c.setAttribute("fill", color || "#3aa0ff");
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
    const a = pose.heading_valid ? (Math.PI / 2 - (pose.heading_rad || 0)) : 0, s = 9 * mpp;
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

  function taskCardHTML(card) {
    const b = taskBadge(card.state);
    // 字段3 内容 = command_text (语音/文本原文); 系统自发任务(返航/充电)无原文 ->
    // 用类型名兜底, 不留空白.
    const content = card.command_text || (card.task_type ? "(" + card.task_type + ")" : "--");
    const pct = card.progress ? card.progress.percent : null;
    // 字段5: percent 为 null(路由未展开)时显 '--', 🚫 不伪造 0/100 (同后端).
    const pctText = (pct == null) ? "--" : Math.round(pct) + "%";
    const pctW = (pct == null) ? 0 : Math.max(0, Math.min(100, pct));
    // 字段4 巡逻点: 走过=finished(绿点绿字) / 当前=current / 未到=pending(灰).
    // targets 现恒空(待关键点层 F06), 空则整段不渲染.
    const targets = (card.targets || []).map((t) => {
      const cls = t.status === "finished" ? "finished"
        : (t.status === "current" ? "current" : "pending");
      return `<li class="${cls}"><i></i><span>${esc(t.name || t.waypoint_id || "")}</span></li>`;
    }).join("");
    return (
      `<div class="task-card-head"><strong class="task-id">${esc(card.task_id || "")}</strong>` +
      `<span class="task-state ${b.cls}">${esc(b.label)}</span></div>` +
      `<div class="task-content">${esc(content)}</div>` +
      `<div class="task-time"><span>下发</span><time>${esc(fmtDispatchTime(card.created_at))}</time></div>` +
      (targets ? `<div class="task-targets-title">巡逻点</div><ol class="task-targets">${targets}</ol>` : "") +
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
      // History cards default collapsed (only id/content/time); a card the
      // operator opened stays expanded. Current cards are always full.
      if (isHistory)
        art.classList.add(expandedHistory.has(card.task_id) ? "expanded" : "collapsed");
      art.innerHTML = taskCardHTML(card);
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
      const card = e.target.closest(".task-card"); if (!card) return;
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
    // ★ WIDTH is SHARED by both panels (they stack on the HMI's right edge, so a
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
    // Footer status (2026-08-16): fix / heading / sync as coloured text, no dots.
    setStat("fixText", fixStat(pose));
    setStat("hdgText", hdgStat(pose));
    // sync (18-C G47): 授时同步 green / 授时未同步 red. Absent source -> unsynced.
    const synced = !!(clock && clock.available && clock.sync);
    setStat("syncText", synced ? { text: "授时同步", cls: "ok" }
                                : { text: "授时未同步", cls: "bad" });
    renderClock();
    $("precText").textContent = pose.cov_h_m != null ? `${pose.cov_h_m.toFixed(2)}m` : "--";
    // link + ESTOP arming (NAV-64): estop_path ok -> button enabled + pulse.
    const ok = status.estop_path === "ok";
    setStat("linkText", ok ? { text: "链路正常", cls: "ok" }
                           : { text: "急停链路断", cls: "bad" });
    const btn = $("estopBtn");
    btn.disabled = !ok;
    btn.classList.toggle("armed", ok);
    btn.title = ok ? "紧急停止" : "急停不可用(链路断)";
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
    const [bx, by, bw, bh] = baseVB;
    const w = bw / view.zoom, h = bh / view.zoom;
    const cx = bx + bw / 2 + view.pan.x, cy = by + bh / 2 + view.pan.y;
    const svg = $("mapSvg");
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
    ws.onopen = () => { stopPoll(); refreshTasks(); };  // push takes over; re-pull tasks (reconnect full-pull)
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
    wireInteraction(); applyView(); buildDialFace(); wireHistoryFolding();
    window.addEventListener("resize", applyView);   // req3: rescale map labels on resize
    wirePanelScroll(); wirePanelResize();
    try { applyUiConfig(await getJSON("/api/hmi/ui_config")); }
    catch (e) { console.error("ui_config load failed", e); return; }
    await tick();                                     // instant first paint (REST)
    refreshTasks();                                   // 17 S6.8.4 task panel (own poll)
    if (!taskTimer) taskTimer = setInterval(refreshTasks, 4000);
    connectWS();                                      // then live server push (W6)
  }
  init();
})();
