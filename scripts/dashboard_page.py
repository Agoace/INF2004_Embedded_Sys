"""Inline dashboard page: HTML, CSS and JavaScript in one string. No CDN, no files.

The page draws whatever panels the server sends. A panel with rows gets a table,
one with a table (columns and rows) gets a grid, one with items gets a list, and a
chart type with a renderer in CHART_RENDERERS gets a canvas. A new panel needs no
change here.
"""

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot dashboard</title>
<meta name="csrf" content="__CSRF__">
<meta name="instance" content="__INSTANCE__">
<style nonce="__NONCE__">
:root { color-scheme: dark; --bg:#14171c; --card:#1e232b; --text:#e6e9ee; --dim:#8b95a3;
  --ok:#3fb950; --warn:#d29922; --bad:#f85149; --line:#2d333d; }
body { margin:0; background:var(--bg); color:var(--text); font:14px/1.4 system-ui, sans-serif; }
header { display:flex; flex-wrap:wrap; gap:10px; align-items:center; padding:10px 14px;
  background:var(--card); border-bottom:1px solid var(--line); position:sticky; top:0;
  z-index:2; }
.pill { padding:2px 10px; border-radius:12px; background:#2d333d; }
.pill.ok { background:#1f4d2b; } .pill.bad { background:#6b1f1b; }
.pill.warn { background:#5c4410; }
#buttons { display:flex; flex-wrap:wrap; gap:8px; margin-left:auto; align-items:center; }
button { font:inherit; padding:6px 14px; border-radius:6px; border:1px solid var(--line);
  background:#2d333d; color:var(--text); cursor:pointer; }
button:disabled { opacity:.35; cursor:not-allowed; }
button.armed { background:var(--warn); color:#000; }
#estop { background:var(--bad); color:#fff; font-weight:700; font-size:18px; padding:10px 26px;
  border-color:#ff9087; }
#message { padding:6px 14px; min-height:20px; color:var(--dim); }
#banner { padding:10px 14px; background:var(--bad); color:#fff; font-weight:700; font-size:16px; }
#banner[hidden] { display:none; }
#restart, #cmderror { padding:10px 14px; background:var(--bad); color:#fff; font-weight:700;
  font-size:16px; border-top:2px solid #ff9087; }
#restart[hidden], #cmderror[hidden] { display:none; }
#grid { display:grid; grid-template-columns:minmax(0, 1.8fr) minmax(0, 1fr) minmax(0, 1fr);
  gap:16px; padding:16px; max-width:1600px; margin:auto; }
#panel-ultrasonic { grid-column:1; grid-row:1; }
#panel-line { grid-column:2; grid-row:1; }
#panel-wheels { grid-column:3; grid-row:1; }
#panel-path { grid-column:1; grid-row:2; }
#panel-summary { grid-column:2 / 4; grid-row:2; }
#panel-ultrasonic { order:1; } #panel-line { order:2; } #panel-wheels { order:3; }
#panel-path { order:4; } #panel-summary { order:5; }
#health { grid-column:1 / -1; min-width:0; order:6; }
#health summary { cursor:pointer; font-weight:700; padding:12px; }
#health-grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); gap:12px; }
#health-grid > .panel { order:2; } #health-grid > #panel-health { order:1; }
#diagnostics { display:flex; flex-wrap:wrap; gap:8px; padding:12px; }
#chain { display:flex; gap:6px; align-items:center; }
#chain span { padding:4px 6px; border-radius:4px; color:var(--dim); }
#chain .active { background:#1f5072; color:#b8e0ff; font-weight:700; }
.panel { min-width:0; }
.panel.paused { filter:grayscale(1); opacity:.6; }
@media(max-width:800px) {
  #grid { grid-template-columns:minmax(0,1fr); padding:10px; gap:10px; }
  #panel-ultrasonic, #panel-line, #panel-wheels, #panel-path, #panel-summary {
    grid-column:1; grid-row:auto; }
  header { gap:6px; padding:8px; } #buttons { width:100%; margin:0; gap:6px; }
  #buttons button { padding:8px; } #buttons #estop { margin-left:auto; padding:10px 20px; }
  #chain { font-size:12px; } #chain span { padding:2px; }
  #health-grid { grid-template-columns:minmax(0,1fr); }
}

.panel { background:var(--card); border:1px solid var(--line); border-radius:8px;
  padding:10px 12px; }
.panel.stale { opacity:.45; }
.panel h2 { margin:0 0 6px; font-size:15px; display:flex; justify-content:space-between; }
.age { color:var(--dim); font-weight:400; font-size:12px; }
.panel table { width:100%; border-collapse:collapse; }
.panel td { padding:1px 0; vertical-align:top; }
.panel td:first-child { color:var(--dim); width:55%; }
.panel table.rows td:first-child { width:auto; }
.panel table.rows th { text-align:left; color:var(--dim); font-weight:400; font-size:12px; }
.panel table.rows { table-layout:fixed; margin-top:6px; }
.panel table.rows td { padding:1px 4px 1px 0; font-size:12px; overflow-wrap:anywhere; }
.items { max-height:220px; overflow-y:auto; font-family:ui-monospace, monospace; font-size:12px; }
.note { color:var(--dim); font-style:italic; }
canvas { width:100%; height:120px; background:#10131a; border-radius:4px; margin-top:6px; }
#panel-ultrasonic canvas { height:350px; }
#panel-line canvas, #panel-wheels canvas { height:240px; }
#panel-path canvas { height:270px; }
.visual-summary { font-size:17px; font-weight:700; margin:8px 0; }
.legend { font-size:12px; color:var(--dim); margin-top:6px; }
@media(max-width:800px) { #panel-ultrasonic canvas { height:240px; } }
</style>
</head>
<body>
<header>
  <strong>Robot dashboard</strong>
  <span id="robot" class="pill">robot ?</span>
  <span id="mode" class="pill">mode ?</span>
  <nav id="chain" aria-label="Active module">
    <span id="module-1">LINE</span> &gt; <span id="module-3">SCAN</span> &gt;
    <span id="module-4">DETOUR</span> &gt; <span id="module-2">SEARCH</span>
  </nav>
  <div id="buttons">
    <button id="calibrate" data-command="calibrate">CAL</button>
    <button id="start" data-command="start">START</button>
    <button id="dump" data-command="dump">DUMP</button>
    <button id="estop" data-command="estop">ESTOP</button>
  </div>
</header>
<div id="restart" hidden></div>
<div id="cmderror" hidden></div>
<div id="banner" hidden></div>
<div id="message"></div>
<main id="grid">
  <section id="panel-path" class="panel"><h2><span class="title">RUN PATH</span><span class="age"></span></h2>
    <div class="note">Waiting for live run rows.</div><div class="visual-summary"></div>
    <canvas hidden></canvas><div class="legend"></div><table class="facts"></table>
    <table class="rows" hidden></table><div class="items"></div></section>
  <details id="health" class="panel"><summary>HEALTH · connection, timing and counters</summary>
    <div id="diagnostics"><span id="broker" class="pill">broker ?</span>
      <span id="telemetry" class="pill">telemetry ?</span>
      <span id="gaps" class="pill">row gaps 0</span><span id="server" class="pill">server ?</span></div>
    <div id="health-grid"></div>
  </details>
</main>
<script nonce="__NONCE__">
"use strict";
const csrf = document.querySelector('meta[name="csrf"]').content;
const instance = document.querySelector('meta[name="instance"]').content;
const armed = {};
let latest = null;
let restarted = false;
const ARM_MS = 4000;
const RELOAD_GUARD_MS = 5000;

function text(node, value) { node.textContent = String(value); }
function pill(id, label, level) {
  const node = document.getElementById(id);
  text(node, label);
  node.className = "pill " + level;
}
function fmtAge(ms) {
  if (ms === null || ms === undefined) return "no data";
  return ms >= 65535 ? "no data" : (ms < 1000 ? ms + " ms" : (ms / 1000).toFixed(1) + " s");
}

function drawBars(ctx, canvas, chart) {
  const w = canvas.width, h = canvas.height;
  const x = angle => (angle + 90) / 180 * (w - 20) + 10;
  chart.points.forEach(p => {
    const angle = p[0], mm = p[1], age = p[2];
    ctx.globalAlpha = age > 10000 ? 0.3 : 1;
    if (mm === null) { ctx.fillStyle = "#f85149"; ctx.fillRect(x(angle) - 2, h - 6, 4, 4); }
    else { const bar = Math.max(2, mm / chart.max_mm * (h - 10));
      ctx.fillStyle = "#58a6ff"; ctx.fillRect(x(angle) - 2, h - 2 - bar, 4, bar); }
  });
  ctx.globalAlpha = 1;
  return x;
}
// Positive servo angles are LEFT. Dots are measurements, not an obstacle outline.
function sonarPoint(angle, mm, cx, cy, scale) {
  const a = angle * Math.PI / 180;
  return [cx - Math.sin(a) * mm * scale, cy - Math.cos(a) * mm * scale];
}
function drawSonar(canvas, chart) {
  const ctx = setup(canvas), w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h - 28, scale = Math.min((w - 80) / 2, h - 55) / 511;
  const point = (a, mm) => sonarPoint(a, mm, cx, cy, scale);
  const line = (a, mm) => { const p = point(a, mm); ctx.lineTo(p[0], p[1]); };
  const arc = (radius, start, end) => {
    ctx.beginPath();
    for (let a = start; a <= end; a += 1) {
      const p = point(a, radius); if (a === start) ctx.moveTo(...p); else ctx.lineTo(...p);
    }
    ctx.stroke();
  };
  const wedge = (start, end, colour) => {
    ctx.fillStyle = colour; ctx.beginPath(); ctx.moveTo(cx, cy);
    for (let a = start; a <= end; a += 1) line(a, 511);
    ctx.closePath(); ctx.fill();
  };
  const profile = chart.profile;
  if (profile && profile.decision) wedge(profile.decision === 2 ? 0 : -30,
    profile.decision === 2 ? 30 : 0, "rgba(63,185,80,.16)");
  wedge(-30, 30, "rgba(88,166,255,.05)");
  ctx.strokeStyle = "#344151"; ctx.fillStyle = "#a3aebc";
  [128,256,384,511].forEach(mm => { arc(mm, -70, 70);
    const p = point(-65, mm); ctx.fillText(mm + " mm", p[0] + 4, p[1]); });
  [-30,0,30].forEach(a => {
    ctx.beginPath(); ctx.moveTo(cx,cy); line(a,511); ctx.stroke();
    const p = point(a,511); ctx.fillText((a > 0 ? "+" : "") + a + "°",p[0]-12,p[1]-7);
  });
  ctx.strokeStyle = "#e6aa45"; arc(258,-60,60);
  const stop = point(55,258); ctx.fillStyle = "#e6aa45";
  ctx.fillText("STOP 258 mm", stop[0]-75,stop[1]-5);
  wedge(chart.beam_deg - 20, chart.beam_deg + 20,"rgba(88,166,255,.09)");
  const near = chart.latest_mm !== null && chart.latest_mm < 258;
  ctx.strokeStyle = near ? "#ff625b" : "#70b9ff"; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(cx,cy); line(chart.beam_deg,511); ctx.stroke(); ctx.lineWidth=1;
  chart.recent.forEach(p => {
    if (p.mm === null && !p.no_return) return;
    const xy = point(p.angle, Math.min(511,p.mm === null ? 511 : p.mm));
    ctx.globalAlpha = Math.max(.12, 1 - p.age_ms / 10000);
    ctx.fillStyle = "#78c4ff"; ctx.strokeStyle = "#78c4ff";
    if (p.no_return) { ctx.strokeRect(xy[0]-4,xy[1]-4,8,8); }
    else { ctx.beginPath(); ctx.arc(...xy,3,0,Math.PI*2); ctx.fill(); }
  });
  ctx.globalAlpha=1;
  if (profile) profile.points.forEach(p => {
    const known = p.mm !== null || p.no_return;
    const xy = point(p.angle, Math.min(511,p.mm === null ? 511 : p.mm));
    ctx.strokeStyle = !known ? "#778391" : p.no_return || p.mm >= profile.free_threshold_mm ? "#59d878" : "#ff7066";
    ctx.lineWidth=2; ctx.beginPath(); ctx.arc(...xy,5,0,Math.PI*2); ctx.stroke();
    if (!known) { ctx.fillStyle="#a3aebc"; ctx.fillText("?",xy[0]-3,xy[1]-8); }
    if (p.no_return) ctx.strokeRect(xy[0]-4,xy[1]-4,8,8);
  });
  if (profile && profile.closest) {
    ctx.strokeStyle="#ffce67"; const p=point(profile.closest.angle,Math.min(511,profile.closest.mm));
    ctx.beginPath(); ctx.arc(...p,10,0,Math.PI*2); ctx.stroke();
    ctx.fillStyle="#ffce67"; ctx.fillText("nearest",p[0]+12,p[1]);
  }
  ctx.lineWidth=1; ctx.fillStyle="#d9e6f4"; ctx.fillRect(cx-10,cy-5,20,16);
  ctx.fillText("LEFT (+)",10,h-10); ctx.fillText("ROBOT · 0 mm",cx-40,h-7);
  ctx.fillText("RIGHT (−)",w-80,h-10);
}
function bounded(value, low, high) { return Math.max(low,Math.min(high,value)); }
function drawLine(canvas, chart) {
  const ctx=setup(canvas), w=canvas.width, h=canvas.height, f=chart.state;
  ctx.fillStyle="#a3aebc"; ctx.fillText("0 floor → 1000 line",12,20);
  [["L",f.norm_l],["C",f.norm_c]].forEach(([label,value],i) => {
    const y=45+i*50; ctx.fillStyle="#dce4ee";
    ctx.fillText(label + "  " + value,12,y);
    ctx.fillStyle="#2d3847"; ctx.fillRect(12,y+8,w-24,15);
    ctx.fillStyle="#7fc9ff"; ctx.fillRect(12,y+8,(w-24)*bounded(value,0,1000)/1000,15);
  });
  const y=h-57, x=18+(w-36)*(bounded(f.kr1_position,-100,100)+100)/200;
  ctx.strokeStyle="#748494"; ctx.beginPath(); ctx.moveTo(18,y); ctx.lineTo(w-18,y); ctx.stroke();
  ctx.fillStyle=f.line_valid ? "#59d878" : "#ff7066";
  ctx.beginPath(); ctx.arc(x,y,7,0,Math.PI*2); ctx.fill();
  ctx.fillStyle="#dce4ee"; ctx.fillText("position " + f.kr1_position +
    (f.line_valid ? " · valid" : " · NOT VALID"),12,h-23);
  ctx.fillStyle="#a3aebc"; ctx.fillText("−100",12,y-13); ctx.fillText("0",w/2-3,y-13);
  ctx.fillText("+100",w-45,y-13);
}
function drawDrive(canvas, chart) {
  const ctx=setup(canvas), w=canvas.width, h=canvas.height, f=chart.state;
  const scale=(w/2-24)/500;
  ctx.fillStyle="#a3aebc"; ctx.fillText("signed mm/s · 0 at centre",12,20);
  [["LEFT",f.kr1_asked_l,f.kr1_measured_l],["RIGHT",f.kr1_asked_r,f.kr1_measured_r]]
    .forEach(([label,asked,measured],i) => {
      const y=45+i*90; ctx.fillStyle="#dce4ee"; ctx.fillText(label,12,y);
      ctx.fillStyle="#2d3847"; ctx.fillRect(12,y+10,w-24,42);
      [[asked,"#7bbcff",y+13],[measured,"#59d878",y+33]].forEach(([value,colour,top]) => {
        const length=bounded(value,-500,500)*scale;
        ctx.fillStyle=colour; ctx.fillRect(Math.min(w/2,w/2+length),top,Math.abs(length),13);
      });
      ctx.strokeStyle="#91a1b4"; ctx.beginPath(); ctx.moveTo(w/2,y+8); ctx.lineTo(w/2,y+55);ctx.stroke();
      ctx.fillStyle="#dce4ee";ctx.fillText("asked " + asked + " / measured " + measured,12,y+72);
    });
  ctx.fillStyle="#a3aebc";ctx.fillText("blue asked · green measured · bars capped ±500",12,h-8);
}
function drawPath(canvas, chart) {
  const ctx=setup(canvas), w=canvas.width, h=canvas.height, points=chart.points;
  if (!points.length) { ctx.fillStyle="#a3aebc";ctx.fillText("Waiting for live run rows",12,24);return; }
  const xs=points.map(p=>p.x), ys=points.map(p=>p.y);
  const minX=Math.min(0,...xs), maxX=Math.max(0,...xs), minY=Math.min(0,...ys), maxY=Math.max(0,...ys);
  const scale=Math.min((w-60)/Math.max(200,maxX-minX),(h-65)/Math.max(200,maxY-minY));
  const centreX=(minX+maxX)/2, centreY=(minY+maxY)/2;
  const xy=p=>[w/2+(p.x-centreX)*scale,h/2-(p.y-centreY)*scale];
  ctx.fillStyle="#899bad";ctx.fillText("dead reckoning · forward ↑ · mm",12,18);
  ctx.strokeStyle="#354254";ctx.beginPath();ctx.moveTo(...xy({x:0,y:0}));
  const origin=xy({x:0,y:0});ctx.arc(...origin,4,0,Math.PI*2);ctx.stroke();
  for (let i=1;i<points.length;i++) {
    if (!points[i].connected) continue;
    const module=points[i-1].module;
    ctx.strokeStyle=({3:"#f5c55e",4:"#c49aff",5:"#ff8b79"})[module] || "#69c1ff";
    ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(...xy(points[i-1]));ctx.lineTo(...xy(points[i]));ctx.stroke();
  }
  points.forEach((p,i) => {
    if (![3,4,5].includes(p.module) || (i && points[i-1].module === p.module)) return;
    const at=xy(p); ctx.fillStyle=({3:"#f5c55e",4:"#c49aff",5:"#ff8b79"})[p.module];
    ctx.beginPath();ctx.arc(...at,p.module === 3 ? 7 : 4,0,Math.PI*2);ctx.fill();
    ctx.fillText(({3:"SCAN",4:"DETOUR",5:"REVERSE"})[p.module],
      at[0] + (p.module === 3 ? -40 : 8),at[1]-5);
  });
  const end=xy(points[points.length-1]);ctx.fillStyle="#ffffff";
  ctx.beginPath();ctx.arc(...end,4,0,Math.PI*2);ctx.fill();ctx.lineWidth=1;
  ctx.strokeStyle="#a3aebc";ctx.beginPath();ctx.moveTo(15,h-20);ctx.lineTo(15+100*scale,h-20);ctx.stroke();
  ctx.fillStyle="#a3aebc";ctx.fillText("100 mm",15,h-27);
}
const CHART_RENDERERS = {
  path: drawPath,
  sensors: drawLine,
  drive: drawDrive,
  fan: drawSonar,
  dial(canvas, chart) {
    // Heading dial: 0 is straight ahead at the top; left (anticlockwise) is positive.
    const ctx = setup(canvas), w = canvas.width, h = canvas.height;
    const cx = w / 2, cy = h / 2, radius = Math.min(w, h) / 2 - 14;
    ctx.strokeStyle = "#2d333d"; ctx.beginPath(); ctx.arc(cx, cy, radius, 0, 6.283);
    ctx.stroke();
    ctx.fillStyle = "#8b95a3";
    ctx.fillText("0", cx - 3, cy - radius - 3); ctx.fillText("L90", cx - radius - 24, cy + 4);
    ctx.fillText("180", cx - 9, cy + radius + 11); ctx.fillText("R90", cx + radius + 3, cy + 4);
    const clockwise = -chart.heading_deg * Math.PI / 180;
    ctx.strokeStyle = "#58a6ff"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + radius * Math.sin(clockwise), cy - radius * Math.cos(clockwise));
    ctx.stroke(); ctx.lineWidth = 1;
    ctx.fillStyle = "#e6e9ee";
    ctx.fillText(chart.turn_rate_dps.toFixed(1) + " deg/s", cx - 28, cy + 16);
  },
  obstacle(canvas, chart) {
    const ctx = setup(canvas), w = canvas.width, h = canvas.height;
    const x = drawBars(ctx, canvas, chart);
    ctx.strokeStyle = "#2d333d"; ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
    // Clear sectors start at 0 deg: right is negative, left is positive.
    const sector = (deg, sign) => {
      if (deg === 255) return;
      ctx.fillStyle = "rgba(63,185,80,0.18)";
      ctx.fillRect(Math.min(x(0), x(sign * deg)), 0, Math.abs(x(sign * deg) - x(0)), h);
    };
    sector(chart.right_clear_deg, -1); sector(chart.left_clear_deg, 1);
    if (chart.closest) {
      const cx = x(chart.closest.angle);
      const cy = h - 2 - chart.closest.mm / chart.max_mm * (h - 10);
      const radius = 6 * (window.devicePixelRatio || 1);
      ctx.strokeStyle = "#f0883e"; ctx.beginPath(); ctx.arc(cx, cy, radius, 0, 6.283);
      ctx.stroke();
    }
    ctx.fillStyle = "#8b95a3";
    ctx.fillText("right", 2, 10); ctx.fillText("0", w / 2 - 3, 10);
    ctx.fillText("left", w - 28, 10);
  },
  profile(canvas, chart) {
    const ctx = setup(canvas), w = canvas.width, h = canvas.height;
    ctx.strokeStyle = "#2d333d"; ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
    drawBars(ctx, canvas, chart);
    ctx.fillStyle = "#8b95a3";
    ctx.fillText("-90", 2, 10); ctx.fillText("0 deg", w / 2 - 12, 10);
    ctx.fillText("+90", w - 26, 10);
  },
  trend(canvas, chart) {
    const ctx = setup(canvas), w = canvas.width, h = canvas.height;
    ctx.strokeStyle = "#2d333d"; ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
    const values = chart.values;
    if (values.length < 2) return;
    ctx.strokeStyle = "#3fb950"; ctx.beginPath();
    values.forEach((v, i) => {
      const px = i / (values.length - 1) * (w - 2) + 1;
      const py = h - 2 - (v - chart.min) / (chart.max - chart.min) * (h - 4);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
  },
};
function setup(canvas) {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * ratio; canvas.height = canvas.clientHeight * ratio;
  const ctx = canvas.getContext("2d"); ctx.font = (11 * ratio) + "px sans-serif";
  ctx.scale(1, 1); ctx.lineWidth = ratio;
  return ctx;
}

function panelNode(panel) {
  let node = document.getElementById("panel-" + panel.id);
  if (!node) {
    node = document.createElement("div");
    node.id = "panel-" + panel.id; node.className = "panel";
    node.innerHTML = '<h2><span class="title"></span><span class="age"></span></h2>' +
      '<div class="note"></div><div class="visual-summary"></div>' +
      '<canvas hidden></canvas><div class="legend"></div><table class="facts"></table>' +
      '<table class="rows" hidden></table><div class="items"></div>';
    const primary = ["ultrasonic", "line", "wheels", "summary"].includes(panel.id);
    document.getElementById(primary ? "grid" : "health-grid").appendChild(node);
  }
  return node;
}
function renderGrid(grid, data) {
  grid.hidden = !data;
  grid.replaceChildren();
  if (!data) return;
  const head = grid.insertRow();
  data.columns.forEach(name => { const th = document.createElement("th");
    text(th, name); head.appendChild(th); });
  data.rows.forEach(row => {
    const tr = grid.insertRow();
    row.forEach(value => text(tr.insertCell(), value));
  });
}
function renderPanel(panel) {
  // Keep all original facts in HEALTH, with short readouts on the driving cards.
  const labels = {line:["Line valid", "Line board"], wheels:["PWM duty left / right"],
    summary:["Run ID", "Duration", "Distance (path length)", "Detours started / done / succeeded",
             "Line losses", "Stop reason"]};
  if (labels[panel.id]) {
    renderPanel({...panel, id:panel.id + "-details", title:panel.title + " · details", chart:null});
    panel = {...panel, rows:panel.rows.filter(row => labels[panel.id].includes(row.label))};
  }
  if (panel.id === "obstacle") panel = {...panel, chart:null};
  const node = panelNode(panel);
  node.classList.toggle("stale", panel.stale);
  node.classList.toggle("paused", Boolean(panel.paused));
  const titles = {ultrasonic:"ULTRASONIC", line:"LINE", wheels:"DRIVE", summary:"LAST RUN"};
  text(node.querySelector(".title"), titles[panel.id] || panel.title);
  text(node.querySelector(".age"), panel.id === "path" ? (panel.paused ? "idle (paused)" : "live rows")
       : panel.placeholder ? "placeholder"
       : panel.paused ? "idle (paused)" + (panel.age_ms === null ? "" : ", data " +
                                           fmtAge(panel.age_ms))
       : fmtAge(panel.age_ms) + (panel.stale && panel.age_ms !== null ? " - stale" : ""));
  text(node.querySelector(".note"), panel.note);
  const table = node.querySelector("table.facts");
  table.replaceChildren();
  panel.rows.forEach(row => {
    const tr = table.insertRow();
    text(tr.insertCell(), row.label); text(tr.insertCell(), row.value);
  });
  renderGrid(node.querySelector("table.rows"), panel.table);
  const canvas = node.querySelector("canvas");
  const draw = panel.chart && CHART_RENDERERS[panel.chart.type];
  canvas.hidden = !draw;
  // A collapsed HEALTH section has zero-sized canvases (a dial radius would be negative).
  if (draw && canvas.clientWidth > 0 && canvas.clientHeight > 0) draw(canvas, panel.chart);
  const list = node.querySelector(".items");
  list.replaceChildren();
  panel.items.forEach(item => {
    const line = document.createElement("div");
    text(line, (item.age_ms === null ? "" : fmtAge(item.age_ms) + " ago  ") + item.text);
    list.appendChild(line);
  });
}

function renderButtons(buttons) {
  Object.keys(buttons).forEach(name => {
    const info = buttons[name], node = document.getElementById(name);
    if (!node) return;
    node.disabled = restarted || !info.enabled;
    node.title = info.reason;
    if (!info.enabled) delete armed[name];
    const isArmed = armed[name] && armed[name] > Date.now();
    node.classList.toggle("armed", Boolean(isArmed));
    text(node, isArmed ? "CONFIRM " + name.toUpperCase() :
      name === "calibrate" ? "CAL" : name.toUpperCase());
  });
}
// A different server instance means the dashboard process restarted, so this
// page holds a stale CSRF token and every command would be refused.
function checkInstance(snapshot) {
  if (!snapshot.server_instance || snapshot.server_instance === instance) return false;
  restarted = true;
  Object.keys(armed).forEach(name => { delete armed[name]; });
  document.querySelectorAll("#buttons button").forEach(node => { node.disabled = true; });
  const node = document.getElementById("restart");
  node.hidden = false;
  let last = 0;
  try { last = Number(sessionStorage.getItem("dashboard-reload-at")) || 0; } catch (e) { last = 0; }
  if (Date.now() - last > RELOAD_GUARD_MS) {
    try { sessionStorage.setItem("dashboard-reload-at", String(Date.now())); } catch (e) { /* none */ }
    text(node, "DASHBOARD RESTARTED: reloading the page. Commands are disabled until it " +
      "reloads. Use GP21 to stop the robot.");
    location.reload();
  } else {
    text(node, "DASHBOARD RESTARTED: reload the page. Commands are disabled. " +
      "Use GP21 to stop the robot.");
  }
  return true;
}
function render(snapshot) {
  if (restarted || checkInstance(snapshot)) return;
  latest = snapshot;
  pill("broker", snapshot.broker.connected ? "broker connected" : "broker disconnected",
       snapshot.broker.connected ? "ok" : "bad");
  const level = snapshot.robot.level;
  const levelClass = {online: "ok", offline: "bad"}[level] || "warn";
  pill("robot", snapshot.robot.text, levelClass);
  const banner = document.getElementById("banner");
  banner.hidden = !(level === "offline" || level === "silent");
  text(banner, level === "offline"
    ? "ROBOT OFFLINE: the broker reports that the robot dropped. START, CALIBRATE and DUMP are " +
      "disabled. An ESTOP will probably not arrive: use GP21."
    : "ROBOT SILENT: no state data. START, CALIBRATE and DUMP are disabled. Use GP21 to stop.");
  const tele = snapshot.telemetry;
  const teleAge = tele.state_age_ms === null ? "" : ", state " + fmtAge(tele.state_age_ms);
  pill("telemetry", tele.mode === "idle" ? "idle, telemetry paused" + teleAge
       : tele.mode === "run" ? "telemetry full rate" + teleAge
       : "telemetry mode unknown (old firmware)" + teleAge, tele.mode === "idle" ? "warn" : "");
  const gaps = snapshot.gaps;
  pill("gaps", "row gaps " + gaps.total + " (lost " + gaps.lost_rows + ", skipped " +
       gaps.jumped_rows + ")", gaps.total ? "warn" : "ok");
  pill("server", "server live", "ok");
  const view = snapshot.view || {};
  const mode = view.mode === "RUNNING" ? "RUN" : view.mode || "mode ?";
  pill("mode", mode + (view.state_stale ? " · stale" : ""),
       view.state_stale ? "warn" : view.mode === "RUNNING" ? "ok" : "");
  [1,3,4,2].forEach(id => {
    const node = document.getElementById("module-" + id);
    node.className = !view.state_stale && (view.module === id || (id === 3 && view.module === 5))
      ? "active" : "";
  });
  renderButtons(snapshot.buttons);
  if (view.path) renderPanel({id:"path", title:"RUN PATH", age_ms:null, stale:false,
    paused:view.paused, rows:[], items:[], table:null, chart:{type:"path", ...view.path},
    note:"Measured wheel speeds · track 111 mm · resets each run · drifts with slip. " +
      "Blue LINE/SEARCH, gold SCAN, purple DETOUR, coral REVERSE. " +
      view.path.gaps + " gaps (travel unknown; no connecting line)" +
      (view.path.trimmed ? " · older points trimmed" : "")});
  snapshot.panels.forEach(panel => {
    if (panel.id === "ultrasonic" && view.sonar) {
      renderPanel({...panel, chart:{type:"fan", ...view.sonar}, paused:view.paused});
      const sonar = view.sonar, profile = sonar.profile;
      const decision = profile ? ({0:"NONE",1:"RIGHT →",2:"← LEFT"}[profile.decision]) : "awaiting scan";
      const observed = profile ? profile.points.filter(p => p.mm !== null || p.no_return).length : 0;
      text(document.getElementById("panel-ultrasonic").querySelector(".visual-summary"),
        (sonar.reversing ? "REVERSING · " : "") + "DECISION: " + decision + " · " +
        "attempt " + sonar.attempt + "/3" +
        (profile ? " · observed " + observed + "/13 · profile " + fmtAge(profile.age_ms) + " ago" : ""));
      text(document.getElementById("panel-ultrasonic").querySelector(".legend"),
        "Solid dots: recent echoes (fade over 10 s) · □ NO_RETURN at range edge · " +
        "rings: last profile samples, green free (NO_RETURN or ≥nearest+150 mm), red blocked, ? unknown · gold: nearest. " +
        "Distances >511 mm clipped. Beam cone ≈ ±20° (approximate, measured 2026-11-07). " +
        "Profile dots are received samples, not reconstructed geometry.");
    } else if ((panel.id === "line" || panel.id === "wheels") && view.state &&
               Object.keys(view.state).length) {
      renderPanel({...panel, chart:{type:panel.id === "line" ? "sensors" : "drive", state:view.state}});
    } else renderPanel(panel);
  });
  if (snapshot.last_command) {
    const c = snapshot.last_command;
    text(document.getElementById("message"), c.command + " #" + c.request_id + ": " + c.status);
  }
}

// Stays visible until a later command succeeds; render() never clears it.
function showCommandError(name, detail, reload) {
  const node = document.getElementById("cmderror");
  text(node, name.toUpperCase() + " NOT SENT: " + detail + "." +
    (reload ? " Reload the page." : "") + " Use GP21 to stop the robot.");
  node.hidden = false;
}
async function sendCommand(name) {
  if (restarted) { showCommandError(name, "dashboard server restarted", true); return; }
  const body = {command: name, confirm: Boolean(latest && latest.buttons[name].needs_confirm)};
  let response;
  try {
    response = await fetch("/api/command", {method: "POST",
      headers: {"Content-Type": "application/json", "X-CSRF-Token": csrf},
      body: JSON.stringify(body)});
  } catch (error) {
    showCommandError(name, "dashboard server unreachable", true);
    return;
  }
  let result = null;
  try { result = await response.json(); } catch (error) { result = null; }
  if (!response.ok || !result || !result.ok) {
    const reason = result && typeof result.error === "string" ? result.error : "no reply";
    // 409 is a robot-state refusal (confirm or not idle); reloading does not change it.
    showCommandError(name, reason + " (HTTP " + response.status + ")", response.status !== 409);
    return;
  }
  document.getElementById("cmderror").hidden = true;
  text(document.getElementById("message"), name.toUpperCase() +
    " sent (broker " + (result.puback ? "acknowledged" : "has not acknowledged yet") +
    "). " + result.note);
}
document.getElementById("buttons").addEventListener("click", event => {
  const node = event.target.closest("button");
  if (!node || node.disabled) return;
  const name = node.dataset.command;
  const info = latest && latest.buttons[name];
  if (info && info.needs_confirm) {
    if (!(armed[name] && armed[name] > Date.now())) {
      armed[name] = Date.now() + ARM_MS;
      renderButtons(latest.buttons);
      setTimeout(() => latest && renderButtons(latest.buttons), ARM_MS + 50);
      return;
    }
    delete armed[name];
  }
  sendCommand(name);
});

document.getElementById("health").addEventListener("toggle", () => { if (latest) render(latest); });

function poll() {
  fetch("/api/state").then(r => r.json()).then(render).catch(() => {
    pill("server", "server unreachable", "bad");
  });
}
let timer = null;
function start() {
  if (!window.EventSource) { timer = setInterval(poll, 500); return; }
  const source = new EventSource("/events");
  source.onmessage = message => { if (timer) { clearInterval(timer); timer = null; }
    render(JSON.parse(message.data)); };
  source.onerror = () => { pill("server", "server connection lost", "bad");
    if (!timer) timer = setInterval(poll, 1000); };
}
start();
</script>
</body>
</html>
"""
