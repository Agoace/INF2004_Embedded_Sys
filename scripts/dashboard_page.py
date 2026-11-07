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
#grid { display:grid; grid-template-columns:repeat(auto-fill, minmax(330px, 1fr)); gap:12px;
  padding:12px 14px; }
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
</style>
</head>
<body>
<header>
  <strong>Robot dashboard</strong>
  <span id="broker" class="pill">broker ?</span>
  <span id="robot" class="pill">robot ?</span>
  <span id="gaps" class="pill">row gaps 0</span>
  <span id="server" class="pill">server ?</span>
  <div id="buttons">
    <button id="dump" data-command="dump">DUMP</button>
    <button id="calibrate" data-command="calibrate">CALIBRATE</button>
    <button id="start" data-command="start">START</button>
    <button id="estop" data-command="estop">ESTOP</button>
  </div>
</header>
<div id="restart" hidden></div>
<div id="cmderror" hidden></div>
<div id="banner" hidden></div>
<div id="message"></div>
<div id="grid"></div>
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
const CHART_RENDERERS = {
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
      '<div class="note"></div><table class="facts"></table><canvas hidden></canvas>' +
      '<table class="rows" hidden></table><div class="items"></div>';
    document.getElementById("grid").appendChild(node);
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
  const node = panelNode(panel);
  node.classList.toggle("stale", panel.stale);
  text(node.querySelector(".title"), panel.title);
  text(node.querySelector(".age"), panel.placeholder ? "placeholder" : fmtAge(panel.age_ms) +
       (panel.stale && panel.age_ms !== null ? " - stale" : ""));
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
  if (draw) draw(canvas, panel.chart);
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
    text(node, isArmed ? "CONFIRM " + name.toUpperCase() : name.toUpperCase());
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
  const gaps = snapshot.gaps;
  pill("gaps", "row gaps " + gaps.total + " (lost " + gaps.lost_rows + ", skipped " +
       gaps.jumped_rows + ")", gaps.total ? "warn" : "ok");
  pill("server", "server live", "ok");
  renderButtons(snapshot.buttons);
  snapshot.panels.forEach(renderPanel);
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
