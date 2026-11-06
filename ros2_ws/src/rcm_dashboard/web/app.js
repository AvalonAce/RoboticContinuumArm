// RCM dashboard client. Data in via SSE (/events), commands out via POST /api/<action>.
"use strict";
const $ = (id) => document.getElementById(id);
const COLORS = ["#4c8dff", "#f0a020", "#2fbf71", "#c86bfa", "#e5484d"];
const WINDOW_S = 30;

let info = {}, last = null, ids = [], plots = null, lastArmT = 0;
let buf = null;                       // plot ring buffers
let protocols = {};
let promptKey = null;
let lastNotice = null;           // highest notice counter already shown
let inflight = 0;

// ---------------------------------------------------------------- api
async function api(action, body = {}) {
  inflight++; syncButtons();
  try {
    const r = await fetch(`/api/${action}`, { method: "POST", headers: { "Content-Type": "application/json" },
                                              body: JSON.stringify(body) });
    const j = await r.json();
    if (!j.ok) toast(`${action}: ${j.message}`, true);
    return j;
  } catch (e) { toast(`${action}: ${e}`, true); return { ok: false }; }
  finally { inflight--; syncButtons(); }
}
// Start/Calibrate/Rest can't be double-fired, and are locked while a test runs
let testBusy = false;
function syncButtons() {
  const lock = inflight > 0 || testBusy;
  for (const id of ["b-start", "b-cal", "b-rest"]) $(id).disabled = lock;
  for (const id of ["b-send", "b-straight"]) $(id).disabled = testBusy;
}
function toast(msg, bad = false) {
  const d = document.createElement("div");
  d.textContent = msg; if (bad) d.className = "bad";
  $("toast").appendChild(d); setTimeout(() => d.remove(), bad ? 6000 : 2500);
}
function chip(el, text, cls) { el.textContent = text; el.className = "chip" + (cls ? " " + cls : ""); }

// ---------------------------------------------------------------- plots
function makePlots() {
  const w = () => Math.max(240, $("main").clientWidth - 58);
  const series = (label) => ids.map((i, k) => ({ label: `${label} ${i}`, stroke: COLORS[k % COLORS.length], width: 1.5 }));
  const opts = (s, h = 170) => ({ width: w(), height: h, series: [{}, ...s], legend: { live: false },
    axes: [{ stroke: "#8a93a0", grid: { stroke: "#262c35" }, values: (u, v) => { const d = v.length > 1 && Math.abs(v[1] - v[0]) < 1 ? 1 : 0; return v.map((x) => x.toFixed(d) + "s"); } },
           { stroke: "#8a93a0", grid: { stroke: "#262c35" } }],
    scales: { x: { time: false } } });
  buf = { t: [], load: ids.map(() => []), err: ids.map(() => []), th: [], thc: [] };
  plots = {
    load: new uPlot(opts(series("load")), [buf.t, ...buf.load], $("p-load")),
    err: new uPlot(opts(series("err")), [buf.t, ...buf.err], $("p-err")),
    theta: new uPlot(opts([{ label: "estimate", stroke: COLORS[0], width: 1.5 },
                           { label: "command", stroke: "#8a93a0", width: 1, dash: [4, 4] }]),
                     [buf.t, buf.th, buf.thc], $("p-theta")),
  };
  const fit = () => Object.values(plots).forEach((p) => p.setSize({ width: w(), height: p.height }));
  window.addEventListener("resize", fit);
  $("adv").addEventListener("toggle", fit);
}
const t0 = Date.now() / 1000;
function pushSample(d) {
  if (!plots) return;
  const t = d.arm.t - t0;
  buf.t.push(t);
  ids.forEach((i, k) => {
    const s = d.arm.servos.find((x) => x.id === i);
    buf.load[k].push(s && !s.error ? s.load : null);
    buf.err[k].push(s && s.goal >= 0 ? s.pos - s.goal : null);
  });
  buf.th.push(d.bend.theta_deg ?? null);
  buf.thc.push(Number.isFinite(d.bend.cmd_theta_deg) ? d.bend.cmd_theta_deg : null);
  while (buf.t.length && buf.t[0] < t - WINDOW_S) {
    buf.t.shift(); buf.load.forEach((a) => a.shift()); buf.err.forEach((a) => a.shift()); buf.th.shift(); buf.thc.shift();
  }
}
let redrawPending = false;
function redraw() {
  if (!plots || redrawPending) return;
  redrawPending = true;
  requestAnimationFrame(() => {
    redrawPending = false;
    plots.load.setData([buf.t, ...buf.load]);
    plots.err.setData([buf.t, ...buf.err]);
    plots.theta.setData([buf.t, buf.th, buf.thc]);
  });
}

// ---------------------------------------------------------------- render
function renderServos(arm) {
  const tb = $("servos").querySelector("tbody");
  tb.innerHTML = "";
  for (const [k, s] of arm.servos.entries()) {
    const err = s.goal >= 0 ? s.pos - s.goal : "";
    const tr = document.createElement("tr");
    tr.innerHTML = `<td><span class="dot" style="background:${COLORS[k % COLORS.length]}"></span>${s.id}</td>
      <td>${s.pos}</td><td>${s.goal >= 0 ? s.goal : "–"}</td><td>${err}</td><td>${s.load}</td>
      <td>${s.temperature}</td><td>${s.voltage.toFixed(1)}</td>
      <td class="${s.error ? "err" : ""}">${s.error ? "NO DATA" : s.moving ? "moving" : ""}</td>`;
    tb.appendChild(tr);
  }
}
function buildJog() {
  const j = $("jog"); j.innerHTML = "";
  for (const i of ids) {
    const r = document.createElement("div"); r.className = "row";
    r.innerHTML = `<span>#${i}</span>` + [-50, -10, 10, 50].map((d) =>
      `<button class="btn small" data-id="${i}" data-d="${d}">${d > 0 ? "+" : ""}${d}</button>`).join("");
    j.appendChild(r);
  }
  j.onclick = (e) => {
    const b = e.target.closest("button"); if (!b || !last) return;
    const i = +b.dataset.id, s = last.arm.servos.find((x) => x.id === i); if (!s) return;
    api("raw", { id: i, pos: (s.goal >= 0 ? s.goal : s.pos) + +b.dataset.d, speed: 150 });
  };
}
function renderTest(t) {
  if (!t || !t.state) return;
  const busy = ["running", "prompt"].includes(t.state);
  if (busy !== testBusy) { testBusy = busy; syncButtons(); }
  const cls = { running: "ok", prompt: "warn", done: "ok", aborted: "warn", tripped: "bad", error: "bad" }[t.state] || "";
  chip($("t-state"), t.state + (t.protocol ? ` · ${t.protocol}` : ""), cls);
  $("t-elapsed").textContent = ["running", "prompt"].includes(t.state) ? `${t.elapsed_s.toFixed(0)} s` : "";
  $("t-session").textContent = t.session_dir ? `→ ${t.session_dir}` : "";
  const log = $("t-log"), atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
  const text = (t.log || []).join("\n");
  if (log.textContent !== text) { log.textContent = text; if (atBottom) log.scrollTop = log.scrollHeight; }
  const p = t.prompt, box = $("prompt");
  const key = p ? p.text + "|" + p.kind : null;
  if (key !== promptKey) {
    promptKey = key;
    box.classList.toggle("hidden", !p);
    if (p) {
      $("prompt-text").textContent = p.text;
      const inp = $("prompt-in");
      inp.style.display = p.kind === "confirm" ? "none" : "";
      inp.type = p.kind === "number" ? "number" : "text";
      inp.value = p.default ?? "";
      (p.kind === "confirm" ? $("b-answer") : inp).focus();
      if (p.kind !== "confirm") inp.select();
    }
  }
}
function render(d) {
  last = d;
  if (d.info && Object.keys(d.info).length) {
    info = d.info;
    $("theta").max = info.theta_max_deg || 90;
    const rng = info.range;
    chip($("c-cal"), !info.calibrated ? "uncalibrated" : rng && !rng.ok ? `calibrated · reach ${rng.theta_reach_deg}°` : "calibrated",
         !info.calibrated ? "warn" : rng && !rng.ok ? "warn" : "ok");
    $("b-rest").title = `Move slowly to rest ${JSON.stringify(info.rest_counts || [])} and hold`;
  }
  const arm = d.arm || {};
  if (arm.servos) {
    const newIds = arm.servos.map((s) => s.id);
    if (newIds.join() !== ids.join()) { ids = newIds; makePlots(); buildJog(); }
    renderServos(arm);
    chip($("c-bridge"), arm.tripped ? `TRIPPED: ${arm.tripped}` : arm.torque ? "holding" : "limp",
         arm.tripped ? "bad" : arm.torque ? "ok" : "");
    $("b-rearm").classList.toggle("hidden", !arm.tripped);
    $("c-sim").classList.toggle("hidden", !arm.sim);
    const stale = d.now - arm.t > 1.0;
    chip($("c-link"), stale ? "no telemetry" : "live", stale ? "bad" : "ok");
    if (arm.t !== lastArmT) { lastArmT = arm.t; pushSample(d); redraw(); }
  }
  if (d.bend && d.bend.theta_deg !== undefined) {
    const b = d.bend, tip = b.tip.map((x) => (x * 1000).toFixed(1)).join(", ");
    $("estimate").textContent = `now θ ${b.theta_deg.toFixed(1)}°  φ ${b.phi_deg.toFixed(0)}°`;
    $("estimate").title = `tip [${tip}] mm, common-mode ${b.common_mm.toFixed(2)} mm`;
  }
  renderTest(d.test);
  for (const n of d.notices || []) {
    if (lastNotice !== null && n.n > lastNotice) toast(n.text, true);
  }
  if (d.notices && d.notices.length) lastNotice = Math.max(lastNotice ?? 0, d.notices[d.notices.length - 1].n);
  else if (lastNotice === null) lastNotice = 0;
}

// ---------------------------------------------------------------- SSE
function connect() {
  const es = new EventSource("/events");
  es.onmessage = (e) => render(JSON.parse(e.data));
  es.onerror = () => { chip($("c-link"), "dashboard offline", "bad"); };
}

// ---------------------------------------------------------------- controls
const bendBody = () => ({ theta_deg: +$("theta").value, phi_deg: +$("phi").value, speed: +$("speed").value });
function onSlider() {
  $("o-theta").textContent = $("theta").value + "°";
  $("o-phi").textContent = $("phi").value + "°";
}
$("theta").oninput = onSlider; $("phi").oninput = onSlider;
$("b-send").onclick = () => api("bend", bendBody());
$("b-straight").onclick = () => { $("theta").value = 0; onSlider(); api("bend", { ...bendBody(), theta_deg: 0 }); };
$("b-limp").onclick = () => { if (confirm("Torque off ALL servos? A lifted payload will drop.")) api("torque", { id: 0, enable: false }); };
$("b-estop").onclick = () => api("estop").then((j) => j.ok && toast("E-STOP: torque off"));
$("b-rearm").onclick = () => api("rearm").then((j) => j.ok && toast(j.message));
document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("b-estop").click(); });

async function loadProtocols() {
  const j = await api("test/list");
  if (!j.ok) return;
  protocols = j.protocols;
  const sel = $("proto"), cur = sel.value;
  sel.innerHTML = Object.keys(protocols).filter((k) => k !== "calibrate").map((k) => `<option>${k}</option>`).join("");
  if (cur && protocols[cur]) sel.value = cur;
  fillProto();
}
function fillProto() {
  const p = protocols[$("proto").value]; if (!p) return;
  $("proto-desc").textContent = p.error ? `YAML error: ${p.error}` : `${p.protocol}: ${p.description || ""}`;
  $("params").value = p.params_yaml || "";
}
$("proto").onchange = fillProto;
$("b-start").onclick = async () => {
  const j = await api("test/start", { protocol: $("proto").value, overrides: $("params").value, notes: $("notes").value });
  if (j.ok) toast(`started → ${j.session_dir}`);
};
$("b-rest").onclick = () => api("rest").then((j) => j.ok && toast(j.message));
$("b-cal").onclick = async () => {
  const p = protocols.calibrate;
  const j = await api("test/start", { protocol: "calibrate", overrides: p ? p.params_yaml : "", notes: $("notes").value });
  if (j.ok) toast("calibration started — follow the prompts");
};
$("b-abort").onclick = () => api("test/abort").then((j) => j.ok && toast(j.message));
function answer() {
  const p = last && last.test && last.test.prompt; if (!p) return;
  api("test/respond", { value: p.kind === "confirm" ? "ok" : $("prompt-in").value });
}
$("b-answer").onclick = answer;
$("prompt-in").addEventListener("keydown", (e) => { if (e.key === "Enter") answer(); });

onSlider();
connect();
setTimeout(loadProtocols, 800);
