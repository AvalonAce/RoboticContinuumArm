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
let testBusy = false, armConnected = false;
function syncButtons() {
  const lock = inflight > 0 || testBusy || !armConnected;
  for (const id of ["b-start", "b-home", "b-rest", "b-mb", "b-cal", "b-fb", "b-gt"]) $(id).disabled = lock;
  for (const id of ["b-send", "b-straight"]) $(id).disabled = testBusy || !armConnected;
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

// ---------------------------------------------------------------- tip IMU
let imuPlot = null, ibuf = { t: [], tilt: [], roll: [], pitch: [] }, lastImuT = 0;
function makeImuPlot() {
  const w = () => Math.max(240, $("main").clientWidth - 58);
  imuPlot = new uPlot({ width: w(), height: 130, legend: { live: false }, scales: { x: { time: false } },
    series: [{}, { label: "tilt from ref", stroke: COLORS[0], width: 1.8 },
                 { label: "roll", stroke: COLORS[1], width: 1 }, { label: "pitch", stroke: COLORS[2], width: 1 }],
    axes: [{ stroke: "#8a93a0", grid: { stroke: "#262c35" }, values: (u, v) => v.map((x) => x.toFixed(0) + "s") },
           { stroke: "#8a93a0", grid: { stroke: "#262c35" }, values: (u, v) => v.map((x) => x + "°") }] },
    [ibuf.t, ibuf.tilt, ibuf.roll, ibuf.pitch], $("p-imu"));
  window.addEventListener("resize", () => imuPlot.setSize({ width: w(), height: 130 }));
}
// quaternion (sensor->world, world z up) -> CSS matrix3d. CSS axes: x right, y DOWN, z toward viewer.
// C maps world -> CSS: x->x, z(up)->-y, y->-z. M = C R C (C is its own inverse).
function quatToCss([w, x, y, z]) {
  const R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
             [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
             [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
  const C = [[1, 0, 0], [0, 0, -1], [0, -1, 0]];
  const mul = (A, B) => A.map((r) => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
  const M = mul(mul(C, R), C);
  return `matrix3d(${M[0][0]},${M[1][0]},${M[2][0]},0,${M[0][1]},${M[1][1]},${M[2][1]},0,` +
         `${M[0][2]},${M[1][2]},${M[2][2]},0,0,0,0,1)`;
}
const fmt = (v, d = 1) => (v === null || v === undefined ? "–" : v.toFixed(d));
function renderImu(m, now) {
  const stale = !m || !m.t || now - m.t > 1.0;
  const st = stale ? "offline" : m.state;
  const cls = { ok: "ok", calibrating: "warn", offline: "" }[st] ?? "";
  chip($("c-imu"), st === "ok" ? `IMU ${m.rate.toFixed(0)} Hz` : `IMU ${st}`, cls === "" ? (stale ? "bad" : "") : cls);
  $("imu-msg").textContent = stale ? "no data — sensor board plugged in? sensor_bridge running?" :
    st === "calibrating" ? m.message : (m.has_ref ? "showing rotation since reference" : "set a reference to zero the view");
  $("b-imu-ref").disabled = st !== "ok";
  if (stale) return;
  if (st === "ok") $("imu-body").style.transform = quatToCss(m.has_ref ? m.q_rel : m.q);
  $("imu-tilt").textContent = m.tilt === null ? "–" : `${m.tilt.toFixed(1)}°`;
  $("imu-roll").textContent = fmt(m.roll); $("imu-pitch").textContent = fmt(m.pitch);
  $("imu-extra").textContent = `|a| ${Math.hypot(...m.accel).toFixed(2)} m/s²  ω [${m.gyro_dps.map((g) => g.toFixed(1)).join(" ")}] °/s  ` +
    `${fmt(m.temp)} °C${m.bad ? `  · ${m.bad} bad lines` : ""}`;
  if (!imuPlot) makeImuPlot();
  if (m.t !== lastImuT && st === "ok") {
    lastImuT = m.t;
    const t = m.t - t0;
    ibuf.t.push(t); ibuf.tilt.push(m.tilt); ibuf.roll.push(m.roll); ibuf.pitch.push(m.pitch);
    while (ibuf.t.length && ibuf.t[0] < t - WINDOW_S) { ibuf.t.shift(); ibuf.tilt.shift(); ibuf.roll.shift(); ibuf.pitch.shift(); }
    imuPlot.setData([ibuf.t, ibuf.tilt, ibuf.roll, ibuf.pitch]);
  }
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
// ---------------------------------------------------------------- protocol live plot (x-y, multi-series)
let liveKey = "";
function drawLive(L) {
  const box = $("t-live");
  box.classList.toggle("hidden", !L);
  if (!L) { liveKey = ""; return; }
  const key = JSON.stringify(L);
  if (key === liveKey || $("t-live-c").clientWidth === 0) return;   // unchanged, or tab hidden (draw later)
  liveKey = key;
  $("t-live-title").textContent = L.title || "live";
  $("t-live-sum").textContent = L.summary || "";
  const hasSeries = Object.keys(L.series || {}).length > 0;
  $("t-live-c").style.display = hasSeries ? "" : "none";      // e.g. grasp trials: summary only, no curve
  if (!hasSeries) return;
  const c = $("t-live-c"), dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
  c.width = W * dpr; c.height = H * dpr;
  const g = c.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const ser = Object.entries(L.series || {});
  const pts = ser.flatMap(([, v]) => v);
  let x1 = Math.max(1, ...pts.map((p) => p[0])), y1 = Math.max(L.target || 0, 5, ...pts.map((p) => p[1]));
  const y0 = Math.min(0, ...pts.map((p) => p[1]));
  y1 *= 1.05;
  const m = { l: 42, r: 26, t: 10, b: 30 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
  const X = (x) => m.l + (x / x1) * pw, Y = (y) => m.t + ph - ((y - y0) / (y1 - y0)) * ph;
  g.font = "11px system-ui"; g.fillStyle = "#8a93a0"; g.strokeStyle = "#262c35"; g.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const yv = y0 + (i / 4) * (y1 - y0), xv = (i / 4) * x1;
    g.beginPath(); g.moveTo(m.l, Y(yv)); g.lineTo(W - m.r, Y(yv)); g.stroke();
    g.fillText(yv.toFixed(0) + "°", 4, Y(yv) + 4);
    g.fillText(xv.toFixed(0), X(xv) - 10, H - 14);
  }
  g.fillText(L.x_label || "", m.l, H - 2);
  if (L.target) {
    g.strokeStyle = "#e5484d"; g.setLineDash([5, 4]);
    g.beginPath(); g.moveTo(m.l, Y(L.target)); g.lineTo(W - m.r, Y(L.target)); g.stroke(); g.setLineDash([]);
    g.fillStyle = "#e5484d"; g.fillText(`target ${L.target}°`, W - m.r - 70, Y(L.target) - 4);
  }
  ser.forEach(([name, v], si) => {
    const col = COLORS[si % COLORS.length];
    g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 1.8;
    g.beginPath(); v.forEach((p, i) => (i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1])))); g.stroke();
    v.forEach((p) => { g.beginPath(); g.arc(X(p[0]), Y(p[1]), 2.2, 0, 6.3); g.fill(); });
    g.fillText(name, m.l + 8 + si * 70, m.t + 12);
  });
}

function renderTest(t) {
  if (!t || !t.state) return;
  renderFinal(t);
  renderGrasp(t);
  renderHold(t);
  drawLive(t.live);
  const busy = ["running", "prompt"].includes(t.state);
  if (busy !== testBusy) { testBusy = busy; syncButtons(); }
  $("tab-dot").classList.toggle("hidden", !busy);
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
    $("c-prompt").classList.toggle("hidden", !p);
    if (p) {
      if (!["final", "manip"].includes(curTab())) showTab("tests");
      box.scrollIntoView({ behavior: "smooth", block: "center" });     // bring the question on screen
      box.classList.remove("flash"); void box.offsetWidth; box.classList.add("flash");
      toast(`Test is waiting for you: ${p.text.split("\n")[0]}`);
      $("prompt-text").textContent = p.text;
      const ch = $("prompt-choices"), hasCh = Array.isArray(p.choices) && p.choices.length;
      ch.classList.toggle("hidden", !hasCh);
      $("prompt-row").classList.toggle("hidden", !!hasCh);
      ch.innerHTML = hasCh ? p.choices.map((c, k) =>
        `<button class="btn ${k === 0 ? "primary" : ""} choice" data-v="${c}">${c.toUpperCase()}</button>`).join("") : "";
      ch.querySelectorAll("button").forEach((b) => (b.onclick = () => api("test/respond", { value: b.dataset.v })));
      if (hasCh) { ch.querySelector("button").focus(); return; }
      const inp = $("prompt-in");
      inp.style.display = p.kind === "confirm" ? "none" : "";
      inp.type = p.kind === "number" ? "number" : "text";
      inp.value = p.default ?? "";
      (p.kind === "confirm" ? $("b-answer") : inp).focus();
      if (p.kind !== "confirm") inp.select();
    }
  }
}
// ---------------------------------------------------------------- flex sensors
const FLEX_MAX = 120;                                      // bar full scale (deg); tick at 45 / 90
function renderFlex(f, now) {
  const stale = !f || !f.t || now - f.t > 1.5;
  const st = stale ? "offline" : f.state;
  chip($("c-flex"), st === "ok" || st === "sim" ? `flex ${f.rate} Hz` : `flex ${st}`, st === "ok" || st === "sim" ? "ok" : "");
  $("flex-msg").textContent = stale ? "no data — flex board plugged in?" : (f.message || "");
  const sens = stale ? [] : f.sensors || [];
  const box = $("flex-rows");
  if (box.childElementCount !== sens.length) {
    box.innerHTML = sens.map((_, k) => `<div class="flex-row"><div class="nm" id="fx-n${k}"></div>
      <div class="ang" id="fx-a${k}"></div><div class="flex-bar"><i id="fx-b${k}"></i>
      <b style="left:${(45 / FLEX_MAX) * 100}%"></b><b style="left:${(90 / FLEX_MAX) * 100}%"></b></div></div>`).join("");
  }
  let sum = 0, nsum = 0;
  sens.forEach((x, k) => {
    const span = x.from !== null && x.to !== null ? ` · ${x.from}–${x.to} cm` : "";
    const nc = x.v !== null && x.v < 0.15;
    $(`fx-n${k}`).innerHTML = `${x.name}<small>${nc ? "not connected" : `${x.v === null ? "–" : x.v.toFixed(2)} V · ${x.r_k === null ? "–" : x.r_k.toFixed(1)} kΩ`}${span}</small>`;
    const a = $(`fx-a${k}`);
    a.textContent = x.angle === null ? "–" : `${x.angle.toFixed(0)}°`;
    a.classList.toggle("est", x.angle !== null && !x.cal);
    a.title = x.cal ? "calibrated (flat + 90°)" : "estimate: 90° point not calibrated yet";
    $(`fx-b${k}`).style.width = `${Math.max(0, Math.min(1, (x.angle || 0) / FLEX_MAX)) * 100}%`;
    if (x.angle !== null) { sum += x.angle; nsum++; }
  });
  const imuT = last && last.imu && last.imu.tilt;
  const cov = !stale && f.length_cm ? ` (${f.covered_cm}/${f.length_cm} cm)` : "";
  $("flex-sum").textContent = (!stale && f.total !== null && nsum ? `whole arm ${f.total.toFixed(0)}° flex${cov}` : "") +
    (imuT !== null && imuT !== undefined ? ` · IMU ${imuT.toFixed(0)}°` : "");
}

let clockSkew = 0;                       // browser clock - server clock (s)
function render(d) {
  if (d.now) clockSkew = Date.now() / 1000 - d.now;
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
    if (arm.connected !== armConnected) { armConnected = !!arm.connected; syncButtons(); }
    if (!arm.connected) chip($("c-bridge"), arm.tripped ? `servo link lost: ${arm.tripped}` : "servo board not connected", "warn");
    else chip($("c-bridge"), arm.tripped ? `TRIPPED: ${arm.tripped}` : arm.torque ? "servos holding" : "servos limp",
              arm.tripped ? "bad" : arm.torque ? "ok" : "");
    $("b-rearm").classList.toggle("hidden", !(arm.tripped && arm.connected));
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
  renderImu(d.imu, d.now);
  renderFlex(d.flex, d.now);
  renderCal(d.arm);
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
  sel.innerHTML = Object.keys(protocols).filter((k) => !["calibrate", "max_bend", "final_bend", "grasp_trials"].includes(k)).map((k) => `<option>${k}</option>`).join("");
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
$("b-imu-ref").onclick = () => api("imu/set_reference").then((j) => j.ok && toast(j.message));
$("b-imu-cal").onclick = () => api("imu/calibrate_gyro").then((j) => j.ok && toast(j.message));
$("b-rest").onclick = () => { calReset(); api("rest").then((j) => j.ok && toast(j.message)); };
$("b-home").onclick = async () => {
  if (!confirm("Save the CURRENT motor positions as home?\n\nEach servo's encoder is re-centred here (no motion) and the arm holds this pose. Make sure the tendons are tensioned and the arm is straight.")) return;
  const j = await api("set_home");
  calReset();
  if (j.ok) toast(j.message);
};
$("b-mb").onclick = async () => {
  const v = (id) => $(id).value;
  const ov = `tendon: ${v("mb-tendon")}\ntarget_deg: ${v("mb-target")}\nstep_counts: ${v("mb-step")}\n` +
             `release_ratio: ${v("mb-ratio")}\nload_limit: ${v("mb-load")}`;
  const j = await api("test/start", { protocol: "max_bend", overrides: ov, notes: $("notes").value });
  if (j.ok) toast(`started max bend (tendon ${v("mb-tendon")})`);
};
$("b-abort").onclick = () => api("test/abort").then((j) => j.ok && toast(j.message));
function answer() {
  const p = last && last.test && last.test.prompt; if (!p) return;
  api("test/respond", { value: p.kind === "confirm" ? "ok" : $("prompt-in").value });
}
$("b-answer").onclick = answer;
$("c-prompt").onclick = () => {
  if (!["final", "manip"].includes(curTab())) showTab("tests");
  $("prompt").scrollIntoView({ behavior: "smooth", block: "center" });
};
$("prompt-in").addEventListener("keydown", (e) => { if (e.key === "Enter") answer(); });

onSlider();
connect();
setTimeout(loadProtocols, 800);

// ---------------------------------------------------------------- tabs
function showTab(name) {
  document.querySelectorAll(".tab").forEach((b) => b.classList.toggle("active", b.dataset.tab === name));
  document.querySelectorAll("#main > [data-tab]").forEach((el) =>
    el.classList.toggle("hidden", !el.dataset.tab.split(" ").includes(name)));
  try { localStorage.setItem("rcm-tab", name); } catch (e) { /* storage unavailable */ }
  liveKey = "";                                   // redraw the protocol plot at its visible size
  window.dispatchEvent(new Event("resize"));      // plots size to the visible column
}
document.querySelectorAll(".tab").forEach((b) => (b.onclick = () => showTab(b.dataset.tab)));
try { showTab(localStorage.getItem("rcm-tab") || "control"); } catch (e) { showTab("control"); }
$("b-flex-flat").onclick = () => api("flex/set_flat").then((j) => j.ok && toast(j.message));
$("b-flex-90").onclick = () => api("flex/set_90").then((j) => toast(j.message, !j.ok));

// ---------------------------------------------------------------- calibrate: pull / release per tendon
const calTarget = {};                 // id -> last commanded count, so fast clicks accumulate
function calReset() { for (const k in calTarget) delete calTarget[k]; }
$("b-cal").onclick = () => {
  const p = $("cal-panel"), open = p.classList.toggle("hidden") === false;
  $("b-cal").textContent = open ? "Calibrate ▴" : "Calibrate ▾";
  calReset();
};
function calNudge(id, dir) {                       // dir +1 = pull (reel in), -1 = release
  const sv = (last && last.arm && last.arm.servos || []).find((x) => x.id === id);
  if (!sv) return toast("no servo data", true);
  const ks = info.ids || ids, k = ks.indexOf(id);
  const sign = ((info.reel_in_sign || [])[k]) || 1;
  const base = calTarget[id] !== undefined ? calTarget[id] : (sv.goal >= 0 ? sv.goal : sv.pos);
  const tgt = Math.round(base + dir * sign * Number($("cal-step").value));
  calTarget[id] = tgt;
  api("raw", { id, pos: tgt, speed: 150 });
}
function renderCal(arm) {
  if ($("cal-panel").classList.contains("hidden")) return;
  const sv = (arm && arm.servos) || [];
  const box = $("cal-rows");
  if (box.childElementCount !== sv.length) {
    box.innerHTML = sv.map((x) => `<div class="cal-row"><b>T${x.id}</b><span class="v" id="cal-v${x.id}"></span>
      <button class="btn small" data-id="${x.id}" data-dir="-1">− Release</button>
      <button class="btn small" data-id="${x.id}" data-dir="1">+ Pull</button></div>`).join("");
    box.querySelectorAll("button").forEach((b) => (b.onclick = () => calNudge(Number(b.dataset.id), Number(b.dataset.dir))));
  }
  const ks = info.ids || ids, mmpc = info.mm_per_count || 0.0169;
  sv.forEach((x) => {
    const k = ks.indexOf(x.id), z = ((info.zero_counts || [])[k]) ?? 2048, sign = ((info.reel_in_sign || [])[k]) || 1;
    const d = (x.pos - z) * sign;                  // + = reeled in past home
    const el = $(`cal-v${x.id}`);
    el.textContent = `pos ${x.pos} · ${d >= 0 ? "+" : ""}${d} from home (${Math.abs(d * mmpc).toFixed(2)} mm ${d >= 0 ? "in" : "out"})`;
    el.classList.toggle("off", Math.abs(d) > 5);
  });
}

// ---------------------------------------------------------------- final tests
function curTab() { const a = document.querySelector(".tab.active"); return a ? a.dataset.tab : "control"; }
$("b-fb").onclick = async () => {
  const tg = $("fb-targets").value.split(/[ ,;]+/).map(Number).filter((x) => x > 0 && x <= 150);
  if (!tg.length) return toast("enter target angles, e.g. 30, 60, 75, 90", true);
  const lab = $("fb-label").value.replace(/"/g, "'");
  const ov = `targets: [${tg.join(", ")}]\ntrials: ${Number($("fb-trials").value) || 1}\ntendon: ${$("fb-tendon").value}\n` +
             `hold_s: ${Number($("fb-hold").value) || 5}\nimu_offset_deg: ${Number($("fb-off").value) || 0}\n` +
             `imu_scale: ${Number($("fb-scale").value) || 1}\nrelease_ratio: ${$("fb-ratio").value}\nlabel: "${lab}"`;
  const j = await api("test/start", { protocol: "final_bend", overrides: ov, notes: lab });
  if (j.ok) { toast(`final test started (${tg.length} targets × ${$("fb-trials").value})`); $("fb-table").tBodies[0].innerHTML = ""; }
};
let fbKey = "";
function renderFinal(t) {
  if (!t || t.protocol !== "final_bend") return;
  $("fb-dir").textContent = t.session_dir ? t.session_dir.split("/").slice(-1)[0] : "";
  const rows = (t.live && t.live.table) || [];
  const key = JSON.stringify(rows);
  if (key === fbKey) return;
  fbKey = key;
  const f = (x, d = 1) => (x === null || x === undefined ? "–" : Number(x).toFixed(d));
  $("fb-table").tBodies[0].innerHTML = rows.map((r) => `<tr><td>${r.bend}</td><td>${r.trial}</td><td>${f(r.target_deg, 0)}°</td>
    <td>${f(r.tilt_deg)}°</td><td class="${Math.abs(r.error_deg || 0) > 2 ? "err" : ""}">${f(r.error_deg, 2)}°</td>
    <td>${r.counts}</td><td>${f(r.mm, 2)}</td><td>${f(r.flex_total_deg)}°</td><td>${f(r.residual_after_return_deg)}°</td>
    <td class="small">${r.stop === "target reached" ? "ok" : r.stop}</td></tr>`).join("");
}

// ---------------------------------------------------------------- hold indicator (final tests)
let holdInfo = null;
function renderHold(t) {
  holdInfo = t && t.live && t.live.hold && (t.state === "running" || t.state === "prompt") ? t.live.hold : null;
  tickHold();
}
function tickHold() {
  const box = $("t-hold");
  const now = Date.now() / 1000 - clockSkew;
  const left = holdInfo ? holdInfo.until - now : -1;
  if (!holdInfo || left <= -0.5) { box.classList.add("hidden"); return; }
  box.classList.remove("hidden");
  $("t-hold-txt").textContent = typeof holdInfo.target === "number"
    ? ` target ${holdInfo.target}° · at ${holdInfo.tilt}° — averaging, keep clear`
    : ` ${holdInfo.target} — keep holding, click DROPPED / SLIPPED only if it happens`;
  $("t-hold-left").textContent = `${Math.max(0, left).toFixed(1)} s`;
  $("t-hold-bar").style.width = `${Math.min(100, Math.max(0, 100 * (1 - left / holdInfo.s)))}%`;
}
setInterval(tickHold, 100);

// ---------------------------------------------------------------- manipulator: grasp trials
const OBJECTS = [   // 3D-printed shape set (Shapes.3mf): name, size mm
  ["Dodecahedron", "49 x 48 x 40"], ["Polyhedron (icosahedron-like)", "43 x 49 x 40"], ["Rectangular block", "30 x 50 x 30"],
  ["Cube", "30 x 30 x 30"], ["Tetrahedron", "42 x 49 x 40"], ["Cone", "D30 x 40"], ["Cone B (short)", "D30 x 20"],
  ["Sphere", "D30"], ["Octahedron", "28 x 33 x 23"], ["Cylinder", "D26 x 30"], ["Polygon plate 1", "41 x 42 x 10"],
  ["Polygon plate 2", "45 x 28 x 10"], ["Polygon plate 3", "41 x 40 x 10"], ["Polygon plate 4", "43 x 37 x 10"],
  ["Triangle plate", "35 x 40 x 10"], ["Half cube", "30 x 30 x 15"], ["Custom...", ""]];
(function initObjects() {
  const sel = $("gt-obj");
  sel.innerHTML = OBJECTS.map(([n], k) => `<option value="${k}">${n}</option>`).join("");
  const sync = () => { const [n, sz] = OBJECTS[Number(sel.value)]; $("gt-size").value = sz;
    $("gt-name").disabled = n !== "Custom..."; };
  sel.onchange = sync; sync();
})();
$("b-gt").onclick = async () => {
  const [n] = OBJECTS[Number($("gt-obj").value)];
  const name = (n === "Custom..." ? $("gt-name").value : n).trim().replace(/"/g, "'");
  if (!name) return toast("enter the custom object name", true);
  const q = (x) => String(x).replace(/"/g, "'");
  const ov = `object: "${name}"\nobject_size_mm: "${q($("gt-size").value)}"\nobject_mass_g: ${Number($("gt-mass").value) || 0}\n` +
             `trials: ${Number($("gt-trials").value) || 5}\nhold_s: ${Number($("gt-hold").value) || 10}\nlabel: "${q($("gt-label").value)}"`;
  const j = await api("test/start", { protocol: "grasp_trials", overrides: ov, notes: `${name} ${$("gt-label").value}` });
  if (j.ok) { toast(`grasp trials started: ${name}`); $("gt-table").tBodies[0].innerHTML = ""; }
};
let gtKey = "";
function renderGrasp(t) {
  if (!t || t.protocol !== "grasp_trials") return;
  $("gt-dir").textContent = t.session_dir ? "· " + t.session_dir.split("/").slice(-1)[0] : "";
  const rows = (t.live && t.live.table) || [];
  const key = JSON.stringify(rows);
  if (key === gtKey) return;
  gtKey = key;
  const done = rows.filter((r) => r.outcome !== "skipped"), ok = done.filter((r) => r.success).length;
  $("gt-tally").textContent = done.length ? `${ok}/${done.length} successful` : "";
  const f = (x, d = 1) => (x === null || x === undefined ? "–" : Number(x).toFixed(d));
  $("gt-table").tBodies[0].innerHTML = rows.map((r) => `<tr><td>${r.trial}</td>
    <td class="${r.success ? "" : "err"}">${r.outcome}</td><td>${f(r.held_s)}</td><td>${f(r.time_to_grasp_s)}</td>
    <td>${r.slipped ? "yes" : ""}</td><td>${f(r.droop_during_hold_deg, 2)}</td><td>${f(r.supply_v)}</td></tr>`).join("");
}
