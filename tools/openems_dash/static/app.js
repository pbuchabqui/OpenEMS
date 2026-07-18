/* OpenEMS Calibration Cockpit — telemetria WS 30Hz + editores de tabela */
"use strict";

/* Pure helpers from js/helpers.js (dual-export; unit-tested). */
const H = (typeof OpenEMSHelpers !== "undefined") ? OpenEMSHelpers : null;
if (!H) console.error("OpenEMSHelpers missing — load js/helpers.js before app.js");

/* ── Multi-cell selection state ──────────────────────────────────────────── */
let selCells = new Set();      // Set of "r,c" keys
let selWeights = new Map();    // "r,c" → peso 0-1 (1.0 = selecção manual; <1 = trace bilinear)
let selPage = 0;               // page of current selection
let selPane = null;            // pane element
let selAnchor = null;          // [r,c] da célula "actual" — origem da navegação por setas
let selFocus = null;           // extremo móvel do rectângulo (Shift+setas)
let dragSel = null;            // {pane, page, from:[r,c]} durante clique-e-arraste
let dragMoved = false;         // suprime o click que dispara após um arraste

// Pilha de undo por página: snapshot dos valores antes de cada operação
// mutante (A/Z, H/V, edição de célula). Tecla X restaura o último.
function pushUndo(st) {
  if (!st.values) return;
  if (!st.undo) st.undo = [];
  st.undo.push(st.values.map(row => row.slice()));
  if (st.undo.length > 50) st.undo.shift();
}

function applyUndo(st, page) {
  if (!st.undo || !st.undo.length) { toast("nada para desfazer"); return; }
  const snap = st.undo.pop();
  const changed = [];
  for (let r = 0; r < snap.length; r++)
    for (let c = 0; c < snap[r].length; c++)
      if (st.values[r][c] !== snap[r][c]) {
        st.values[r][c] = snap[r][c];
        st.modified.add(`${r},${c}`);
        changed.push([r, c]);
      }
  if (!changed.length) { toast("nada para desfazer"); return; }
  const flat = st.values.flat();
  const [lo, hi] = [Math.min(...flat), Math.max(...flat)];
  for (const [r, c] of changed) {
    const td = st.pane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
    if (td) {
      td.textContent = st.disp ? st.disp.fmt(st.values[r][c]) : st.values[r][c];
      td.classList.add("mod");
      td.style.background = H.heatColor(st.values[r][c], lo, hi);
      td.style.color = H.heatTextColor(st.values[r][c], lo, hi);
    }
  }
  toast(`desfeito: ${changed.length} célula(s)`);
  if (st.send) st.send().catch(err => toast(err.message, true));
}

// Selecciona o rectângulo entre dois cantos (inclusivo), substituindo a
// selecção actual. Usado pelo arraste do mouse e por Shift+setas/click.
function selectRect(pane, page, a, b) {
  clearSel();
  selPage = page; selPane = pane;
  const r0 = Math.min(a[0], b[0]), r1 = Math.max(a[0], b[0]);
  const c0 = Math.min(a[1], b[1]), c1 = Math.max(a[1], b[1]);
  for (let r = r0; r <= r1; r++) {
    for (let c = c0; c <= c1; c++) {
      const key = `${r},${c}`;
      selCells.add(key); selWeights.set(key, 1.0);
      const td = pane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
      if (td) td.classList.add("sel");
    }
  }
}

function clearSel() {
  if (selPane) $$("td.sel", selPane).forEach(td => td.classList.remove("sel"));
  selCells.clear(); selWeights.clear(); selPage = 0; selPane = null;
}

document.addEventListener("keydown", e => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
  if (!selPane) return;
  if ($(".grid-pane.active") !== selPane) return;
  const st = gridState[selPage];
  if (!st) return;

  // Setas: navegam pela tabela (movem a âncora, substituem a selecção por
  // 1 célula) — só em modo Manual: em Trace a selecção é conduzida pela
  // posição do motor, navegar manualmente entraria em conflito com isso.
  const nav = { ArrowUp: [1, 0], ArrowDown: [-1, 0], ArrowRight: [0, 1], ArrowLeft: [0, -1] }[e.key];
  if (nav) {
    if (st.mode !== "manual") return;
    e.preventDefault();
    if (!selAnchor) return;
    const maxR = (INFO ? INFO.axes.map_kpa.length : 20) - 1;
    const maxC = (INFO ? INFO.axes.rpm.length : 20) - 1;
    if (e.shiftKey) {
      // Shift+seta: âncora fica, o foco move — selecção = rectângulo entre os dois
      const base = selFocus ?? selAnchor;
      selFocus = [Math.max(0, Math.min(maxR, base[0] + nav[0])),
                  Math.max(0, Math.min(maxC, base[1] + nav[1]))];
      const pane = selPane, page = selPage;
      selectRect(pane, page, selAnchor, selFocus);
      return;
    }
    const r = Math.max(0, Math.min(maxR, selAnchor[0] + nav[0]));
    const c = Math.max(0, Math.min(maxC, selAnchor[1] + nav[1]));
    $$("td.sel", selPane).forEach(td => td.classList.remove("sel"));
    selCells.clear(); selWeights.clear();
    const key = `${r},${c}`;
    selCells.add(key);
    selWeights.set(key, 1.0);
    selAnchor = [r, c];
    selFocus = null;
    const td = selPane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
    if (td) td.classList.add("sel");
    return;
  }

  // X: desfaz a última operação nesta página (não exige selecção activa)
  if (e.key === "x" || e.key === "X") {
    e.preventDefault();
    applyUndo(st, selPage);
    return;
  }

  if (!selCells.size) return;

  // H/V: interpolação linear entre as células EXTREMAS da selecção — por
  // linha (H, ao longo do eixo de RPM) ou por coluna (V, eixo de MAP).
  // Interpola pelo VALOR do eixo (não pelo índice): os eixos não são
  // uniformes (ex.: RPM 500→750 vs 7000→8000), então a rampa fica
  // fisicamente correta. Extremos mantêm o valor; intermediárias recebem.
  if (e.key === "h" || e.key === "H" || e.key === "v" || e.key === "V") {
    e.preventDefault();
    const horiz = (e.key === "h" || e.key === "H");
    const axis = horiz ? INFO.axes.rpm : INFO.axes.map_kpa;
    const clampV = (v) => H.clampCell(selPage, v);
    pushUndo(st);
    // agrupa as células seleccionadas por linha (H) ou por coluna (V)
    const groups = new Map();  // fixo → [móvel, ...]
    selCells.forEach(key => {
      const [r, c] = key.split(",").map(Number);
      const fixed = horiz ? r : c;
      const moving = horiz ? c : r;
      if (!groups.has(fixed)) groups.set(fixed, []);
      groups.get(fixed).push(moving);
    });
    let changed = 0;
    groups.forEach((movs, fixed) => {
      movs.sort((a, b) => a - b);
      const lo = movs[0], hi = movs[movs.length - 1];
      if (hi - lo < 2) return;  // sem intermediárias
      const vAt = (m) => horiz ? st.values[fixed][m] : st.values[m][fixed];
      const setAt = (m, v) => {
        if (horiz) st.values[fixed][m] = v; else st.values[m][fixed] = v;
        st.modified.add(horiz ? `${fixed},${m}` : `${m},${fixed}`);
        ++changed;
      };
      const v0 = vAt(lo), v1 = vAt(hi);
      const x0 = axis[lo], x1 = axis[hi];
      for (const m of movs) {
        if (m === lo || m === hi) continue;
        const f = (axis[m] - x0) / ((x1 - x0) || 1);
        setAt(m, clampV(Math.round(v0 + f * (v1 - v0))));
      }
    });
    if (!changed) { toast("interpolação: seleccione ≥3 células na direção"); return; }
    // actualiza as células visíveis (mesmo padrão do A/Z)
    const flat2 = st.values.flat();
    const [m2lo, m2hi] = [Math.min(...flat2), Math.max(...flat2)];
    selCells.forEach(key => {
      const [r, c] = key.split(",");
      const td = selPane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
      if (td) {
        td.textContent = st.disp ? st.disp.fmt(st.values[r][c]) : st.values[r][c];
        if (st.modified.has(key)) td.classList.add("mod");
        td.style.background = H.heatColor(st.values[r][c], m2lo, m2hi);
        td.style.color = H.heatTextColor(st.values[r][c], m2lo, m2hi);
      }
    });
    if (st.send) st.send().catch(err => toast(err.message, true));
    return;
  }

  const step = H.stepSize(selPage);
  let delta = 0;
  // "A" = step mais, "Z" = step menos (substituem ArrowUp/ArrowDown, agora
  // usadas para navegar); +/=/- continuam a funcionar também.
  if (e.key === "+" || e.key === "=" || e.key === "a" || e.key === "A")      delta = step;
  else if (e.key === "-" || e.key === "z" || e.key === "Z") delta = -step;
  else return;
  e.preventDefault();
  pushUndo(st);
  selCells.forEach(key => {
    const [r, c] = key.split(",").map(Number);
    // Peso bilinear (trace auto-select) escala o delta — célula dominante
    // move-se mais depressa que as vizinhas de influência menor.
    const w = selWeights.get(key) ?? 1.0;
    const weighted = H.weightedStep(delta, w);
    st.values[r][c] = H.clampCell(selPage, st.values[r][c] + weighted);
    st.modified.add(key);
  });
  // Update visible cells — recalcula min/max da tabela para a cor do
  // heatmap acompanhar o novo valor (só era feito dentro do render()
  // completo, que deixou de correr a cada tecla).
  const flat = st.values.flat();
  const [tmin, tmax] = [Math.min(...flat), Math.max(...flat)];
  selCells.forEach(key => {
    const [r, c] = key.split(",");
    const td = selPane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
    if (td) {
      td.textContent = st.disp ? st.disp.fmt(st.values[r][c]) : st.values[r][c];
      td.classList.add("mod");
      td.style.background = H.heatColor(st.values[r][c], tmin, tmax);
      td.style.color = H.heatTextColor(st.values[r][c], tmin, tmax);
    }
  });
  if (st.send) st.send().catch(err => toast(err.message, true));
});

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];

document.addEventListener("mouseup", () => { dragSel = null; });

let INFO = null;          // /api/info
let RT = null;            // último frame de telemetria

function toast(msg, err = false) {
  const d = document.createElement("div");
  d.className = "msg" + (err ? " err" : "");
  d.textContent = msg;
  $("#toast").appendChild(d);
  setTimeout(() => d.remove(), 4200);
}

async function api(path, methodOrOpts, body) {
  let opts = methodOrOpts;
  if (typeof methodOrOpts === "string") {
    opts = { method: methodOrOpts, headers: {"Content-Type":"application/json"},
             body: body !== undefined ? JSON.stringify(body) : undefined };
  }
  // Sem isto o browser pode servir um GET repetido (ex.: botão Reload) da
  // cache HTTP em vez de pedir de novo ao servidor — a página lê sempre
  // g_pageN (RAM) fresco no firmware; é a cache do browser que mostrava
  // "valores base" desactualizados, não o caminho de leitura.
  opts = { cache: "no-store", ...opts };
  const r = await fetch(path, opts);
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status} ${await r.text()}`);
  return r.json();
}

/* ── tabs ─────────────────────────────────────────────────────────────── */
$$("#sb-nav .tab").forEach(b => b.onclick = () => {
  clearSel();
  $$("#sb-nav .tab").forEach(x => x.classList.toggle("active", x === b));
  $$(".pane").forEach(p => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  const pane = $("#tab-" + b.dataset.tab);
  if (pane.classList.contains("grid-pane") && !pane.dataset.loaded) loadGrid(pane);
  if (b.dataset.tab === "params"    && !$("#paramsRoot").dataset.loaded)   loadParams();
  if (b.dataset.tab === "pedal-map" && !$("#pedalMapRoot").dataset.loaded) loadPedalMap();
  if (b.dataset.tab === "boost"     && !$("#boostRoot").dataset.loaded)    loadBoostMap();
  if (b.dataset.tab === "ltft-accum" && !$("#ltftAccumRoot").dataset.loaded) loadLtftAccum();
  if (b.dataset.tab === "output-test" && !$("#outputTestRoot").dataset.loaded) loadOutputTest();
  if (b.dataset.tab === "telemetry")
    charts.forEach(c => c.u.setSize({ width: c.u.root.parentElement.clientWidth - 8, height: 300 }));
});

/* ── telemetria: gauges + charts ──────────────────────────────────────── */
// Statusbar: 10 primários (largura toda — chips vivem no rodapé). Resto só em Telemetry.
const GAUGES = [
  ["rpm",                 "RPM",     v => H.formatGauge("rpm", v)],
  ["map_kpa",             "MAP kPa", v => H.formatGauge("map_kpa", v)],
  ["tps_pct",             "TPS %",   v => H.formatGauge("tps_pct", v)],
  ["ve",                  "VE %",    v => H.formatGauge("ve", v)],
  ["lambda_x1000",        "λ",       v => H.formatGauge("lambda_x1000", v)],
  ["lambda_target_x1000", "λ tgt",   v => H.formatGauge("lambda_target_x1000", v)],
  ["pw_ms",               "PW ms",   v => H.formatGauge("pw_ms", v)],
  ["advance_deg",         "Ign °",   v => H.formatGauge("advance_deg", v)],
  ["clt_c",               "CLT °C",  v => H.formatGauge("clt_c", v)],
  ["iat_c",               "IAT °C",  v => H.formatGauge("iat_c", v)],
];
const GAUGES_EXTRA = [
  ["stft_pct",    "STFT %", v => H.formatGauge("stft_pct", v)],
  ["ltft_pct",    "LTFT %", v => H.formatGauge("ltft_pct", v)],
  ["ethanol_pct", "E%",     v => H.formatGauge("ethanol_pct", v)],
  ["dc_pct",      "DC %",   v => H.formatGauge("dc_pct", v ?? 0)],
];
function renderGaugeRow(el, list, idPrefix = "g_") {
  el.innerHTML = list.map(([k, l]) =>
    `<div class="gauge"><div class="v" id="${idPrefix}${k}">—</div><div class="l">${l}</div></div>`
  ).join("");
}
renderGaugeRow($("#gauges"), GAUGES);
renderGaugeRow($("#gaugesExtra"), GAUGES_EXTRA, "gx_");

const INJ_MODES = { 0: "SIM", 1: "SEMI", 2: "SEQ" };
// Chips: off=muted; goodWhenOn → verde se on; senão vermelho se on (fault/cut).
// kind "warn" = âmbar quando on (activo mas não fault).
// FAULT agrega sensores analógicos + WBO2 (CAN) + TLE8888 + LIMP/ETB-limp —
// clique lista as fontes ativas (sensor a sensor via sensor_fault_bits, r[34]).
const STATUS_CHIPS = [
  { id: "SYNC",  label: "SYNC",     goodWhenOn: true,
    on: d => !!(d.status && d.status.FULL_SYNC) },
  { id: "REV",   label: "REV LIM",  goodWhenOn: false,
    on: d => !!(d.status && d.status.REV_LIMIT) },
  { id: "FAULT", label: "FAULT",    goodWhenOn: false,
    // TLE8888 é só informativo (nenhum gating no firmware; INJ/IGN saem por
    // GPIOE) e em bancada o chip está ausente → suprimido com BENCH ativo.
    // WBO2 NÃO é suprimido: bloqueia o closed-loop, e o bench simula λ que
    // o limpa — se acender em bench, é falha real da simulação.
    on: d => !!(d.status && (d.status.SENSOR_FAULT || d.status.WBO2_FAULT ||
                             (d.status.TLE8888_FAULT && !d.status.BENCH_MODE) ||
                             d.status.LIMP_MODE || d.status.ETB_LIMP)) },
  { id: "TC",    label: "TRACTION", goodWhenOn: false, warn: true,
    on: d => !!(d.status && d.status.TC_ACTIVE) },
];
$("#statusLeds").innerHTML =
  STATUS_CHIPS.slice(0, 1).map(c =>
    `<span class="led" id="led_${c.id}" title="${c.label}">${c.label}</span>`).join("") +
  `<span class="led on-info" id="ignMode" title="Modo de ignição">—</span>` +
  `<span class="led on-info" id="injMode" title="Modo de injeção">—</span>` +
  STATUS_CHIPS.slice(1).map(c =>
    `<span class="led" id="led_${c.id}" title="${c.label}">${c.label}</span>`).join("");

// Detalhe do FAULT: nomes por bit de sensor_fault_bits (SensorId 0..7).
const SENSOR_FAULT_NAMES =
  ["MAP", "MAF", "TPS", "CLT", "IAT", "O2", "FUEL PRESS", "OIL PRESS"];
let lastRT = null;
function faultDetails(d) {
  if (!d || !d.status) return [];
  const out = [];
  const sb = d.sensor_fault_bits || 0;
  SENSOR_FAULT_NAMES.forEach((n, i) => {
    if (sb & (1 << i)) out.push(`${n} fora de range`);
  });
  if (d.status.LIMP_MODE) out.push("LIMP mode ativo (corte de PW)");
  if (d.status.ETB_LIMP) out.push("ETB limp ativo");
  if (d.status.WBO2_FAULT) out.push("WBO2 (CAN) sem sinal/fault");
  if (d.status.TLE8888_FAULT)
    out.push(`TLE8888 driver${d.tle8888_fault_bm
      ? ` (bm 0x${d.tle8888_fault_bm.toString(16).toUpperCase()})` : " ausente"}${
      d.status.BENCH_MODE ? " — suprimido em bench" : ""}`);
  return out;
}
$("#led_FAULT").style.cursor = "pointer";
$("#led_FAULT").onclick = () => {
  const list = faultDetails(lastRT);
  toast(list.length ? "FAULT: " + list.join(" · ") : "Sem falhas ativas", list.length > 0);
};

/* strip-chart uPlot único: 60 s sliding window, séries por checkbox.
   Cada série tem a SUA escala (autorange independente) — sem eixo Y comum;
   valores lêem-se na legenda (hover) e nos gauges. */
const WINDOW_S = 60, MAX_PTS = 60 * 35;
const CHART_SERIES = [
  // [key, cor, label, on por default]
  ["rpm",          "#e8a020", "RPM",    true],
  ["map_kpa",      "#4ea1ff", "MAP",    true],
  ["tps_pct",      "#22c55e", "TPS",    true],
  ["lambda_x1000", "#ef4444", "λ",      false],
  ["stft_pct",     "#a78bfa", "STFT %", false],
  ["pw_ms",        "#f0d030", "PW ms",  false],
  ["advance_deg",  "#2dd4a0", "Ign °",  false],
];
const charts = (() => {
  const wrap = $("#charts");
  const toggles = document.createElement("div");
  toggles.className = "chart-toggles";
  toggles.innerHTML = CHART_SERIES.map(([k, c, lab, on], i) =>
    `<label style="--sc:${c}"><input type="checkbox" data-si="${i + 1}"
       ${on ? "checked" : ""}> ${lab}</label>`).join("");
  wrap.appendChild(toggles);
  const box = document.createElement("div");
  box.className = "chart-box chart-box-single";
  wrap.appendChild(box);
  const data = [[], ...CHART_SERIES.map(() => [])];
  const u = new uPlot({
    width: box.clientWidth - 8 || 480, height: 300,
    scales: { x: { time: false } },
    axes: [{ stroke: "#555555", grid: { stroke: "#1a1a1a" } }],
    series: [
      { label: "t" },
      ...CHART_SERIES.map(([k, c, lab, on]) => ({
        label: lab, stroke: c, width: 1.5, show: on, scale: k,
      })),
    ],
  }, data, box);
  toggles.querySelectorAll("input").forEach(inp => {
    inp.onchange = () => u.setSeries(+inp.dataset.si, { show: inp.checked });
  });
  return [{ u, data, keys: CHART_SERIES.map(([k]) => k) }];
})();
window.addEventListener("resize", () =>
  charts.forEach(c => c.u.setSize({ width: c.u.root.parentElement.clientWidth - 8, height: 300 })));

const t0 = performance.now();
function pushTelemetry(d) {
  const t = (performance.now() - t0) / 1000;
  for (const c of charts) {
    c.data[0].push(t);
    c.keys.forEach((k, i) => {
      let v = d[k];
      // chart de λ em unidades humanas
      if (k === "lambda_x1000") v = v / 1000;
      c.data[i + 1].push(v);
    });
    while (c.data[0].length > MAX_PTS || t - c.data[0][0] > WINDOW_S)
      c.data.forEach(a => a.shift());
    c.u.setData(c.data);
  }
  for (const [k, , fmt] of GAUGES) {
    const el = $(`#g_${k}`);
    if (el) el.textContent = fmt(d[k]);
  }
  for (const [k, , fmt] of GAUGES_EXTRA) {
    const el = $(`#gx_${k}`);
    if (el) el.textContent = fmt(d[k]);
  }
  $("#ignMode").textContent =
    (d.status && d.status.IGN_SEQUENTIAL) ? "IGN:SEQ" : "IGN:WASTED";
  $("#injMode").textContent = "INJ:" + (INJ_MODES[d.inj_mode] || "?");
  lastRT = d;
  const fd = faultDetails(d);
  $("#led_FAULT").title = fd.length ? fd.join(" · ") : "Sem falhas ativas";
  for (const c of STATUS_CHIPS) {
    const on = c.on(d);
    let cls = "led";
    if (on) cls += c.info ? " on-info"
                          : (c.goodWhenOn ? " on-good" : (c.warn ? " on-warn" : " on-bad"));
    $(`#led_${c.id}`).className = cls;
  }
  // Chip BENCH segue o bit real da ECU (STATUS_BENCH_MODE) — apanha o caso
  // do bench (RAM) morrer num reset da ECU com o toggle do host ainda ON.
  const ecuBench = !!(d.status && d.status.BENCH_MODE);
  if (ecuBench !== benchOn) setBenchBtn(ecuBench);
  const tleBm = d.tle8888_fault_bm || 0;
  const tleStr = tleBm ? ` · TLE8888 fault 0x${tleBm.toString(16).toUpperCase()}` : "";
  $("#diag").textContent =
    `loop2ms ${d.loop2ms_last_us}µs (max ${d.loop2ms_max_us}µs) · ` +
    `late ${d.late_events} · drops ${d.sched_drops} · clamps ${d.cal_clamps} · ` +
    `sync_state ${d.sync_state} · cmp_confirms ${d.cmp_confirms} · ` +
    `cmp_glitch ${d.cmp_glitch}${tleStr}`;
}

/* ── WebSocket ────────────────────────────────────────────────────────── */
function setConnUI(online, detail) {
  const el = $("#conn");
  if (!el) return;
  el.className = "conn-pill " + (online ? "on" : "off");
  const text = el.querySelector(".conn-text");
  if (text) text.textContent = online ? "ONLINE" : "OFFLINE";
  else el.textContent = online ? "ONLINE" : "OFFLINE";
  if (detail) el.title = detail;
}

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onmessage = ev => {
    const d = JSON.parse(ev.data);
    const conn = d.connected;
    setConnUI(conn, d.error || (conn ? "ECU linked" : "sem ECU"));
    if (conn && d.rpm !== undefined) {
      RT = d; pushTelemetry(d); highlightLiveCell();
      $$(".live-raw").forEach(el => el.textContent = `live: ${d[el.dataset.src]}`);
    }
  };
  ws.onclose = () => {
    setConnUI(false, "WS closed · reconnecting");
    setTimeout(connectWS, 1000);
  };
}

/* Heatmap / axis / clamp: see js/helpers.js (H.*) */

/* ── editores de grid (VE/Spark/Lambda) ───────────────────────────────── */
const gridState = {};   // page → {values, modified:Set("r,c"), table}

async function loadGrid(pane) {
  const page = +pane.dataset.page;
  const meta = INFO.grid_pages[page];
  // Exibição: página 4 guarda λ×1000 no wire/estado; mostrar λ real (1.050).
  // parse converte o texto editado de volta para o valor cru do protocolo.
  const disp = {
    fmt: v => H.formatCell(page, v),
    parse: s => H.parseCell(page, s),
  };
  pane.innerHTML = `
    <div class="grid-toolbar">
      <strong>${meta.name}</strong> <span class="muted">(${meta.unit})</span>
      <span class="mode-toggle">
        <button class="mode-btn active" data-mode="trace">◉ Trace</button>
        <button class="mode-btn" data-mode="manual">✎ Manual</button>
      </span>
      <button data-act="reload" title="Lê a página da ECU (RAM)">Read</button>
      <button class="primary" data-act="send" title="Escreve células pendentes na RAM (edições já vão ao teclar)">Write</button>
      <button class="danger" data-act="burn" title="Grava a página actual (RAM) no flash">Save</button>
      <span class="dirty"></span>
      <span class="flash-dirty" hidden>não gravado em flash</span>
      <span class="grid-hint">Trace: rastro · Manual: clique/arraste · A/Z ± · H/V interp · X undo · dbl-click edita</span>
    </div>
    <div class="grid-wrap"></div>`;
  pane.dataset.loaded = "1";

  // Modos mutuamente exclusivos: "trace" (destaque ao vivo da posição do
  // motor, 30Hz) e "manual" (selecção múltipla + edição por teclado). Os
  // dois partilhavam o mesmo outline CSS (.sel/.live2) e o mesmo ciclo de
  // desenho, tornando-se indistinguíveis e a competir pela mesma célula —
  // agora só um corre de cada vez por pane.
  const st = gridState[page] = {
    values: null, modified: new Set(), pane, mode: "trace", disp, flashDirty: false,
  };

  function updateFlashDirty() {
    const el = $(".flash-dirty", pane);
    if (el) el.hidden = !st.flashDirty;
  }

  function setMode(mode) {
    if (st.mode === mode) return;
    st.mode = mode;
    $$(".mode-btn", pane).forEach(b => b.classList.toggle("active", b.dataset.mode === mode));
    if (mode === "manual") {
      // Sai do trace: apaga o rasto para não ficar preso na última posição
      // desenhada. A selecção herdada do trace (até 4 células, pesos
      // bilineares <1) mantém-se por continuidade, mas o modo Manual
      // trabalha sempre a peso cheio — normaliza para 1.0.
      const cv = pane.querySelector("canvas.trail");
      if (cv) cv.getContext("2d").clearRect(0, 0, cv.width, cv.height);
      if (selPane === pane) selCells.forEach(key => selWeights.set(key, 1.0));
    } else {
      // Sai do manual: limpa qualquer selecção pendente.
      if (selPane === pane) clearSel();
    }
  }
  pane.querySelectorAll(".mode-btn").forEach(b => b.onclick = () => setMode(b.dataset.mode));

  async function reload() {
    const r = await api(`/api/pages/${page}`);
    st.values = r.grid;
    st.modified.clear();
    render();
  }

  function render() {
    const flat = st.values.flat();
    const [min, max] = [Math.min(...flat), Math.max(...flat)];
    let html = `<table class="tune"><tr><th></th>` +
      INFO.axes.rpm.map(r => `<th>${r}</th>`).join("") + "</tr>";
    // eixo Y invertido na exibição: MAP cresce de baixo p/ cima (data-r mantém
    // o índice real da page — edição/tracing não mudam)
    for (let row = INFO.axes.map_kpa.length - 1; row >= 0; row--) {
      html += `<tr><th>${INFO.axes.map_kpa[row]}</th>`;
      for (let col = 0; col < INFO.axes.rpm.length; col++) {
        const v = st.values[row][col];
        const key = `${row},${col}`;
        const mod = st.modified.has(key) ? " mod" : "";
        // Reaplica a selecção multi-célula: render() corre a cada auto-send
        // (uma vez por tecla +/-) e regenera todos os <td> do zero — sem
        // isto a classe "sel" perdia-se após o 1º step, mesmo com selCells
        // (o estado lógico) intacto.
        const sel = (selPane === pane && selCells.has(key)) ? " sel" : "";
        html += `<td class="${mod}${sel}" data-r="${row}" data-c="${col}"
                     style="background:${H.heatColor(v, min, max)};color:${H.heatTextColor(v, min, max)}">${disp.fmt(v)}</td>`;
      }
      html += "</tr>";
    }
    $(".grid-wrap", pane).innerHTML = html + "</table>" +
      `<canvas class="trail"></canvas>`;
    const wrap = $(".grid-wrap", pane), tbl = $("table.tune", wrap),
          cv = $("canvas.trail", wrap);
    cv.width = tbl.offsetWidth;
    cv.height = tbl.offsetHeight;
    $(".dirty", pane).textContent = st.modified.size
      ? `${st.modified.size} célula(s) por enviar` : "";
    updateFlashDirty();
    bindCells();
  }

  function bindCells() {
    // Modelo padronizado: clique simples e Ctrl+click usam o MESMO
    // mecanismo de selecção (contorno .sel + selCells) — clique simples
    // substitui a selecção por só esta célula, Ctrl+click acrescenta/
    // remove. Duplo-clique abre o campo de texto para valor exacto
    // (antes, clique simples ia logo para o texto, sem contorno —
    // selecção individual e múltipla pareciam mecanismos diferentes).
    $$("td[data-r]", pane).forEach(td => {
      // Clique-e-arraste: mousedown ancora, mouseenter estende o rectângulo.
      td.onmousedown = (e) => {
        if (e.button !== 0 || e.target.tagName === "INPUT") return;
        if (st.mode !== "manual") return;
        dragSel = { pane, page, from: [+td.dataset.r, +td.dataset.c] };
        dragMoved = false;
        e.preventDefault();  // evita selecção de texto durante o arraste
      };
      td.onmouseenter = () => {
        if (!dragSel || dragSel.pane !== pane) return;
        const to = [+td.dataset.r, +td.dataset.c];
        if (to[0] !== dragSel.from[0] || to[1] !== dragSel.from[1]) dragMoved = true;
        selAnchor = dragSel.from;
        selFocus = to;
        selectRect(pane, page, dragSel.from, to);
      };
      td.onclick = (e) => {
        if (e.target.tagName === "INPUT") return;  // clique dentro do campo em edição
        if (dragMoved) { dragMoved = false; return; }  // fim de arraste ≠ clique
        if (st.mode !== "manual") {
          toast('Mudar para modo "✎ Manual" para seleccionar células');
          return;
        }
        const key = `${td.dataset.r},${td.dataset.c}`;
        const clicked = [+td.dataset.r, +td.dataset.c];
        selFocus = null;
        if (e.shiftKey && selAnchor && selPane === pane) {
          // Shift+click: rectângulo da âncora (não a 1ª célula do Set)
          selectRect(pane, page, selAnchor, clicked);
          return;
        }
        selAnchor = clicked;  // origem p/ navegação por setas / próximo Shift
        if (e.ctrlKey || e.metaKey) {
          if (selCells.has(key)) { selCells.delete(key); selWeights.delete(key); td.classList.remove("sel"); }
          else {
            if (!selCells.size) { selPage = page; selPane = pane; }
            selCells.add(key); selWeights.set(key, 1.0); td.classList.add("sel");
          }
        } else {
          clearSel();
          selPage = page; selPane = pane;
          selCells.add(key); selWeights.set(key, 1.0); td.classList.add("sel");
        }
      };
      td.ondblclick = (e) => {
        if (st.mode !== "manual") {
          toast('Mudar para modo "✎ Manual" para editar');
          return;
        }
        clearSel();
        beginEdit(td);
      };
    });
  }

  function beginEdit(td) {
    if (td.querySelector("input")) return;
    pushUndo(st);
    const r = +td.dataset.r, c = +td.dataset.c;
    const orig = st.values[r][c];
    const inp = document.createElement("input");
    inp.value = disp.fmt(orig);
    td.textContent = "";
    td.appendChild(inp);
    inp.focus(); inp.select();
    // Envia enquanto ainda se edita (antes do blur): debounce curto para
    // não disparar um PUT a cada tecla, mas sem esperar sair da célula.
    let liveTimer = null;
    const applyLive = () => {
      const v = disp.parse(inp.value);
      if (Number.isNaN(v) || v === st.values[r][c]) return;
      st.values[r][c] = v;
      st.modified.add(`${r},${c}`);
      // Cor do heatmap acompanha o valor em tempo real (input tem fundo
      // transparente — o estilo do <td> é o que aparece por trás do texto).
      const flat = st.values.flat();
      td.style.background = H.heatColor(v, Math.min(...flat), Math.max(...flat));
      td.style.color = H.heatTextColor(v, Math.min(...flat), Math.max(...flat));
      clearTimeout(liveTimer);
      liveTimer = setTimeout(() => {
        // forceBlur=false: isto corre EM PARALELO com a edição ainda em
        // curso (input focado); forçar blur aqui terminava a edição a meio
        // — só um step era possível antes do foco saltar para <body>.
        sendModified(false).catch(err => toast(err.message, true));
      }, 200);
    };
    const commit = () => {
      clearTimeout(liveTimer);
      const v = disp.parse(inp.value);
      if (!Number.isNaN(v) && v !== orig) {
        st.values[r][c] = v;
        st.modified.add(`${r},${c}`);
      }
      render();
      // Envia de imediato para a RAM — não requer clicar "Send" à parte.
      sendModified().catch(err => toast(err.message, true));
    };
    inp.oninput = applyLive;
    inp.onblur = commit;
    let delta = 0;
    inp.onkeydown = e => {
      if (e.key === "Enter") inp.blur();
      if (e.key === "Escape") { inp.value = disp.fmt(orig); inp.blur(); }
      const step = H.stepSize(page);
      if (e.key === "+" || e.key === "=" || e.key === "ArrowUp")   { delta += step; inp.value = disp.fmt(orig + delta); e.preventDefault(); applyLive(); }
      if (e.key === "-" || e.key === "ArrowDown") { delta -= step; inp.value = disp.fmt(orig + delta); e.preventDefault(); applyLive(); }
    };
  }

  async function sendModified(forceBlur = true) {
    // Força o commit de uma edição em curso (input ainda focado): clicar
    // directamente em Burn/Send sem sair da célula antes disparava o click
    // sem o blur ter corrido ainda — st.modified ficava vazio e nada era
    // enviado (Network tab mostrava só o POST /burn, sem PUT /cells).
    // forceBlur=false (chamado pelo auto-envio em segundo plano durante a
    // edição): NÃO tira o foco do input — só queremos mandar o valor já
    // aplicado a st.values, sem interromper quem ainda está a escrever.
    if (forceBlur) {
      const activeInput = pane.querySelector("input");
      if (activeInput) activeInput.blur();
    }
    if (!st.modified.size) return 0;
    const cells = [...st.modified].map(k => {
      const [row, col] = k.split(",").map(Number);
      return { row, col, value: st.values[row][col] };
    });
    await api(`/api/pages/${page}/cells`, {
      method: "PUT", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cells }),
    });
    // NÃO chama render() completo aqui: recriar a tabela inteira a cada
    // envio automático (uma vez por tecla) competia com o trace ao vivo
    // (highlightLiveCell, 30Hz) e destruía a selecção multi-célula (.sel)
    // e o foco do input em edição a cada envio. Só limpa a marca "não
    // enviada" das células que acabaram de sair, deixando o resto do DOM
    // intocado — trace e selecção passam a ser independentes da edição.
    cells.forEach(({ row, col }) => {
      st.modified.delete(`${row},${col}`);
      const td = pane.querySelector(`td[data-r="${row}"][data-c="${col}"]`);
      if (td) td.classList.remove("mod");
    });
    st.flashDirty = true;
    $(".dirty", pane).textContent = st.modified.size
      ? `${st.modified.size} célula(s) por enviar` : "";
    updateFlashDirty();
    return cells.length;
  }
  st.send = sendModified;  // usado pelo atalho de teclado multi-célula

  pane.querySelector('[data-act="send"]').onclick = async () => {
    try {
      const n = await sendModified();
      if (!n) { toast("nada a escrever"); return; }
      toast(`Write · ${n} célula(s) → RAM`);
    } catch (e) { toast(e.message, true); }
  };
  pane.querySelector('[data-act="burn"]').onclick = async () => {
    // Save grava o que já está em RAM no firmware — sem isto, uma edição
    // por enviar ficava só no browser: o save "sucedia" mas persistia o
    // valor ANTIGO, sem qualquer aviso.
    try {
      const n = await sendModified();
      await api(`/api/pages/${page}/burn`, { method: "POST" });
      st.flashDirty = false;
      updateFlashDirty();
      toast(n ? `Save OK · pág. ${page} · ${n} célula(s)` : `Save OK · pág. ${page}`);
    } catch (e) { toast(e.message, true); }
  };
  pane.querySelector('[data-act="reload"]').onclick = async () => {
    try {
      await reload();
      toast(`Read · pág. ${page}`);
    } catch (e) { toast(e.message, true); }
  };

  try {
    await reload();
  } catch (e) {
    // Offline / no ECU: keep toolbar so user can retry Read without a blank pane.
    // Still paint axis skeleton so the VE surface is a real grid structure, not empty void.
    const wrap = $(".grid-wrap", pane);
    if (wrap && INFO && INFO.axes) {
      const rpm = INFO.axes.rpm || [];
      const map = INFO.axes.map_kpa || [];
      let html = `<table class="tune"><tr><th></th>` +
        rpm.map(r => `<th>${r}</th>`).join("") + "</tr>";
      for (let row = map.length - 1; row >= 0; row--) {
        html += `<tr><th>${map[row]}</th>`;
        for (let col = 0; col < rpm.length; col++) {
          html += `<td data-r="${row}" data-c="${col}" style="background:#1a1e28;color:#6b7280">—</td>`;
        }
        html += "</tr>";
      }
      wrap.innerHTML = html + "</table>" +
        `<div class="ot-banner" style="margin-top:10px">Grid offline — ${e.message || e}. Connect ECU and press Read.</div>`;
    } else if (wrap) {
      wrap.innerHTML = `<div class="ot-banner">Grid offline — ${e.message || e}. Connect ECU and press Read.</div>`;
    }
    toast(String(e.message || e), true);
  }
}

/* célula corrente do motor destacada ao vivo.
   Espelha axis_lookup() do firmware (table3d.cpp): eixos são NÓS de
   interpolação — o ponto de operação mistura os 4 nós vizinhos com peso
   pela posição real. Destacamos os 4 (dominante mais forte). */
const TRAIL_MS = 10000;
function highlightLiveCell() {
  if (!RT || !INFO) return;
  const pane = $(".grid-pane.active");
  if (!pane) return;
  const page = +pane.dataset.page;
  const st = gridState[page];
  const cv = pane.querySelector("canvas.trail");
  if (!st || !cv) return;
  // Modo manual: trace completamente desligado para este pane — não toca
  // em .sel/canvas nem compete com a selecção/edição em curso.
  if (st.mode !== "trace") return;

  const lx = H.axisLookup(INFO.axes.rpm, RT.rpm);
  const ly = H.axisLookup(INFO.axes.map_kpa, RT.map_kpa);

  // posição real interpolada em pixels: centro do nó idx + frac até o nó idx+1
  const center = (sel) => {
    const td = pane.querySelector(sel);
    return td ? { x: td.offsetLeft + td.offsetWidth / 2,
                  y: td.offsetTop + td.offsetHeight / 2 } : null;
  };
  const c0 = center(`td[data-r="${ly.idx}"][data-c="${lx.idx}"]`);
  const c1 = center(`td[data-r="${ly.idx + 1}"][data-c="${lx.idx + 1}"]`);
  if (!c0 || !c1) return;
  const x = c0.x + lx.frac * (c1.x - c0.x);
  const y = c0.y + ly.frac * (c1.y - c0.y);

  // rastro dos últimos TRAIL_MS
  const now = performance.now();
  st.trail = (st.trail || []).filter(p => now - p.t < TRAIL_MS);
  st.trail.push({ x, y, t: now });

  // célula dominante (nó mais próximo da posição interpolada) — usada só
  // para o rasto/trilha (st.cells) abaixo, já não define sozinha a selecção.
  const domR = ly.idx + (ly.frac >= 0.5 ? 1 : 0);
  const domC = lx.idx + (lx.frac >= 0.5 ? 1 : 0);

  // "Trace auto-select": as 4 células bilineares (mesmas usadas para
  // interpolar o valor real usado pelo motor) ficam seleccionadas, cada
  // uma com o seu peso — A/Z/+/- distribui o delta proporcionalmente em
  // vez de aplicar tudo só à dominante. Pesos a 0 (fronteira exacta da
  // tabela, frac=0/1) são omitidos. Só mexe no DOM quando o CONJUNTO de
  // nós muda (não a cada frame a 30Hz).
  const corners = H.bilinearCorners(lx, ly);
  const newKeys = corners.map(p => `${p.r},${p.c}`);
  const changed = selPane !== pane || selPage !== page ||
    selCells.size !== newKeys.length || newKeys.some(k => !selCells.has(k));
  if (changed) {
    $$("td.sel", pane).forEach(td => td.classList.remove("sel"));
    selCells.clear(); selWeights.clear();
    corners.forEach(p => {
      const key = `${p.r},${p.c}`;
      selCells.add(key);
      selWeights.set(key, p.w);
      const td = pane.querySelector(`td[data-r="${p.r}"][data-c="${p.c}"]`);
      if (td) td.classList.add("sel");
    });
    selPage = page; selPane = pane; selAnchor = [domR, domC];
  }

  st.cells = (st.cells || []).filter(p => now - p.t < TRAIL_MS);
  const hit = st.cells.find(p => p.r === domR && p.c === domC);
  if (hit) hit.t = now;                       // renova o tempo se já visitada
  else st.cells.push({ r: domR, c: domC, t: now });

  const g = cv.getContext("2d");
  g.clearRect(0, 0, cv.width, cv.height);
  g.lineCap = g.lineJoin = "round";

  // realce das células percorridas nos últimos TRAIL_MS (desvanece com a idade)
  for (const p of st.cells) {
    const td = pane.querySelector(`td[data-r="${p.r}"][data-c="${p.c}"]`);
    if (!td) continue;
    const age = (now - p.t) / TRAIL_MS;        // 0 = recente, 1 = velho
    g.fillStyle = `rgba(255,213,79,${(1 - age) * 0.30})`;
    g.fillRect(td.offsetLeft, td.offsetTop, td.offsetWidth, td.offsetHeight);
  }
  for (let i = 1; i < st.trail.length; i++) {
    const a = st.trail[i - 1], b = st.trail[i];
    const age = (now - b.t) / TRAIL_MS;          // 0 = recente, 1 = velho
    g.strokeStyle = `rgba(255,255,255,${(1 - age) * 0.85})`;
    g.lineWidth = 2 + (1 - age) * 3;             // afina conforme envelhece
    g.beginPath(); g.moveTo(a.x, a.y); g.lineTo(b.x, b.y); g.stroke();
  }
  // ponto atual — bem visível
  g.beginPath(); g.arc(x, y, 9, 0, 2 * Math.PI);
  g.fillStyle = "#fff"; g.fill();
  g.lineWidth = 3; g.strokeStyle = "#111"; g.stroke();
}

/* ── parâmetros ───────────────────────────────────────────────────────── */
const PARAM_PAGES = { 0: "Engine Config", 5: "1D Corrections",
                      6: "X-Tau / AE / Crank", 7: "Dwell 2D" };

/* Metadados dos grupos de parâmetros: ícone, rótulo da seção, página(s) */
const PARAM_GROUPS = [
  { id: "pg-motor",    icon: "⚙",  label: "ENGINE",      pages: [0] },
  { id: "pg-inject",   icon: "⛽", label: "FUELING",     pages: [5, 6] },
  { id: "pg-ignition", icon: "⚡", label: "IGNITION",    pages: [7] },
  { id: "pg-canrx",    icon: "⇄",  label: "CAN RX",      pages: [], canRx: true },
];

/* Page 0 scalar sub-groups */
const PAGE_0_SECTIONS = [
  {
    label: "ENGINE",
    fields: ["displacement_cc","trigger_tooth0_engine_deg",
             "default_eoi_lead_deg","config_magic"],
  },
  {
    label: "FUELING",
    fields: ["injector_flow_cc_min","stoich_afr_x100","map_ref_bar_x100"],
  },
  {
    label: "APP SENSORS",
    fields: ["app1_raw_min","app1_raw_max","app2_raw_min","app2_raw_max",
             "app_max_delta_pct_x10"],
  },
  {
    label: "THROTTLE BODY (ETB)",
    fields: ["etb_tps1_raw_min","etb_tps1_raw_max","etb_tps2_raw_min","etb_tps2_raw_max",
             "etb_max_delta_pct_x10","etb_max_open_pct_x10_limp",
             "etb_max_rate_pct_per_s","etb_idle_open_pct_x10","etb_cal_valid",
             "etb_harness_present","etb_kp_x10","etb_ki_x10","etb_kd_x10"],
  },
  {
    label: "CYLINDER TRIM",
    fields: ["cyl_fuel_trim_pct_0","cyl_fuel_trim_pct_1","cyl_fuel_trim_pct_2","cyl_fuel_trim_pct_3",
             "cyl_ign_trim_deg_0","cyl_ign_trim_deg_1","cyl_ign_trim_deg_2","cyl_ign_trim_deg_3"],
  },
  {
    label: "CMP",
    fields: ["cmp_window_open_tooth","cmp_window_close_tooth"],
  },
  {
    label: "IDLE",
    fields: ["etb_idle_rpm_target","etb_idle_min_opening_x10","etb_idle_max_opening_x10"],
    curves: [
      { title: "Idle target RPM vs CLT", axis: "iac_clt_axis_x10", axisLabel: "CLT (°C)",
        rows: [["iac_idle_target_rpm_x10", "RPM alvo"]] },
    ],
  },
  {
    label: "CLOSED-LOOP (STFT/LTFT)",
    fields: ["closed_loop_enable","closed_loop_post_start_s","ltft_adapt_min_rpm_x10",
             "stft_kp_x100","stft_ki_x1000","stft_clamp_pct_x10",
             "ltft_mult_clamp_pct_x10","ltft_add_clamp_us","ltft_learn_div",
             "ltft_commit_gain_pct","ltft_max_step_x10","ltft_adapt_enable",
             "ltft_learn_ready_hits","ltft_learn_max_err_x1000",
             "ltft_learn_ready_max_mean_err","ltft_learn_ready_min_stft_x10",
             "ltft_learn_ready_max_stft_x10","ltft_add_pw_threshold_us",
             "ltft_apply_burn_ve",
             "xtau_x_min_q8","xtau_x_max_q8","xtau_tau_min","xtau_tau_max"],
    actions: [{ label: "Reset LTFT", cls: "danger", endpoint: "/api/ltft/reset",
                confirm: "Zero STFT + LEARN accum + LTFT maps (NVM adaptive shadow)?" }],
  },
  {
    label: "EWG (ELECTRONIC WASTEGATE)",
    fields: ["ewg_kp_x10","ewg_ki_x10","ewg_kd_x10","ewg_pos_min_raw","ewg_pos_max_raw"],
  },
  {
    label: "EOI BLEND (FASE DE INJEÇÃO)",
    fields: ["eoi_idle_deg","eoi_blend_rpm_lo","eoi_blend_rpm_hi"],
  },
  {
    label: "DRIVABILITY",
    fields: ["antijerk_tpsdot_threshold_x10","antijerk_retard_deg","antijerk_decay_cycles",
             "rev_limit_rpm_x10","rev_limit_soft_window_x10",
             "decel_cut_tps_threshold_x10","decel_cut_entry_rpm_x10",
             "decel_cut_exit_rpm_x10","decel_cut_min_clt_x10"],
  },
];

/* friendly labels — unidades = valor já escalado por protocol.py (não o raw wire) */
const FIELD_LABELS = {
  displacement_cc:           "Displacement (cc)",
  injector_flow_cc_min:      "Injector flow (cc/min)",
  stoich_afr_x100:           "Stoich AFR",
  map_ref_bar_x100:          "Reference MAP (bar)",
  trigger_tooth0_engine_deg: "Trigger tooth 0 offset (°, 0-719)",
  default_eoi_lead_deg:      "EOI target (° BTDC — fim da injeção)",
  config_magic:              "Magic (0x4544 = valid config v2/EOI)",
  app1_raw_min:  "APP1 released (raw)",   app1_raw_max:  "APP1 floored (raw)",
  app2_raw_min:  "APP2 released (raw)",   app2_raw_max:  "APP2 floored (raw)",
  etb_tps1_raw_min: "ETB TPS1 closed (raw)", etb_tps1_raw_max: "ETB TPS1 open (raw)",
  etb_tps2_raw_min: "ETB TPS2 closed (raw)", etb_tps2_raw_max: "ETB TPS2 open (raw)",
  app_max_delta_pct_x10: "APP plausibility Δmax (%)",
  etb_max_delta_pct_x10: "ETB plausibility Δmax (%)",
  etb_max_open_pct_x10_limp: "ETB max opening limp (%)",
  etb_max_rate_pct_per_s: "ETB max rate (%/s)",
  etb_idle_open_pct_x10: "ETB idle opening (%)",
  etb_cal_valid: "ETB cal valid (0/1)",
  etb_harness_present: "ETB harness present (0/1)",
  etb_kp_x10: "ETB PID Kp", etb_ki_x10: "ETB PID Ki", etb_kd_x10: "ETB PID Kd",
  cyl_fuel_trim_pct_0: "Fuel trim cyl.1 (%)", cyl_fuel_trim_pct_1: "Fuel trim cyl.2 (%)",
  cyl_fuel_trim_pct_2: "Fuel trim cyl.3 (%)", cyl_fuel_trim_pct_3: "Fuel trim cyl.4 (%)",
  cyl_ign_trim_deg_0:  "Ign trim cyl.1 (°)",  cyl_ign_trim_deg_1:  "Ign trim cyl.2 (°)",
  cyl_ign_trim_deg_2:  "Ign trim cyl.3 (°)",  cyl_ign_trim_deg_3:  "Ign trim cyl.4 (°)",
  cmp_window_open_tooth:  "CMP window open (tooth)",
  cmp_window_close_tooth: "CMP window close (tooth; 0/0=disabled)",
  // Page 5 scalars (valores já em unidade natural)
  ae_tpsdot_threshold_x10: "AE TPSdot threshold (%/s)",
  ae_taper_cycles:         "AE taper (cycles)",
  ae_max_pw_us:            "AE max PW (ms)",
  idle_spark_tps_max_x10:              "Idle spark TPS max (%)",
  idle_spark_map_max_bar_x100:         "Idle spark MAP max (bar)",
  idle_spark_rpm_min_x10:              "Idle spark RPM min",
  idle_spark_window_above_target_x10:  "Idle spark window above target (RPM)",
  idle_spark_deadband_rpm_x10:         "Idle spark deadband (RPM)",
  idle_spark_rpm_per_deg_x10:          "Idle spark RPM/°",
  idle_spark_retard_limit_deg:         "Idle spark max retard (°)",
  idle_spark_advance_limit_deg:        "Idle spark max advance (°)",
  // Marcha lenta ETB + IAC
  etb_idle_rpm_target:      "Idle target RPM",
  etb_idle_min_opening_x10: "ETB idle min opening (%)",
  etb_idle_max_opening_x10: "ETB idle max opening (%)",
  iac_clt_axis_x10:         "Eixo CLT — Idle target RPM (°C, 8pts)",
  iac_idle_target_rpm_x10:  "Idle target RPM vs CLT (8pts)",
  // Closed-loop STFT/LTFT
  // Nota: stft_kp/ki/clamp ainda vêm raw no wire (scale=1) — labels mantêm factor.
  closed_loop_enable:       "Closed-loop enable (0=open-loop freeze, 1=on)",
  closed_loop_post_start_s: "Post-start delay before STFT (s)",
  ltft_adapt_min_rpm_x10:   "LTFT/LEARN min RPM (STFT free below)",
  stft_kp_x100:             "STFT Kp (raw ×100 no wire)",
  stft_ki_x1000:            "STFT Ki (raw ×1000 no wire)",
  stft_clamp_pct_x10:       "STFT clamp (raw ×10 % no wire)",
  ltft_mult_clamp_pct_x10:  "LTFT mult clamp ±(%)",
  ltft_add_clamp_us:        "LTFT add clamp (ms)",
  ltft_learn_div:           "LTFT IIR divisor (cell+=(stft-cell)/div)",
  ltft_commit_gain_pct:     "LEARN bake gain (% of mean STFT)",
  ltft_max_step_x10:        "LTFT max step %/tick (0=unlimited)",
  ltft_adapt_enable:        "LTFT adapt enable (0=STFT only, 1=learn LTFT)",
  ltft_learn_ready_hits:    "LEARN ready min hits",
  ltft_learn_max_err_x1000: "LEARN sample max |λ err|",
  ltft_learn_ready_max_mean_err: "LEARN ready max mean |λ err|",
  ltft_learn_ready_min_stft_x10: "LEARN ready min |mean STFT| (%)",
  ltft_learn_ready_max_stft_x10: "LEARN ready max |mean STFT| (%)",
  ltft_apply_burn_ve:       "After APPLY: burn VE (0=RAM only, 1=burn page1 @ RPM safe)",
  xtau_x_min_q8:            "X-τ X min (Q8)",
  xtau_x_max_q8:            "X-τ X max (Q8)",
  xtau_tau_min:             "X-τ τ min (cycles)",
  xtau_tau_max:             "X-τ τ max (cycles)",
  // EWG — scale=1 no wire (raw ×10)
  ewg_kp_x10:               "EWG Kp (raw ×10 no wire)",
  ewg_ki_x10:               "EWG Ki (raw ×10 no wire)",
  ewg_kd_x10:               "EWG Kd (raw ×10 no wire)",
  ewg_pos_min_raw:          "EWG pos closed (raw)",
  ewg_pos_max_raw:          "EWG pos open (raw)",
  eoi_idle_deg:             "EOI idle (° BTDC — 60=compressão, 365=pré-IVO)",
  eoi_blend_rpm_lo:         "Blend RPM início (0/0 = desligado)",
  eoi_blend_rpm_hi:         "Blend RPM fim (→ EOI target 355°)",
  mspark_max_rpm_x10:       "Multi-spark max RPM",
  mspark_count:             "Multi-spark extra sparks (0-3)",
  mspark_inter_dwell_ms_x10: "Multi-spark inter-dwell (ms)",
  // Dirigibilidade
  antijerk_tpsdot_threshold_x10: "Anti-jerk TPSdot threshold (%/s)",
  antijerk_retard_deg:           "Anti-jerk ignition retard (°)",
  antijerk_decay_cycles:         "Anti-jerk decay (cycles)",
  rev_limit_rpm_x10:             "Rev limiter hard cut (RPM)",
  rev_limit_soft_window_x10:     "Rev limit hysteresis (RPM)",
  ltft_add_pw_threshold_us:      "LTFT PW threshold (ms)",
  decel_cut_tps_threshold_x10:   "Decel cut TPS max (%)",
  decel_cut_entry_rpm_x10:       "Decel cut entry RPM",
  decel_cut_exit_rpm_x10:        "Decel cut exit RPM",
  decel_cut_min_clt_x10:         "Decel cut min CLT (°C)",
};

/* calibração assistida: campo → fonte do raw ao vivo na telemetria */
const CAL_CAPTURE = {
  app1_raw_min: "an1_raw", app1_raw_max: "an1_raw",
  app2_raw_min: "an2_raw", app2_raw_max: "an2_raw",
  etb_tps1_raw_min: "an3_raw", etb_tps1_raw_max: "an3_raw",
  etb_tps2_raw_min: "an4_raw", etb_tps2_raw_max: "an4_raw",
};
const READONLY_FIELDS = new Set(["config_magic"]);

/* Layout explícito: curvas 1D (eixo + 1..n linhas de valores), tabelas 2D
   e escalares. Nomes de campo = protocol.py / ui_protocol.cpp. */
const PAGE_LAYOUT = {
  0: {
    curves: [],
    tables2d: [],
    scalarSections: PAGE_0_SECTIONS,
  },
  5: {
    curves: [
      { title: "CLT correction",        axis: "clt_corr_axis_x10",     axisLabel: "CLT (°C)",   rows: [["clt_corr_x256", "factor ×256"]] },
      { title: "IAT correction",        axis: "iat_corr_axis_x10",     axisLabel: "IAT (°C)",   rows: [["iat_corr_x256", "factor ×256"]] },
      { title: "Warmup",              axis: "warmup_corr_axis_x10",  axisLabel: "CLT (°C)",   rows: [["warmup_corr_x256", "factor ×256"]] },
      { title: "Injector dead time vs VBat",     axis: "vbatt_corr_axis_mv",    axisLabel: "VBat (V)",     rows: [["injector_dead_time_us", "dead time (ms)"]] },
      { title: "AE vs CLT",           axis: "ae_clt_corr_axis_x10",  axisLabel: "CLT (°C)",   rows: [["ae_clt_sens", "sensitivity"]] },
      { title: "Dwell vs VBat",       axis: "dwell_vbatt_axis_mv",   axisLabel: "VBat (V)",     rows: [["dwell_ms_x10_table", "dwell (ms)"]] },
    ],
    tables2d: [
      { title: "Lambda delay (ms)", x: "lambda_delay_rpm_axis_x10", xLabel: "RPM",
        y: "lambda_delay_load_axis_bar_x100", yLabel: "MAP (bar)",
        values: "lambda_delay_ms_table" },
    ],
    scalarSections: [
      { label: "ACCEL ENRICHMENT (AE)",
        fields: ["ae_tpsdot_threshold_x10","ae_taper_cycles","ae_max_pw_us"] },
      { label: "IDLE SPARK",
        fields: ["idle_spark_tps_max_x10","idle_spark_map_max_bar_x100",
                 "idle_spark_rpm_min_x10","idle_spark_window_above_target_x10",
                 "idle_spark_deadband_rpm_x10","idle_spark_rpm_per_deg_x10",
                 "idle_spark_retard_limit_deg","idle_spark_advance_limit_deg"] },
    ],
  },
  6: {
    curves: [
      { title: "X-Tau vs CLT", axis: "xtau_clt_axis_x10", axisLabel: "CLT (°C)",
        rows: [["xtau_x_fraction_q8", "X (Q8)"], ["xtau_tau_cycles", "τ (ciclos)"]] },
      { title: "AE rate", axis: "ae_tpsdot_axis_x10", axisLabel: "TPSdot (%/s)",
        rows: [["ae_pw_adder_us", "PW adder (ms)"]] },
    ],
    tables2d: [],
  },
  7: {
    curves: [
      { title: "Dwell factor vs RPM", axis: "dwell_rpm_axis_rpm", axisLabel: "RPM",
        rows: [["dwell_rpm_factor_q8", "factor (Q8, 256=1.0×)"]] },
    ],
    tables2d: [],
  },
};

/* ── Boost map editor (page 9) ───────────────────────────────────────── */
const BOOST_RPM_AXIS    = [1500, 2000, 2500, 3000, 4000, 5000, 6500, 8000];
const BOOST_GEAR_LABELS = ["NEUTRAL", "1ª", "2ª", "3ª", "4ª", "5ª", "6ª"];
const BOOST_DEFAULTS = [
  [1000, 1020, 1050, 1080, 1100, 1120, 1150, 1180],
  [1000, 1050, 1100, 1150, 1200, 1250, 1280, 1300],
  [1000, 1080, 1150, 1220, 1280, 1340, 1380, 1420],
  [1000, 1100, 1180, 1260, 1330, 1400, 1450, 1500],
  [1000, 1120, 1210, 1300, 1380, 1460, 1520, 1580],
  [1000, 1140, 1240, 1340, 1430, 1520, 1600, 1680],
  [1000, 1150, 1260, 1370, 1470, 1570, 1660, 1750],
];

const BOOST_GEAR_COLORS = ["#888","#e8a020","#4caf50","#2196f3","#ff5722","#9c27b0","#00bcd4"];

async function loadBoostMap() {
  const root = $("#boostRoot");
  root.dataset.loaded = "1";
  root.innerHTML = `
    <div class="pm-header">
      <strong>BOOST TARGET</strong>
      <div class="pm-tabs" role="tablist">${BOOST_GEAR_LABELS.map((l,i)=>`<button type="button" role="tab" class="pm-tab${i===0?" active":""}" data-gear="${i}" aria-selected="${i===0?"true":"false"}">${l}</button>`).join("")}</div>
      <button type="button" id="boostReload" title="Lê boost map da ECU">Read</button>
      <button type="button" class="primary" id="boostSend" title="Escreve mapa na RAM (edições já vão ao soltar o ponto)">Write</button>
      <button type="button" class="danger"  id="boostBurn" title="Grava boost map no flash">Save</button>
      <span class="dirty" id="boostDirty"></span>
      <span class="flash-dirty" id="boostFlashDirty" hidden>não gravado em flash</span>
    </div>
    <canvas id="boostCanvas" width="760" height="500"></canvas>`;

  let rows = BOOST_DEFAULTS.map(r => [...r]);
  let activeGear = 0;
  let curve = null;

  async function sendBoost() {
    await api("/api/pages/9/cells", "PUT", { boost_map: rows });
    $("#boostDirty").textContent = "";
    const fd = $("#boostFlashDirty");
    if (fd) fd.hidden = false;
  }

  function buildCurve() {
    curve = makeDragCurve({
      canvas: $("#boostCanvas"),
      getValues: () => rows,
      getActive: () => activeGear,
      colors: BOOST_GEAR_COLORS,
      nPts: 8,
      xAxis: BOOST_RPM_AXIS,
      xLabel: "RPM",
      yLabel: "Boost (bar×1000)",
      // Range completo 1.0–3.0 bar, mas escala Y dividida: a faixa de foco
      // 1.0–2.0 bar ocupa 80% da altura (ticks de 0.1) e 2.0–3.0 os 20%
      // restantes (ticks de 0.25) — mais precisão onde se ajusta.
      yMin: 1000, yMax: 3000,
      yNorm: v => v <= 2000
        ? (v - 1000) / 1000 * 0.8
        : 0.8 + (v - 2000) / 1000 * 0.2,
      yDenorm: n => n <= 0.8
        ? 1000 + (n / 0.8) * 1000
        : 2000 + ((n - 0.8) / 0.2) * 1000,
      yTicks: [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900,
               2000, 2250, 2500, 2750, 3000],
      mono: false,
      onchange: () => { $("#boostDirty").textContent = "unsent"; },
      // Envia ao soltar o rato/dedo — não requer clicar "Send" à parte.
      ondragend: () => sendBoost().catch(e => toast(e.message, true)),
    });
  }

  function activateGear(g) {
    activeGear = g;
    root.querySelectorAll(".pm-tab").forEach(b => {
      const on = +b.dataset.gear === g;
      b.classList.toggle("active", on);
      b.setAttribute("aria-selected", on ? "true" : "false");
    });
    if (curve) curve.draw();
  }

  async function reload() {
    try {
      const d = await api("/api/pages/9");
      rows = d.boost_map;
    } catch { toast("ECU offline — showing defaults", false); }
    $("#boostDirty").textContent = "";
    if (curve) curve.draw();
  }

  buildCurve();
  // Capture on the tab strip so Read/Write/Save (or any overlay) cannot
  // steal the click; stopPropagation keeps global handlers out of the way.
  const boostTabs = root.querySelector(".pm-tabs");
  if (boostTabs) {
    boostTabs.addEventListener("click", e => {
      const b = e.target.closest(".pm-tab");
      if (!b || !boostTabs.contains(b)) return;
      e.preventDefault();
      e.stopPropagation();
      activateGear(+b.dataset.gear);
    });
  }

  $("#boostSend", root).onclick = async () => {
    try {
      await sendBoost();
      toast("Write · boost map → RAM");
    } catch (e) { toast(e.message, true); }
  };
  $("#boostBurn", root).onclick = async () => {
    // Write sempre antes de Save — sem isto, arrastar a curva e clicar
    // logo em Save persistia o valor ANTIGO em flash.
    try {
      await sendBoost();
      await api("/api/pages/9/burn", "POST");
      const fd = $("#boostFlashDirty");
      if (fd) fd.hidden = true;
      toast("Save OK · boost map");
    } catch (e) { toast(e.message, true); }
  };
  $("#boostReload", root).onclick = async () => {
    try { await reload(); toast("Read · boost map"); }
    catch (e) { toast(e.message, true); }
  };
  await reload();
}

/* ── LEARN: mean STFT % (page 12) ─────────────────────────────────────── */
// Apply = bake-in manual de TODAS as células com hits>0 → VE (não só ready).
// Contorno verde = célula "ready" (critérios de qualidade do FW); APPLY não
// se limita a ready — usa hits>0 (comando 'Y').
async function loadLtftAccum() {
  const root = $("#ltftAccumRoot");
  root.dataset.loaded = "1";
  root.innerHTML = `
    <div class="grid-toolbar">
      <strong>LEARN</strong>
      <span class="muted" title="Mapa de aprendizado: mean STFT % por célula (RPM×MAP); contorno verde = ready; APPLY grava em VE células com hits>0; RESET zera o acumulador">mean STFT %</span>
      <span class="learn-stats" id="ltftStats"></span>
      <button data-act="read">READ</button>
      <button data-act="apply">APPLY → VE</button>
      <button class="danger" data-act="reset">RESET</button>
    </div>
    <div class="grid-wrap" id="ltftAccumGrid"></div>`;

  let data = null;

  function render() {
    if (!data) {
      const g = $("#ltftAccumGrid");
      if (!g.innerHTML) g.innerHTML = `<p class="muted">—</p>`;
      return;
    }
    const n = data.mean_stft_x10.length;
    const flat = [];
    let nReady = 0, nHits = 0;
    for (let r = 0; r < n; r++)
      for (let c = 0; c < n; c++) {
        flat.push(data.mean_stft_x10[r][c]);
        if (data.ready[r][c]) nReady++;
        if (data.hits[r][c] > 0) nHits++;
      }
    $("#ltftStats").textContent = `ready ${nReady} · hits>0 ${nHits}`;
    const min = Math.min(...flat), max = Math.max(...flat);
    const rpm = (INFO && INFO.axes && INFO.axes.rpm) || [];
    const map = (INFO && INFO.axes && INFO.axes.map_kpa) || [];

    let html = `<table class="tune"><tr><th></th>` +
      rpm.map(x => `<th>${x}</th>`).join("") + "</tr>";
    for (let row = n - 1; row >= 0; row--) {
      html += `<tr><th>${map[row] ?? row}</th>`;
      for (let col = 0; col < n; col++) {
        const x10 = data.mean_stft_x10[row][col];
        const ready = data.ready[row][col];
        const hits = data.hits[row][col];
        const bg = H.heatColor(x10, min, max);
        // Verde = ready (qualidade). Hits>0 sem ready ficam só no heatmap.
        const outline = ready ? "outline:2px solid #22c55e;outline-offset:-2px" : "";
        const txt = (x10 / 10).toFixed(1);
        html += `<td style="background:${bg};color:${H.heatTextColor()};${outline}"
          title="${map[row]} kPa · ${rpm[col]} rpm · hits ${hits}${ready ? " · ready" : ""}">${txt}</td>`;
      }
      html += "</tr>";
    }
    html += "</table>";
    $("#ltftAccumGrid").innerHTML = html;
  }

  async function read() {
    try {
      data = await api("/api/pages/12");
      render();
    } catch (e) {
      toast(String(e.message || e), true);
      data = null;
      $("#ltftAccumGrid").innerHTML = `<p class="muted">sem dados</p>`;
    }
  }

  root.querySelector("[data-act=read]").onclick = read;
  root.querySelector("[data-act=apply]").onclick = async () => {
    if (!confirm("APPLY: bake-in LEARN → VE (todas as células com hits>0). Continuar?"))
      return;
    try {
      const r = await api("/api/ltft/apply-ready", { method: "POST" });
      toast(`APPLY · ${r.committed ?? 0} célula(s) → VE`);
      await read();
    } catch (e) { toast(e.message, true); }
  };
  root.querySelector("[data-act=reset]").onclick = async () => {
    if (!confirm("RESET: zera acumulador LEARN / adaptives. Continuar?"))
      return;
    try {
      await api("/api/adaptives/reset", { method: "POST" });
      toast("RESET OK");
      await read();
    } catch (e) { toast(e.message, true); }
  };

  await read();
}

/* ── CAN RX Map editor ────────────────────────────────────────────────── */
const CAN_RX_FIELDS = [
  { key: "id",          label: "Frame ID (hex)",   hex: true  },
  { key: "byte_lo",     label: "Byte LSB (0-7)",   hex: false },
  { key: "byte_hi",     label: "Byte MSB (255=8bit)",hex:false },
  { key: "shift_right", label: "Right shift",    hex: false },
  { key: "mask",        label: "Mask",          hex: true  },
  { key: "offset",      label: "Additive offset",   hex: false },
  { key: "timeout_ms",  label: "Timeout (ms)",     hex: false },
];

async function buildCanRxUI(container) {
  const div = document.createElement("div");
  div.className = "param-group";
  div.innerHTML = "";
  container.appendChild(div);

  let cfg = {};
  try { cfg = (await api("/api/can_rx_map")).signals; }
  catch { toast("CAN RX map: server offline", true); }

  const SIG_LABELS = {
    GEAR: "GEAR",
    SPEED_KMH: "VEHICLE SPEED (km/h) — body / ABS ref",
    WHEEL_SPEED_KMH: "DRIVEN WHEEL (km/h) — TC slip vs vehicle",
  };
  for (const sig of ["GEAR", "SPEED_KMH", "WHEEL_SPEED_KMH"]) {
    const sec = document.createElement("div");
    const sigLabel = SIG_LABELS[sig] || sig;
    sec.innerHTML = `<div class="pg-section-header">${sigLabel}</div>`;
    div.appendChild(sec);

    const vals = cfg[sig] || {};
    for (const f of CAN_RX_FIELDS) {
      const raw = vals[f.key] ?? 0;
      const disp = f.hex ? "0x" + raw.toString(16).toUpperCase() : raw;
      const row = document.createElement("div");
      row.className = "param-row";
      row.innerHTML = `<label>${f.label}</label>
        <input data-sig="${sig}" data-key="${f.key}" data-hex="${f.hex}" value="${disp}">`;
      sec.appendChild(row);
    }
  }

  // ── WBO2 CAN ID ────────────────────────────────────────────────────
  const wboSec = document.createElement("div");
  wboSec.innerHTML = `<div class="pg-section-header">WBO2 (WIDEBAND LAMBDA)</div>`;
  div.appendChild(wboSec);

  let wboId = 0x180;
  try { wboId = (await api("/api/wbo2_can_id")).id; } catch { /* offline */ }

  const wboRow = document.createElement("div");
  wboRow.className = "param-row";
  wboRow.innerHTML = `<label>Frame ID (hex)</label>
    <input id="wboIdInput" value="0x${wboId.toString(16).toUpperCase()}" style="width:80px">`;
  wboSec.appendChild(wboRow);

  // ── botões únicos que enviam tudo ──────────────────────────────────
  async function sendAll() {
    const inputs = div.querySelectorAll("input[data-sig]");
    const bySignal = {};
    for (const inp of inputs) {
      const { sig, key, hex } = inp.dataset;
      if (!bySignal[sig]) bySignal[sig] = {};
      bySignal[sig][key] = hex === "true"
        ? parseInt(inp.value.trim(), 16)
        : parseInt(inp.value.trim(), 10);
    }
    for (const [sig, fields] of Object.entries(bySignal))
      await api(`/api/can_rx_map/${sig}`, "PUT", fields);
    const id = parseInt($("#wboIdInput").value.trim(), 16);
    if (!isNaN(id) && id >= 1 && id <= 0x7FF)
      await api("/api/wbo2_can_id", "PUT", { id });
  }

  const btnRow = document.createElement("div");
  btnRow.className = "param-row pg-btns";
  btnRow.innerHTML = `
    <button class="primary" id="canRxSend" title="Escreve CAN RX na RAM">Write</button>
    <button class="danger"  id="canRxBurn" title="Grava CAN RX no flash">Save</button>`;
  div.appendChild(btnRow);

  div.querySelector("#canRxSend").onclick = async () => {
    try { await sendAll(); toast("Write · CAN RX → RAM"); }
    catch (e) { toast(e.message, true); }
  };

  div.querySelector("#canRxBurn").onclick = async () => {
    try {
      await sendAll();
      await api("/api/pages/0/burn", "POST", {});
      toast("Save OK · CAN RX");
    } catch (e) { toast(e.message, true); }
  };
}

async function loadParams() {
  const root = $("#paramsRoot");
  root.dataset.loaded = "1";
  root.innerHTML = `
    <div class="params-filter">
      <input type="search" id="paramsFilter" placeholder="Filtrar parâmetros…" autocomplete="off">
    </div>`;

  const filterInp = $("#paramsFilter", root);
  filterInp.oninput = () => {
    const q = filterInp.value.trim().toLowerCase();
    $$(".param-row", root).forEach(row => {
      if (row.classList.contains("pg-btns")) return;
      const text = row.textContent.toLowerCase();
      row.style.display = !q || text.includes(q) ? "" : "none";
    });
    // Mostra secções/acordeões que ainda têm linhas visíveis
    $$(".pg-section-header", root).forEach(h => {
      let el = h.nextElementSibling, any = false;
      while (el && !el.classList.contains("pg-section-header") && !el.classList.contains("pg-btns")) {
        if (el.classList.contains("param-row") && el.style.display !== "none") any = true;
        if (el.tagName === "H4" || el.tagName === "TABLE") {
          // curva/tabela: mostra se filtro vazio ou título casa
          const hit = !q || el.textContent.toLowerCase().includes(q) ||
            (el.previousElementSibling === h);
          if (el.tagName === "TABLE" && q) {
            const prev = el.previousElementSibling;
            const titleHit = prev && prev.tagName === "H4" && prev.textContent.toLowerCase().includes(q);
            el.style.display = titleHit || el.textContent.toLowerCase().includes(q) ? "" : "none";
            if (el.style.display !== "none") any = true;
          } else if (el.tagName === "H4") {
            el.style.display = !q || el.textContent.toLowerCase().includes(q) ? "" : "none";
            if (el.style.display !== "none") any = true;
          }
        }
        el = el.nextElementSibling;
      }
      h.style.display = !q || any || h.textContent.toLowerCase().includes(q) ? "" : "none";
    });
    $$(".pg-accordion", root).forEach(acc => {
      if (q && !acc.open) acc.open = true;
    });
  };

  for (const grp of PARAM_GROUPS) {
    const details = document.createElement("details");
    details.className = "pg-accordion";
    details.id = grp.id;
    // ENGINE aberto por default na 1ª visita
    if (grp.id === "pg-motor") details.open = true;
    details.innerHTML = `<summary class="pg-summary">
      <span class="pg-icon">${grp.icon}</span>
      <span class="pg-label">${grp.label}</span>
      <span class="pg-arrow">▶</span>
    </summary>`;
    root.appendChild(details);

    let opened = false;
    const loadGroup = () => {
      if (opened) return;
      opened = true;
      if (grp.canRx) {
        buildCanRxUI(details);
      } else {
        for (const page of grp.pages) {
          const div = document.createElement("div");
          div.className = "param-group";
          div.innerHTML = `
            <div class="pg-page-label">PG${page} — ${PARAM_PAGES[page]}</div>
            <div class="rows muted">a carregar…</div>
            <div class="param-row pg-btns">
              <button data-act="reload" title="Lê a página da ECU (RAM)">Read</button>
              <button class="primary" data-act="send" title="Escreve campos pendentes na RAM">Write</button>
              <button class="danger"  data-act="burn" title="Grava a página no flash">Save</button>
              <span class="flash-dirty" hidden>não gravado em flash</span>
            </div>`;
          details.appendChild(div);
          bindParamGroup(div, page);
        }
      }
    };
    details.addEventListener("toggle", () => {
      if (details.open) loadGroup();
    });
    if (details.open) loadGroup();
  }
}

async function bindParamGroup(div, page) {
  const rowsEl = $(".rows", div);
  let fields = {};
  const modified = new Set();
  let flashDirty = false;

  function updateFlashDirty() {
    const el = $(".flash-dirty", div);
    if (el) el.hidden = !flashDirty;
  }

  async function reload() {
    fields = (await api(`/api/pages/${page}`)).fields;
    modified.clear();
    render();
  }
  function render() {
    rowsEl.className = "rows";
    const layout = PAGE_LAYOUT[page] || { curves: [], tables2d: [] };
    const used = new Set();
    let html = "";

    const inp = (f, i, v, cls = "") =>
      `<input class="${cls}" data-f="${f}" data-i="${i}" value="${v}"
              ${READONLY_FIELDS.has(f) ? "disabled" : ""}>`;

    // curvas 1D: linha de eixo (editável) + linhas de valores
    for (const c of layout.curves) {
      used.add(c.axis);
      c.rows.forEach(([f]) => used.add(f));
      html += `<h4>${c.title}</h4><table class="ptable"><tr>
        <th>${c.axisLabel}</th>` +
        fields[c.axis].map((v, i) => `<td class="axis">${inp(c.axis, i, v)}</td>`).join("") +
        "</tr>" +
        c.rows.map(([f, label]) => `<tr><th>${label}</th>` +
          fields[f].map((v, i) => `<td>${inp(f, i, v)}</td>`).join("") + "</tr>").join("") +
        "</table>";
    }

    // tabelas 2D: eixo X no cabeçalho, eixo Y na 1ª coluna, valores row-major [y][x]
    for (const t of layout.tables2d) {
      used.add(t.x); used.add(t.y); used.add(t.values);
      const nx = fields[t.x].length;
      html += `<h4>${t.title}</h4><table class="ptable"><tr>
        <th>${t.yLabel} \\ ${t.xLabel}</th>` +
        fields[t.x].map((v, i) => `<td class="axis">${inp(t.x, i, v)}</td>`).join("") +
        "</tr>" +
        fields[t.y].map((yv, yi) => `<tr><td class="axis">${inp(t.y, yi, yv)}</td>` +
          fields[t.x].map((_, xi) => {
            const idx = yi * nx + xi;
            return `<td>${inp(t.values, idx, fields[t.values][idx])}</td>`;
          }).join("") + "</tr>").join("") +
        "</table>";
    }

    // escalares: page 0 e páginas com scalarSections usam subgrupos; demais exibem plano
    const remaining = Object.entries(fields).filter(([n]) => !used.has(n));
    const scalarSections = layout.scalarSections || (page === 0 ? PAGE_0_SECTIONS : []);
    if (scalarSections.length && remaining.length) {
      const fieldMap = Object.fromEntries(remaining);
      const rendered = new Set();
      for (const sec of scalarSections) {
        const secFields = sec.fields.filter(f => f in fieldMap);
        if (!secFields.length) continue;
        html += `<div class="pg-section-header">${sec.label}</div>`;
        if (sec.curves) {
          for (const c of sec.curves) {
            rendered.add(c.axis);
            c.rows.forEach(([f]) => rendered.add(f));
            if (fields[c.axis]) {
              html += `<h4>${c.title}</h4><table class="ptable"><tr>
                <th>${c.axisLabel}</th>` +
                fields[c.axis].map((v, i) => `<td class="axis">${inp(c.axis, i, v)}</td>`).join("") +
                "</tr>" +
                c.rows.map(([f, label]) => `<tr><th>${label}</th>` +
                  fields[f].map((v, i) => `<td>${inp(f, i, v)}</td>`).join("") + "</tr>").join("") +
                "</table>";
            }
          }
        }
        for (const name of secFields) {
          rendered.add(name);
          const v = fieldMap[name];
          const vals = Array.isArray(v) ? v : [v];
          const cap = CAL_CAPTURE[name]
            ? `<button class="cap" data-cap="${name}" data-src="${CAL_CAPTURE[name]}"
                       title="capture live value">◉ capture</button>
               <span class="muted live-raw" data-src="${CAL_CAPTURE[name]}"></span>`
            : "";
          html += `<div class="param-row"><label>${FIELD_LABELS[name] || name}</label>` +
            vals.map((x, i) => inp(name, i, x)).join("") + cap + "</div>";
        }
        if (sec.actions) {
          for (const act of sec.actions) {
            html += `<div class="param-row"><button class="${act.cls || 'primary'}"
              onclick="if(confirm('${act.confirm || 'Are you sure?'}'))fetch('${act.endpoint}',{method:'POST'}).then(r=>r.json()).then(()=>toast('${act.label} OK')).catch(e=>toast(e.message,true))">${act.label}</button></div>`;
          }
        }
      }
      // qualquer campo não coberto pelos subgrupos
      for (const [name, v] of remaining) {
        if (rendered.has(name)) continue;
        const vals = Array.isArray(v) ? v : [v];
        html += `<div class="param-row"><label>${FIELD_LABELS[name] || name}</label>` +
          vals.map((x, i) => inp(name, i, x)).join("") + "</div>";
      }
    } else {
      for (const [name, v] of remaining) {
        const vals = Array.isArray(v) ? v : [v];
        const cap = CAL_CAPTURE[name]
          ? `<button class="cap" data-cap="${name}" data-src="${CAL_CAPTURE[name]}"
                     title="capture live value">◉ capture</button>
             <span class="muted live-raw" data-src="${CAL_CAPTURE[name]}"></span>`
          : "";
        html += `<div class="param-row"><label>${FIELD_LABELS[name] || name}</label>` +
          vals.map((x, i) => inp(name, i, x)).join("") + cap + "</div>";
      }
    }
    rowsEl.innerHTML = html;
    $$("button.cap", rowsEl).forEach(btn => btn.onclick = () => {
      if (!RT) return toast("sem telemetria", true);
      const v = RT[btn.dataset.src];
      const input = rowsEl.querySelector(`input[data-f="${btn.dataset.cap}"]`);
      input.value = v;
      input.dispatchEvent(new Event("change"));
      toast(`${btn.dataset.cap} ← ${v}`);
    });
    $$("input", rowsEl).forEach(inp => inp.onchange = () => {
      const { f, i } = inp.dataset;
      const v = parseInt(inp.value, 10);
      if (Number.isNaN(v)) return;
      if (Array.isArray(fields[f])) fields[f][+i] = v; else fields[f] = v;
      modified.add(f);
      inp.classList.add("mod");
      // Envia de imediato para a RAM — não requer clicar "Send" à parte.
      sendModified().catch(err => toast(err.message, true));
    });
  }

  async function sendModified() {
    // Força o commit de um input em edição antes de checar `modified` —
    // clicar em Burn/Send sem sair do campo antes disparava o click sem o
    // onchange ter corrido, deixando `modified` vazio (nada enviado).
    const active = document.activeElement;
    if (active && active.tagName === "INPUT" && rowsEl.contains(active)) active.blur();
    if (!modified.size) return 0;
    const body = { fields: {} };
    modified.forEach(f => body.fields[f] = fields[f]);
    await api(`/api/pages/${page}/cells`, {
      method: "PUT", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const n = modified.size;
    // Não recria os inputs (render() completo) a cada envio automático —
    // só limpa a marca "não enviado" dos campos que acabaram de sair.
    modified.forEach(f => {
      rowsEl.querySelectorAll(`input[data-f="${f}"]`).forEach(el => el.classList.remove("mod"));
    });
    modified.clear();
    if (n) { flashDirty = true; updateFlashDirty(); }
    return n;
  }

  div.querySelector('[data-act="send"]').onclick = async () => {
    try {
      const n = await sendModified();
      if (!n) { toast("nada a escrever"); return; }
      toast(`Write · pág. ${page}: ${n} campo(s) → RAM`);
    } catch (e) { toast(e.message, true); }
  };
  div.querySelector('[data-act="burn"]').onclick = async () => {
    // Save grava o que já está em RAM no firmware — sem Write prévio dos
    // campos editados, o save persistia o valor ANTIGO silenciosamente.
    try {
      const n = await sendModified();
      await api(`/api/pages/${page}/burn`, { method: "POST" });
      flashDirty = false;
      updateFlashDirty();
      toast(n ? `Save OK · pág. ${page} · ${n} campo(s)` : `Save OK · pág. ${page}`);
    } catch (e) { toast(e.message, true); }
  };
  div.querySelector('[data-act="reload"]').onclick = async () => {
    try {
      await reload();
      toast(`Read · pág. ${page}`);
    } catch (e) { toast(e.message, true); }
  };
  await reload();
}

/* ── bench-mode (comando 'B' no protocolo) ────────────────────────────── */
// Força CLT=90°C / IAT=25°C (sem SENSOR_FAULT), λ=1.000 simulado no CAN stack,
// e relaxa timeouts CKP/CMP no firmware (HIL com estimulador). O FW expõe
// STATUS_BENCH_MODE (bit 15) no realtime — benchOn segue a ECU, não o host.
let benchOn = false;
function setBenchBtn(on) {
  benchOn = on;
  const b = $("#benchBtn");
  if (!b) return;
  b.textContent = on ? "BENCH ON" : "BENCH OFF";
  b.classList.toggle("on", on);
}
$("#benchBtn").onclick = async () => {
  const next = !benchOn;
  try {
    await api("/api/bench_mode", "POST", { on: next });
    setBenchBtn(next);
    toast(next
      ? "Bench ON · CLT=90°C IAT=25°C λ=1.000"
      : "Bench OFF · sensores reais");
  } catch (e) { toast(e.message, true); }
};

/* ── datalog ──────────────────────────────────────────────────────────── */
let logging = false;
function setLogBtn(rec) {
  $("#logBtn").textContent = rec ? "■ STOP LOG" : "● LOG";
  $("#logBtn").classList.toggle("rec", rec);
}
$("#logBtn").onclick = async () => {
  try {
    if (!logging) {
      const r = await api("/api/log/start", { method: "POST" });
      logging = true;
      setLogBtn(true);
      toast("a gravar: " + r.path);
    } else {
      const r = await api("/api/log/stop", { method: "POST" });
      logging = false;
      setLogBtn(false);
      toast("log guardado: " + r.path);
      window.open("/api/log/download");
    }
  } catch (e) { toast(e.message, true); }
};

$("#logExportBtn").onclick = () => window.open("/api/log/export");

/* ── Pedal Map (page 8) ──────────────────────────────────────────────── */
const PEDAL_MODES = ["ECO", "NORMAL", "SPORT", "RAIN"];
const PEDAL_AXIS  = [0, 10, 20, 30, 40, 50, 60, 70, 80, 100]; // labels do eixo pedal (10 pontos)
const PEDAL_COLORS = ["#4caf50","#2196f3","#ff5722","#9c27b0"];

let pedalMaps = null; // current edited state: float[4][10]

/* ── shared drag-curve engine ─────────────────────────────────────────── */
// Draws an interactive curve on a canvas.
// cfg: { canvas, values[], colors[], labels[], nPts, xAxis[], xLabel, yLabel,
//        yMin, yMax, onchange(idx,v), hitRadius=10, mono=false }
function makeDragCurve(cfg) {
  const cv = cfg.canvas;
  const ctx = cv.getContext("2d");
  const PAD = {l:52, r:18, t:18, b:42};
  let dragIdx = -1;

  // Resolve live series list / active index every call so preset tabs
  // (getters or plain fields) always drive the draw/edit path.
  function seriesList() {
    const v = typeof cfg.getValues === "function" ? cfg.getValues() : cfg.values;
    return v || [];
  }
  function activeIdx() {
    if (typeof cfg.getActive === "function") return cfg.getActive() | 0;
    return (cfg.active | 0);
  }
  function activeSeries() {
    const list = seriesList();
    const i = Math.max(0, Math.min(list.length - 1, activeIdx()));
    return list[i] || [];
  }

  function ptX(i) {
    const iW = cv.width - PAD.l - PAD.r;
    return PAD.l + i * iW / (cfg.nPts - 1);
  }
  // Mapeamento Y opcionalmente não-linear: yNorm(v)→[0..1] e o inverso
  // yDenorm — permite dar mais altura a uma faixa de interesse (ex.: boost
  // 1.0–2.0 bar maior que 2.0–3.0). Default = linear.
  const yNorm = cfg.yNorm ||
    (v => (v - cfg.yMin) / (cfg.yMax - cfg.yMin));
  const yDenorm = cfg.yDenorm ||
    (n => cfg.yMin + n * (cfg.yMax - cfg.yMin));
  function ptY(v) {
    const iH = cv.height - PAD.t - PAD.b;
    return PAD.t + iH - yNorm(v) * iH;
  }
  function yFromPx(py) {
    const iH = cv.height - PAD.t - PAD.b;
    return yDenorm(1 - (py - PAD.t) / iH);
  }

  function draw() {
    const W = cv.width, H = cv.height;
    const iW = W - PAD.l - PAD.r, iH = H - PAD.t - PAD.b;
    const act = activeIdx();
    const list = seriesList();
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#111"; ctx.fillRect(0, 0, W, H);

    // grid vertical (X) uniforme; horizontal (Y) segue os ticks — com yTicks
    // custom as linhas acompanham o mapeamento não-linear.
    ctx.strokeStyle = "#222"; ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      const x = PAD.l + i * iW / 10;
      ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, PAD.t + iH); ctx.stroke();
    }
    const yStep = (cfg.yMax - cfg.yMin) / 10;
    const yTicks = cfg.yTicks ||
      Array.from({ length: 11 }, (_, i) => cfg.yMin + i * yStep);
    for (const v of yTicks) {
      const y = ptY(v);
      ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + iW, y); ctx.stroke();
    }

    // axis labels
    ctx.fillStyle = "#666"; ctx.font = "11px monospace";
    ctx.textAlign = "right";
    for (const v of yTicks) {
      ctx.fillText(v % 1 === 0 ? v : v.toFixed(1), PAD.l - 5, ptY(v) + 4);
    }
    ctx.textAlign = "center";
    if (cfg.xAxis) {
      cfg.xAxis.forEach((x, i) => ctx.fillText(x, ptX(i), PAD.t + iH + 14));
    }
    ctx.fillStyle = "#888";
    ctx.fillText(cfg.xLabel || "", PAD.l + iW / 2, H - 4);
    ctx.save(); ctx.translate(12, PAD.t + iH / 2); ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center"; ctx.fillText(cfg.yLabel || "", 0, 0); ctx.restore();

    // all series faint
    list.forEach((vals, si) => {
      if (si === act || !vals) return;
      ctx.strokeStyle = (cfg.colors[si] || "#888") + "33"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      vals.forEach((v, i) => { i === 0 ? ctx.moveTo(ptX(i), ptY(v)) : ctx.lineTo(ptX(i), ptY(v)); });
      ctx.stroke();
    });

    // active series
    const vals = list[act] || [];
    const col = cfg.colors[act] || "#e8a020";
    ctx.strokeStyle = col; ctx.lineWidth = 2.5;
    ctx.beginPath();
    vals.forEach((v, i) => { i === 0 ? ctx.moveTo(ptX(i), ptY(v)) : ctx.lineTo(ptX(i), ptY(v)); });
    ctx.stroke();

    // points
    vals.forEach((v, i) => {
      const x = ptX(i), y = ptY(v);
      ctx.fillStyle = dragIdx === i ? "#fff" : col;
      ctx.beginPath(); ctx.arc(x, y, dragIdx === i ? 7 : 5, 0, Math.PI * 2); ctx.fill();
      // value label on hover
      if (dragIdx === i) {
        ctx.fillStyle = "#fff"; ctx.font = "bold 12px monospace"; ctx.textAlign = "center";
        ctx.fillText(v % 1 === 0 ? v : v.toFixed(1), x, y - 12);
      }
    });
  }

  function nearestPt(ex, ey) {
    const vals = activeSeries();
    const R = cfg.hitRadius || 12;
    let best = -1, bestD = R * R;
    vals.forEach((v, i) => {
      const dx = ex - ptX(i), dy = ey - ptY(v);
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    });
    return best;
  }

  function clampVal(idx, v) {
    v = Math.max(cfg.yMin, Math.min(cfg.yMax, v));
    if (cfg.mono) {
      const vals = activeSeries();
      if (idx > 0 && v < vals[idx - 1]) v = vals[idx - 1];
      if (idx < cfg.nPts - 1 && v > vals[idx + 1]) v = vals[idx + 1];
    }
    return v;
  }

  function writeActive(idx, v) {
    const list = seriesList();
    const act = activeIdx();
    if (!list[act]) return;
    list[act][idx] = v;
  }

  function evPos(e) {
    const r = cv.getBoundingClientRect();
    const src = e.touches ? e.touches[0] : e;
    // Scale CSS pixels → canvas buffer (handles max-width:100% shrink)
    const sx = cv.width / (r.width || cv.width);
    const sy = cv.height / (r.height || cv.height);
    return [(src.clientX - r.left) * sx, (src.clientY - r.top) * sy];
  }

  cv.addEventListener("mousedown", e => {
    const [ex, ey] = evPos(e);
    dragIdx = nearestPt(ex, ey);
    if (dragIdx >= 0) { e.preventDefault(); draw(); }
  });
  cv.addEventListener("mousemove", e => {
    const [ex, ey] = evPos(e);
    if (dragIdx >= 0) {
      e.preventDefault();
      const v = clampVal(dragIdx, yFromPx(ey));
      writeActive(dragIdx, v);
      cfg.onchange && cfg.onchange(dragIdx, v);
      draw();
    } else {
      const h = nearestPt(ex, ey);
      cv.style.cursor = h >= 0 ? "grab" : "default";
    }
  });
  const stopDrag = () => {
    const wasDragging = dragIdx >= 0;
    dragIdx = -1;
    draw();
    // Envia ao soltar o rato/dedo (fim do gesto) — não a cada mousemove,
    // que inundaria a ligação série durante o arrasto.
    if (wasDragging) cfg.ondragend && cfg.ondragend();
  };
  cv.addEventListener("mouseup",   stopDrag);
  cv.addEventListener("mouseleave", stopDrag);

  // touch
  cv.addEventListener("touchstart", e => {
    const [ex, ey] = evPos(e);
    dragIdx = nearestPt(ex, ey);
    if (dragIdx >= 0) e.preventDefault();
  }, {passive: false});
  cv.addEventListener("touchmove", e => {
    if (dragIdx < 0) return;
    e.preventDefault();
    const [, ey] = evPos(e);
    const v = clampVal(dragIdx, yFromPx(ey));
    writeActive(dragIdx, v);
    cfg.onchange && cfg.onchange(dragIdx, v);
    draw();
  }, {passive: false});
  cv.addEventListener("touchend", stopDrag);

  return { draw };
}

function buildPedalMapUI(data) {
  const root = $("#pedalMapRoot");
  root.dataset.loaded = "1";
  pedalMaps = data.pedal_maps.map(m => [...m]);

  root.innerHTML = `
    <div class="pm-header">
      <div class="pm-tabs" role="tablist">${PEDAL_MODES.map((m,i)=>`<button type="button" role="tab" class="pm-tab${i===0?" active":""}" data-mode="${i}" aria-selected="${i===0?"true":"false"}">${m}</button>`).join("")}</div>
      <button type="button" id="pmReload" title="Lê pedal map da ECU">Read</button>
      <button type="button" class="primary" id="pmSend" title="Escreve mapa na RAM (edições já vão ao soltar o ponto)">Write</button>
      <button type="button" class="danger"  id="pmBurn" title="Grava pedal map no flash">Save</button>
      <span class="flash-dirty" id="pmFlashDirty" hidden>não gravado em flash</span>
    </div>
    <canvas id="pmCanvas" width="760" height="500"></canvas>`;

  async function sendPedal() {
    await api("/api/pages/8/cells", "PUT", {pedal_maps: pedalMaps});
    const fd = $("#pmFlashDirty");
    if (fd) fd.hidden = false;
  }

  let activeMode = 0;
  const curve = makeDragCurve({
    canvas: $("#pmCanvas"),
    getValues: () => pedalMaps,
    getActive: () => activeMode,
    colors: PEDAL_COLORS,
    nPts: 10,
    xAxis: PEDAL_AXIS,
    xLabel: "Pedal %",
    yLabel: "Throttle %",
    yMin: 0, yMax: 100,
    mono: true,
    onchange: () => {},
    // Envia ao soltar o rato/dedo — não requer clicar "Send" à parte.
    ondragend: () => sendPedal().catch(e => toast(e.message, true)),
  });

  function activate(mode) {
    activeMode = mode;
    root.querySelectorAll(".pm-tab").forEach(b => {
      const on = +b.dataset.mode === mode;
      b.classList.toggle("active", on);
      b.setAttribute("aria-selected", on ? "true" : "false");
    });
    curve.draw();
  }

  const pedalTabs = root.querySelector(".pm-tabs");
  if (pedalTabs) {
    pedalTabs.addEventListener("click", e => {
      const b = e.target.closest(".pm-tab");
      if (!b || !pedalTabs.contains(b)) return;
      e.preventDefault();
      e.stopPropagation();
      activate(+b.dataset.mode);
    });
  }

  root.querySelector("#pmSend").onclick = async () => {
    try {
      await sendPedal();
      toast("Write · Pedal Map → RAM");
    } catch(e) { toast(e.message, true); }
  };
  root.querySelector("#pmBurn").onclick = async () => {
    // Write sempre antes de Save — sem isto, arrastar a curva e clicar
    // logo em Save persistia o valor ANTIGO em flash.
    try {
      await sendPedal();
      await api("/api/pages/8/burn", "POST");
      const fd = $("#pmFlashDirty");
      if (fd) fd.hidden = true;
      toast("Save OK · Pedal Map");
    } catch(e) { toast(e.message, true); }
  };
  root.querySelector("#pmReload").onclick = async () => {
    try {
      const data = await api("/api/pages/8");
      pedalMaps = data.pedal_maps.map(m => [...m]);
      activate(activeMode);
      toast("Read · Pedal Map");
    } catch(e) { toast(e.message, true); }
  };

  activate(0);
}

async function loadPedalMap() {
  try {
    const data = await api("/api/pages/8");
    buildPedalMapUI(data);
  } catch(e) {
    // ECU offline — show UI with defaults for offline editing
    buildPedalMapUI({
      pedal_maps: [
        [0,8,15,22,30,40,52,65,80,100],
        [0,10,20,30,40,50,60,70,80,100],
        [0,18,35,50,60,70,78,85,92,100],
        [0,5,10,15,22,30,40,52,65,100],
      ],
      modes: PEDAL_MODES,
      axis: PEDAL_AXIS,
    });
    toast("ECU disconnected — showing defaults", false);
  }
}

/* ── osciloscópio CKP/CMP ─────────────────────────────────────────────── */
// Desenha as bordas cruas dos rings do firmware ('K' via /api/scope):
// CKP na pista de cima com o GAP 60-2 destacado; CMP na pista de baixo com
// o dente âncora anotado; régua de ângulo do ciclo 720° (dente 0 pós-gap =
// 0°/360° conforme a fase CMP). Poll 3 Hz só com a aba TELEMETRY visível.
let scopeFrozen = false;
// Vista de ciclo completo: linha do tempo FIXA 0-720° (2 voltas de
// virabrequim). Cada poll traz ~1.1 volta de bordas; o acumulador
// client-side preenche o ciclo inteiro em ~2 polls e mantém o desenho
// estático — só o cursor (ângulo actual) se move. Onda quadrada estilo
// analisador lógico: dente = pulso de 3°, GAP = ausência de pulsos,
// CMP = pulso na pista de baixo no seu ângulo do ciclo.
const scopeSeen = { ckp: new Map(), cmp: new Map() };  // pos → wall-clock ms

async function drawScope() {
  const pane = $("#tab-telemetry");
  if (!pane.classList.contains("active") || scopeFrozen) return;
  let s;
  try { s = await api("/api/scope"); } catch { return; }
  // desenha o conteúdo estático num canvas offscreen; o cursor é animado
  // por requestAnimationFrame (scopeAnim) extrapolando o ângulo pelo RPM.
  if (!drawScope.off) drawScope.off = document.createElement("canvas");
  const vis = $("#scopeCanvas");
  const cv = drawScope.off;
  cv.width = vis.parentElement.clientWidth - 8;
  cv.height = vis.height;
  const ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  ctx.font = "10px monospace";
  const info = $("#scopeInfo");

  const synced = s.sync_state === 1 || s.sync_state === 2;
  if (!s.ckp_ms || s.ckp_ms.length < 4 || !synced) {
    scopeSeen.ckp.clear(); scopeSeen.cmp.clear();
    scopeAnchor = null;
    info.textContent = !s.ckp_ms || s.ckp_ms.length < 4
      ? "sem bordas CKP — sensor/estimulador parado"
      : "sem sync — aguardando referência angular (gap)";
    return;
  }

  // ângulo absoluto por borda (âncora: tooth_index/fase do dump)
  const n = s.ckp_ms.length;
  const deltas = [];
  for (let i = 1; i < n; i++) deltas.push(s.ckp_ms[i] - s.ckp_ms[i-1]);
  const med = [...deltas].sort((a, b) => a - b)[Math.floor(deltas.length / 2)];
  const angles = new Array(n);
  let a = (s.phase_a ? 0 : 360) + s.tooth_index * 6;
  angles[n-1] = ((a % 720) + 720) % 720;
  for (let i = n - 2; i >= 0; i--) {
    a -= Math.max(1, Math.round(deltas[i] / med)) * 6;
    angles[i] = ((a % 720) + 720) % 720;
  }

  // acumula no mapa de posições (arredonda a 6° = 1 dente; expira em 3s)
  const now = performance.now();
  for (const ang of angles)
    scopeSeen.ckp.set(Math.round(ang / 6) * 6 % 720, now);
  for (const t of (s.cmp_ms || [])) {
    if (t < s.ckp_ms[0] || t > s.ckp_ms[n-1]) continue;
    let i = 0;
    while (i < n - 1 && s.ckp_ms[i+1] < t) i++;
    const f = (t - s.ckp_ms[i]) / ((s.ckp_ms[i+1] - s.ckp_ms[i]) || 1);
    const ang = angles[i] + f * Math.max(1, Math.round(deltas[i] / med)) * 6;
    scopeSeen.cmp.set(Math.round(((ang % 720) + 720) % 720), now);
  }
  for (const m of [scopeSeen.ckp, scopeSeen.cmp])
    for (const [k, v] of m) if (now - v > 3000) m.delete(k);

  const x = deg => deg / 720 * (W - 20) + 10;
  const pw = x(3) - x(0);  // largura do pulso (3°)
  const yTop = 26, hPulse = 34, yCmp = 80, hCmp = 38, yBase = H - 16;

  // régua fixa: 0/90/…/720 + marcas de TDC (0°, 360°)
  ctx.strokeStyle = "#1c1c1c";
  ctx.fillStyle = "#4a4a4a";
  for (let d = 0; d <= 720; d += 90) {
    const px = x(d);
    ctx.beginPath(); ctx.moveTo(px, yTop - 8); ctx.lineTo(px, yBase); ctx.stroke();
    ctx.fillText(`${d}°`, px - (d === 720 ? 22 : 8), H - 4);
  }

  // pista CKP: onda quadrada (baseline + pulso por dente visto)
  ctx.strokeStyle = "#e8a020";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x(0), yTop + hPulse);
  ctx.lineTo(x(720), yTop + hPulse);
  ctx.stroke();
  ctx.fillStyle = "#e8a020";
  for (const pos of scopeSeen.ckp.keys())
    ctx.fillRect(x(pos), yTop, pw, hPulse);

  // GAPs (fim de cada volta: 348-360 e 708-720)
  ctx.fillStyle = "rgba(239,68,68,.18)";
  for (const g0 of [348, 708]) {
    ctx.fillRect(x(g0), yTop, x(g0 + 12) - x(g0), hPulse);
    ctx.fillStyle = "#ef4444";
    ctx.fillText("GAP", x(g0) - 2, yTop - 4);
    ctx.fillStyle = "rgba(239,68,68,.18)";
  }

  // pista CMP: onda quadrada com pulso no ângulo do came
  ctx.strokeStyle = "#8b5cf6";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x(0), yCmp + hCmp);
  ctx.lineTo(x(720), yCmp + hCmp);
  ctx.stroke();
  ctx.fillStyle = "#8b5cf6";
  const cmpList = [...scopeSeen.cmp.keys()].sort((p, q) => p - q);
  for (const pos of cmpList) {
    ctx.fillRect(x(Math.max(0, pos - 3)), yCmp, pw * 2, hCmp);
    ctx.fillText(`${pos}°`, x(pos) + 5, yCmp + 12);
  }

  // âncora do cursor animado: o ring é história — a borda mais recente
  // ocorreu há (agora - t_última_borda); extrapola pelo RPM (6°/ms por RPM/1000)
  const angNow = angles[n-1];
  scopeAnchor = { angle: angNow, t: performance.now(),
                  degPerMs: (RT && RT.rpm ? RT.rpm : 0) * 6 / 1000,
                  W, yTop, yBase };

  // rótulos
  ctx.fillStyle = "#4a4a4a";
  ctx.fillText("CKP", 10, yTop - 4);
  ctx.fillText("CMP", 10, yCmp - 4);

  const ref = s.cmp_ref_tooth;
  const gapDelta = deltas.find(d => d > med * 1.5);
  info.textContent =
    `ângulo actual: ${Math.round(angNow)}° de 720° · dente ${med.toFixed(2)}ms` +
    (gapDelta ? ` · GAP ${gapDelta.toFixed(2)}ms (${(gapDelta/med).toFixed(1)}×)` : "") +
    (cmpList.length ? ` · CMP @ ${cmpList.join("°, ")}°` : " · sem CMP visto") +
    (ref !== 255 ? ` (ancorado no dente ${ref})` : " (não-ancorado)");
}
setInterval(drawScope, 333);

// Animação do cursor: blita o traço estático e desenha o cursor com o
// ângulo extrapolado (60fps) — movimento contínuo entre polls de 333ms.
let scopeAnchor = null;
function scopeAnim() {
  requestAnimationFrame(scopeAnim);
  const off = drawScope.off;
  if (!off) return;
  if (!$("#tab-telemetry").classList.contains("active")) return;
  const vis = $("#scopeCanvas");
  if (vis.width !== off.width) vis.width = off.width;
  const ctx = vis.getContext("2d");
  ctx.clearRect(0, 0, vis.width, vis.height);
  ctx.drawImage(off, 0, 0);
  if (!scopeAnchor) return;
  const a = scopeAnchor;
  const adv = scopeFrozen ? 0 : (performance.now() - a.t) * a.degPerMs;
  const ang = (((a.angle + adv) % 720) + 720) % 720;
  const cx = ang / 720 * (a.W - 20) + 10;
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx, a.yTop - 8); ctx.lineTo(cx, a.yBase); ctx.stroke();
  ctx.fillStyle = "#22c55e";
  ctx.font = "10px monospace";
  ctx.fillText(`▼${Math.round(ang)}°`, Math.min(cx + 3, a.W - 48), a.yTop - 12);
}
requestAnimationFrame(scopeAnim);

$("#scopeFreezeBtn").onclick = () => {
  // congela o cursor no ângulo do momento (o traço estático já para no poll)
  if (!scopeFrozen && scopeAnchor) {
    const a = scopeAnchor;
    a.angle = (((a.angle + (performance.now() - a.t) * a.degPerMs) % 720) + 720) % 720;
    a.t = performance.now();
  }
  scopeFrozen = !scopeFrozen;
  const b = $("#scopeFreezeBtn");
  b.textContent = scopeFrozen ? "▶ RUN" : "⏸ FREEZE";
  b.classList.toggle("frozen", scopeFrozen);
};

/* ── teste de saídas ──────────────────────────────────────────────────── */
let otArmed = false;
let otTimer = null;   // keepalive + status poll (1s) enquanto armado

const OT_ABORT_REASONS = {0: "", 1: "abortado: RPM > 0 detectado",
                          2: "abortado: keepalive expirou"};

function otSetArmedUI(armed) {
  otArmed = armed;
  const root = $("#outputTestRoot");
  $("#otArmBtn", root).textContent = armed ? "DESARMAR" : "ARMAR";
  $("#otArmBtn", root).classList.toggle("armed", armed);
  $$(".ot-ctl", root).forEach(el => el.disabled = !armed);
  $("#otBanner", root).textContent = armed
    ? "⚠ MODO TESTE ARMADO — saídas sob controle manual"
    : "Modo teste desarmado — controles bloqueados";
  $("#otBanner", root).classList.toggle("armed", armed);
  if (!armed && otTimer) { clearInterval(otTimer); otTimer = null; }
}

async function otDisarm(silent = false) {
  try { await api("/api/output_test/exit", "POST"); } catch (e) {
    if (!silent) toast(e.message, true);
  }
  otSetArmedUI(false);
}

async function otArm() {
  if (RT && RT.rpm > 0) { toast(`bloqueado: motor a ${RT.rpm} RPM`, true); return; }
  try {
    await api("/api/output_test/enter", "POST");
  } catch (e) { toast(e.message, true); return; }
  otSetArmedUI(true);
  otTimer = setInterval(async () => {
    try {
      await api("/api/output_test/keepalive", "POST");
      const st = await api("/api/output_test/status");
      if (!st.active) {
        otSetArmedUI(false);
        toast("Modo teste " + (OT_ABORT_REASONS[st.abort_reason] || "encerrado pelo firmware"), true);
      }
    } catch (e) {
      otSetArmedUI(false);
      toast("keepalive falhou — teste desarmado: " + e.message, true);
    }
  }, 1000);
}

async function otFire(kind, cyl, usInputId) {
  const us = parseInt($(usInputId).value, 10);
  if (!us || us <= 0) { toast("largura de pulso inválida", true); return; }
  try {
    await api("/api/output_test/fire", "POST", { kind, cyl, us });
  } catch (e) { toast(e.message, true); }
}

async function otSet(target, value) {
  try {
    await api("/api/output_test/set", "POST", { target, value });
  } catch (e) { toast(e.message, true); }
}

function loadOutputTest() {
  const root = $("#outputTestRoot");
  root.dataset.loaded = "1";
  root.innerHTML = `
    <div class="ot-wrap">
      <div id="otBanner" class="ot-banner">Modo teste desarmado — controles bloqueados</div>
      <div class="ot-arm-row">
        <button id="otArmBtn" class="ot-arm">ARMAR</button>
        <span class="ot-hint">Só com motor parado. Auto-desarma se RPM &gt; 0 ou sem comunicação por 5s.</span>
      </div>

      <h4>Injetores (pulso único)</h4>
      <div class="ot-row">
        <label>PW (µs): <input id="otInjPw" class="ot-ctl" type="number" value="5000" min="100" max="30000" step="100" disabled></label>
        ${[0,1,2,3].map(i => `<button class="ot-ctl ot-fire" data-inj="${i}" disabled>INJ${i+1}</button>`).join("")}
      </div>

      <h4>Bobinas (dwell + faísca)</h4>
      <div class="ot-row">
        <label>Dwell (µs): <input id="otIgnDwell" class="ot-ctl" type="number" value="3000" min="500" max="10000" step="100" disabled></label>
        ${[0,1,2,3].map(i => `<button class="ot-ctl ot-fire" data-ign="${i}" disabled>IGN${i+1}</button>`).join("")}
      </div>

      <h4>Relés</h4>
      <div class="ot-row">
        <button class="ot-ctl" data-set="pump" data-val="1" disabled>BOMBA ON</button>
        <button class="ot-ctl" data-set="pump" data-val="0" disabled>BOMBA OFF</button>
        <button class="ot-ctl" data-set="fan" data-val="1" disabled>VENTOINHA ON</button>
        <button class="ot-ctl" data-set="fan" data-val="0" disabled>VENTOINHA OFF</button>
      </div>

      <h4>VVT (duty %)</h4>
      <div class="ot-row">
        <label>Escape: <input class="ot-ctl ot-slider" data-vvt="vvt_exh" type="range" min="0" max="100" value="0" disabled>
          <span id="otVvtExhVal">0%</span></label>
        <label>Admissão: <input class="ot-ctl ot-slider" data-vvt="vvt_int" type="range" min="0" max="100" value="0" disabled>
          <span id="otVvtIntVal">0%</span></label>
      </div>

      <h4>Motores (ETB / EWG — duty limitado a ±30%)</h4>
      <div class="ot-row">
        <label>ETB: <input class="ot-ctl ot-slider" data-motor="etb" data-scale="10.23" type="range" min="-30" max="30" value="0" disabled>
          <span id="otEtbVal">0%</span></label>
        <label>EWG: <input class="ot-ctl ot-slider" data-motor="ewg" data-scale="10" type="range" min="-30" max="30" value="0" disabled>
          <span id="otEwgVal">0%</span></label>
        <button id="otStopBtn" class="ot-ctl ot-stop" disabled>■ STOP MOTORES</button>
      </div>
    </div>`;

  $("#otArmBtn", root).onclick = () => otArmed ? otDisarm() : otArm();

  $$(".ot-fire", root).forEach(b => b.onclick = () => {
    if (b.dataset.inj !== undefined) otFire("inj", +b.dataset.inj, "#otInjPw");
    else otFire("ign", +b.dataset.ign, "#otIgnDwell");
  });

  $$("[data-set]", root).forEach(b => b.onclick = () =>
    otSet(b.dataset.set, +b.dataset.val));

  $$("[data-vvt]", root).forEach(sl => sl.oninput = () => {
    const pct = +sl.value;
    const span = sl.dataset.vvt === "vvt_exh" ? "#otVvtExhVal" : "#otVvtIntVal";
    $(span, root).textContent = pct + "%";
    otSet(sl.dataset.vvt, pct * 10);   // duty_pct_x10
  });

  $$("[data-motor]", root).forEach(sl => sl.oninput = () => {
    const pct = +sl.value;
    const span = sl.dataset.motor === "etb" ? "#otEtbVal" : "#otEwgVal";
    $(span, root).textContent = pct + "%";
    otSet(sl.dataset.motor, Math.round(pct * +sl.dataset.scale));
  });

  $("#otStopBtn", root).onclick = () => {
    $$("[data-motor]", root).forEach(sl => { sl.value = 0; });
    $("#otEtbVal", root).textContent = "0%";
    $("#otEwgVal", root).textContent = "0%";
    otSet("etb", 0); otSet("ewg", 0);
  };

  // Segurança extra: sair da página/aba desarma
  window.addEventListener("beforeunload", () => {
    if (otArmed) navigator.sendBeacon("/api/output_test/exit");
  });
}

/* ── init ─────────────────────────────────────────────────────────────── */
(async () => {
  connectWS();
  try {
    INFO = await api("/api/info");
    const fw = $("#fw");
    if (fw) {
      fw.textContent = INFO.connected
        ? `${INFO.signature || "OpenEMS"} · ${INFO.fw || "?"} · ${INFO.port || ""}`
        : (INFO.port ? `waiting · ${INFO.port}` : "server OK · no ECU");
    }
    setConnUI(!!INFO.connected, INFO.error || "");
    logging = !!INFO.logging;
    setLogBtn(logging);
  } catch (e) {
    setConnUI(false, e.message);
    toast(e.message, true);
  }
  // Default surface: VE grid (tune workflow)
  if (INFO && INFO.grid_pages) loadGrid($("#tab-grid-1"));
  else {
    const pane = $("#tab-grid-1");
    if (pane) {
      pane.innerHTML = `<div class="pane-head"><h2>VE</h2>
        <p class="pane-sub">Waiting for /api/info · server must be running</p></div>
        <div class="ot-banner">Cannot load grid metadata yet.</div>`;
    }
  }
})();
