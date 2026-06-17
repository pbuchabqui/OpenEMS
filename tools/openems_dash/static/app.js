/* OpenEMS Dashboard — telemetria WS 30Hz + editores de tabela */
"use strict";

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];

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
  const r = await fetch(path, opts);
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status} ${await r.text()}`);
  return r.json();
}

/* ── tabs ─────────────────────────────────────────────────────────────── */
$$("#sb-nav .tab").forEach(b => b.onclick = () => {
  $$("#sb-nav .tab").forEach(x => x.classList.toggle("active", x === b));
  $$(".pane").forEach(p => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  const pane = $("#tab-" + b.dataset.tab);
  if (pane.classList.contains("grid-pane") && !pane.dataset.loaded) loadGrid(pane);
  if (b.dataset.tab === "params"    && !$("#paramsRoot").dataset.loaded)   loadParams();
  if (b.dataset.tab === "pedal-map" && !$("#pedalMapRoot").dataset.loaded) loadPedalMap();
  if (b.dataset.tab === "boost"     && !$("#boostRoot").dataset.loaded)    loadBoostMap();
});

/* ── telemetria: gauges + charts ──────────────────────────────────────── */
const GAUGES = [
  ["rpm",          "RPM",     v => v],
  ["map_kpa",      "MAP bar", v => (v / 100).toFixed(2)],
  ["tps_pct",      "TPS %",   v => v],
  ["lambda_x1000", "λ",       v => (v / 1000).toFixed(2)],
  ["pw_ms",        "PW ms",   v => v.toFixed(2)],
  ["advance_deg",  "Ign °",   v => v],
  ["clt_c",        "CLT °C",  v => v],
  ["iat_c",        "IAT °C",  v => v],
  ["ve",           "VE %",    v => v],
  ["stft_pct",     "STFT %",  v => v],
];
$("#gauges").innerHTML = GAUGES.map(([k, l]) =>
  `<div class="gauge"><div class="v" id="g_${k}">—</div><div class="l">${l}</div></div>`).join("");


const LEDS = [
  ["FULL_SYNC", true], ["PHASE_A", true], ["SENSOR_FAULT", false],
  ["SCHED_LATE", false], ["SCHED_DROP", false], ["SCHED_CLAMP", false],
  ["WBO2_FAULT", false],
];
$("#statusLeds").innerHTML = LEDS.map(([k]) =>
  `<span class="led" id="led_${k}">[${k.replace(/_/g,"-")}]</span>`).join("");

/* strip-charts uPlot: janela deslizante de 60 s */
const WINDOW_S = 60, MAX_PTS = 60 * 35;
const chartDefs = [
  { title: "RPM",          series: [["rpm",           "#e8a020"]] },
  { title: "MAP / TPS",    series: [["map_kpa",       "#e8a020"], ["tps_pct",     "#22c55e"]] },
  { title: "λ / STFT",     series: [["lambda_x1000",  "#ef4444"], ["stft_pct",    "#e8a020"]] },
  { title: "PW / AVANÇO",  series: [["pw_ms",         "#e8a020"], ["advance_deg", "#22c55e"]] },
];
const charts = chartDefs.map(def => {
  const box = document.createElement("div");
  box.className = "chart-box";
  $("#charts").appendChild(box);
  const data = [[], ...def.series.map(() => [])];
  const u = new uPlot({
    width: box.clientWidth - 8 || 480, height: 160, title: def.title,
    scales: { x: { time: false } },
    axes: [
      { stroke: "#555555", grid: { stroke: "#1a1a1a" } },
      { stroke: "#555555", grid: { stroke: "#1a1a1a" } },
    ],
    series: [
      { label: "t" },
      ...def.series.map(([k, c]) => ({ label: k, stroke: c, width: 1.5 })),
    ],
  }, data, box);
  return { u, data, keys: def.series.map(([k]) => k) };
});
window.addEventListener("resize", () =>
  charts.forEach(c => c.u.setSize({ width: c.u.root.parentElement.clientWidth - 8, height: 160 })));

const t0 = performance.now();
function pushTelemetry(d) {
  const t = (performance.now() - t0) / 1000;
  for (const c of charts) {
    c.data[0].push(t);
    c.keys.forEach((k, i) => c.data[i + 1].push(d[k]));
    while (c.data[0].length > MAX_PTS || t - c.data[0][0] > WINDOW_S)
      c.data.forEach(a => a.shift());
    c.u.setData(c.data);
  }
  for (const [k, , fmt] of GAUGES)
    $(`#g_${k}`).textContent = fmt(d[k]);
  for (const [k, goodWhenOn] of LEDS) {
    const on = d.status[k];
    $(`#led_${k}`).className = "led" + (on ? (goodWhenOn ? " on-good" : " on-bad") : "");
  }
  $("#diag").textContent =
    `loop2ms ${d.loop2ms_last_us}µs (máx ${d.loop2ms_max_us}µs) · ` +
    `late ${d.late_events} · drops ${d.sched_drops} · clamps ${d.cal_clamps} · ` +
    `sync_state ${d.sync_state}`;
}

/* ── WebSocket ────────────────────────────────────────────────────────── */
function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onmessage = ev => {
    const d = JSON.parse(ev.data);
    const conn = d.connected;
    $("#conn").className = conn ? "on" : "off";
    $("#conn").textContent = conn ? "[ONLINE]" : "[OFFLINE]";
    if (conn && d.rpm !== undefined) {
      RT = d; pushTelemetry(d); highlightLiveCell();
      $$(".live-raw").forEach(el => el.textContent = `ao vivo: ${d[el.dataset.src]}`);
    }
  };
  ws.onclose = () => setTimeout(connectWS, 1000);
}

/* ── heatmap helpers ──────────────────────────────────────────────────── */
function heatColor(v, min, max) {
  const f = max > min ? (v - min) / (max - min) : 0;
  const h = (1 - f) * 240;          // 240=azul → 0=vermelho (faixa completa)
  const s = 90 + f * 10;            // 90–100% saturação
  const l = 28 + (1 - Math.abs(f - 0.5) * 2) * 14; // 28% nas extremidades, 42% no meio
  return `hsl(${h} ${s}% ${l}%)`;
}

/* ── editores de grid (VE/Spark/Lambda) ───────────────────────────────── */
const gridState = {};   // page → {values, modified:Set("r,c"), table}

async function loadGrid(pane) {
  const page = +pane.dataset.page;
  const meta = INFO.grid_pages[page];
  pane.innerHTML = `
    <div class="grid-toolbar">
      <strong>${meta.name}</strong> <span class="muted">(${meta.unit})</span>
      <button class="primary" data-act="send">Enviar (RAM)</button>
      <button class="danger" data-act="burn">Burn → flash</button>
      <button data-act="reload">Reler</button>
      <span class="dirty"></span>
      <span class="muted">eixo X = RPM, eixo Y = MAP (kPa)</span>
    </div>
    <div class="grid-wrap"></div>`;
  pane.dataset.loaded = "1";

  const st = gridState[page] = { values: null, modified: new Set(), pane };

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
    // o índice real da página — edição/tracing não mudam)
    for (let row = 15; row >= 0; row--) {
      html += `<tr><th>${INFO.axes.map_kpa[row]}</th>`;
      for (let col = 0; col < 16; col++) {
        const v = st.values[row][col];
        const mod = st.modified.has(`${row},${col}`) ? " mod" : "";
        html += `<td class="${mod}" data-r="${row}" data-c="${col}"
                     style="background:${heatColor(v, min, max)}">${v}</td>`;
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
      ? `${st.modified.size} célula(s) não enviada(s)` : "";
    bindCells();
  }

  function bindCells() {
    $$("td[data-r]", pane).forEach(td => {
      td.onclick = () => beginEdit(td);
    });
  }

  function beginEdit(td) {
    if (td.querySelector("input")) return;
    const r = +td.dataset.r, c = +td.dataset.c;
    const inp = document.createElement("input");
    inp.value = st.values[r][c];
    td.textContent = "";
    td.appendChild(inp);
    inp.focus(); inp.select();
    const commit = () => {
      const v = parseInt(inp.value, 10);
      if (!Number.isNaN(v) && v !== st.values[r][c]) {
        st.values[r][c] = v;
        st.modified.add(`${r},${c}`);
      }
      render();
    };
    inp.onblur = commit;
    inp.onkeydown = e => {
      if (e.key === "Enter") inp.blur();
      if (e.key === "Escape") { inp.value = st.values[r][c]; inp.blur(); }
    };
  }

  pane.querySelector('[data-act="send"]').onclick = async () => {
    if (!st.modified.size) return toast("nada a enviar");
    const cells = [...st.modified].map(k => {
      const [row, col] = k.split(",").map(Number);
      return { row, col, value: st.values[row][col] };
    });
    try {
      await api(`/api/pages/${page}/cells`, {
        method: "PUT", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ cells }),
      });
      st.modified.clear(); render();
      toast(`${cells.length} célula(s) enviadas (RAM)`);
    } catch (e) { toast(e.message, true); }
  };
  pane.querySelector('[data-act="burn"]').onclick = async () => {
    try { await api(`/api/pages/${page}/burn`, { method: "POST" }); toast("burn OK"); }
    catch (e) { toast(e.message, true); }
  };
  pane.querySelector('[data-act="reload"]').onclick = reload;

  await reload();
}

/* célula corrente do motor destacada ao vivo.
   Espelha axis_lookup() do firmware (table3d.cpp): eixos são NÓS de
   interpolação — o ponto de operação mistura os 4 nós vizinhos com peso
   pela posição real. Destacamos os 4 (dominante mais forte). */
function axisLookup(axis, v) {
  const last = axis.length - 1;
  if (v <= axis[0]) return { idx: 0, frac: 0 };
  if (v >= axis[last]) return { idx: last - 1, frac: 1 };
  let i = last - 1;
  while (i > 0 && v < axis[i]) i--;
  return { idx: i, frac: (v - axis[i]) / (axis[i + 1] - axis[i]) };
}
const TRAIL_MS = 10000;
function highlightLiveCell() {
  if (!RT || !INFO) return;
  const pane = $(".grid-pane.active");
  if (!pane) return;
  const page = +pane.dataset.page;
  const st = gridState[page];
  const cv = pane.querySelector("canvas.trail");
  if (!st || !cv) return;

  const lx = axisLookup(INFO.axes.rpm, RT.rpm);
  const ly = axisLookup(INFO.axes.map_kpa, RT.map_kpa);

  // células de contexto da bilinear (tracejado fraco) — sem contorno no dominante
  $$("td.live2", pane).forEach(td => td.classList.remove("live2"));
  for (const r of [ly.idx, ly.idx + 1])
    for (const c of [lx.idx, lx.idx + 1]) {
      const td = pane.querySelector(`td[data-r="${r}"][data-c="${c}"]`);
      if (td) td.classList.add("live2");
    }

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

  // célula dominante (nó mais próximo da posição interpolada) visitada agora
  const domR = ly.idx + (ly.frac >= 0.5 ? 1 : 0);
  const domC = lx.idx + (lx.frac >= 0.5 ? 1 : 0);
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
const PARAM_PAGES = { 0: "Configuração do motor", 5: "Correções 1D",
                      6: "X-Tau / AE / Crank", 7: "Dwell 2D" };

/* Metadados dos grupos de parâmetros: ícone, rótulo da seção, página(s) */
const PARAM_GROUPS = [
  { id: "pg-motor",    icon: "⚙",  label: "MOTOR",       pages: [0] },
  { id: "pg-inject",   icon: "⛽", label: "INJEÇÃO",      pages: [5, 6] },
  { id: "pg-ignition", icon: "⚡", label: "IGNIÇÃO",      pages: [7] },
  { id: "pg-canrx",    icon: "⇄",  label: "CAN RX",       pages: [], canRx: true },
];

/* Subgrupos de campos escalares dentro da página 0 */
const PAGE_0_SECTIONS = [
  {
    label: "MOTOR",
    fields: ["ivc_abdc_deg","displacement_cc","trigger_tooth0_engine_deg",
             "default_soi_lead_deg","config_magic"],
  },
  {
    label: "INJEÇÃO",
    fields: ["injector_flow_cc_min","stoich_afr_x100","map_ref_bar_x100"],
  },
  {
    label: "SENSORES APP",
    fields: ["app1_raw_min","app1_raw_max","app2_raw_min","app2_raw_max",
             "app_max_delta_pct_x10"],
  },
  {
    label: "BORBOLETA ETB",
    fields: ["etb_tps1_raw_min","etb_tps1_raw_max","etb_tps2_raw_min","etb_tps2_raw_max",
             "etb_max_delta_pct_x10","etb_max_open_pct_x10_limp",
             "etb_max_rate_pct_per_s","etb_idle_open_pct_x10","etb_cal_valid",
             "etb_harness_present","etb_kp_x10","etb_ki_x10","etb_kd_x10"],
  },
  {
    label: "TRIM CILINDROS",
    fields: ["cyl_fuel_trim_pct_0","cyl_fuel_trim_pct_1","cyl_fuel_trim_pct_2","cyl_fuel_trim_pct_3",
             "cyl_ign_trim_deg_0","cyl_ign_trim_deg_1","cyl_ign_trim_deg_2","cyl_ign_trim_deg_3"],
  },
  {
    label: "CMP",
    fields: ["cmp_window_open_tooth","cmp_window_close_tooth"],
  },
  {
    label: "MARCHA LENTA",
    fields: ["etb_idle_rpm_target","etb_idle_min_opening_x10","etb_idle_max_opening_x10"],
  },
  {
    label: "DIRIGIBILIDADE",
    fields: ["antijerk_tpsdot_threshold_x10","antijerk_retard_deg","antijerk_decay_cycles",
             "rev_limit_rpm_x10","rev_limit_soft_window_x10","rev_limit_spark_window_x10",
             "rev_limit_max_retard_deg","ltft_add_pw_threshold_us",
             "decel_cut_tps_threshold_x10","decel_cut_entry_rpm_x10",
             "decel_cut_exit_rpm_x10","decel_cut_min_clt_x10"],
  },
];

/* rótulos amigáveis (página 0) e campos read-only */
const FIELD_LABELS = {
  ivc_abdc_deg:              "Fecho admissão IVC (° ABDC)",
  displacement_cc:           "Cilindrada (cc)",
  injector_flow_cc_min:      "Vazão do bico (cc/min)",
  stoich_afr_x100:           "AFR estequiométrico",
  map_ref_bar_x100:          "MAP de referência (bar)",
  trigger_tooth0_engine_deg: "Offset trigger dente 0 (°, 0-719)",
  default_soi_lead_deg:      "Avanço SOI padrão (°)",
  config_magic:              "Magic (0x4543 = config válida)",
  app1_raw_min:  "APP1 pedal solto (raw)",   app1_raw_max:  "APP1 pedal fundo (raw)",
  app2_raw_min:  "APP2 pedal solto (raw)",   app2_raw_max:  "APP2 pedal fundo (raw)",
  etb_tps1_raw_min: "ETB TPS1 fechada (raw)", etb_tps1_raw_max: "ETB TPS1 aberta (raw)",
  etb_tps2_raw_min: "ETB TPS2 fechada (raw)", etb_tps2_raw_max: "ETB TPS2 aberta (raw)",
  app_max_delta_pct_x10: "Plausibilidade APP Δmáx (%)",
  etb_max_delta_pct_x10: "Plausibilidade ETB Δmáx (%)",
  etb_max_open_pct_x10_limp: "ETB abertura máx limp (%)",
  etb_max_rate_pct_per_s: "ETB taxa máx (%/s)",
  etb_idle_open_pct_x10: "ETB abertura idle (%)",
  etb_cal_valid: "Calibração ETB válida (0/1)",
  etb_harness_present: "Chicote ETB presente (0/1)",
  etb_kp_x10: "ETB PID Kp", etb_ki_x10: "ETB PID Ki", etb_kd_x10: "ETB PID Kd",
  cyl_fuel_trim_pct_0: "Trim comb. cil.1 (%)", cyl_fuel_trim_pct_1: "Trim comb. cil.2 (%)",
  cyl_fuel_trim_pct_2: "Trim comb. cil.3 (%)", cyl_fuel_trim_pct_3: "Trim comb. cil.4 (%)",
  cyl_ign_trim_deg_0:  "Trim ign. cil.1 (°)",  cyl_ign_trim_deg_1:  "Trim ign. cil.2 (°)",
  cyl_ign_trim_deg_2:  "Trim ign. cil.3 (°)",  cyl_ign_trim_deg_3:  "Trim ign. cil.4 (°)",
  cmp_window_open_tooth:  "CMP janela abre (dente)",
  cmp_window_close_tooth: "CMP janela fecha (dente; 0/0=desabilitado)",
  // Page 5 scalars
  ae_tpsdot_threshold_x10: "AE TPSdot threshold (%/s)",
  ae_taper_cycles:         "AE taper (ciclos)",
  ae_max_pw_us:            "AE PW máximo (ms)",
  idle_spark_tps_max_x10:              "Idle spark TPS máx. (%)",
  idle_spark_map_max_bar_x100:         "Idle spark MAP máx. (bar)",
  idle_spark_rpm_min_x10:              "Idle spark RPM mín.",
  idle_spark_window_above_target_x10:  "Idle spark janela acima target (RPM)",
  idle_spark_deadband_rpm_x10:         "Idle spark deadband (RPM)",
  idle_spark_rpm_per_deg_x10:          "Idle spark RPM/°",
  idle_spark_retard_limit_deg:         "Idle spark retardo máx. (°)",
  idle_spark_advance_limit_deg:        "Idle spark avanço máx. (°)",
  // Marcha lenta ETB + IAC
  etb_idle_rpm_target:      "RPM alvo marcha lenta",
  etb_idle_min_opening_x10: "Abertura mínima idle ETB (%)",
  etb_idle_max_opening_x10: "Abertura máxima idle ETB (%)",
  iac_clt_axis_x10:         "Eixo CLT — RPM alvo marcha lenta (°C, 8pts)",
  iac_idle_target_rpm_x10:  "RPM alvo marcha lenta vs CLT (8pts)",
  // Dirigibilidade
  antijerk_tpsdot_threshold_x10: "Anti-jerk TPSdot threshold (%/s)",
  antijerk_retard_deg:            "Anti-jerk retardo ignição (°)",
  antijerk_decay_cycles:          "Anti-jerk decay (ciclos)",
  rev_limit_rpm_x10:              "Limitador RPM",
  rev_limit_soft_window_x10:      "Rev limit janela corte injeção (RPM)",
  rev_limit_spark_window_x10:     "Rev limit janela retardo ign. (RPM)",
  rev_limit_max_retard_deg:       "Rev limit retardo máx. ignição (°)",
  ltft_add_pw_threshold_us:       "LTFT threshold PW (ms)",
  decel_cut_tps_threshold_x10:    "Decel cut TPS máx. (%)",
  decel_cut_entry_rpm_x10:        "Decel cut RPM entrada",
  decel_cut_exit_rpm_x10:         "Decel cut RPM saída/histerese",
  decel_cut_min_clt_x10:          "Decel cut CLT mín. (°C)",
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
    curves: [
      { title: "RPM alvo marcha lenta vs CLT", axis: "iac_clt_axis_x10", axisLabel: "CLT (°C)",
        rows: [["iac_idle_target_rpm_x10", "RPM alvo"]] },
    ],
    tables2d: [],
    scalarSections: PAGE_0_SECTIONS,
  },
  5: {
    curves: [
      { title: "Correção CLT",        axis: "clt_corr_axis_x10",     axisLabel: "CLT (°C)",   rows: [["clt_corr_x256", "fator ×256"]] },
      { title: "Correção IAT",        axis: "iat_corr_axis_x10",     axisLabel: "IAT (°C)",   rows: [["iat_corr_x256", "fator ×256"]] },
      { title: "Warmup",              axis: "warmup_corr_axis_x10",  axisLabel: "CLT (°C)",   rows: [["warmup_corr_x256", "fator ×256"]] },
      { title: "Injetor vs VBat",     axis: "vbatt_corr_axis_mv",    axisLabel: "VBat (V)",     rows: [["injector_dead_time_us", "dead time (ms)"]] },
      { title: "AE vs CLT",           axis: "ae_clt_corr_axis_x10",  axisLabel: "CLT (°C)",   rows: [["ae_clt_sens", "sensibilidade"]] },
      { title: "Dwell vs VBat",       axis: "dwell_vbatt_axis_mv",   axisLabel: "VBat (V)",     rows: [["dwell_ms_x10_table", "dwell (ms)"]] },
    ],
    tables2d: [
      { title: "Atraso lambda (ms)", x: "lambda_delay_rpm_axis_x10", xLabel: "RPM",
        y: "lambda_delay_load_axis_bar_x100", yLabel: "MAP (bar)",
        values: "lambda_delay_ms_table" },
    ],
    scalarSections: [
      { label: "ENRIQUECIMENTO ACELERAÇÃO (AE)",
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
      { title: "Fator de dwell vs RPM", axis: "dwell_rpm_axis_rpm", axisLabel: "RPM",
        rows: [["dwell_rpm_factor_q8", "fator (Q8, 256=1.0×)"]] },
    ],
    tables2d: [],
  },
};

/* ── Boost map editor (page 9) ───────────────────────────────────────── */
const BOOST_RPM_AXIS    = [1500, 2000, 2500, 3000, 4000, 5000, 6500, 8000];
const BOOST_GEAR_LABELS = ["NEUTRO", "1ª", "2ª", "3ª", "4ª", "5ª", "6ª"];
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
      <div class="pm-tabs">${BOOST_GEAR_LABELS.map((l,i)=>`<button class="pm-tab${i===0?" active":""}" data-gear="${i}">${l}</button>`).join("")}</div>
      <button class="primary" id="boostSend">Enviar (RAM)</button>
      <button class="danger"  id="boostBurn">Burn → flash</button>
      <button id="boostReload">Reler</button>
      <span class="dirty" id="boostDirty"></span>
    </div>
    <canvas id="boostCanvas" width="760" height="500"></canvas>`;

  let rows = BOOST_DEFAULTS.map(r => [...r]);
  let activeGear = 0;
  let curve = null;

  function buildCurve() {
    curve = makeDragCurve({
      canvas: $("#boostCanvas"),
      get values() { return rows; },
      get active() { return activeGear; },
      colors: BOOST_GEAR_COLORS,
      nPts: 8,
      xAxis: BOOST_RPM_AXIS,
      xLabel: "RPM",
      yLabel: "Boost (bar×1000)",
      yMin: 800, yMax: 2200,
      mono: false,
      onchange: () => { $("#boostDirty").textContent = "não enviado"; },
    });
  }

  function activateGear(g) {
    activeGear = g;
    root.querySelectorAll(".pm-tab").forEach(b => b.classList.toggle("active", +b.dataset.gear === g));
    curve.draw();
  }

  async function reload() {
    try {
      const d = await api("/api/pages/9");
      rows = d.boost_map;
    } catch { toast("ECU offline — mostrando defaults", false); }
    $("#boostDirty").textContent = "";
    curve.draw();
  }

  buildCurve();
  root.querySelectorAll(".pm-tab").forEach(b => b.onclick = () => activateGear(+b.dataset.gear));

  $("#boostSend", root).onclick = async () => {
    try {
      await api("/api/pages/9/cells", "PUT", { boost_map: rows });
      $("#boostDirty").textContent = "";
      toast("boost map enviado (RAM)");
    } catch (e) { toast(e.message, true); }
  };
  $("#boostBurn", root).onclick = async () => {
    try { await api("/api/pages/9/burn", "POST"); toast("burn OK"); }
    catch (e) { toast(e.message, true); }
  };
  $("#boostReload", root).onclick = reload;
  await reload();
}

/* ── CAN RX Map editor ────────────────────────────────────────────────── */
const CAN_RX_FIELDS = [
  { key: "id",          label: "Frame ID (hex)",   hex: true  },
  { key: "byte_lo",     label: "Byte LSB (0-7)",   hex: false },
  { key: "byte_hi",     label: "Byte MSB (255=8bit)",hex:false },
  { key: "shift_right", label: "Shift direito",    hex: false },
  { key: "mask",        label: "Máscara",          hex: true  },
  { key: "offset",      label: "Offset aditivo",   hex: false },
  { key: "timeout_ms",  label: "Timeout (ms)",     hex: false },
];

async function buildCanRxUI(container) {
  const div = document.createElement("div");
  div.className = "param-group";
  div.innerHTML = `<div class="muted" style="font-size:10px;margin-bottom:8px">
    Configuração RAM-only — ativa até o próximo reset da ECU.</div>`;
  container.appendChild(div);

  let cfg = {};
  try { cfg = (await api("/api/can_rx_map")).signals; }
  catch { toast("CAN RX map: servidor offline", true); }

  for (const sig of ["GEAR", "SPEED_KMH"]) {
    const sec = document.createElement("div");
    const sigLabel = sig === "GEAR" ? "MARCHA" : "VELOCIDADE (km/h)";
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

  const btnRow = document.createElement("div");
  btnRow.className = "param-row pg-btns";
  btnRow.innerHTML = `<button class="primary" id="canRxSend">Enviar (RAM)</button>`;
  div.appendChild(btnRow);

  div.querySelector("#canRxSend").onclick = async () => {
    const inputs = div.querySelectorAll("input[data-sig]");
    const bySignal = {};
    for (const inp of inputs) {
      const { sig, key, hex } = inp.dataset;
      if (!bySignal[sig]) bySignal[sig] = {};
      const raw = inp.value.trim();
      bySignal[sig][key] = hex === "true" ? parseInt(raw, 16) : parseInt(raw, 10);
    }
    try {
      for (const [sig, fields] of Object.entries(bySignal)) {
        await api(`/api/can_rx_map/${sig}`, {
          method: "PUT", headers: { "Content-Type": "application/json" },
          body: JSON.stringify(fields),
        });
      }
      toast("CAN RX map enviado");
    } catch (e) { toast(e.message, true); }
  };
}

async function loadParams() {
  const root = $("#paramsRoot");
  root.dataset.loaded = "1";
  root.innerHTML = "";

  for (const grp of PARAM_GROUPS) {
    const details = document.createElement("details");
    details.className = "pg-accordion";
    details.id = grp.id;
    details.innerHTML = `<summary class="pg-summary">
      <span class="pg-icon">${grp.icon}</span>
      <span class="pg-label">${grp.label}</span>
      <span class="pg-arrow">▶</span>
    </summary>`;
    root.appendChild(details);

    let opened = false;
    details.addEventListener("toggle", () => {
      if (details.open && !opened) {
        opened = true;
        if (grp.canRx) {
          buildCanRxUI(details);
        } else {
          for (const page of grp.pages) {
            const div = document.createElement("div");
            div.className = "param-group";
            div.innerHTML = `
              <div class="pg-page-label">PG${page} — ${PARAM_PAGES[page]}</div>
              <div class="rows muted">carregando…</div>
              <div class="param-row pg-btns">
                <button class="primary" data-act="send">Enviar (RAM)</button>
                <button class="danger"  data-act="burn">Burn → flash</button>
                <button data-act="reload">Reler</button>
              </div>`;
            details.appendChild(div);
            bindParamGroup(div, page);
          }
        }
      }
    });
  }
}

async function bindParamGroup(div, page) {
  const rowsEl = $(".rows", div);
  let fields = {};
  const modified = new Set();

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

    // escalares: página 0 e páginas com scalarSections usam subgrupos; demais exibem plano
    const remaining = Object.entries(fields).filter(([n]) => !used.has(n));
    const scalarSections = layout.scalarSections || (page === 0 ? PAGE_0_SECTIONS : []);
    if (scalarSections.length && remaining.length) {
      const fieldMap = Object.fromEntries(remaining);
      const rendered = new Set();
      for (const sec of scalarSections) {
        const secFields = sec.fields.filter(f => f in fieldMap);
        if (!secFields.length) continue;
        html += `<div class="pg-section-header">${sec.label}</div>`;
        for (const name of secFields) {
          rendered.add(name);
          const v = fieldMap[name];
          const vals = Array.isArray(v) ? v : [v];
          const cap = CAL_CAPTURE[name]
            ? `<button class="cap" data-cap="${name}" data-src="${CAL_CAPTURE[name]}"
                       title="capturar valor ao vivo">◉ capturar</button>
               <span class="muted live-raw" data-src="${CAL_CAPTURE[name]}"></span>`
            : "";
          html += `<div class="param-row"><label>${FIELD_LABELS[name] || name}</label>` +
            vals.map((x, i) => inp(name, i, x)).join("") + cap + "</div>";
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
                     title="capturar valor ao vivo">◉ capturar</button>
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
    });
  }

  div.querySelector('[data-act="send"]').onclick = async () => {
    if (!modified.size) return toast("nada a enviar");
    const body = { fields: {} };
    modified.forEach(f => body.fields[f] = fields[f]);
    try {
      await api(`/api/pages/${page}/cells`, {
        method: "PUT", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      modified.clear(); render();
      toast(`página ${page}: campos enviados (RAM)`);
    } catch (e) { toast(e.message, true); }
  };
  div.querySelector('[data-act="burn"]').onclick = async () => {
    try { await api(`/api/pages/${page}/burn`, { method: "POST" }); toast("burn OK"); }
    catch (e) { toast(e.message, true); }
  };
  div.querySelector('[data-act="reload"]').onclick = reload;
  await reload();
}

/* ── datalog ──────────────────────────────────────────────────────────── */
let logging = false;
$("#logBtn").onclick = async () => {
  try {
    if (!logging) {
      const r = await api("/api/log/start", { method: "POST" });
      logging = true;
      $("#logBtn").textContent = "■ Parar log";
      $("#logBtn").classList.add("rec");
      toast("gravando: " + r.path);
    } else {
      const r = await api("/api/log/stop", { method: "POST" });
      logging = false;
      $("#logBtn").textContent = "● Gravar log";
      $("#logBtn").classList.remove("rec");
      toast("log salvo: " + r.path);
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

  function ptX(i) {
    const iW = cv.width - PAD.l - PAD.r;
    return PAD.l + i * iW / (cfg.nPts - 1);
  }
  function ptY(v) {
    const iH = cv.height - PAD.t - PAD.b;
    return PAD.t + iH - (v - cfg.yMin) / (cfg.yMax - cfg.yMin) * iH;
  }
  function yFromPx(py) {
    const iH = cv.height - PAD.t - PAD.b;
    return cfg.yMin + (1 - (py - PAD.t) / iH) * (cfg.yMax - cfg.yMin);
  }

  function draw() {
    const W = cv.width, H = cv.height;
    const iW = W - PAD.l - PAD.r, iH = H - PAD.t - PAD.b;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#111"; ctx.fillRect(0, 0, W, H);

    // grid
    ctx.strokeStyle = "#222"; ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      const x = PAD.l + i * iW / 10, y = PAD.t + i * iH / 10;
      ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, PAD.t + iH); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + iW, y); ctx.stroke();
    }

    // axis labels
    ctx.fillStyle = "#666"; ctx.font = "11px monospace";
    const yStep = (cfg.yMax - cfg.yMin) / 10;
    ctx.textAlign = "right";
    for (let i = 0; i <= 10; i++) {
      const v = cfg.yMin + i * yStep;
      ctx.fillText(v % 1 === 0 ? v : v.toFixed(1), PAD.l - 5, PAD.t + iH - i * iH / 10 + 4);
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
    cfg.values.forEach((vals, si) => {
      if (si === cfg.active) return;
      ctx.strokeStyle = cfg.colors[si] + "33"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      vals.forEach((v, i) => { i === 0 ? ctx.moveTo(ptX(i), ptY(v)) : ctx.lineTo(ptX(i), ptY(v)); });
      ctx.stroke();
    });

    // active series
    const vals = cfg.values[cfg.active];
    ctx.strokeStyle = cfg.colors[cfg.active]; ctx.lineWidth = 2.5;
    ctx.beginPath();
    vals.forEach((v, i) => { i === 0 ? ctx.moveTo(ptX(i), ptY(v)) : ctx.lineTo(ptX(i), ptY(v)); });
    ctx.stroke();

    // points
    vals.forEach((v, i) => {
      const x = ptX(i), y = ptY(v);
      ctx.fillStyle = dragIdx === i ? "#fff" : cfg.colors[cfg.active];
      ctx.beginPath(); ctx.arc(x, y, dragIdx === i ? 7 : 5, 0, Math.PI * 2); ctx.fill();
      // value label on hover
      if (dragIdx === i) {
        ctx.fillStyle = "#fff"; ctx.font = "bold 12px monospace"; ctx.textAlign = "center";
        ctx.fillText(v % 1 === 0 ? v : v.toFixed(1), x, y - 12);
      }
    });
  }

  function nearestPt(ex, ey) {
    const vals = cfg.values[cfg.active];
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
      const vals = cfg.values[cfg.active];
      if (idx > 0 && v < vals[idx - 1]) v = vals[idx - 1];
      if (idx < cfg.nPts - 1 && v > vals[idx + 1]) v = vals[idx + 1];
    }
    return v;
  }

  function evPos(e) {
    const r = cv.getBoundingClientRect();
    const src = e.touches ? e.touches[0] : e;
    return [src.clientX - r.left, src.clientY - r.top];
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
      cfg.values[cfg.active][dragIdx] = v;
      cfg.onchange && cfg.onchange(dragIdx, v);
      draw();
    } else {
      const h = nearestPt(ex, ey);
      cv.style.cursor = h >= 0 ? "grab" : "default";
    }
  });
  const stopDrag = () => { dragIdx = -1; draw(); };
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
    cfg.values[cfg.active][dragIdx] = v;
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
      <div class="pm-tabs">${PEDAL_MODES.map((m,i)=>`<button class="pm-tab${i===0?" active":""}" data-mode="${i}">${m}</button>`).join("")}</div>
      <button id="pmSave">Salvar na ECU</button>
      <button id="pmReset">Restaurar padrões</button>
    </div>
    <canvas id="pmCanvas" width="760" height="500"></canvas>`;

  let activeMode = 0;
  const curve = makeDragCurve({
    canvas: $("#pmCanvas"),
    get values() { return pedalMaps; },
    get active() { return activeMode; },
    colors: PEDAL_COLORS,
    nPts: 10,
    xAxis: PEDAL_AXIS,
    xLabel: "Pedal %",
    yLabel: "Borboleta %",
    yMin: 0, yMax: 100,
    mono: true,
    onchange: () => {},
  });

  function activate(mode) {
    activeMode = mode;
    root.querySelectorAll(".pm-tab").forEach(b => b.classList.toggle("active", +b.dataset.mode === mode));
    curve.draw();
  }

  root.querySelectorAll(".pm-tab").forEach(b => b.onclick = () => activate(+b.dataset.mode));

  root.querySelector("#pmSave").onclick = async () => {
    try {
      await api("/api/pages/8/cells", "PUT", {pedal_maps: pedalMaps});
      toast("Pedal Map salvo na ECU ✓");
    } catch(e) { toast(e.message, true); }
  };

  const DEFAULTS = [
    [0,8,15,22,30,40,52,65,80,100],
    [0,10,20,30,40,50,60,70,80,100],
    [0,18,35,50,60,70,78,85,92,100],
    [0,5,10,15,22,30,40,52,65,100],
  ];
  root.querySelector("#pmReset").onclick = () => {
    pedalMaps = DEFAULTS.map(m => [...m]);
    activate(activeMode);
  };

  activate(0);
}

async function loadPedalMap() {
  try {
    const data = await api("/api/pages/8");
    buildPedalMapUI(data);
  } catch(e) {
    // ECU desconectada — mostra UI com defaults para edição offline
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
    toast("ECU desconectada — exibindo defaults", false);
  }
}

/* ── init ─────────────────────────────────────────────────────────────── */
(async () => {
  try {
    INFO = await api("/api/info");
    $("#fw").textContent = INFO.connected ? `${INFO.signature} · ${INFO.fw} · ${INFO.port}` : "";
    logging = INFO.logging;
  } catch (e) { toast(e.message, true); }
  connectWS();
  // Carregar VE automaticamente (aba padrão)
  loadGrid($("#tab-grid-1"));
})();
