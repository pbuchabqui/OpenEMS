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

async function api(path, opts) {
  const r = await fetch(path, opts);
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status} ${await r.text()}`);
  return r.json();
}

/* ── tabs ─────────────────────────────────────────────────────────────── */
$$("nav .tab").forEach(b => b.onclick = () => {
  $$("nav .tab").forEach(x => x.classList.toggle("active", x === b));
  $$(".pane").forEach(p => p.classList.toggle("active", p.id === "tab-" + b.dataset.tab));
  const pane = $("#tab-" + b.dataset.tab);
  if (pane.classList.contains("grid-pane") && !pane.dataset.loaded) loadGrid(pane);
  if (b.dataset.tab === "params" && !$("#paramsRoot").dataset.loaded) loadParams();
});

/* ── telemetria: gauges + charts ──────────────────────────────────────── */
const GAUGES = [
  ["rpm", "RPM", v => v],
  ["map_kpa", "MAP kPa", v => v],
  ["tps_pct", "TPS %", v => v],
  ["lambda_x1000", "λ", v => (v / 1000).toFixed(2)],
  ["pw_ms", "PW ms", v => v.toFixed(1)],
  ["advance_deg", "Avanço °", v => v],
  ["clt_c", "CLT °C", v => v],
  ["iat_c", "IAT °C", v => v],
  ["ve", "VE", v => v],
  ["stft_pct", "STFT %", v => v],
];
$("#gauges").innerHTML = GAUGES.map(([k, l]) =>
  `<div class="gauge"><div class="v" id="g_${k}">—</div><div class="l">${l}</div></div>`).join("");

const LEDS = [
  ["FULL_SYNC", true], ["PHASE_A", true], ["SENSOR_FAULT", false],
  ["SCHED_LATE", false], ["SCHED_DROP", false], ["SCHED_CLAMP", false],
  ["WBO2_FAULT", false],
];
$("#statusLeds").innerHTML = LEDS.map(([k]) =>
  `<span class="led" id="led_${k}">${k}</span>`).join("");

/* strip-charts uPlot: janela deslizante de 60 s */
const WINDOW_S = 60, MAX_PTS = 60 * 35;
const chartDefs = [
  { title: "RPM", series: [["rpm", "#4cc2ff"]] },
  { title: "MAP / TPS", series: [["map_kpa", "#ffb347"], ["tps_pct", "#3ad06c"]] },
  { title: "Lambda / STFT", series: [["lambda_x1000", "#ff5562"], ["stft_pct", "#b58cff"]] },
  { title: "PW / Avanço", series: [["pw_ms", "#4cc2ff"], ["advance_deg", "#ffd24c"]] },
];
const charts = chartDefs.map(def => {
  const box = document.createElement("div");
  box.className = "chart-box";
  $("#charts").appendChild(box);
  const data = [[], ...def.series.map(() => [])];
  const u = new uPlot({
    width: box.clientWidth - 12 || 560, height: 180, title: def.title,
    scales: { x: { time: false } },
    axes: [
      { stroke: "#7b828c", grid: { stroke: "#2a2e36" } },
      { stroke: "#7b828c", grid: { stroke: "#2a2e36" } },
    ],
    series: [
      { label: "t" },
      ...def.series.map(([k, c]) => ({ label: k, stroke: c, width: 1.5 })),
    ],
  }, data, box);
  return { u, data, keys: def.series.map(([k]) => k) };
});
window.addEventListener("resize", () =>
  charts.forEach(c => c.u.setSize({ width: c.u.root.parentElement.clientWidth - 12, height: 180 })));

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
    $("#conn").className = "badge " + (conn ? "on" : "off");
    $("#conn").textContent = conn ? "conectado" : (d.error || "desconectado");
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
  const h = (1 - f) * 120;          // 120=verde → 0=vermelho
  return `hsl(${h} 65% 62%)`;
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
    for (let row = 0; row < 16; row++) {
      html += `<tr><th>${INFO.axes.map_kpa[row]}</th>`;
      for (let col = 0; col < 16; col++) {
        const v = st.values[row][col];
        const mod = st.modified.has(`${row},${col}`) ? " mod" : "";
        html += `<td class="${mod}" data-r="${row}" data-c="${col}"
                     style="background:${heatColor(v, min, max)}">${v}</td>`;
      }
      html += "</tr>";
    }
    $(".grid-wrap", pane).innerHTML = html + "</table>";
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

/* célula corrente do motor destacada ao vivo */
function axisIndex(axis, v) {
  for (let i = axis.length - 1; i >= 0; i--) if (v >= axis[i]) return Math.min(i, 15);
  return 0;
}
function highlightLiveCell() {
  if (!RT || !INFO) return;
  const col = axisIndex(INFO.axes.rpm, RT.rpm);
  const row = axisIndex(INFO.axes.map_kpa, RT.map_kpa);
  $$(".grid-pane.active td.live").forEach(td => td.classList.remove("live"));
  const pane = $(".grid-pane.active");
  if (pane) {
    const td = pane.querySelector(`td[data-r="${row}"][data-c="${col}"]`);
    if (td) td.classList.add("live");
  }
}

/* ── parâmetros (páginas 5/6/7) ───────────────────────────────────────── */
const PARAM_PAGES = { 0: "Configuração do motor", 5: "Correções 1D",
                      6: "X-Tau / AE / Crank", 7: "Dwell 2D" };

/* rótulos amigáveis (página 0) e campos read-only */
const FIELD_LABELS = {
  ivc_abdc_deg:              "Fecho da admissão IVC (° ABDC)",
  displacement_cc:           "Cilindrada (cc)",
  injector_flow_cc_min:      "Vazão do bico (cc/min)",
  stoich_afr_x100:           "AFR estequiométrico (×100)",
  map_ref_bar_x100:          "MAP de referência (bar×100)",
  trigger_tooth0_engine_deg: "Offset trigger dente 0 (° motor, 0-719)",
  default_soi_lead_deg:      "Avanço SOI padrão (°)",
  config_magic:              "Magic (0x4543 = config válida)",
  app1_raw_min:  "APP1 pedal solto (raw)",   app1_raw_max:  "APP1 pedal a fundo (raw)",
  app2_raw_min:  "APP2 pedal solto (raw)",   app2_raw_max:  "APP2 pedal a fundo (raw)",
  etb_tps1_raw_min: "ETB TPS1 fechada (raw)", etb_tps1_raw_max: "ETB TPS1 aberta (raw)",
  etb_tps2_raw_min: "ETB TPS2 fechada (raw)", etb_tps2_raw_max: "ETB TPS2 aberta (raw)",
  tps_raw_min: "TPS cabo fechado (raw)",      tps_raw_max: "TPS cabo aberto (raw)",
  app_max_delta_pct_x10: "Plausibilidade APP Δmáx (%×10)",
  etb_max_delta_pct_x10: "Plausibilidade ETB Δmáx (%×10)",
  etb_max_open_pct_x10_limp: "ETB abertura máx limp (%×10)",
  etb_max_rate_pct_per_s: "ETB taxa máx (%/s)",
  etb_idle_open_pct_x10: "ETB abertura idle (%×10)",
  etb_cal_valid: "Calibração ETB válida (0/1)",
  etb_harness_present: "Chicote ETB presente (0/1)",
  etb_kp_x10: "ETB PID Kp (×10)", etb_ki_x10: "ETB PID Ki (×10)", etb_kd_x10: "ETB PID Kd (×10)",
};

/* calibração assistida: campo → fonte do raw ao vivo na telemetria */
const CAL_CAPTURE = {
  app1_raw_min: "an1_raw", app1_raw_max: "an1_raw",
  app2_raw_min: "an2_raw", app2_raw_max: "an2_raw",
  etb_tps1_raw_min: "an3_raw", etb_tps1_raw_max: "an3_raw",
};
const READONLY_FIELDS = new Set(["config_magic"]);

/* Layout explícito: curvas 1D (eixo + 1..n linhas de valores), tabelas 2D
   e escalares. Nomes de campo = protocol.py / ui_protocol.cpp. */
const PAGE_LAYOUT = {
  5: {
    curves: [
      { title: "Correção CLT",        axis: "clt_corr_axis_x10",     axisLabel: "CLT (°C×10)",   rows: [["clt_corr_x256", "fator ×256"]] },
      { title: "Correção IAT",        axis: "iat_corr_axis_x10",     axisLabel: "IAT (°C×10)",   rows: [["iat_corr_x256", "fator ×256"]] },
      { title: "Warmup",              axis: "warmup_corr_axis_x10",  axisLabel: "CLT (°C×10)",   rows: [["warmup_corr_x256", "fator ×256"]] },
      { title: "Injetor vs VBat",     axis: "vbatt_corr_axis_mv",    axisLabel: "VBat (mV)",     rows: [["injector_dead_time_us", "dead time (µs)"]] },
      { title: "AE vs CLT",           axis: "ae_clt_corr_axis_x10",  axisLabel: "CLT (°C×10)",   rows: [["ae_clt_sens", "sensibilidade"]] },
      { title: "Dwell vs VBat",       axis: "dwell_vbatt_axis_mv",   axisLabel: "VBat (mV)",     rows: [["dwell_ms_x10_table", "dwell (ms×10)"]] },
    ],
    tables2d: [
      { title: "Atraso lambda (ms)", x: "lambda_delay_rpm_axis_x10", xLabel: "RPM ×10",
        y: "lambda_delay_load_axis_bar_x100", yLabel: "MAP (bar×100)",
        values: "lambda_delay_ms_table" },
    ],
  },
  6: {
    curves: [
      { title: "X-Tau vs CLT", axis: "xtau_clt_axis_x10", axisLabel: "CLT (°C×10)",
        rows: [["xtau_x_fraction_q8", "X (Q8)"], ["xtau_tau_cycles", "τ (ciclos)"]] },
      { title: "AE rate", axis: "ae_tpsdot_axis_x10", axisLabel: "TPSdot (%/s×10)",
        rows: [["ae_pw_adder_us", "PW adder (µs)"]] },
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

async function loadParams() {
  const root = $("#paramsRoot");
  root.dataset.loaded = "1";
  root.innerHTML = "";
  for (const [page, title] of Object.entries(PARAM_PAGES)) {
    const div = document.createElement("div");
    div.className = "param-group";
    div.innerHTML = `<h3>Página ${page} — ${title}</h3>
      <div class="rows muted">carregando…</div>
      <div class="param-row">
        <button class="primary" data-act="send">Enviar (RAM)</button>
        <button class="danger" data-act="burn">Burn → flash</button>
        <button data-act="reload">Reler</button>
      </div>`;
    root.appendChild(div);
    bindParamGroup(div, +page);
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

    // escalares e campos fora do layout
    for (const [name, v] of Object.entries(fields)) {
      if (used.has(name)) continue;
      const vals = Array.isArray(v) ? v : [v];
      const cap = CAL_CAPTURE[name]
        ? `<button class="cap" data-cap="${name}" data-src="${CAL_CAPTURE[name]}"
                   title="capturar valor ao vivo">◉ capturar</button>
           <span class="muted live-raw" data-src="${CAL_CAPTURE[name]}"></span>`
        : "";
      html += `<div class="param-row"><label>${FIELD_LABELS[name] || name}</label>` +
        vals.map((x, i) => inp(name, i, x)).join("") + cap + "</div>";
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

/* ── init ─────────────────────────────────────────────────────────────── */
(async () => {
  try {
    INFO = await api("/api/info");
    $("#fw").textContent = INFO.connected ? `${INFO.signature} · ${INFO.fw} · ${INFO.port}` : "";
    logging = INFO.logging;
  } catch (e) { toast(e.message, true); }
  connectWS();
})();
