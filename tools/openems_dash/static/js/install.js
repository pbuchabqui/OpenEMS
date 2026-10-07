/**
 * OpenEMS dash — "Instalação" tab: guided install / calibration.
 * Steps follow the README install guide. Uses app.js globals ($, api, toast,
 * RT) and OpenEMSHelpers. Nothing here blocks: each step only shows ✓ when
 * the live value looks plausible.
 */
(function () {
  "use strict";
  const H = window.OpenEMSHelpers;

  // Engine-config ranges come from the server (protocol.FIELD_LIMITS) via the
  // 400 error; these are only the input hints.
  const ENGINE_FIELDS = [
    ["displacement_cc", "Cilindrada", "cc", 200, 10000, 1],
    ["injector_flow_cc_min", "Vazão do injetor (a 3 bar)", "cc/min", 50, 3000, 1],
    ["stoich_afr_x100", "AFR estequiométrico", "AFR", 9, 18, 0.01],
  ];

  let page0 = {};

  async function readPage0() {
    page0 = (await api("/api/pages/0")).fields;
    return page0;
  }

  async function writeFields(fields) {
    try {
      await api("/api/pages/0/cells", "PUT", { fields });
      Object.assign(page0, fields);
      return true;
    } catch (e) {
      const m = /"error":\s*"([^"]+)"/.exec(String(e.message));
      toast(m ? m[1] : String(e.message), true);
      return false;
    }
  }

  function stepBox(n, title, body, okId) {
    return `<section class="inst-step">
      <h3><span class="inst-n">${n}</span> ${title}
        <span class="inst-ok" id="${okId}" title="valor plausível">·</span></h3>
      ${body}</section>`;
  }

  function engineStep() {
    const rows = ENGINE_FIELDS.map(([k, label, unit, lo, hi, step]) =>
      `<label class="inst-row">${label}
        <input type="number" id="inst_${k}" min="${lo}" max="${hi}" step="${step}">
        <span class="inst-unit">${unit} (${lo} … ${hi})</span></label>`).join("");
    return stepBox(1, "Motor", `${rows}
      <button id="instEngineApply">Aplicar</button>
      <div class="inst-hint" id="instReject"></div>`, "ok_engine");
  }

  function polarityStep() {
    return stepBox(2, "Sensores de rotação (CKP / CMP)", `
      <label class="inst-row"><input type="checkbox" id="instCkpFall"> CKP na borda de descida</label>
      <label class="inst-row"><input type="checkbox" id="instCmpFall"> CMP na borda de descida</label>
      <div class="inst-hint">Hall open-collector (repouso alto): CMP descida.
        Gire o motor: os contadores abaixo devem subir.</div>
      <div class="inst-live" id="instEdges">CKP — · CMP —</div>`, "ok_edges");
  }

  function deadTimeStep() {
    return stepBox(3, "Dead time do injetor", `
      <div class="inst-hint">Copie a curva da folha de dados do injetor (ms × tensão).</div>
      <button data-goto="params">Abrir curvas (Params → página 5)</button>`, "ok_dead");
  }

  function outputStep() {
    return stepBox(4, "Teste de saídas (motor parado)", `
      <div class="inst-hint">Confirme que injetor 1 e bobina 1 são do cilindro 1, etc.</div>
      <button data-goto="output-test">Abrir teste de saídas</button>`, "ok_out");
  }

  function offsetStep() {
    return stepBox(5, "Offset do trigger (grosso)", `
      <label class="inst-row">Dentes do 1º dente após a falha até o PMS do cilindro 1
        <input type="number" id="instTeeth" min="0" max="119" step="1"></label>
      <div class="inst-live">Offset = <b id="instOffsetCalc">—</b> ° (atual: <b id="instOffsetNow">—</b> °)</div>
      <button id="instOffsetApply">Aplicar offset</button>`, "ok_offset");
  }

  function strobeStep() {
    return stepBox(6, "Offset fino com lâmpada de ponto", `
      <div class="inst-hint">Motor em marcha lenta, lâmpada no cabo da vela 1. Ligue o modo: a centelha
        fica fixa no avanço abaixo (sem correções nem knock). Ajuste até a lâmpada marcar o mesmo valor.</div>
      <label class="inst-row">Avanço fixo
        <input type="number" id="instStrobeAdv" min="0" max="30" step="0.5"> °</label>
      <div class="inst-row">
        <button id="instStrobeOn">Ligar modo lâmpada</button>
        <button id="instStrobeOff">Desligar</button></div>
      <div class="inst-row inst-adj">Offset
        <button data-adj="-1">−1°</button><button data-adj="1">+1°</button>
        Fino <button data-fine="-0.1">−0,1°</button><button data-fine="0.1">+0,1°</button></div>
      <div class="inst-live">Offset <b id="instOffsetLive">—</b> ° + fino <b id="instFineLive">—</b> ° ·
        avanço na bobina <b id="instAdvLive">—</b> ° · modo <b id="instStrobeState">—</b></div>`, "ok_strobe");
  }

  function burnStep() {
    return stepBox(7, "Gravar", `
      <div class="inst-hint">Grava a página 0 e todas as páginas com alterações (motor parado).</div>
      <button id="instBurn">Gravar tudo</button>`, "ok_burn");
  }

  function tuneBox() {
    return `<section class="inst-step inst-tune"><h3>Tune</h3>
      <div class="inst-row">
        <button id="instExport">Exportar tune…</button>
        <label class="inst-file">Importar tune… <input type="file" id="instImport" accept=".json"></label>
        <button id="instFactory" class="danger">Restaurar padrões de fábrica</button>
      </div>
      <div class="inst-hint">Importar e restaurar gravam na flash (motor parado).</div></section>`;
  }

  function fillFromPage0() {
    for (const [k] of ENGINE_FIELDS) $("#inst_" + k).value = page0[k];
    $("#instCkpFall").checked = !!(page0.capture_polarity & 1);
    $("#instCmpFall").checked = !!(page0.capture_polarity & 2);
    $("#instStrobeAdv").value = page0.timing_light_advance_x10 || 10;
    $("#instOffsetNow").textContent = page0.trigger_tooth0_engine_deg;
    $("#instOffsetLive").textContent = page0.trigger_tooth0_engine_deg;
    $("#instFineLive").textContent = Number(page0.trigger_fine_x10 || 0).toFixed(1);
  }

  function setOk(id, ok) {
    const el = $("#" + id);
    if (!el) return;
    el.textContent = ok ? "✓" : "·";
    el.classList.toggle("on", !!ok);
  }

  // Live part, called from app.js on every telemetry frame.
  window.installOnTelemetry = function (d) {
    if (!$("#installRoot") || !$("#installRoot").dataset.loaded) return;
    $("#instEdges").textContent =
      `CKP ${d.ckp_edge_count} bordas · CMP ${d.cmp_edge_count} bordas · RPM ${d.rpm}`;
    setOk("ok_edges", d.ckp_edge_count > 0);
    $("#instAdvLive").textContent = Number(d.advance_deg_fine).toFixed(1);
    $("#instStrobeState").textContent = d.timing_light ? "LÂMPADA (fixo)" : "normal";
    const rej = H.rejectedFields(d.config_reject_mask);
    $("#instReject").textContent = rej.length ? "ECU rejeitou: " + rej.join(", ") : "";
    setOk("ok_engine", rej.length === 0 && page0.displacement_cc > 0);
  };

  async function wire() {
    $("#instEngineApply").onclick = async () => {
      const f = {};
      for (const [k] of ENGINE_FIELDS) f[k] = Number($("#inst_" + k).value);
      if (await writeFields(f)) toast("Motor aplicado (lembre de gravar)");
    };
    const pol = async () => {
      const v = ($("#instCkpFall").checked ? 1 : 0) | ($("#instCmpFall").checked ? 2 : 0);
      if (await writeFields({ capture_polarity: v })) toast("Polaridade aplicada");
    };
    $("#instCkpFall").onchange = pol;
    $("#instCmpFall").onchange = pol;
    $("#instTeeth").oninput = () => {
      const o = H.offsetFromTeeth($("#instTeeth").value);
      $("#instOffsetCalc").textContent = o === null ? "—" : o;
    };
    $("#instOffsetApply").onclick = async () => {
      const o = H.offsetFromTeeth($("#instTeeth").value);
      if (o === null) { toast("Informe o número de dentes (0 … 119)", true); return; }
      if (await writeFields({ trigger_tooth0_engine_deg: o })) {
        $("#instOffsetNow").textContent = o;
        $("#instOffsetLive").textContent = o;
        setOk("ok_offset", true);
      }
    };
    $("#instStrobeOn").onclick = async () => {
      const adv = Number($("#instStrobeAdv").value);
      if (await writeFields({ timing_light_advance_x10: adv, timing_light_enable: 1 }))
        toast(`Modo lâmpada: centelha fixa em ${adv.toFixed(1)}°`);
    };
    $("#instStrobeOff").onclick = async () => {
      if (await writeFields({ timing_light_enable: 0 })) {
        toast("Modo lâmpada desligado");
        setOk("ok_strobe", true);
      }
    };
    $$("#installRoot [data-adj]").forEach(b => b.onclick = async () => {
      const o = ((page0.trigger_tooth0_engine_deg + Number(b.dataset.adj)) % 720 + 720) % 720;
      if (await writeFields({ trigger_tooth0_engine_deg: o })) {
        $("#instOffsetLive").textContent = o;
        $("#instOffsetNow").textContent = o;
      }
    });
    $$("#installRoot [data-fine]").forEach(b => b.onclick = async () => {
      const f = Math.round((Number(page0.trigger_fine_x10 || 0) + Number(b.dataset.fine)) * 10) / 10;
      if (await writeFields({ trigger_fine_x10: f }))
        $("#instFineLive").textContent = f.toFixed(1);
    });
    $$("#installRoot [data-goto]").forEach(b => b.onclick = () =>
      $(`#sb-nav .tab[data-tab="${b.dataset.goto}"]`).click());
    $("#instBurn").onclick = async () => {
      try {
        const r = await api("/api/burn_all", "POST");
        toast("Gravado: páginas " + r.pages.join(", "));
        setOk("ok_burn", true);
      } catch (e) { toast(String(e.message), true); }
    };
    $("#instExport").onclick = async () => {
      try {
        const tune = await api("/api/tune/export");
        const a = document.createElement("a");
        a.href = URL.createObjectURL(new Blob([JSON.stringify(tune, null, 1)],
                                              { type: "application/json" }));
        a.download = `openems-tune-${new Date().toISOString().slice(0, 10)}.json`;
        a.click();
      } catch (e) { toast(String(e.message), true); }
    };
    $("#instImport").onchange = async ev => {
      const file = ev.target.files[0];
      if (!file) return;
      try {
        const tune = JSON.parse(await file.text());
        if (!confirm(`Carregar ${file.name} e gravar na flash?`)) return;
        await api("/api/tune/import", "POST", tune);
        toast("Tune carregado e gravado");
        await readPage0(); fillFromPage0();
      } catch (e) { toast(String(e.message), true); }
      ev.target.value = "";
    };
    $("#instFactory").onclick = async () => {
      if (!confirm("Substituir TODA a calibração pelos padrões de fábrica e gravar?")) return;
      try {
        await api("/api/tune/factory", "POST");
        toast("Padrões de fábrica restaurados");
        await readPage0(); fillFromPage0();
      } catch (e) { toast(String(e.message), true); }
    };
  }

  window.loadInstall = async function () {
    const root = $("#installRoot");
    root.innerHTML = `<div class="inst-wrap">
      <p class="inst-intro">Guia de instalação — siga em ordem. Valores fora da faixa são recusados
        com o nome do campo; nada é gravado na flash até “Gravar tudo”.</p>
      ${engineStep()}${polarityStep()}${deadTimeStep()}${outputStep()}
      ${offsetStep()}${strobeStep()}${burnStep()}${tuneBox()}</div>`;
    root.dataset.loaded = "1";
    try {
      await readPage0();
      fillFromPage0();
    } catch (e) {
      toast("ECU offline — conecte para usar o guia", true);
    }
    wire();
  };
})();
