/**
 * OpenEMS dash — pure calibration helpers (no DOM, no I/O).
 * Dual-export: browser global `OpenEMSHelpers` + Node `module.exports`.
 * Used by the UI and by `tools/openems_dash/tests/helpers.test.js`.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (typeof root === "object") {
    root.OpenEMSHelpers = api;
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  /** Heatmap: blue (low) → red (high). Returns CSS hsl(). */
  function heatColor(v, min, max) {
    const f = max > min ? (v - min) / (max - min) : 0;
    const h = (1 - f) * 240;
    const s = 72 + f * 10;
    const l = 52 + (1 - Math.abs(f - 0.5) * 2) * 12;
    return `hsl(${h} ${s}% ${l}%)`;
  }

  /** High-L heatmap cells → dark text for contrast (TunerStudio-style). */
  function heatTextColor() {
    return "#1a1a1a";
  }

  /**
   * Nearest-bin + fraction along an axis of interpolation nodes.
   * Mirrors firmware axis_lookup() in table3d.cpp.
   * @returns {{ idx: number, frac: number }}
   */
  function axisLookup(axis, v) {
    if (!axis || !axis.length) return { idx: 0, frac: 0 };
    const last = axis.length - 1;
    if (v <= axis[0]) return { idx: 0, frac: 0 };
    if (v >= axis[last]) return { idx: last - 1, frac: 1 };
    let i = last - 1;
    while (i > 0 && v < axis[i]) i--;
    const span = axis[i + 1] - axis[i];
    return { idx: i, frac: span ? (v - axis[i]) / span : 0 };
  }

  /**
   * Bilinear corner cells for operating point (lx, ly from axisLookup).
   * Omits zero-weight corners. Weights sum to ~1.
   * @returns {Array<{ r: number, c: number, w: number }>}
   */
  function bilinearCorners(lx, ly) {
    return [
      { r: ly.idx,     c: lx.idx,     w: (1 - ly.frac) * (1 - lx.frac) },
      { r: ly.idx,     c: lx.idx + 1, w: (1 - ly.frac) * lx.frac },
      { r: ly.idx + 1, c: lx.idx,     w: ly.frac * (1 - lx.frac) },
      { r: ly.idx + 1, c: lx.idx + 1, w: ly.frac * lx.frac },
    ].filter((p) => p.w > 0);
  }

  /**
   * Clamp raw cell value by page type.
   * page 2 = spark signed int8; page 4 = λ×1000 u16; else VE-like 0–255.
   */
  function clampCell(page, v) {
    const n = Math.round(Number(v));
    if (!Number.isFinite(n)) return 0;
    if (page === 2) return Math.max(-128, Math.min(127, n));
    if (page === 4) return Math.max(0, Math.min(65535, n));
    return Math.max(0, Math.min(255, n));
  }

  /** Display string for a grid cell (page 4 shows human λ). */
  function formatCell(page, v) {
    if (page === 4) return (Number(v) / 1000).toFixed(2);
    return String(v);
  }

  /** Parse user input back to wire/raw cell value. */
  function parseCell(page, s) {
    if (page === 4) return Math.round(parseFloat(s) * 1000);
    return parseInt(s, 10);
  }

  /** Keyboard step size for multi-cell A/Z/+/-. */
  function stepSize(page) {
    return page === 4 ? 10 : 1;
  }

  /**
   * Scale a step by bilinear weight; never rounds a non-zero delta to 0
   * (so low-weight nodes still move on step-1 tables).
   */
  function weightedStep(delta, weight) {
    const w = weight == null ? 1 : weight;
    const raw = delta * w;
    if (raw === 0) return 0;
    return Math.sign(raw) * Math.max(1, Math.round(Math.abs(raw)));
  }

  /** Primary gauge formatters used by the status bar. */
  function formatGauge(key, v) {
    if (v == null || Number.isNaN(v)) return "—";
    switch (key) {
      case "lambda_x1000":
      case "lambda_target_x1000":
        return (v / 1000).toFixed(2);
      case "pw_ms":
        return Number(v).toFixed(2);
      case "dc_pct":
        return Number(v).toFixed(1);
      default:
        return String(v);
    }
  }

  return {
    heatColor,
    heatTextColor,
    axisLookup,
    bilinearCorners,
    clampCell,
    formatCell,
    parseCell,
    stepSize,
    weightedStep,
    formatGauge,
  };
});
