/**
 * Unit tests for shipped pure helpers (tools/openems_dash/static/js/helpers.js).
 * Run: node --test tools/openems_dash/tests/helpers.test.js
 */
"use strict";

const { describe, it } = require("node:test");
const assert = require("node:assert/strict");
const path = require("node:path");

const helpers = require(path.join(__dirname, "..", "static", "js", "helpers.js"));

describe("heatColor / heatTextColor", () => {
  it("returns hsl strings spanning blue→red", () => {
    const lo = helpers.heatColor(0, 0, 100);
    const mid = helpers.heatColor(50, 0, 100);
    const hi = helpers.heatColor(100, 0, 100);
    assert.match(lo, /^hsl\(/);
    assert.match(hi, /^hsl\(/);
    // low → hue near 240 (blue), high → hue near 0 (red)
    const hue = (s) => Number(s.match(/hsl\(([-\d.]+)/)[1]);
    assert.ok(hue(lo) > 200);
    assert.ok(hue(hi) < 20);
    assert.ok(hue(mid) > 80 && hue(mid) < 160);
  });

  it("handles min===max without NaN", () => {
    const c = helpers.heatColor(42, 42, 42);
    assert.match(c, /^hsl\(/);
    assert.ok(!c.includes("NaN"));
  });

  it("heatTextColor is dark for light heatmap cells", () => {
    assert.equal(helpers.heatTextColor(), "#1a1a1a");
  });
});

describe("axisLookup", () => {
  const rpm = [500, 1000, 2000, 4000, 8000];

  it("clamps below first node", () => {
    assert.deepEqual(helpers.axisLookup(rpm, 100), { idx: 0, frac: 0 });
  });

  it("clamps above last node", () => {
    assert.deepEqual(helpers.axisLookup(rpm, 12000), { idx: 3, frac: 1 });
  });

  it("returns mid-bin fraction", () => {
    const r = helpers.axisLookup(rpm, 1500);
    assert.equal(r.idx, 1);
    assert.ok(Math.abs(r.frac - 0.5) < 1e-9);
  });

  it("handles empty axis", () => {
    assert.deepEqual(helpers.axisLookup([], 1), { idx: 0, frac: 0 });
  });
});

describe("bilinearCorners", () => {
  it("returns four corners interior with weights summing to 1", () => {
    const corners = helpers.bilinearCorners(
      { idx: 2, frac: 0.25 },
      { idx: 3, frac: 0.5 }
    );
    assert.equal(corners.length, 4);
    const sum = corners.reduce((a, p) => a + p.w, 0);
    assert.ok(Math.abs(sum - 1) < 1e-9);
    assert.ok(corners.every((p) => p.w > 0));
  });

  it("omits zero-weight corners on axis boundary", () => {
    const corners = helpers.bilinearCorners(
      { idx: 1, frac: 0 },
      { idx: 2, frac: 0 }
    );
    assert.equal(corners.length, 1);
    assert.equal(corners[0].r, 2);
    assert.equal(corners[0].c, 1);
    assert.equal(corners[0].w, 1);
  });
});

describe("clampCell", () => {
  it("clamps VE-like 0–255", () => {
    assert.equal(helpers.clampCell(1, -3), 0);
    assert.equal(helpers.clampCell(1, 300), 255);
    assert.equal(helpers.clampCell(1, 100.6), 101);
  });

  it("clamps spark signed int8", () => {
    assert.equal(helpers.clampCell(2, -200), -128);
    assert.equal(helpers.clampCell(2, 200), 127);
    assert.equal(helpers.clampCell(2, 12), 12);
  });

  it("clamps lambda×1000 as u16", () => {
    assert.equal(helpers.clampCell(4, -1), 0);
    assert.equal(helpers.clampCell(4, 70000), 65535);
    assert.equal(helpers.clampCell(4, 1050), 1050);
  });
});

describe("formatCell / parseCell", () => {
  it("formats lambda as human value", () => {
    assert.equal(helpers.formatCell(4, 1050), "1.05");
    assert.equal(helpers.formatCell(4, 800), "0.80");
  });

  it("formats VE/spark as raw string", () => {
    assert.equal(helpers.formatCell(1, 87), "87");
    assert.equal(helpers.formatCell(2, -5), "-5");
  });

  it("parses lambda display back to ×1000", () => {
    assert.equal(helpers.parseCell(4, "1.05"), 1050);
    assert.equal(helpers.parseCell(4, "0.8"), 800);
  });

  it("parses integer cells", () => {
    assert.equal(helpers.parseCell(1, "90"), 90);
    assert.equal(helpers.parseCell(2, "-3"), -3);
  });
});

describe("stepSize / weightedStep", () => {
  it("lambda steps by 10, others by 1", () => {
    assert.equal(helpers.stepSize(4), 10);
    assert.equal(helpers.stepSize(1), 1);
    assert.equal(helpers.stepSize(2), 1);
  });

  it("weights steps and never zeros non-zero delta", () => {
    assert.equal(helpers.weightedStep(1, 1), 1);
    assert.equal(helpers.weightedStep(1, 0.1), 1);
    assert.equal(helpers.weightedStep(10, 0.25), 3);
    assert.equal(helpers.weightedStep(-10, 0.25), -3);
    assert.equal(helpers.weightedStep(0, 0.5), 0);
  });
});

describe("formatGauge", () => {
  it("formats primary gauges", () => {
    assert.equal(helpers.formatGauge("lambda_x1000", 980), "0.98");
    assert.equal(helpers.formatGauge("lambda_target_x1000", 1000), "1.00");
    assert.equal(helpers.formatGauge("pw_ms", 3.456), "3.46");
    assert.equal(helpers.formatGauge("rpm", 3500), "3500");
    assert.equal(helpers.formatGauge("rpm", null), "—");
  });
});
