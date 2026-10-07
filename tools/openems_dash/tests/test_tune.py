"""Field limits, tune export / import and factory defaults.

The ECU side is the simulated ECU (firmware code on a pty); see
tests/test_sim_ecu.py. Skipped when the sim binary is missing.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import protocol as P  # noqa: E402
from tests.test_sim_ecu import SIM, SimEcu  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


class TestLimits(unittest.TestCase):
    def test_engine_config_limits_equal_firmware(self):
        src = open(os.path.join(REPO, "src/engine/engine_config.cpp"), encoding="utf-8").read()
        fw = {m.group(3): (int(m.group(1)), int(m.group(2))) for m in re.finditer(
            r"\{kOffset\w+,\s*(\d+)u,\s*(\d+)u,\s*&EngineConfigRam::(\w+)\}", src)}
        self.assertEqual(len(fw), 6, "kFields parse")
        scale = {e[0]: (e[4] if len(e) > 4 else 1.0) for e in P.PAGE0_FIELDS}
        for name, (lo, hi) in fw.items():
            dlo, dhi = P.FIELD_LIMITS[(0, name)]
            self.assertEqual((round(dlo / scale[name]), round(dhi / scale[name])), (lo, hi), name)

    def test_validate_field(self):
        self.assertIsNone(P.validate_field(0, "displacement_cc", 1600))
        self.assertIn("displacement_cc", P.validate_field(0, "displacement_cc", 150))
        self.assertIn("fora da faixa", P.validate_field(0, "stoich_afr_x100", 20.0))
        self.assertIsNone(P.validate_field(0, "trigger_fine_x10", -0.3))
        self.assertIsNotNone(P.validate_field(0, "trigger_fine_x10", 6))
        self.assertIsNone(P.validate_field(0, "rev_limit_rpm_x10", 99999))  # no limit set


@unittest.skipUnless(os.path.exists(SIM), "sim ECU not built (make sim-ecu-build)")
class TestTune(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.nvm = os.path.join(self.tmp.name, "nvm.bin")
        self.ecu = SimEcu(self.nvm)

    def tearDown(self):
        self.ecu.close()
        self.tmp.cleanup()

    def restart(self):
        self.ecu.close()
        self.ecu = SimEcu(self.nvm)

    def test_base_tune_matches_firmware_defaults(self):
        out = os.path.join(self.tmp.name, "dump.json")
        subprocess.run([SIM, "--dump-pages", out], check=True, stdout=subprocess.DEVNULL)
        self.assertEqual(json.load(open(out)), P.load_base_tune(),
                         "base_tune.json is stale: regenerate with sim_ecu --dump-pages")

    def test_export_import_survives_restart(self):
        link = self.ecu.link
        off, data = P.encode_field(0, "displacement_cc", 1798)
        link.read_page(0)
        link.write_page_ram(0, off, data)
        link.write_page_ram(1, 3 * 20 + 4, P.encode_cell_u8(66))
        tune = P.export_tune(link)
        self.assertIsNone(P.check_tune(tune))
        # Fresh ECU (factory), import, power cycle, compare.
        self.ecu.close()
        os.remove(self.nvm) if os.path.exists(self.nvm) else None
        self.ecu = SimEcu(self.nvm)
        P.import_tune(self.ecu.link, tune)
        self.restart()
        self.assertEqual(P.export_tune(self.ecu.link), tune)

    def test_factory_defaults_restore(self):
        link = self.ecu.link
        link.read_page(0)
        off, data = P.encode_field(0, "displacement_cc", 1400)
        link.write_page_ram(0, off, data)
        link.burn_page(0)
        P.import_tune(link, P.load_base_tune())
        self.restart()
        f = P.decode_fields(0, self.ecu.link.read_page(0))
        self.assertEqual(f["displacement_cc"], 2000)

    def test_page_sizes_equal_firmware(self):
        import base64
        fw = {int(k): len(base64.b64decode(v)) for k, v in P.load_base_tune()["pages"].items()}
        for pg, n in fw.items():
            self.assertEqual(P.PAGE_SIZES[pg], n, f"dashboard page {pg}")
        ini = open(os.path.join(REPO, "tools/ts/openems.ini"), encoding="utf-8").read()
        sizes = [int(x) for x in re.search(r"pageSize\s*=\s*([\d,\s]+)", ini).group(1).split(",")]
        ini_pages = [0, 1, 2, 4, 5, 6, 7, 8, 9, 11]  # ini page n -> firmware page
        for pg, n in zip(ini_pages, sizes):
            self.assertEqual(n, fw[pg], f"TunerStudio page for firmware page {pg}")

    def test_bad_tunes_refused(self):
        base = P.load_base_tune()
        self.assertIsNotNone(P.check_tune({"format": "x"}))
        bad = dict(base, layout_version=base["layout_version"] + 1)
        self.assertIn("layout", P.check_tune(bad))
        short = json.loads(json.dumps(base))
        short["pages"]["1"] = "AAAA"
        self.assertIn("tamanho", P.check_tune(short))


if __name__ == "__main__":
    unittest.main()
