"""Dashboard protocol against the simulated ECU (firmware code on a pty).

Needs the sim binary: `make sim-ecu-build` (path via SIM_ECU, default
/tmp/openems-build/host/sim_ecu). Skipped when it is missing.
"""
import os
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import protocol as P  # noqa: E402

SIM = os.environ.get("SIM_ECU", "/tmp/openems-build/host/sim_ecu")


class SimEcu:
    def __init__(self, nvm: str):
        self.proc = subprocess.Popen([SIM, "--nvm", nvm], stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True)
        line = self.proc.stdout.readline()
        while "sim ECU on" not in line:  # e.g. "loaded flash image ..."
            line = self.proc.stdout.readline()
        self.port = line.split()[-1]
        self.link = P.OpenEMSLink(self.port, timeout=1.0)

    def close(self):
        self.link.close()
        self.proc.kill()
        self.proc.wait()
        self.proc.stdout.close()


@unittest.skipUnless(os.path.exists(SIM), "sim ECU not built (make sim-ecu-build)")
class TestSimEcu(unittest.TestCase):
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

    def test_identity_and_realtime(self):
        self.assertTrue(self.ecu.link.query().startswith("OpenEMS"))
        rt = self.ecu.link.read_realtime()
        self.assertEqual(rt.rpm, 0)

    def test_field_write_burn_survives_restart(self):
        link = self.ecu.link
        page0 = link.read_page(0)
        fields = P.decode_fields(0, page0)
        self.assertEqual(fields["displacement_cc"], 2000)
        off, data = P.encode_field(0, "displacement_cc", 1598)
        link.write_page_ram(0, off, data)
        link.burn_page(0)
        self.restart()
        fields = P.decode_fields(0, self.ecu.link.read_page(0))
        self.assertEqual(fields["displacement_cc"], 1598)

    def test_ve_cell_burn_survives_restart(self):
        link = self.ecu.link
        link.write_page_ram(1, 5 * 20 + 7, P.encode_cell_u8(91))
        link.burn_page(1)
        self.restart()
        grid = P.decode_grid_u8(self.ecu.link.read_page(1))
        self.assertEqual(grid[5][7], 91)

    def test_output_test_cycle(self):
        link = self.ecu.link
        link.test_enter()
        self.assertTrue(link.test_status()["active"])
        link.test_fire_inj(0, 3000)
        link.test_exit()
        self.assertFalse(link.test_status()["active"])


if __name__ == "__main__":
    unittest.main()
