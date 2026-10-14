from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from aquabot_simulator.config import RunConfig, load_config
from aquabot_simulator.standalone import simulate
from aquabot_simulator.vehicle_state import FlightMode


PROJECT = Path(__file__).resolve().parents[1]


class RunnerTests(unittest.TestCase):
    def test_default_config_is_disarmed(self):
        cfg = load_config(PROJECT / "config" / "default.yaml")
        self.assertFalse(cfg.armed)
        self.assertEqual(list(simulate(cfg))[-1].north_m, 0)

    def test_partial_final_step_and_repeatability(self):
        cfg = RunConfig(duration_s=0.105, dt_s=0.02,
                        armed=True, mode=FlightMode.GUIDED)
        first = list(simulate(cfg))
        self.assertEqual(first, list(simulate(cfg)))
        self.assertAlmostEqual(first[-1].time_s, 0.105)
        self.assertEqual(len(first), 7)
        self.assertGreater(first[-1].north_m, 0)

    def test_nonzero_initial_time_and_zero_duration(self):
        cfg = RunConfig(duration_s=0)
        cfg = replace(cfg, initial=replace(cfg.initial, time_s=10))
        self.assertEqual(list(simulate(cfg)), [cfg.initial])
        cfg = replace(cfg, duration_s=1)
        self.assertAlmostEqual(list(simulate(cfg))[-1].time_s, 11)

    def test_strict_config(self):
        for data in ({"unknown": 1}, {"operation": {"armed": "false"}},
                     {"dynamics": {"mass_kg": 0}},
                     {"simulation": {"dt_s": -1}},
                     {"operation": {"mode": "AUTO"}}):
            with self.subTest(data=data), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "config.yaml"
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_config(path)

    def test_cli_forward_and_inhibited(self):
        for extra, moving in ((["--armed", "--mode", "GUIDED"], True),
                              (["--armed", "--mode", "HOLD"], False),
                              (["--no-armed", "--mode", "GUIDED"], False)):
            result = subprocess.run(
                [sys.executable, "-B", "-m", "aquabot_simulator.standalone",
                 "--duration", "2", *extra], cwd=PROJECT,
                capture_output=True, text=True, check=True)
            data = json.loads(result.stdout)
            self.assertTrue(data["simulation_only"])
            self.assertEqual(data["state"]["north_m"] > 0, moving)

    def test_cli_invalid_input(self):
        result = subprocess.run(
            [sys.executable, "-B", "-m", "aquabot_simulator.standalone",
             "--dt", "nan"], cwd=PROJECT, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("finite", result.stderr)

    def test_standalone_blocks_all_hardware_and_ros_imports(self):
        # A fresh process prevents cached modules from bypassing this guard.
        script = '''
import importlib.abc
import sys
class BlockHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'serial', 'rclpy', 'boat_control',
                                      'boat_iface', 'pyubx2', 'pymavlink'}:
            raise AssertionError('Forbidden import: ' + fullname)
sys.meta_path.insert(0, BlockHardware())
from aquabot_simulator.standalone import main
main(['--duration', '1', '--armed', '--mode', 'GUIDED'])
'''
        result = subprocess.run([sys.executable, "-B", "-c", script],
                                cwd=PROJECT, capture_output=True, text=True,
                                check=True)
        self.assertGreater(json.loads(result.stdout)["state"]["north_m"], 0)
