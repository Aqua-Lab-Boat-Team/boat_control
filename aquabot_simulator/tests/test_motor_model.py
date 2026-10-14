import math
import unittest

from aquabot_simulator.motor_model import MotorConfig, MotorModel


class MotorTests(unittest.TestCase):
    def test_response_time_constant(self):
        motor = MotorModel(MotorConfig(dead_zone=0, response_time_s=0.5))
        motor.step(1, 0.5)
        self.assertAlmostEqual(motor.state, 1 - math.exp(-1))
        self.assertLess(motor.thrust_n, 20)

    def test_response_is_independent_of_step_partition(self):
        first, second = MotorModel(), MotorModel()
        first.step(0.8, 1)
        for _ in range(100):
            second.step(0.8, 0.01)
        self.assertAlmostEqual(first.state, second.state)

    def test_zero_lag_and_saturation(self):
        motor = MotorModel(MotorConfig(response_time_s=0))
        self.assertEqual(motor.step(10, 0.02), 20)
        self.assertEqual(motor.step(-10, 0.02), -20)

    def test_dead_zone_and_rescaling(self):
        motor = MotorModel(MotorConfig(response_time_s=0, dead_zone=0.2))
        for command in (0, 0.1, -0.1, 0.2, -0.2):
            self.assertEqual(motor.step(command, 0.1), 0)
        self.assertAlmostEqual(motor.step(0.6, 0.1), 5)
        self.assertAlmostEqual(motor.step(-0.6, 0.1), -5)

    def test_asymmetry_scales_thrust(self):
        motor = MotorModel(MotorConfig(response_time_s=0, efficiency=0.8))
        self.assertEqual(motor.step(1, 0.1), 16)

    def test_command_zero_decays_without_instant_stop(self):
        motor = MotorModel()
        motor.step(1, 1)
        before = motor.thrust_n
        after = motor.step(0, 0.1)
        self.assertGreater(after, 0)
        self.assertLess(after, before)

    def test_invalid_commands_do_not_mutate(self):
        for command in (float("nan"), float("inf"), True, "1"):
            motor = MotorModel()
            with self.assertRaises(ValueError):
                motor.step(command, 0.1)
            self.assertEqual(motor.state, 0)

    def test_invalid_dt(self):
        for dt in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                MotorModel().step(1, dt)

    def test_invalid_configs(self):
        for kwargs in ({"dead_zone": 1}, {"response_time_s": -1},
                       {"efficiency": -1}, {"exponent": 0.5},
                       {"max_thrust_n": float("nan")},
                       {"efficiency": True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MotorConfig(**kwargs)
