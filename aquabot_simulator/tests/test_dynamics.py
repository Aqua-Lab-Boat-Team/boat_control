from dataclasses import FrozenInstanceError, replace
import unittest

from aquabot_simulator import DynamicsConfig, FlightMode, MotorConfig, Simulator, VehicleState
from aquabot_simulator.vehicle_state import wrap_angle_deg


def moving_sim(**kwargs):
    sim = Simulator(**kwargs)
    sim.set_operating_state(True, FlightMode.GUIDED)
    return sim


class DynamicsTests(unittest.TestCase):
    def test_equal_forward_commands_move_north(self):
        sim = moving_sim()
        sim.set_commands(0.8, 0.8)
        state = sim.step(10)
        self.assertGreater(state.north_m, 1)
        self.assertEqual(state.east_m, 0)
        self.assertEqual(state.heading_deg, 0)
        self.assertGreater(state.forward_mps, 0)

    def test_reverse_commands_move_south(self):
        sim = moving_sim()
        sim.set_commands(-0.8, -0.8)
        state = sim.step(5)
        self.assertLess(state.north_m, 0)
        self.assertLess(state.forward_mps, 0)

    def test_heading_east_moves_east(self):
        sim = moving_sim(initial=VehicleState(heading_deg=90))
        sim.set_commands(0.5, 0.5)
        state = sim.step(5)
        self.assertGreater(state.east_m, 0)
        self.assertAlmostEqual(state.north_m, 0)

    def test_differential_turn_direction(self):
        clockwise = moving_sim()
        clockwise.set_commands(0.8, 0.2)
        state = clockwise.step(2)
        self.assertGreater(state.heading_deg, 0)
        self.assertLess(state.heading_deg, 90)
        self.assertGreater(state.yaw_rate_deg_s, 0)
        counterclockwise = moving_sim()
        counterclockwise.set_commands(0.2, 0.8)
        state = counterclockwise.step(2)
        self.assertLess(state.yaw_rate_deg_s, 0)
        self.assertGreater(state.heading_deg, 270)

    def test_opposed_commands_turn_without_translation(self):
        sim = moving_sim()
        sim.set_commands(0.8, -0.8)
        state = sim.step(5)
        self.assertEqual(state.north_m, 0)
        self.assertEqual(state.east_m, 0)
        self.assertGreater(state.yaw_rate_deg_s, 0)

    def test_motor_asymmetry_curves_trajectory(self):
        sim = moving_sim(right_motor=MotorConfig(efficiency=0.6))
        sim.set_commands(0.8, 0.8)
        state = sim.step(3)
        self.assertGreater(state.left_thrust_n, state.right_thrust_n)
        self.assertGreater(state.east_m, 0)
        self.assertGreater(state.yaw_rate_deg_s, 0)

    def test_drag_coasts_and_decelerates(self):
        sim = moving_sim(initial=VehicleState(forward_mps=2))
        first = sim.step(0.02)
        self.assertGreater(first.forward_mps, 0)
        self.assertLess(first.forward_mps, 2)
        last = sim.step(10)
        self.assertGreater(last.forward_mps, 0)
        self.assertLess(last.forward_mps, first.forward_mps)
        self.assertGreater(last.north_m, first.north_m)

    def test_zero_drag_preserves_momentum(self):
        cfg = DynamicsConfig(linear_drag_n_per_mps=0, quadratic_drag_n_per_mps2=0)
        sim = Simulator(config=cfg, initial=VehicleState(forward_mps=2))
        state = sim.step(1)
        self.assertAlmostEqual(state.forward_mps, 2)
        self.assertAlmostEqual(state.north_m, 2)

    def test_yaw_drag_decelerates(self):
        sim = Simulator(initial=VehicleState(yaw_rate_deg_s=-30))
        state = sim.step(1)
        self.assertGreater(state.yaw_rate_deg_s, -30)
        self.assertLess(state.yaw_rate_deg_s, 0)

    def test_acceleration_limits(self):
        cfg = DynamicsConfig(max_acceleration_mps2=0.1,
                             max_yaw_acceleration_deg_s2=1)
        motor = MotorConfig(response_time_s=0, max_thrust_n=1000)
        sim = moving_sim(config=cfg, left_motor=motor, right_motor=motor)
        sim.set_commands(1, 0)
        state = sim.step(0.02)
        self.assertLessEqual(state.forward_mps, 0.1 * 0.02 + 1e-12)
        self.assertLessEqual(state.yaw_rate_deg_s, 1 * 0.02 + 1e-12)

    def test_saturation(self):
        sim = moving_sim()
        sim.set_commands(50, -50)
        state = sim.step(1)
        self.assertEqual((state.left_command, state.right_command), (1, -1))

    def test_default_disarmed_hold(self):
        sim = Simulator()
        self.assertFalse(sim.armed)
        self.assertEqual(sim.mode, FlightMode.HOLD)
        sim.set_commands(1, 1)
        self.assertEqual(sim.step(2).north_m, 0)

    def test_inhibited_modes_and_disarming(self):
        for armed, mode in ((False, FlightMode.GUIDED), (True, FlightMode.HOLD),
                            (True, 10), (True, 999)):
            with self.subTest(armed=armed, mode=mode):
                sim = moving_sim()
                sim.set_commands(0.8, 0.8)
                before = sim.step(5)
                sim.set_operating_state(armed, mode)
                sim.set_commands(1, 1)
                state = sim.step(0.02)
                self.assertEqual(state.left_command, 0)
                self.assertEqual(state.right_command, 0)
                self.assertGreater(state.forward_mps, 0)
                self.assertLess(state.left_thrust_n, before.left_thrust_n)
                sim.set_operating_state(True, FlightMode.GUIDED)
                self.assertEqual(sim.step(0.02).left_command, 0)

    def test_manual_mode_accepts_commands(self):
        sim = Simulator()
        sim.set_operating_state(True, FlightMode.MANUAL)
        sim.set_commands(0.5, 0.5)
        self.assertGreater(sim.step(2).north_m, 0)

    def test_repeated_supervisor_state_does_not_clear_commands(self):
        sim = moving_sim()
        sim.set_commands(0.5, 0.5)
        sim.set_operating_state(True, FlightMode.GUIDED)
        self.assertEqual(sim.step(0.02).left_command, 0.5)

    def test_rest_stays_at_rest(self):
        sim = moving_sim()
        self.assertEqual(sim.step(20), VehicleState(time_s=20))

    def test_repeatable(self):
        first, second = moving_sim(), moving_sim()
        for index in range(100):
            left = 0.8 if index < 50 else 0
            for sim in (first, second):
                sim.set_commands(left, 0.2)
            self.assertEqual(first.step(0.02), second.step(0.02))

    def test_substeps_match_small_steps(self):
        first, second = moving_sim(), moving_sim()
        for sim in (first, second):
            sim.set_commands(0.8, 0.3)
        state = first.step(1)
        for _ in range(50):
            second.step(0.02)
        for name in ("east_m", "north_m", "heading_deg", "forward_mps"):
            self.assertAlmostEqual(getattr(state, name), getattr(second.state, name))

    def test_snapshot_is_immutable(self):
        sim = moving_sim()
        old = sim.state
        with self.assertRaises(FrozenInstanceError):
            old.north_m = 4
        sim.set_commands(1, 1)
        sim.step(1)
        self.assertEqual(old.north_m, 0)

    def test_heading_normalization(self):
        self.assertEqual(VehicleState(heading_deg=-90).heading_deg, 270)
        self.assertEqual(wrap_angle_deg(359), -1)
        self.assertEqual(wrap_angle_deg(181), -179)
        self.assertEqual(wrap_angle_deg(-181), 179)

    def test_invalid_dt_does_not_advance(self):
        sim = moving_sim()
        for dt in (0, -1, 61, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                sim.step(dt)
            self.assertEqual(sim.state, VehicleState())

    def test_invalid_state_and_configuration(self):
        for kwargs in ({"mass_kg": 0}, {"max_substep_s": 0},
                       {"linear_drag_n_per_mps": -1},
                       {"max_acceleration_mps2": float("nan")}):
            with self.assertRaises(ValueError):
                DynamicsConfig(**kwargs)
        with self.assertRaises(ValueError):
            VehicleState(north_m=float("nan"))
        with self.assertRaises(ValueError):
            Simulator(initial=replace(VehicleState(), left_thrust_n=1))
