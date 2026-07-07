import unittest
import logging

from drone_precision import (
    FIXED_CONTROL_SIGNS,
    ControlSample,
    PIDController,
    PrecisionDroneController,
    SensorSnapshot,
    calculate_stopping_distance_mm,
    default_pid_profile,
    rotate_body_to_world,
    rotate_world_to_body,
)


def snapshot(*, wall_time: float, sensor_time: float, x_mm: float) -> SensorSnapshot:
    return SensorSnapshot(
        wall_time=wall_time,
        pos_age_sec=0.0,
        motion_age_sec=0.0,
        pos_sensor_time=sensor_time,
        motion_sensor_time=sensor_time,
        x_mm=x_mm,
        y_mm=0.0,
        z_mm=500.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        accel_x=0.0,
        accel_y=0.0,
        accel_z=0.0,
        gyro_roll=0.0,
        gyro_pitch=0.0,
        gyro_yaw=0.0,
    )


class StoppingDistanceTests(unittest.TestCase):
    def test_includes_physical_braking_latency_and_margin(self) -> None:
        distance = calculate_stopping_distance_mm(500.0, 900.0, 0.33)
        self.assertAlmostEqual(distance, 338.8888889)

    def test_stationary_distance_is_margin_only(self) -> None:
        self.assertEqual(calculate_stopping_distance_mm(0.0, 900.0, 0.5), 35.0)


class StateEstimatorTests(unittest.TestCase):
    def test_velocity_uses_time_between_fresh_position_packets(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        first = snapshot(wall_time=0.0, sensor_time=1.0, x_mm=0.0)
        first_state = controller._compute_state_estimate(first, None, None)

        stale = snapshot(wall_time=0.1, sensor_time=1.0, x_mm=0.0)
        stale_state = controller._compute_state_estimate(stale, first, first_state)
        fresh = snapshot(wall_time=0.5, sensor_time=2.0, x_mm=50.0)
        fresh_state = controller._compute_state_estimate(fresh, stale, stale_state)

        # Raw velocity is 50 / 0.5 = 100mm/s and alpha is 0.4.
        self.assertAlmostEqual(fresh_state.x_velocity_mm_s, 40.0)


class CoordinateTransformTests(unittest.TestCase):
    def test_world_to_body_keeps_axes_when_yaw_is_zero(self) -> None:
        forward, right = rotate_world_to_body(100.0, 50.0, 0.0)
        self.assertAlmostEqual(forward, 100.0)
        self.assertAlmostEqual(right, 50.0)

    def test_world_to_body_rotates_error_by_current_yaw(self) -> None:
        forward, right = rotate_world_to_body(100.0, 0.0, 90.0)
        self.assertAlmostEqual(forward, 0.0, places=6)
        self.assertAlmostEqual(right, -100.0, places=6)

    def test_body_to_world_inverts_world_to_body(self) -> None:
        forward, right = rotate_world_to_body(120.0, -35.0, 24.0)
        x_value, y_value = rotate_body_to_world(forward, right, 24.0)
        self.assertAlmostEqual(x_value, 120.0)
        self.assertAlmostEqual(y_value, -35.0)


class ConfigurationTests(unittest.TestCase):
    def test_yaw_command_sign_matches_observed_codrone_direction(self) -> None:
        self.assertEqual(FIXED_CONTROL_SIGNS["yaw"], -1.0)


def control_sample(*, step: int, error_x_mm: float, error_y_mm: float) -> ControlSample:
    return ControlSample(
        step=step,
        wall_time=float(step),
        elapsed_sec=float(step),
        dt_sec=0.1,
        pos_fresh=True,
        motion_fresh=True,
        stale_cycles=0,
        stale_duration_sec=0.0,
        target_x_mm=0.0,
        target_y_mm=0.0,
        target_z_mm=0.0,
        target_yaw_deg=0.0,
        x_mm=0.0,
        y_mm=0.0,
        z_mm=800.0,
        yaw_deg=0.0,
        error_x_mm=error_x_mm,
        error_y_mm=error_y_mm,
        error_z_mm=0.0,
        error_yaw_deg=0.0,
        x_velocity_mm_s=0.0,
        y_velocity_mm_s=0.0,
        z_velocity_mm_s=0.0,
        yaw_rate_deg_s=0.0,
        pitch_cmd=0,
        roll_cmd=0,
        throttle_cmd=0,
        yaw_cmd=0,
        pitch_limit=0.0,
        roll_limit=0.0,
        throttle_limit=0.0,
        yaw_limit=0.0,
        pitch_integral=0.0,
        roll_integral=0.0,
        throttle_integral=0.0,
        yaw_integral=0.0,
        control_phase="settle",
        predicted_remaining_x_mm=0.0,
        stopping_distance_x_mm=0.0,
    )


class InternalSensorGuardTests(unittest.TestCase):
    def test_horizontal_drift_guard_limits_growing_oscillation(self) -> None:
        samples = []
        for step in range(72):
            magnitude = 40.0 + step * 2.0
            signed = magnitude if step % 2 == 0 else -magnitude
            samples.append(control_sample(step=step, error_x_mm=signed, error_y_mm=0.0))

        scale = PrecisionDroneController._horizontal_drift_guard_scale(samples, 210.0)

        self.assertLess(scale, 1.0)


class PIDControllerTests(unittest.TestCase):
    def make_pid(self, *, kp: float, ki: float = 0.0) -> PIDController:
        return PIDController(
            kp=kp,
            ki=ki,
            kd=0.0,
            kv=0.0,
            output_limit=20.0,
            integral_limit=100.0,
            min_output=6.0,
        )

    def test_minimum_output_is_disabled_near_target(self) -> None:
        pid = self.make_pid(kp=0.1)
        output = pid.compute(
            error=10.0,
            measurement=0.0,
            measurement_rate=0.0,
            dt=0.1,
            output_limit=20.0,
            min_output=6.0,
            min_output_error=30.0,
        )
        self.assertEqual(output, 1.0)

    def test_integral_does_not_grow_while_output_is_saturated(self) -> None:
        pid = self.make_pid(kp=1.0, ki=1.0)
        pid.compute(
            error=30.0,
            measurement=0.0,
            measurement_rate=0.0,
            dt=1.0,
            output_limit=20.0,
            min_output=0.0,
        )
        self.assertEqual(pid.integral, 0.0)


class SettleCommandShapeTests(unittest.TestCase):
    def test_premature_reverse_command_is_forced_back_toward_target(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        shaped = controller._shape_settle_axis_command(
            command=-1,
            sign=1.0,
            error_mm=54.0,
            velocity_mm_s=44.0,
            output_limit=40.0,
            min_drive=6.0,
            min_brake=6.0,
            position_interval_sec=0.12,
            brake_accel_mm_s2=90.0,
            close_error_mm=10.0,
        )
        self.assertGreaterEqual(shaped, 6)

    def test_close_fast_motion_is_braked(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        shaped = controller._shape_settle_axis_command(
            command=12,
            sign=1.0,
            error_mm=24.0,
            velocity_mm_s=80.0,
            output_limit=40.0,
            min_drive=6.0,
            min_brake=7.0,
            position_interval_sec=0.12,
            brake_accel_mm_s2=90.0,
            close_error_mm=10.0,
        )
        self.assertLessEqual(shaped, -7)

    def test_velocity_servo_drives_back_when_far_and_slow(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        command = controller._settle_velocity_axis_command(
            command=0,
            sign=-1.0,
            error_mm=70.0,
            velocity_mm_s=0.0,
            output_limit=42.0,
            max_speed_mm_s=130.0,
            min_speed_mm_s=24.0,
            velocity_gain=0.160,
            position_gain=0.022,
            min_drive=6.0,
            min_brake=7.0,
            position_interval_sec=0.12,
            brake_accel_mm_s2=90.0,
            close_error_mm=12.0,
            close_speed_mm_s=18.0,
        )
        self.assertLessEqual(command, -12)

    def test_velocity_servo_brakes_when_close_and_fast(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        command = controller._settle_velocity_axis_command(
            command=0,
            sign=1.0,
            error_mm=20.0,
            velocity_mm_s=80.0,
            output_limit=42.0,
            max_speed_mm_s=130.0,
            min_speed_mm_s=24.0,
            velocity_gain=0.160,
            position_gain=0.022,
            min_drive=6.0,
            min_brake=7.0,
            position_interval_sec=0.12,
            brake_accel_mm_s2=90.0,
            close_error_mm=12.0,
            close_speed_mm_s=18.0,
        )
        self.assertLessEqual(command, -7)


class _FakeLog:
    def __init__(self) -> None:
        self.records = []

    def write(self, event: str, payload: dict) -> None:
        self.records.append((event, payload))


class _FakeDrone:
    def __init__(self) -> None:
        self.pitch = 0
        self.roll = 0
        self.throttle = 0

    def set_roll(self, value: int) -> None:
        self.roll = value

    def set_pitch(self, value: int) -> None:
        self.pitch = value

    def set_throttle(self, value: int) -> None:
        self.throttle = value

    def set_yaw(self, value: int) -> None:
        pass

    def move(self) -> None:
        pass


class RapidTransitTests(unittest.TestCase):
    def test_long_move_uses_full_acceleration_and_full_braking(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        controller.logger = logging.getLogger("rapid-transit-test")
        controller.profile = default_pid_profile()
        controller.pitch_pid = controller._make_pid(controller.profile["pitch"])
        controller.roll_pid = controller._make_pid(controller.profile["roll"])
        controller.throttle_pid = controller._make_pid(controller.profile["throttle"])
        controller.yaw_pid = controller._make_pid(controller.profile["yaw"])
        controller.pitch_sign = 1.0
        controller.roll_sign = -1.0
        controller.throttle_sign = 1.0
        controller.yaw_sign = 1.0
        controller.drone = _FakeDrone()
        controller.log_writer = _FakeLog()

        initial = snapshot(wall_time=0.0, sensor_time=0.0, x_mm=0.0)
        controller.acquire_reference_snapshot = lambda: initial
        controller.stop_and_hover = lambda duration=0.25: None
        plant = {"time": 0.0, "x": 0.0, "velocity": 0.0, "sensor_time": 0.0}

        def read_snapshot() -> SensorSnapshot:
            dt = 0.05
            acceleration = controller.drone.pitch * 10.0
            plant["velocity"] += acceleration * dt
            plant["velocity"] *= 0.99
            plant["x"] += plant["velocity"] * dt
            plant["time"] += dt
            plant["sensor_time"] += 1.0
            return snapshot(
                wall_time=plant["time"],
                sensor_time=plant["sensor_time"],
                x_mm=plant["x"],
            )

        controller.read_snapshot = read_snapshot
        controller.move_relative_mm(
            target_x_mm=600.0,
            timeout_sec=0.80,
            loop_period_sec=0.001,
            settle_time_sec=0.01,
        )

        samples = [payload for event, payload in controller.log_writer.records if event == "sample"]
        phases = {sample["control_phase"] for sample in samples}
        pitch_commands = {sample["pitch_cmd"] for sample in samples}
        self.assertIn("accelerate", phases)
        self.assertIn("brake", phases)
        self.assertIn(100, pitch_commands)
        self.assertIn(-100, pitch_commands)

    def test_lateral_move_uses_full_roll_acceleration_and_braking(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        controller.logger = logging.getLogger("rapid-lateral-test")
        controller.profile = default_pid_profile()
        controller.pitch_pid = controller._make_pid(controller.profile["pitch"])
        controller.roll_pid = controller._make_pid(controller.profile["roll"])
        controller.throttle_pid = controller._make_pid(controller.profile["throttle"])
        controller.yaw_pid = controller._make_pid(controller.profile["yaw"])
        controller.pitch_sign = 1.0
        controller.roll_sign = -1.0
        controller.throttle_sign = 1.0
        controller.yaw_sign = 1.0
        controller.drone = _FakeDrone()
        controller.log_writer = _FakeLog()

        initial = snapshot(wall_time=0.0, sensor_time=0.0, x_mm=0.0)
        controller.acquire_reference_snapshot = lambda: initial
        controller.stop_and_hover = lambda duration=0.25: None
        plant = {"time": 0.0, "y": 0.0, "velocity": 0.0, "sensor_time": 0.0}

        def read_snapshot() -> SensorSnapshot:
            dt = 0.05
            acceleration = -controller.drone.roll * 10.0
            plant["velocity"] += acceleration * dt
            plant["velocity"] *= 0.99
            plant["y"] += plant["velocity"] * dt
            plant["time"] += dt
            plant["sensor_time"] += 1.0
            snap = snapshot(wall_time=plant["time"], sensor_time=plant["sensor_time"], x_mm=0.0)
            snap.y_mm = plant["y"]
            return snap

        controller.read_snapshot = read_snapshot
        controller.move_relative_mm(
            target_x_mm=0.0,
            target_y_mm=600.0,
            timeout_sec=0.80,
            loop_period_sec=0.001,
            settle_time_sec=0.01,
        )

        samples = [payload for event, payload in controller.log_writer.records if event == "sample"]
        phases = {sample["control_phase_y"] for sample in samples}
        roll_commands = {sample["roll_cmd"] for sample in samples}
        self.assertIn("accelerate", phases)
        self.assertIn("brake", phases)
        self.assertIn(100, roll_commands)
        self.assertIn(-100, roll_commands)

    def test_vertical_move_uses_throttle_rapid_phase(self) -> None:
        controller = object.__new__(PrecisionDroneController)
        controller.logger = logging.getLogger("rapid-vertical-test")
        controller.profile = default_pid_profile()
        controller.pitch_pid = controller._make_pid(controller.profile["pitch"])
        controller.roll_pid = controller._make_pid(controller.profile["roll"])
        controller.throttle_pid = controller._make_pid(controller.profile["throttle"])
        controller.yaw_pid = controller._make_pid(controller.profile["yaw"])
        controller.pitch_sign = 1.0
        controller.roll_sign = -1.0
        controller.throttle_sign = 1.0
        controller.yaw_sign = 1.0
        controller.drone = _FakeDrone()
        controller.log_writer = _FakeLog()

        initial = snapshot(wall_time=0.0, sensor_time=0.0, x_mm=0.0)
        controller.acquire_reference_snapshot = lambda: initial
        controller.stop_and_hover = lambda duration=0.25: None
        plant = {"time": 0.0, "z": 500.0, "velocity": 0.0, "sensor_time": 0.0}

        def read_snapshot() -> SensorSnapshot:
            dt = 0.05
            acceleration = controller.drone.throttle * 8.0
            plant["velocity"] += acceleration * dt
            plant["velocity"] *= 0.99
            plant["z"] += plant["velocity"] * dt
            plant["time"] += dt
            plant["sensor_time"] += 1.0
            snap = snapshot(wall_time=plant["time"], sensor_time=plant["sensor_time"], x_mm=0.0)
            snap.z_mm = plant["z"]
            return snap

        controller.read_snapshot = read_snapshot
        controller.move_relative_mm(
            target_x_mm=0.0,
            target_z_mm=300.0,
            timeout_sec=0.80,
            loop_period_sec=0.001,
            settle_time_sec=0.01,
        )

        samples = [payload for event, payload in controller.log_writer.records if event == "sample"]
        phases = {sample["control_phase_z"] for sample in samples}
        throttle_commands = {sample["throttle_cmd"] for sample in samples}
        self.assertIn("accelerate", phases)
        self.assertIn("brake", phases)
        self.assertIn(80, throttle_commands)
        self.assertIn(-80, throttle_commands)


if __name__ == "__main__":
    unittest.main()
