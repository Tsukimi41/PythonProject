import unittest
import logging

from drone_precision import (
    PIDController,
    PrecisionDroneController,
    SensorSnapshot,
    calculate_stopping_distance_mm,
    default_pid_profile,
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
