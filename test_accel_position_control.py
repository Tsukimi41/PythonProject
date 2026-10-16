import unittest
from enum import Enum

from accel_position_control import (
    RAW_ACCEL_TO_MM_S2,
    AccelerationBias,
    AccelerationPositionEstimator,
    AccelerationReading,
    ControlConfig,
    EstimatorConfig,
    PositionController,
    PositionEstimate,
    build_parser,
    calibrate_accelerometer,
    check_takeoff_success,
)


class FakeModeFlight(Enum):
    Flight = 19
    Ready = 16


class FakeModeMovement(Enum):
    Hovering = 2
    Ready = 1


class FakeDroneForTakeoffCheck:
    def __init__(self, *, height_cm: float, state_error_flags: int = 0) -> None:
        self.height_cm = height_cm
        self.state_error_flags = state_error_flags

    def get_battery(self):
        return 65

    def get_state_data(self):
        return [
            7.3,
            "Running",
            FakeModeFlight.Flight,
            "Position",
            "Normal",
            "Normal",
            65,
            FakeModeMovement.Hovering,
            2,
        ]

    def get_error_data(self, print_error=True):
        return [7.4, 0, self.state_error_flags]

    def get_height(self, unit="cm"):
        return self.height_cm


class FakeAccelerationReader:
    def __init__(self, readings: list[AccelerationReading]) -> None:
        self.readings = readings
        self.index = 0

    def read(self) -> AccelerationReading:
        if self.index >= len(self.readings):
            return self.readings[-1]
        reading = self.readings[self.index]
        self.index += 1
        return reading


class CalibrationTests(unittest.TestCase):
    def test_calibration_uses_mean_raw_acceleration(self) -> None:
        reader = FakeAccelerationReader(
            [
                AccelerationReading(0.00, 1.0, -2.0, 10.0),
                AccelerationReading(0.01, 3.0, -4.0, 12.0),
                AccelerationReading(0.02, 5.0, -6.0, 14.0),
            ]
        )

        bias = calibrate_accelerometer(
            reader,
            duration_sec=0.02,
            sample_period_sec=0.0,
            sample_count=3,
        )

        self.assertAlmostEqual(bias.raw_x, 3.0)
        self.assertAlmostEqual(bias.raw_y, -4.0)
        self.assertAlmostEqual(bias.raw_z, 12.0)


class TakeoffCheckTests(unittest.TestCase):
    def test_flight_and_hovering_state_is_success_even_when_height_sensor_is_zero(self) -> None:
        check = check_takeoff_success(FakeDroneForTakeoffCheck(height_cm=0.0), 8.0)

        self.assertTrue(check.success)
        self.assertIn("height sensor reported low altitude", check.warning)

    def test_state_error_keeps_takeoff_check_failed(self) -> None:
        check = check_takeoff_success(
            FakeDroneForTakeoffCheck(height_cm=20.0, state_error_flags=16),
            8.0,
        )

        self.assertFalse(check.success)


class EstimatorTests(unittest.TestCase):
    def test_raw_acceleration_unit_conversion_matches_codrone_documented_scale(self) -> None:
        self.assertEqual(RAW_ACCEL_TO_MM_S2, 100.0)

    def test_bias_corrected_constant_acceleration_integrates_position(self) -> None:
        config = EstimatorConfig(
            accel_low_pass_alpha=1.0,
            accel_deadband_mm_s2=0.0,
            velocity_leak_per_sec=0.0,
        )
        estimator = AccelerationPositionEstimator(
            config,
            AccelerationBias(raw_x=1.0, raw_y=0.0, raw_z=0.0),
        )

        estimate = estimator.update(
            AccelerationReading(0.1, raw_x=3.0, raw_y=0.0, raw_z=0.0),
            dt_sec=0.1,
        )

        self.assertAlmostEqual(estimate.ax_mm_s2, 200.0)
        self.assertAlmostEqual(estimate.vx_mm_s, 20.0)
        self.assertAlmostEqual(estimate.x_mm, 1.0)

    def test_deadband_suppresses_small_bias_noise(self) -> None:
        config = EstimatorConfig(
            accel_low_pass_alpha=1.0,
            accel_deadband_mm_s2=150.0,
            velocity_leak_per_sec=0.0,
        )
        estimator = AccelerationPositionEstimator(config, AccelerationBias())

        estimate = estimator.update(
            AccelerationReading(0.1, raw_x=1.0, raw_y=1.0, raw_z=0.0),
            dt_sec=0.1,
        )

        self.assertEqual(estimate.ax_mm_s2, 0.0)
        self.assertEqual(estimate.ay_mm_s2, 0.0)
        self.assertEqual(estimate.x_mm, 0.0)
        self.assertEqual(estimate.y_mm, 0.0)

    def test_acceleration_axis_sign_can_be_inverted(self) -> None:
        config = EstimatorConfig(
            accel_low_pass_alpha=1.0,
            accel_deadband_mm_s2=0.0,
            velocity_leak_per_sec=0.0,
            accel_x_sign=-1.0,
        )
        estimator = AccelerationPositionEstimator(config, AccelerationBias())

        estimate = estimator.update(
            AccelerationReading(0.1, raw_x=2.0, raw_y=0.0, raw_z=0.0),
            dt_sec=0.1,
        )

        self.assertAlmostEqual(estimate.ax_mm_s2, -200.0)


class ControllerTests(unittest.TestCase):
    def test_positive_x_target_commands_positive_pitch(self) -> None:
        controller = PositionController(ControlConfig(target_x_cm=30.0, target_y_cm=0.0))

        pitch_cmd, roll_cmd, _error_x, _error_y = controller.compute(PositionEstimate())

        self.assertGreater(pitch_cmd, 0)
        self.assertEqual(roll_cmd, 0)

    def test_positive_y_target_commands_left_roll_negative(self) -> None:
        controller = PositionController(ControlConfig(target_x_cm=0.0, target_y_cm=30.0))

        pitch_cmd, roll_cmd, _error_x, _error_y = controller.compute(PositionEstimate())

        self.assertEqual(pitch_cmd, 0)
        self.assertLess(roll_cmd, 0)

    def test_output_is_limited_by_configured_power(self) -> None:
        controller = PositionController(
            ControlConfig(target_x_cm=500.0, target_y_cm=-500.0, max_pitch=12, max_roll=13)
        )

        pitch_cmd, roll_cmd, _error_x, _error_y = controller.compute(PositionEstimate())

        self.assertEqual(pitch_cmd, 12)
        self.assertEqual(roll_cmd, 13)

    def test_command_signs_can_be_inverted_for_field_validation(self) -> None:
        controller = PositionController(
            ControlConfig(
                target_x_cm=30.0,
                target_y_cm=30.0,
                pitch_sign_for_positive_x=-1.0,
                roll_sign_for_positive_y=1.0,
            )
        )

        pitch_cmd, roll_cmd, _error_x, _error_y = controller.compute(PositionEstimate())

        self.assertLess(pitch_cmd, 0)
        self.assertGreater(roll_cmd, 0)


class CliDefaultTests(unittest.TestCase):
    def test_cli_default_inverts_codrone_x_acceleration_seen_in_logs(self) -> None:
        args = build_parser().parse_args(["--x-cm", "30", "--y-cm", "0"])

        self.assertEqual(args.accel_x_sign, -1.0)


if __name__ == "__main__":
    unittest.main()
