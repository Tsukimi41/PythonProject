import unittest

from time_calibrated_xy import (
    Calibration,
    DirectionCalibration,
    plan_move,
    update_direction,
    update_landing,
    update_trim,
)


class TimeCalibratedXYTest(unittest.TestCase):
    def test_plan_move_uses_direction_specific_signs(self):
        calibration = Calibration(
            forward=DirectionCalibration(power=20, speed_cm_s=10.0),
            back=DirectionCalibration(power=-21, speed_cm_s=20.0),
            left=DirectionCalibration(power=-22, speed_cm_s=30.0),
            right=DirectionCalibration(power=23, speed_cm_s=40.0),
            max_segment_sec=10.0,
        )

        segments = plan_move(-40.0, -80.0, calibration)

        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0].direction, "back")
        self.assertEqual(segments[0].power, -21)
        self.assertAlmostEqual(segments[0].duration_sec, 2.0)
        self.assertEqual(segments[1].direction, "right")
        self.assertEqual(segments[1].power, 23)
        self.assertAlmostEqual(segments[1].duration_sec, 2.0)

    def test_plan_move_splits_long_segments(self):
        calibration = Calibration.default()
        segments = plan_move(100.0, 0.0, calibration)

        self.assertGreater(len(segments), 1)
        self.assertTrue(all(segment.duration_sec <= calibration.max_segment_sec for segment in segments))
        self.assertAlmostEqual(sum(segment.distance_cm for segment in segments), 100.0)

    def test_update_direction_uses_measured_speed(self):
        calibration = Calibration.default()
        updated = update_direction(
            calibration=calibration,
            direction="left",
            measured_cm=25.0,
            duration_sec=2.0,
            power=18,
        )

        self.assertEqual(updated.left.power, -18)
        self.assertAlmostEqual(updated.left.speed_cm_s, 12.5)
        self.assertEqual(updated.forward, calibration.forward)

    def test_update_trim_keeps_motion_calibration(self):
        calibration = Calibration.default()
        updated = update_trim(calibration, roll=-4, pitch=6)

        self.assertEqual(updated.trim_roll, -4)
        self.assertEqual(updated.trim_pitch, 6)
        self.assertEqual(updated.forward, calibration.forward)
        self.assertEqual(updated.right, calibration.right)

    def test_update_landing_keeps_takeoff_trim(self):
        calibration = update_trim(Calibration.default(), roll=-3, pitch=-4)
        updated = update_landing(
            calibration,
            roll=-2,
            pitch=-5,
            throttle=-20,
            duration_sec=1.8,
        )

        self.assertEqual(updated.trim_roll, -3)
        self.assertEqual(updated.trim_pitch, -4)
        self.assertEqual(updated.landing_roll, -2)
        self.assertEqual(updated.landing_pitch, -5)
        self.assertEqual(updated.landing_throttle, -20)
        self.assertAlmostEqual(updated.landing_duration_sec, 1.8)

    def test_update_landing_rejects_positive_throttle(self):
        with self.assertRaises(ValueError):
            update_landing(
                Calibration.default(),
                roll=0,
                pitch=0,
                throttle=10,
                duration_sec=1.0,
            )


if __name__ == "__main__":
    unittest.main()
