from __future__ import annotations

import argparse
import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean, pstdev
from typing import Iterable


LOG_DIR = Path(__file__).resolve().parent / "flight_logs"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def rotate_body_to_world(forward_cm: float, left_cm: float, yaw_deg: float) -> tuple[float, float]:
    """Convert drone-body movement to the takeoff-frame XY coordinates."""
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    world_x = forward_cm * cos_yaw - left_cm * sin_yaw
    world_y = forward_cm * sin_yaw + left_cm * cos_yaw
    return world_x, world_y


def rotate_world_to_body(world_x_cm: float, world_y_cm: float, yaw_deg: float) -> tuple[float, float]:
    """Convert takeoff-frame XY error to forward/left error for the current heading."""
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    forward_cm = world_x_cm * cos_yaw + world_y_cm * sin_yaw
    left_cm = -world_x_cm * sin_yaw + world_y_cm * cos_yaw
    return forward_cm, left_cm


@dataclass
class Pose:
    """Estimated pose in the coordinate frame at takeoff.

    x_cm: forward from the takeoff heading.
    y_cm: left from the takeoff heading.
    z_cm: height from the floor, when bottom range is valid.
    yaw_deg: current yaw relative to the takeoff heading.
    """

    x_cm: float = 0.0
    y_cm: float = 0.0
    z_cm: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class MotionModel:
    """Open-loop speed model.

    Tune these values with short test flights. Keep pitch/roll modest because
    large commands are nonlinear and cause extra drift.
    """

    forward_cm_s_per_pitch: float = 1.30
    left_cm_s_per_roll: float = -1.15
    command_deadband: int = 4
    max_dt_sec: float = 0.15

    def speed_from_pitch(self, pitch: int) -> float:
        return self._linear_speed(pitch, self.forward_cm_s_per_pitch)

    def speed_from_roll(self, roll: int) -> float:
        return self._linear_speed(roll, self.left_cm_s_per_roll)

    def _linear_speed(self, command: int, gain: float) -> float:
        if abs(command) <= self.command_deadband:
            return 0.0
        effective_power = abs(command) - self.command_deadband
        return math.copysign(effective_power * abs(gain), command * gain)


@dataclass(frozen=True)
class WallCorrection:
    """Optional correction from a known wall in front of the drone.

    Example:
      axis="x", coordinate_cm=150, yaw_deg=0 means a wall is at x=150 cm.
      If the drone is facing +X and the front range reads 40 cm, x becomes 110 cm.
    """

    axis: str
    coordinate_cm: float
    yaw_deg: float
    max_yaw_error_deg: float = 12.0
    min_range_cm: float = 5.0
    max_range_cm: float = 180.0
    blend: float = 0.35


@dataclass(frozen=True)
class ColorMarker:
    """Optional floor-marker correction from a bottom color sensor."""

    color_name: str
    x_cm: float
    y_cm: float
    max_height_cm: float = 18.0


@dataclass(frozen=True)
class FlightStep:
    name: str
    pitch: int = 0
    roll: int = 0
    duration_sec: float = 0.0


@dataclass(frozen=True)
class Target:
    x_cm: float
    y_cm: float


@dataclass(frozen=True)
class TargetControlConfig:
    max_power: int = 24
    min_power: int = 7
    slow_radius_cm: float = 35.0
    command_per_cm: float = 0.75
    timeout_sec: float = 12.0
    stop_radius_cm: float = 6.0
    stop_window_sec: float = 1.0
    stop_std_cm: float = 2.5
    confidence_z: float = 1.96


@dataclass(frozen=True)
class StopStats:
    sample_count: int
    mean_error_cm: float
    std_error_cm: float
    upper_confidence_cm: float


class StatisticalStopDetector:
    """Stops only when recent estimated error is both small and stable."""

    def __init__(self, window_sec: float, sample_period_sec: float, confidence_z: float) -> None:
        sample_count = max(4, int(round(window_sec / sample_period_sec)))
        self.distances_cm: deque[float] = deque(maxlen=sample_count)
        self.confidence_z = confidence_z

    def add(self, distance_cm: float) -> StopStats | None:
        self.distances_cm.append(distance_cm)
        if len(self.distances_cm) < self.distances_cm.maxlen:
            return None

        values = list(self.distances_cm)
        mean_error = fmean(values)
        std_error = pstdev(values)
        standard_error = std_error / math.sqrt(len(values))
        upper_confidence = mean_error + self.confidence_z * standard_error
        return StopStats(
            sample_count=len(values),
            mean_error_cm=mean_error,
            std_error_cm=std_error,
            upper_confidence_cm=upper_confidence,
        )

    def has_arrived(self, stats: StopStats | None, config: TargetControlConfig) -> bool:
        if stats is None:
            return False
        return (
            stats.upper_confidence_cm <= config.stop_radius_cm
            and stats.std_error_cm <= config.stop_std_cm
        )


class DeadReckoningEstimator:
    """Position estimator that never reads optical-flow position APIs."""

    def __init__(self, motion_model: MotionModel | None = None) -> None:
        self.motion_model = motion_model or MotionModel()
        self.pose = Pose()

    def update_from_command(self, pitch: int, roll: int, yaw_deg: float, dt_sec: float) -> Pose:
        dt = clamp(dt_sec, 0.0, self.motion_model.max_dt_sec)
        self.pose.yaw_deg = wrap_angle_deg(yaw_deg)

        forward_cm = self.motion_model.speed_from_pitch(pitch) * dt
        left_cm = self.motion_model.speed_from_roll(roll) * dt
        dx_cm, dy_cm = rotate_body_to_world(forward_cm, left_cm, self.pose.yaw_deg)

        self.pose.x_cm += dx_cm
        self.pose.y_cm += dy_cm
        return self.pose

    def update_height(self, bottom_range_cm: float | None) -> Pose:
        if bottom_range_cm is not None and 0.0 <= bottom_range_cm <= 220.0:
            self.pose.z_cm = bottom_range_cm
        return self.pose

    def correct_from_front_wall(
        self,
        front_range_cm: float | None,
        correction: WallCorrection | None,
    ) -> Pose:
        if correction is None or front_range_cm is None:
            return self.pose
        if not correction.min_range_cm <= front_range_cm <= correction.max_range_cm:
            return self.pose
        if abs(wrap_angle_deg(self.pose.yaw_deg - correction.yaw_deg)) > correction.max_yaw_error_deg:
            return self.pose

        blend = clamp(correction.blend, 0.0, 1.0)

        if correction.axis == "x":
            direction = math.copysign(1.0, math.cos(math.radians(correction.yaw_deg)))
            measured_position = correction.coordinate_cm - direction * front_range_cm
            self.pose.x_cm = (1.0 - blend) * self.pose.x_cm + blend * measured_position
        elif correction.axis == "y":
            direction = math.copysign(1.0, math.sin(math.radians(correction.yaw_deg)))
            measured_position = correction.coordinate_cm - direction * front_range_cm
            self.pose.y_cm = (1.0 - blend) * self.pose.y_cm + blend * measured_position
        else:
            raise ValueError("WallCorrection.axis must be 'x' or 'y'")

        return self.pose

    def correct_from_color_marker(
        self,
        color_name: str | None,
        markers: Iterable[ColorMarker],
    ) -> Pose:
        if not color_name:
            return self.pose

        normalized_color = color_name.strip().lower()
        for marker in markers:
            if normalized_color == marker.color_name.strip().lower() and self.pose.z_cm <= marker.max_height_cm:
                self.pose.x_cm = marker.x_cm
                self.pose.y_cm = marker.y_cm
                break
        return self.pose


class DroneIO:
    """Small wrapper around CoDrone EDU APIs used by this program."""

    def __init__(self) -> None:
        from codrone_edu.drone import Drone

        self.drone = Drone()

    def pair(self) -> None:
        self.drone.pair()

    def takeoff(self) -> None:
        self.drone.takeoff()

    def land(self) -> None:
        self.drone.land()

    def close(self) -> None:
        self.drone.close()

    def reset_yaw(self) -> None:
        self.drone.reset_gyro()

    def yaw_deg(self) -> float:
        return float(self.drone.get_angle_z())

    def bottom_range_cm(self) -> float | None:
        return self._read_range("get_bottom_range")

    def front_range_cm(self) -> float | None:
        return self._read_range("get_front_range")

    def front_color(self) -> str | None:
        if not hasattr(self.drone, "get_front_color"):
            return None
        try:
            value = self.drone.get_front_color()
        except Exception:
            return None
        return str(value) if value is not None else None

    def command(self, pitch: int, roll: int, yaw: int, duration_sec: float) -> None:
        self.drone.set_pitch(int(pitch))
        self.drone.set_roll(int(roll))
        self.drone.set_yaw(int(yaw))
        self.drone.move(duration_sec)

    def stop_xy(self) -> None:
        self.command(pitch=0, roll=0, yaw=0, duration_sec=0.05)

    def _read_range(self, method_name: str) -> float | None:
        method = getattr(self.drone, method_name)
        try:
            value = method("cm")
        except TypeError:
            value = method()
        except Exception:
            return None

        try:
            range_cm = float(value)
        except (TypeError, ValueError):
            return None

        if range_cm >= 900.0:
            return None
        return range_cm


class CsvLogger:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.file = None
        self.writer = None

    def __enter__(self) -> "CsvLogger":
        if not self.enabled:
            return self

        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"command_dead_reckoning_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "elapsed_sec",
                "step",
                "pitch",
                "roll",
                "yaw_deg",
                "front_range_cm",
                "bottom_range_cm",
                "color",
                "target_x_cm",
                "target_y_cm",
                "error_cm",
                "mean_error_cm",
                "std_error_cm",
                "upper_confidence_cm",
                "x_cm",
                "y_cm",
                "z_cm",
            ],
        )
        self.writer.writeheader()
        print(f"log: {path}")
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.file is not None:
            self.file.close()

    def write(
        self,
        elapsed_sec: float,
        step_name: str,
        pitch: int,
        roll: int,
        yaw_deg: float,
        front_range_cm: float | None,
        bottom_range_cm: float | None,
        color: str | None,
        pose: Pose,
        target: Target | None = None,
        error_cm: float | None = None,
        stats: StopStats | None = None,
    ) -> None:
        if self.writer is None:
            return
        self.writer.writerow(
            {
                "elapsed_sec": round(elapsed_sec, 3),
                "step": step_name,
                "pitch": pitch,
                "roll": roll,
                "yaw_deg": round(yaw_deg, 2),
                "front_range_cm": _round_optional(front_range_cm),
                "bottom_range_cm": _round_optional(bottom_range_cm),
                "color": color or "",
                "target_x_cm": _round_optional(None if target is None else target.x_cm),
                "target_y_cm": _round_optional(None if target is None else target.y_cm),
                "error_cm": _round_optional(error_cm),
                "mean_error_cm": _round_optional(None if stats is None else stats.mean_error_cm),
                "std_error_cm": _round_optional(None if stats is None else stats.std_error_cm),
                "upper_confidence_cm": _round_optional(
                    None if stats is None else stats.upper_confidence_cm
                ),
                "x_cm": round(pose.x_cm, 2),
                "y_cm": round(pose.y_cm, 2),
                "z_cm": round(pose.z_cm, 2),
            }
        )


def _round_optional(value: float | None) -> float | str:
    return "" if value is None else round(value, 2)


def hold_heading_yaw_command(current_yaw_deg: float, target_yaw_deg: float, gain: float = 0.9) -> int:
    error = wrap_angle_deg(target_yaw_deg - current_yaw_deg)
    return int(clamp(error * gain, -15, 15))


def distance_to_target(pose: Pose, target: Target) -> float:
    return math.hypot(target.x_cm - pose.x_cm, target.y_cm - pose.y_cm)


def _command_for_axis(
    error_cm: float,
    speed_gain: float,
    total_error_cm: float,
    config: TargetControlConfig,
) -> int:
    axis_deadzone_cm = config.stop_radius_cm / math.sqrt(2.0)
    if total_error_cm <= config.stop_radius_cm or abs(error_cm) <= axis_deadzone_cm:
        return 0

    command_sign = 1.0 if speed_gain >= 0.0 else -1.0
    raw_power = error_cm * config.command_per_cm * command_sign

    if abs(error_cm) < config.slow_radius_cm:
        raw_power *= abs(error_cm) / config.slow_radius_cm

    power = int(round(clamp(raw_power, -config.max_power, config.max_power)))
    if power == 0:
        return 0
    if abs(power) < config.min_power:
        power = int(math.copysign(config.min_power, power))
    return power


def command_toward_target(
    pose: Pose,
    target: Target,
    motion_model: MotionModel,
    config: TargetControlConfig,
) -> tuple[int, int, float]:
    error_x = target.x_cm - pose.x_cm
    error_y = target.y_cm - pose.y_cm
    total_error = math.hypot(error_x, error_y)
    forward_error, left_error = rotate_world_to_body(error_x, error_y, pose.yaw_deg)

    pitch = _command_for_axis(
        error_cm=forward_error,
        speed_gain=motion_model.forward_cm_s_per_pitch,
        total_error_cm=total_error,
        config=config,
    )
    roll = _command_for_axis(
        error_cm=left_error,
        speed_gain=motion_model.left_cm_s_per_roll,
        total_error_cm=total_error,
        config=config,
    )
    return pitch, roll, total_error


def fly_to_target(
    drone: DroneIO,
    estimator: DeadReckoningEstimator,
    target: Target,
    logger: CsvLogger,
    wall_correction: WallCorrection | None,
    color_markers: list[ColorMarker],
    update_period_sec: float,
    control_config: TargetControlConfig,
    start_time: float,
) -> Pose:
    stop_detector = StatisticalStopDetector(
        window_sec=control_config.stop_window_sec,
        sample_period_sec=update_period_sec,
        confidence_z=control_config.confidence_z,
    )
    end_time = time.perf_counter() + control_config.timeout_sec
    last_update = time.perf_counter()
    target_yaw_deg = estimator.pose.yaw_deg
    last_print = 0.0
    active_pitch = 0
    active_roll = 0

    while time.perf_counter() < end_time:
        now = time.perf_counter()
        dt_sec = now - last_update
        last_update = now

        yaw_deg = drone.yaw_deg()
        bottom_range_cm = drone.bottom_range_cm()
        front_range_cm = drone.front_range_cm()
        color = drone.front_color()

        estimator.update_height(bottom_range_cm)
        estimator.update_from_command(active_pitch, active_roll, yaw_deg, dt_sec)
        estimator.correct_from_front_wall(front_range_cm, wall_correction)
        estimator.correct_from_color_marker(color, color_markers)

        error_cm = distance_to_target(estimator.pose, target)
        stats = stop_detector.add(error_cm)
        yaw_command = hold_heading_yaw_command(yaw_deg, target_yaw_deg)
        next_pitch, next_roll, _ = command_toward_target(
            pose=estimator.pose,
            target=target,
            motion_model=estimator.motion_model,
            config=control_config,
        )

        logger.write(
            elapsed_sec=time.perf_counter() - start_time,
            step_name="target",
            pitch=next_pitch,
            roll=next_roll,
            yaw_deg=yaw_deg,
            front_range_cm=front_range_cm,
            bottom_range_cm=bottom_range_cm,
            color=color,
            pose=estimator.pose,
            target=target,
            error_cm=error_cm,
            stats=stats,
        )

        elapsed_sec = time.perf_counter() - start_time
        if elapsed_sec - last_print >= 0.3:
            print_target_status(target, estimator.pose, error_cm, stats)
            last_print = elapsed_sec

        if stop_detector.has_arrived(stats, control_config):
            drone.stop_xy()
            print_target_status(target, estimator.pose, error_cm, stats)
            print("arrived: statistical stop condition satisfied")
            return estimator.pose

        drone.command(next_pitch, next_roll, yaw_command, update_period_sec)
        active_pitch = next_pitch
        active_roll = next_roll

    drone.stop_xy()
    print_target_status(target, estimator.pose, distance_to_target(estimator.pose, target), None)
    print("timeout: stopped before statistical condition was satisfied")
    return estimator.pose


def print_target_status(
    target: Target,
    pose: Pose,
    error_cm: float,
    stats: StopStats | None,
) -> None:
    stats_text = ""
    if stats is not None:
        stats_text = (
            f", mean={stats.mean_error_cm:4.1f}, "
            f"std={stats.std_error_cm:4.1f}, "
            f"ucb95={stats.upper_confidence_cm:4.1f}"
        )
    print(
        f"target=({target.x_cm:.1f},{target.y_cm:.1f}) "
        f"pose=({pose.x_cm:5.1f},{pose.y_cm:5.1f}) "
        f"error={error_cm:4.1f} cm{stats_text}"
    )


def run_step(
    drone: DroneIO,
    estimator: DeadReckoningEstimator,
    step: FlightStep,
    logger: CsvLogger,
    wall_correction: WallCorrection | None,
    color_markers: list[ColorMarker],
    update_period_sec: float,
    start_time: float,
) -> Pose:
    end_time = time.perf_counter() + step.duration_sec
    last_update = time.perf_counter()
    target_yaw_deg = estimator.pose.yaw_deg

    while time.perf_counter() < end_time:
        now = time.perf_counter()
        dt_sec = now - last_update
        last_update = now

        yaw_deg = drone.yaw_deg()
        bottom_range_cm = drone.bottom_range_cm()
        front_range_cm = drone.front_range_cm()
        color = drone.front_color()

        estimator.update_height(bottom_range_cm)
        estimator.update_from_command(step.pitch, step.roll, yaw_deg, dt_sec)
        estimator.correct_from_front_wall(front_range_cm, wall_correction)
        estimator.correct_from_color_marker(color, color_markers)

        yaw_command = hold_heading_yaw_command(yaw_deg, target_yaw_deg)
        drone.command(step.pitch, step.roll, yaw_command, update_period_sec)

        logger.write(
            elapsed_sec=time.perf_counter() - start_time,
            step_name=step.name,
            pitch=step.pitch,
            roll=step.roll,
            yaw_deg=yaw_deg,
            front_range_cm=front_range_cm,
            bottom_range_cm=bottom_range_cm,
            color=color,
            pose=estimator.pose,
        )

    drone.stop_xy()
    print_pose(step.name, estimator.pose)
    return estimator.pose


def print_pose(label: str, pose: Pose) -> None:
    print(
        f"{label}: x={pose.x_cm:6.1f} cm, y={pose.y_cm:6.1f} cm, "
        f"z={pose.z_cm:5.1f} cm, yaw={pose.yaw_deg:6.1f} deg"
    )


def default_mission() -> list[FlightStep]:
    return [
        FlightStep(name="forward", pitch=22, duration_sec=1.2),
        FlightStep(name="hover", duration_sec=0.5),
        FlightStep(name="left", roll=-22, duration_sec=1.2),
        FlightStep(name="hover", duration_sec=0.5),
        FlightStep(name="back", pitch=-22, duration_sec=1.2),
        FlightStep(name="hover", duration_sec=0.5),
        FlightStep(name="right", roll=22, duration_sec=1.2),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate CoDrone EDU XY position without optical flow position APIs."
    )
    parser.add_argument("--dry-run", action="store_true", help="Run estimator without connecting to a drone.")
    parser.add_argument("--demo-square", action="store_true", help="Run the old square demo instead of target mode.")
    parser.add_argument("--log", action="store_true", help="Write CSV logs to flight_logs/.")
    parser.add_argument("--period", type=float, default=0.05, help="Control/update period in seconds.")
    parser.add_argument("--target-x", type=float, default=30.0, help="Target X position in cm. +X is forward from takeoff.")
    parser.add_argument("--target-y", type=float, default=0.0, help="Target Y position in cm. +Y is left from takeoff.")
    parser.add_argument("--timeout", type=float, default=12.0, help="Maximum flight time for target mode in seconds.")
    parser.add_argument("--stop-radius", type=float, default=6.0, help="Arrival radius in cm.")
    parser.add_argument("--stop-window", type=float, default=1.0, help="Rolling statistics window in seconds.")
    parser.add_argument("--stop-std", type=float, default=2.5, help="Maximum allowed recent error standard deviation in cm.")
    parser.add_argument("--forward-gain", type=float, default=1.30, help="cm/s per pitch power.")
    parser.add_argument("--left-gain", type=float, default=-1.15, help="cm/s per roll power.")
    parser.add_argument(
        "--front-wall-x",
        type=float,
        default=None,
        help="Known +X wall coordinate in cm. Enables front range correction while facing +X.",
    )
    return parser.parse_args()


def simulate_mission(mission: list[FlightStep], estimator: DeadReckoningEstimator) -> None:
    for step in mission:
        remaining = step.duration_sec
        while remaining > 0.0:
            dt_sec = min(0.05, remaining)
            estimator.update_from_command(step.pitch, step.roll, yaw_deg=0.0, dt_sec=dt_sec)
            remaining -= dt_sec
        print_pose(step.name, estimator.pose)


def simulate_target(
    target: Target,
    estimator: DeadReckoningEstimator,
    control_config: TargetControlConfig,
    update_period_sec: float,
) -> None:
    stop_detector = StatisticalStopDetector(
        window_sec=control_config.stop_window_sec,
        sample_period_sec=update_period_sec,
        confidence_z=control_config.confidence_z,
    )
    active_pitch = 0
    active_roll = 0
    elapsed_sec = 0.0

    while elapsed_sec < control_config.timeout_sec:
        estimator.update_from_command(active_pitch, active_roll, yaw_deg=0.0, dt_sec=update_period_sec)
        error_cm = distance_to_target(estimator.pose, target)
        stats = stop_detector.add(error_cm)
        print_target_status(target, estimator.pose, error_cm, stats)

        if stop_detector.has_arrived(stats, control_config):
            print("arrived: statistical stop condition satisfied")
            return

        active_pitch, active_roll, _ = command_toward_target(
            pose=estimator.pose,
            target=target,
            motion_model=estimator.motion_model,
            config=control_config,
        )
        elapsed_sec += update_period_sec

    print("timeout: stopped before statistical condition was satisfied")


def main() -> None:
    args = parse_args()
    motion_model = MotionModel(
        forward_cm_s_per_pitch=args.forward_gain,
        left_cm_s_per_roll=args.left_gain,
    )
    estimator = DeadReckoningEstimator(motion_model)
    mission = default_mission()
    target = Target(x_cm=args.target_x, y_cm=args.target_y)
    control_config = TargetControlConfig(
        timeout_sec=args.timeout,
        stop_radius_cm=args.stop_radius,
        stop_window_sec=args.stop_window,
        stop_std_cm=args.stop_std,
    )

    if args.dry_run:
        if args.demo_square:
            simulate_mission(mission, estimator)
        else:
            simulate_target(target, estimator, control_config, args.period)
        return

    wall_correction = (
        WallCorrection(axis="x", coordinate_cm=args.front_wall_x, yaw_deg=0.0)
        if args.front_wall_x is not None
        else None
    )
    color_markers: list[ColorMarker] = [
        # Example: ColorMarker(color_name="red", x_cm=0.0, y_cm=0.0),
    ]

    drone = DroneIO()
    start_time = time.perf_counter()

    try:
        drone.pair()
        drone.takeoff()
        time.sleep(1.5)
        drone.reset_yaw()

        with CsvLogger(args.log) as logger:
            if args.demo_square:
                for step in mission:
                    run_step(
                        drone=drone,
                        estimator=estimator,
                        step=step,
                        logger=logger,
                        wall_correction=wall_correction,
                        color_markers=color_markers,
                        update_period_sec=args.period,
                        start_time=start_time,
                    )
            else:
                fly_to_target(
                    drone=drone,
                    estimator=estimator,
                    target=target,
                    logger=logger,
                    wall_correction=wall_correction,
                    color_markers=color_markers,
                    update_period_sec=args.period,
                    control_config=control_config,
                    start_time=start_time,
                )
    finally:
        safe_shutdown(drone)


def safe_shutdown(drone: DroneIO) -> None:
    for action in (drone.stop_xy, drone.land, drone.close):
        try:
            action()
        except Exception:
            pass


if __name__ == "__main__":
    main()
