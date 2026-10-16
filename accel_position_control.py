from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass, fields
from enum import Enum
from pathlib import Path
from statistics import fmean
from typing import Protocol


LOG_DIR = Path(__file__).resolve().parent / "accel_position_logs"
RAW_ACCEL_TO_MM_S2 = 100.0
STATE_ERROR_FLAGS = {
    1: "NotRegistered",
    2: "FlashReadLock_UnLocked",
    4: "BootloaderWriteLock_UnLocked",
    8: "LowBattery",
    16: "TakeoffFailure_CheckPropellerAndMotor",
    32: "CheckPropellerVibration",
    64: "Attitude_NotStable",
    256: "CanNotFlip_LowBattery",
    512: "CanNotFlip_TooHeavy",
}
SENSOR_ERROR_FLAGS = {
    1: "Motion_NoAnswer",
    2: "Motion_WrongValue",
    4: "Motion_NotCalibrated",
    8: "Motion_Calibrating",
    16: "Pressure_NoAnswer",
    32: "Pressure_WrongValue",
    256: "RangeGround_NoAnswer",
    512: "RangeGround_WrongValue",
    4096: "Flow_NoAnswer",
    8192: "Flow_WrongValue",
    16384: "Flow_CannotRecognizeGroundImage",
}


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _deadband(value: float, threshold: float) -> float:
    if abs(value) <= threshold:
        return 0.0
    return math.copysign(abs(value) - threshold, value)


def _decay_factor(rate_per_sec: float, dt_sec: float) -> float:
    if rate_per_sec <= 0.0 or dt_sec <= 0.0:
        return 1.0
    return math.exp(-rate_per_sec * dt_sec)


def _json_safe(value):
    if isinstance(value, Enum):
        return {"name": value.name, "value": value.value}
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [_json_safe(item) for item in value]
    return value


def _json_dumps_safe(value, *, indent: int | None = None) -> str:
    return json.dumps(_json_safe(value), ensure_ascii=False, indent=indent)


def _decode_bit_flags(value: object, names_by_bit: dict[int, str]) -> list[str]:
    raw_value = _enum_value(value)
    if not isinstance(raw_value, int):
        return []
    return [name for bit, name in names_by_bit.items() if raw_value & bit]


@dataclass
class AccelerationReading:
    wall_time: float
    raw_x: float
    raw_y: float
    raw_z: float


class AccelerationReader(Protocol):
    def read(self) -> AccelerationReading:
        raise NotImplementedError


class DroneAccelerationReader:
    """Reads only the named accelerometer APIs from CoDrone EDU."""

    def __init__(self, drone: object) -> None:
        self.drone = drone

    def read(self) -> AccelerationReading:
        return AccelerationReading(
            wall_time=time.perf_counter(),
            raw_x=float(self.drone.get_accel_x()),
            raw_y=float(self.drone.get_accel_y()),
            raw_z=float(self.drone.get_accel_z()),
        )


@dataclass
class AccelerationBias:
    raw_x: float = 0.0
    raw_y: float = 0.0
    raw_z: float = 0.0


@dataclass
class EstimatorConfig:
    sample_period_sec: float = 0.05
    max_dt_sec: float = 0.20
    accel_low_pass_alpha: float = 0.35
    accel_deadband_mm_s2: float = 140.0
    velocity_leak_per_sec: float = 0.12
    accel_x_sign: float = 1.0
    accel_y_sign: float = 1.0


@dataclass
class PositionEstimate:
    x_mm: float = 0.0
    y_mm: float = 0.0
    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    ax_mm_s2: float = 0.0
    ay_mm_s2: float = 0.0
    filtered_ax_mm_s2: float = 0.0
    filtered_ay_mm_s2: float = 0.0


@dataclass
class ControlConfig:
    target_x_cm: float
    target_y_cm: float
    takeoff: bool = True
    land: bool = True
    calibration_sec: float = 1.8
    hover_after_takeoff_sec: float = 2.0
    min_takeoff_height_cm: float = 8.0
    require_takeoff_success: bool = True
    max_pitch: int = 35
    max_roll: int = 35
    min_move_power: int = 24
    min_move_error_cm: float = 15.0
    kp_command_per_mm: float = 0.095
    kv_command_per_mm_s: float = 0.018
    pitch_sign_for_positive_x: float = 1.0
    roll_sign_for_positive_y: float = -1.0
    target_tolerance_cm: float = 8.0
    velocity_tolerance_cm_s: float = 22.0
    settle_required_sec: float = 0.55
    timeout_sec: float | None = None
    print_every_sec: float = 0.25
    log_dir: Path = LOG_DIR

    @property
    def target_x_mm(self) -> float:
        return self.target_x_cm * 10.0

    @property
    def target_y_mm(self) -> float:
        return self.target_y_cm * 10.0

    @property
    def target_tolerance_mm(self) -> float:
        return self.target_tolerance_cm * 10.0

    @property
    def velocity_tolerance_mm_s(self) -> float:
        return self.velocity_tolerance_cm_s * 10.0

    @property
    def min_move_error_mm(self) -> float:
        return self.min_move_error_cm * 10.0

    def resolved_timeout_sec(self) -> float:
        if self.timeout_sec is not None:
            return self.timeout_sec
        distance_cm = math.hypot(self.target_x_cm, self.target_y_cm)
        return max(8.0, 7.0 + distance_cm * 0.18)


@dataclass
class ControlSample:
    step: int
    elapsed_sec: float
    dt_sec: float
    raw_accel_x: float
    raw_accel_y: float
    raw_accel_z: float
    bias_raw_x: float
    bias_raw_y: float
    bias_raw_z: float
    ax_mm_s2: float
    ay_mm_s2: float
    filtered_ax_mm_s2: float
    filtered_ay_mm_s2: float
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float
    target_x_mm: float
    target_y_mm: float
    error_x_mm: float
    error_y_mm: float
    pitch_cmd: int
    roll_cmd: int
    settled: bool


@dataclass
class FlightSummary:
    started_at: str
    log_csv: str
    summary_json: str
    termination_reason: str
    target_x_cm: float
    target_y_cm: float
    estimated_x_cm: float
    estimated_y_cm: float
    final_error_x_cm: float
    final_error_y_cm: float
    final_speed_x_cm_s: float
    final_speed_y_cm_s: float
    sample_count: int
    duration_sec: float
    bias_raw_x: float
    bias_raw_y: float
    bias_raw_z: float


@dataclass
class TakeoffCheck:
    battery_percent: object
    state_data: object
    error_data: object
    height_cm: object
    flight_state: object
    movement_state: object
    sensor_error_flags: object
    state_error_flags: object
    sensor_error_names: list[str]
    state_error_names: list[str]
    warning: str
    success: bool


class AccelerationPositionEstimator:
    """Double-integrates bias-corrected accelerometer values into x/y position."""

    def __init__(self, config: EstimatorConfig, bias: AccelerationBias) -> None:
        self.config = config
        self.bias = bias
        self.estimate = PositionEstimate()

    def update(self, reading: AccelerationReading, dt_sec: float) -> PositionEstimate:
        dt = _clamp(dt_sec, 0.0, self.config.max_dt_sec)

        raw_ax_mm_s2 = (reading.raw_x - self.bias.raw_x) * RAW_ACCEL_TO_MM_S2
        raw_ay_mm_s2 = (reading.raw_y - self.bias.raw_y) * RAW_ACCEL_TO_MM_S2
        raw_ax_mm_s2 *= 1.0 if self.config.accel_x_sign >= 0.0 else -1.0
        raw_ay_mm_s2 *= 1.0 if self.config.accel_y_sign >= 0.0 else -1.0

        alpha = _clamp(self.config.accel_low_pass_alpha, 0.0, 1.0)
        filtered_ax = (
            alpha * raw_ax_mm_s2
            + (1.0 - alpha) * self.estimate.filtered_ax_mm_s2
        )
        filtered_ay = (
            alpha * raw_ay_mm_s2
            + (1.0 - alpha) * self.estimate.filtered_ay_mm_s2
        )

        ax = _deadband(filtered_ax, self.config.accel_deadband_mm_s2)
        ay = _deadband(filtered_ay, self.config.accel_deadband_mm_s2)

        old_vx = self.estimate.vx_mm_s
        old_vy = self.estimate.vy_mm_s
        leak = _decay_factor(self.config.velocity_leak_per_sec, dt)
        new_vx = (old_vx + ax * dt) * leak
        new_vy = (old_vy + ay * dt) * leak

        self.estimate = PositionEstimate(
            x_mm=self.estimate.x_mm + ((old_vx + new_vx) * 0.5 * dt),
            y_mm=self.estimate.y_mm + ((old_vy + new_vy) * 0.5 * dt),
            vx_mm_s=new_vx,
            vy_mm_s=new_vy,
            ax_mm_s2=ax,
            ay_mm_s2=ay,
            filtered_ax_mm_s2=filtered_ax,
            filtered_ay_mm_s2=filtered_ay,
        )
        return self.estimate


class PositionController:
    """Maps position error to pitch and roll commands."""

    def __init__(self, config: ControlConfig) -> None:
        self.config = config

    def compute(self, estimate: PositionEstimate) -> tuple[int, int, float, float]:
        error_x_mm = self.config.target_x_mm - estimate.x_mm
        error_y_mm = self.config.target_y_mm - estimate.y_mm

        forward_effort = (
            self.config.kp_command_per_mm * error_x_mm
            - self.config.kv_command_per_mm_s * estimate.vx_mm_s
        )
        left_effort = (
            self.config.kp_command_per_mm * error_y_mm
            - self.config.kv_command_per_mm_s * estimate.vy_mm_s
        )

        pitch_effort = forward_effort * (
            1.0 if self.config.pitch_sign_for_positive_x >= 0.0 else -1.0
        )
        if abs(error_x_mm) >= self.config.min_move_error_mm and abs(pitch_effort) > 0.0:
            pitch_effort = math.copysign(
                max(abs(pitch_effort), self.config.min_move_power),
                pitch_effort,
            )
        pitch_cmd = int(round(_clamp(pitch_effort, -self.config.max_pitch, self.config.max_pitch)))

        # CoDrone EDU roll sign used by the existing keyboard code:
        # negative roll moves left, positive roll moves right.
        roll_effort = left_effort * (
            1.0 if self.config.roll_sign_for_positive_y >= 0.0 else -1.0
        )
        if abs(error_y_mm) >= self.config.min_move_error_mm and abs(roll_effort) > 0.0:
            roll_effort = math.copysign(
                max(abs(roll_effort), self.config.min_move_power),
                roll_effort,
            )
        roll_cmd = int(round(_clamp(roll_effort, -self.config.max_roll, self.config.max_roll)))
        return pitch_cmd, roll_cmd, error_x_mm, error_y_mm


class FlightLogger:
    def __init__(self, log_dir: Path) -> None:
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.log_dir / f"accel_position_{stamp}.csv"
        self.summary_path = self.log_dir / f"accel_position_{stamp}_summary.json"
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=[field.name for field in fields(ControlSample)],
        )
        self._writer.writeheader()

    def write_sample(self, sample: ControlSample) -> None:
        self._writer.writerow(asdict(sample))

    def write_summary(self, summary: FlightSummary) -> None:
        self._csv_file.flush()
        self.summary_path.write_text(
            _json_dumps_safe(asdict(summary), indent=2),
            encoding="utf-8",
        )

    def close(self) -> None:
        self._csv_file.close()


def _import_drone_class():
    try:
        from codrone_edu.drone import Drone

        return Drone
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "codrone_edu is not available. Run this with the project .venv "
            "or install the CoDrone EDU Python library."
        ) from exc


def calibrate_accelerometer(
    reader: AccelerationReader,
    *,
    duration_sec: float,
    sample_period_sec: float,
    sample_count: int | None = None,
) -> AccelerationBias:
    samples: list[AccelerationReading] = []
    if sample_count is not None:
        for _ in range(max(0, sample_count)):
            samples.append(reader.read())
    else:
        deadline = time.perf_counter() + max(0.0, duration_sec)
        while time.perf_counter() < deadline:
            samples.append(reader.read())
            time.sleep(sample_period_sec)

    if not samples:
        raise RuntimeError("accelerometer calibration collected no samples")

    return AccelerationBias(
        raw_x=fmean(sample.raw_x for sample in samples),
        raw_y=fmean(sample.raw_y for sample in samples),
        raw_z=fmean(sample.raw_z for sample in samples),
    )


def _zero_horizontal_commands(drone: object) -> None:
    drone.set_pitch(0)
    drone.set_roll(0)
    drone.set_yaw(0)


def _read_drone_value(label: str, func):
    try:
        return func()
    except Exception as exc:
        return {"read_error": label, "type": type(exc).__name__, "message": str(exc)}


def _list_get(values: object, index: int) -> object:
    if isinstance(values, list) and len(values) > index:
        return values[index]
    return None


def _enum_value(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    return value


def _flag_value(value: object) -> int | None:
    raw = _enum_value(value)
    if isinstance(raw, int):
        return raw
    return None


def check_takeoff_success(drone: object, min_height_cm: float) -> TakeoffCheck:
    battery = _read_drone_value("battery", drone.get_battery)
    state_data = _read_drone_value("state_data", drone.get_state_data)
    error_data = _read_drone_value(
        "error_data",
        lambda: drone.get_error_data(print_error=False),
    )
    height_cm = _read_drone_value("height_cm", lambda: drone.get_height("cm"))
    flight_state = _list_get(state_data, 2)
    movement_state = _list_get(state_data, 7)
    sensor_error_flags = _list_get(error_data, 1)
    state_error_flags = _list_get(error_data, 2)

    flight_state_value = _enum_value(flight_state)
    movement_state_value = _enum_value(movement_state)
    state_error_value = _flag_value(state_error_flags)
    sensor_error_value = _flag_value(sensor_error_flags)

    flight_controller_reports_airborne = flight_state_value in {18, 19}
    movement_reports_airborne = movement_state_value in {2, 3}
    height_reports_airborne = (
        isinstance(height_cm, (int, float)) and float(height_cm) >= min_height_cm
    )
    no_state_error = state_error_value == 0
    no_sensor_error = sensor_error_value == 0

    success = (
        no_state_error
        and no_sensor_error
        and (height_reports_airborne or (flight_controller_reports_airborne and movement_reports_airborne))
    )
    warning = ""
    if success and not height_reports_airborne:
        warning = (
            "height sensor reported low altitude, but flight state reports Flight/Hovering; "
            "continuing without using height as a hard failure."
        )
    return TakeoffCheck(
        battery_percent=battery,
        state_data=state_data,
        error_data=error_data,
        height_cm=height_cm,
        flight_state=flight_state,
        movement_state=movement_state,
        sensor_error_flags=sensor_error_flags,
        state_error_flags=state_error_flags,
        sensor_error_names=_decode_bit_flags(sensor_error_flags, SENSOR_ERROR_FLAGS),
        state_error_names=_decode_bit_flags(state_error_flags, STATE_ERROR_FLAGS),
        warning=warning,
        success=success,
    )


def run_accel_only_xy_move(
    drone: object,
    config: ControlConfig,
    estimator_config: EstimatorConfig | None = None,
) -> FlightSummary:
    estimator_config = estimator_config or EstimatorConfig()
    logger = FlightLogger(config.log_dir)
    reader = DroneAccelerationReader(drone)
    start_wall = time.strftime("%Y-%m-%d %H:%M:%S")
    start_perf = time.perf_counter()
    last_read_time = start_perf
    last_print_time = start_perf
    termination_reason = "unknown"
    samples_written = 0
    estimate = PositionEstimate()
    bias = AccelerationBias()

    try:
        if config.takeoff:
            print("takeoff")
            drone.takeoff()
            time.sleep(max(0.0, config.hover_after_takeoff_sec))
            takeoff_check = check_takeoff_success(drone, config.min_takeoff_height_cm)
            print(
                "takeoff_check: "
                + _json_dumps_safe(asdict(takeoff_check))
            )
            if config.require_takeoff_success and not takeoff_check.success:
                termination_reason = "takeoff_failed"
                raise RuntimeError(
                    "takeoff_failed: CoDrone did not report a usable takeoff state. "
                    f"check={_json_dumps_safe(asdict(takeoff_check))}"
                )

        _zero_horizontal_commands(drone)
        drone.set_throttle(0)
        drone.move(0.05)

        print("calibrating accelerometer")
        bias = calibrate_accelerometer(
            reader,
            duration_sec=config.calibration_sec,
            sample_period_sec=estimator_config.sample_period_sec,
        )
        last_read_time = time.perf_counter()
        estimator = AccelerationPositionEstimator(estimator_config, bias)
        controller = PositionController(config)
        settle_start: float | None = None
        control_start_time = time.perf_counter()
        last_print_time = control_start_time
        timeout_sec = config.resolved_timeout_sec()
        step = 0

        while True:
            loop_now = time.perf_counter()
            elapsed = loop_now - control_start_time
            if elapsed >= timeout_sec:
                termination_reason = "timeout"
                break

            reading = reader.read()
            dt = reading.wall_time - last_read_time
            last_read_time = reading.wall_time
            estimate = estimator.update(reading, dt)
            pitch_cmd, roll_cmd, error_x_mm, error_y_mm = controller.compute(estimate)

            speed_ok = (
                abs(estimate.vx_mm_s) <= config.velocity_tolerance_mm_s
                and abs(estimate.vy_mm_s) <= config.velocity_tolerance_mm_s
            )
            position_ok = (
                abs(error_x_mm) <= config.target_tolerance_mm
                and abs(error_y_mm) <= config.target_tolerance_mm
            )
            settled = position_ok and speed_ok

            if settled:
                if settle_start is None:
                    settle_start = loop_now
                if loop_now - settle_start >= config.settle_required_sec:
                    termination_reason = "settled"
                    break
            else:
                settle_start = None

            drone.set_pitch(pitch_cmd)
            drone.set_roll(roll_cmd)
            drone.set_yaw(0)
            drone.set_throttle(0)
            drone.move(estimator_config.sample_period_sec)

            sample = ControlSample(
                step=step,
                elapsed_sec=elapsed,
                dt_sec=dt,
                raw_accel_x=reading.raw_x,
                raw_accel_y=reading.raw_y,
                raw_accel_z=reading.raw_z,
                bias_raw_x=bias.raw_x,
                bias_raw_y=bias.raw_y,
                bias_raw_z=bias.raw_z,
                ax_mm_s2=estimate.ax_mm_s2,
                ay_mm_s2=estimate.ay_mm_s2,
                filtered_ax_mm_s2=estimate.filtered_ax_mm_s2,
                filtered_ay_mm_s2=estimate.filtered_ay_mm_s2,
                x_mm=estimate.x_mm,
                y_mm=estimate.y_mm,
                vx_mm_s=estimate.vx_mm_s,
                vy_mm_s=estimate.vy_mm_s,
                target_x_mm=config.target_x_mm,
                target_y_mm=config.target_y_mm,
                error_x_mm=error_x_mm,
                error_y_mm=error_y_mm,
                pitch_cmd=pitch_cmd,
                roll_cmd=roll_cmd,
                settled=settled,
            )
            logger.write_sample(sample)
            samples_written += 1

            if loop_now - last_print_time >= config.print_every_sec:
                print(
                    "target=({:.1f},{:.1f})cm estimated=({:.1f},{:.1f})cm "
                    "error=({:.1f},{:.1f})cm cmd=(pitch:{}, roll:{})".format(
                        config.target_x_cm,
                        config.target_y_cm,
                        estimate.x_mm / 10.0,
                        estimate.y_mm / 10.0,
                        error_x_mm / 10.0,
                        error_y_mm / 10.0,
                        pitch_cmd,
                        roll_cmd,
                    )
                )
                last_print_time = loop_now

            step += 1

    except KeyboardInterrupt:
        termination_reason = "keyboard_interrupt"
    except Exception:
        termination_reason = "exception"
        raise
    finally:
        _zero_horizontal_commands(drone)
        drone.set_throttle(0)
        try:
            drone.move(0.05)
        finally:
            if config.land:
                drone.land()

    duration_sec = time.perf_counter() - start_perf
    summary = FlightSummary(
        started_at=start_wall,
        log_csv=str(logger.csv_path),
        summary_json=str(logger.summary_path),
        termination_reason=termination_reason,
        target_x_cm=config.target_x_cm,
        target_y_cm=config.target_y_cm,
        estimated_x_cm=estimate.x_mm / 10.0,
        estimated_y_cm=estimate.y_mm / 10.0,
        final_error_x_cm=(config.target_x_mm - estimate.x_mm) / 10.0,
        final_error_y_cm=(config.target_y_mm - estimate.y_mm) / 10.0,
        final_speed_x_cm_s=estimate.vx_mm_s / 10.0,
        final_speed_y_cm_s=estimate.vy_mm_s / 10.0,
        sample_count=samples_written,
        duration_sec=duration_sec,
        bias_raw_x=bias.raw_x,
        bias_raw_y=bias.raw_y,
        bias_raw_z=bias.raw_z,
    )
    logger.write_summary(summary)
    logger.close()
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move CoDrone EDU using x/y position estimated from accelerometer only."
    )
    parser.add_argument("--x-cm", type=float, required=True, help="Forward target in cm.")
    parser.add_argument("--y-cm", type=float, required=True, help="Left target in cm.")
    parser.add_argument("--max-power", type=int, default=35, help="Pitch/roll command limit.")
    parser.add_argument("--min-move-power", type=int, default=24, help="Minimum pitch/roll command while far from target.")
    parser.add_argument("--min-move-error-cm", type=float, default=15.0, help="Error threshold for minimum move power.")
    parser.add_argument("--timeout-sec", type=float, default=None, help="Override safety timeout.")
    parser.add_argument("--calibration-sec", type=float, default=1.8, help="Stationary bias calibration time.")
    parser.add_argument("--min-takeoff-height-cm", type=float, default=8.0, help="Required height after takeoff.")
    parser.add_argument("--allow-takeoff-failure", action="store_true", help="Continue even if height check fails.")
    parser.add_argument("--sample-period-sec", type=float, default=0.05, help="Control loop command duration.")
    parser.add_argument("--accel-x-sign", type=float, choices=[-1.0, 1.0], default=-1.0, help="Invert estimated x acceleration if needed.")
    parser.add_argument("--accel-y-sign", type=float, choices=[-1.0, 1.0], default=1.0, help="Invert estimated y acceleration if needed.")
    parser.add_argument("--pitch-sign", type=float, choices=[-1.0, 1.0], default=1.0, help="Invert pitch command for positive x if needed.")
    parser.add_argument("--roll-sign", type=float, choices=[-1.0, 1.0], default=-1.0, help="Invert roll command for positive y if needed.")
    parser.add_argument("--tolerance-cm", type=float, default=8.0, help="Estimated position tolerance.")
    parser.add_argument("--velocity-tolerance-cm-s", type=float, default=22.0, help="Estimated speed tolerance.")
    parser.add_argument("--no-takeoff", action="store_true", help="Skip takeoff for bench tests.")
    parser.add_argument("--no-land", action="store_true", help="Leave the drone flying after stop.")
    parser.add_argument("--log-dir", type=Path, default=LOG_DIR, help="Directory for CSV and JSON logs.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    Drone = _import_drone_class()
    drone = Drone()
    try:
        drone.pair()
        config = ControlConfig(
            target_x_cm=args.x_cm,
            target_y_cm=args.y_cm,
            takeoff=not args.no_takeoff,
            land=not args.no_land,
            calibration_sec=args.calibration_sec,
            min_takeoff_height_cm=args.min_takeoff_height_cm,
            require_takeoff_success=not args.allow_takeoff_failure,
            max_pitch=args.max_power,
            max_roll=args.max_power,
            min_move_power=args.min_move_power,
            min_move_error_cm=args.min_move_error_cm,
            pitch_sign_for_positive_x=args.pitch_sign,
            roll_sign_for_positive_y=args.roll_sign,
            timeout_sec=args.timeout_sec,
            target_tolerance_cm=args.tolerance_cm,
            velocity_tolerance_cm_s=args.velocity_tolerance_cm_s,
            log_dir=args.log_dir,
        )
        estimator_config = EstimatorConfig(
            sample_period_sec=args.sample_period_sec,
            accel_x_sign=args.accel_x_sign,
            accel_y_sign=args.accel_y_sign,
        )
        summary = run_accel_only_xy_move(drone, config, estimator_config)
        print(_json_dumps_safe(asdict(summary), indent=2))
        return 0
    finally:
        drone.close()


if __name__ == "__main__":
    raise SystemExit(main())
