from __future__ import annotations

import copy
import json
import logging
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import fmean

from codrone_edu.drone import Drone


LOG_DIR = Path(__file__).resolve().parent / "flight_logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
PID_PROFILE_PATH = LOG_DIR / "last_pid_profile.json"
DEFAULT_CONTROL_SIGNS = {"pitch": 1.0, "roll": -1.0, "throttle": 1.0, "yaw": 1.0}


def profile_bucket_for_distance(target_distance_mm: float) -> str:
    return "2000mm" if abs(target_distance_mm) >= 1000.0 else "500mm"


def pid_profile_path_for_bucket(bucket: str) -> Path:
    return LOG_DIR / f"last_pid_profile_{bucket}.json"


def _clamp(value: float, lower: float, upper: float | None = None) -> float:
    if upper is None:
        limit = float(lower)
        return max(-limit, min(limit, value))
    return max(lower, min(upper, value))


def _sign(value: float) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 0.0


def is_settled_state(
    *,
    error_x_mm: float,
    error_y_mm: float,
    error_z_mm: float,
    error_yaw_deg: float,
    x_velocity_mm_s: float,
    y_velocity_mm_s: float,
    z_velocity_mm_s: float,
) -> bool:
    return (
        abs(error_x_mm) <= 10.0
        and abs(error_y_mm) <= 25.0
        and abs(error_z_mm) <= 25.0
        and abs(error_yaw_deg) <= 8.0
        and abs(x_velocity_mm_s) <= 25.0
        and abs(y_velocity_mm_s) <= 35.0
        and abs(z_velocity_mm_s) <= 35.0
    )


def default_pid_profile() -> dict:
    return {
        "pitch": {"kp": 0.110, "ki": 0.00040, "kd": 0.030, "output_limit": 55.0, "integral_limit": 3000.0},
        "roll": {"kp": 0.090, "ki": 0.00030, "kd": 0.040, "output_limit": 40.0, "integral_limit": 2200.0},
        "throttle": {"kp": 0.135, "ki": 0.00450, "kd": 0.040, "output_limit": 32.0, "integral_limit": 1500.0},
        "yaw": {"kp": 0.500, "ki": 0.00000, "kd": 0.080, "output_limit": 16.0, "integral_limit": 120.0},
        "signs": dict(DEFAULT_CONTROL_SIGNS),
    }


def load_pid_profile(target_distance_mm: float) -> dict:
    profile = default_pid_profile()
    candidate_paths = [pid_profile_path_for_bucket(profile_bucket_for_distance(target_distance_mm)), PID_PROFILE_PATH]

    raw = None
    for profile_path in candidate_paths:
        if not profile_path.exists():
            continue
        try:
            raw = json.loads(profile_path.read_text(encoding="utf-8"))
            break
        except Exception:
            continue
    if raw is None:
        return profile

    for axis in profile:
        if axis not in raw or not isinstance(raw[axis], dict):
            continue
        for key in profile[axis]:
            if key in raw[axis]:
                try:
                    profile[axis][key] = float(raw[axis][key])
                except (TypeError, ValueError):
                    pass
    profile["signs"] = dict(DEFAULT_CONTROL_SIGNS)
    return profile


def save_pid_profile(profile: dict) -> None:
    PID_PROFILE_PATH.write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")


def save_pid_profile_for_distance(profile: dict, target_distance_mm: float) -> None:
    bucket = profile_bucket_for_distance(target_distance_mm)
    pid_profile_path_for_bucket(bucket).write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")


def adjust_pid_profile(profile: dict, summary: FlightSummary | None) -> dict:
    if summary is None:
        return profile

    next_profile = copy.deepcopy(profile)

    pitch = next_profile["pitch"]
    roll = next_profile["roll"]
    throttle = next_profile["throttle"]
    yaw = next_profile["yaw"]

    if summary.oscillation_count >= 3:
        pitch["kp"] *= 0.92
        pitch["kd"] *= 1.10
        pitch["ki"] *= 0.90
    elif abs(summary.final_error_x_mm) > 20.0:
        pitch["kp"] *= 1.08
        pitch["kd"] *= 0.94
        if abs(summary.mean_abs_error_x_mm) > 40.0:
            pitch["ki"] *= 1.08

    if abs(summary.final_error_y_mm) > 15.0:
        roll["kp"] *= 1.05
        roll["kd"] *= 0.98

    if abs(summary.final_error_y_mm) > 8.0:
        roll["kp"] *= 1.03
        roll["kd"] *= 1.02

    if abs(summary.final_error_z_mm) > 15.0:
        throttle["kp"] *= 1.05
        throttle["kd"] *= 0.98

    if abs(summary.final_error_z_mm) > 8.0:
        throttle["kp"] *= 1.03
        throttle["kd"] *= 1.02

    if abs(summary.final_error_yaw_deg) > 6.0:
        yaw["kp"] *= 1.04
        yaw["kd"] *= 1.02

    pitch["kp"] = _clamp(pitch["kp"], 0.05, 0.22)
    pitch["ki"] = _clamp(pitch["ki"], 0.0, 0.004)
    pitch["kd"] = _clamp(pitch["kd"], 0.01, 0.08)

    roll["kp"] = _clamp(roll["kp"], 0.04, 0.18)
    roll["ki"] = _clamp(roll["ki"], 0.0, 0.003)
    roll["kd"] = _clamp(roll["kd"], 0.02, 0.08)

    throttle["kp"] = _clamp(throttle["kp"], 0.08, 0.28)
    throttle["ki"] = _clamp(throttle["ki"], 0.0, 0.01)
    throttle["kd"] = _clamp(throttle["kd"], 0.02, 0.08)

    yaw["kp"] = _clamp(yaw["kp"], 0.2, 0.9)
    yaw["ki"] = _clamp(yaw["ki"], 0.0, 0.01)
    yaw["kd"] = _clamp(yaw["kd"], 0.02, 0.16)

    return next_profile


@dataclass
class SensorSnapshot:
    timestamp: float
    x_mm: float
    y_mm: float
    z_mm: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_roll: float
    gyro_pitch: float
    gyro_yaw: float


@dataclass
class ControlSample:
    step: int
    timestamp: float
    elapsed_sec: float
    dt_sec: float
    target_x_mm: float
    target_y_mm: float
    target_z_mm: float
    target_yaw_deg: float
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float
    error_x_mm: float
    error_y_mm: float
    error_z_mm: float
    error_yaw_deg: float
    pitch_cmd: int
    roll_cmd: int
    throttle_cmd: int
    yaw_cmd: int
    x_velocity_mm_s: float
    y_velocity_mm_s: float
    z_velocity_mm_s: float
    yaw_rate_deg_s: float


@dataclass
class FlightSummary:
    samples: int
    duration_sec: float
    target_distance_mm: float
    start_x_mm: float
    final_x_mm: float
    final_y_mm: float
    final_z_mm: float
    final_yaw_deg: float
    final_error_x_mm: float
    final_error_y_mm: float
    final_error_z_mm: float
    final_error_yaw_deg: float
    max_abs_error_x_mm: float
    mean_abs_error_x_mm: float
    oscillation_count: int
    settling_time_sec: float | None


@dataclass
class AxisSignMonitor:
    axis_name: str
    sign: float = 1.0
    confidence: float = 0.0
    samples: int = 0

    def observe(self, command: int, delta: float, min_command: int = 8, min_delta_mm: float = 6.0) -> None:
        if abs(command) < min_command or abs(delta) < min_delta_mm:
            return

        observed_sign = _sign(command * delta)
        if observed_sign == 0.0:
            return

        weight = min(abs(command) / 18.0, 1.5) * min(abs(delta) / 20.0, 1.5)
        self.confidence = _clamp((self.confidence * 0.82) + (observed_sign * weight), -8.0, 8.0)
        self.samples += 1
        if abs(self.confidence) >= 2.4:
            self.sign = _sign(self.confidence)

    def is_confident(self) -> bool:
        return self.samples >= 4 and abs(self.confidence) >= 2.4


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float,
        integral_limit: float,
        min_output: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.min_output = min_output
        self.integral = 0.0
        self.prev_error = 0.0
        self._has_prev_error = False
        self._filtered_derivative = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self._has_prev_error = False
        self._filtered_derivative = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, target: float, current: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        error = target - current
        self.integral = _clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)

        if self._has_prev_error:
            raw_derivative = (error - self.prev_error) / dt
            self._filtered_derivative = (0.35 * raw_derivative) + (0.65 * self._filtered_derivative)
        else:
            self._filtered_derivative = 0.0
            self._has_prev_error = True

        output = (
            (self.kp * error)
            + (self.ki * self.integral)
            + (self.kd * self._filtered_derivative)
        )

        self.prev_error = error
        output = _clamp(output, -self.output_limit, self.output_limit)

        if self.min_output > 0.0 and abs(error) > 1e-6 and abs(output) < self.min_output:
            output = _sign(error) * self.min_output

        return _clamp(output, -self.output_limit, self.output_limit)


class FlightLogWriter:
    def __init__(self, log_dir: Path) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = log_dir / f"pid_flight_{stamp}.jsonl"
        self._fh = self.log_path.open("w", encoding="utf-8")

    def write(self, event: str, payload: dict) -> None:
        record = {
            "event": event,
            "logged_at": time.time(),
            **payload,
        }
        self._fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.close()

    def __enter__(self) -> "FlightLogWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.close()
        return False


class FlightLogAnalyzer:
    @staticmethod
    def summarize(samples: list[ControlSample]) -> FlightSummary | None:
        if not samples:
            return None

        first = samples[0]
        last = samples[-1]
        abs_errors = [abs(sample.error_x_mm) for sample in samples]

        sign_changes = 0
        prev_sign = _sign(samples[0].error_x_mm)
        for sample in samples[1:]:
            now_sign = _sign(sample.error_x_mm)
            if prev_sign != 0.0 and now_sign != 0.0 and now_sign != prev_sign:
                sign_changes += 1
            if now_sign != 0.0:
                prev_sign = now_sign

        return FlightSummary(
            samples=len(samples),
            duration_sec=last.elapsed_sec,
            target_distance_mm=last.target_x_mm - first.x_mm,
            start_x_mm=first.x_mm,
            final_x_mm=last.x_mm,
            final_y_mm=last.y_mm,
            final_z_mm=last.z_mm,
            final_yaw_deg=last.yaw_deg,
            final_error_x_mm=last.error_x_mm,
            final_error_y_mm=last.error_y_mm,
            final_error_z_mm=last.error_z_mm,
            final_error_yaw_deg=last.error_yaw_deg,
            max_abs_error_x_mm=max(abs_errors),
            mean_abs_error_x_mm=fmean(abs_errors),
            oscillation_count=sign_changes,
            settling_time_sec=FlightLogAnalyzer._estimate_settling_time(samples),
        )

    @staticmethod
    def _estimate_settling_time(samples: list[ControlSample]) -> float | None:
        stable_since = None
        for sample in samples:
            stable = is_settled_state(
                error_x_mm=sample.error_x_mm,
                error_y_mm=sample.error_y_mm,
                error_z_mm=sample.error_z_mm,
                error_yaw_deg=sample.error_yaw_deg,
                x_velocity_mm_s=sample.x_velocity_mm_s,
                y_velocity_mm_s=sample.y_velocity_mm_s,
                z_velocity_mm_s=sample.z_velocity_mm_s,
            )
            if stable:
                if stable_since is None:
                    stable_since = sample.elapsed_sec
                elif sample.elapsed_sec - stable_since >= 0.4:
                    return stable_since
            else:
                stable_since = None
        return None


class PrecisionDroneController:
    def __init__(self, logger: logging.Logger | None = None, pid_profile: dict | None = None) -> None:
        self.drone = Drone()
        self.is_flying = False
        self.logger = logger or logging.getLogger("precision_drone")
        self.log_writer: FlightLogWriter | None = None

        profile = pid_profile or default_pid_profile()
        self.profile = profile
        self.pitch_pid = self._make_pid(profile["pitch"])
        self.roll_pid = self._make_pid(profile["roll"])
        self.throttle_pid = self._make_pid(profile["throttle"])
        self.yaw_pid = self._make_pid(profile["yaw"])
        self.pitch_sign = float(profile.get("signs", {}).get("pitch", 1.0))
        self.roll_sign = float(profile.get("signs", {}).get("roll", 1.0))
        self.throttle_sign = float(profile.get("signs", {}).get("throttle", 1.0))
        self.yaw_sign = float(profile.get("signs", {}).get("yaw", 1.0))
        self.pitch_monitor = AxisSignMonitor("pitch", sign=self.pitch_sign)
        self.roll_monitor = AxisSignMonitor("roll", sign=self.roll_sign)
        self.throttle_monitor = AxisSignMonitor("throttle", sign=self.throttle_sign)

    @staticmethod
    def _make_pid(config: dict) -> PIDController:
        return PIDController(
            kp=float(config["kp"]),
            ki=float(config["ki"]),
            kd=float(config["kd"]),
            output_limit=float(config["output_limit"]),
            integral_limit=float(config["integral_limit"]),
        )

    def open_log(self) -> FlightLogWriter:
        self.log_writer = FlightLogWriter(LOG_DIR)
        return self.log_writer

    def initialize_and_takeoff(self) -> None:
        self.logger.info("ドローンに接続します")
        self.drone.pair()

        battery_level = self.drone.get_battery()
        self.logger.info("バッテリー残量: %s%%", battery_level)
        if battery_level < 20:
            raise RuntimeError(f"バッテリー残量が不足しています: {battery_level}%")

        self.logger.info("ジャイロを再初期化します")
        self.drone.reset_gyro()

        self.logger.info("離陸します")
        self.drone.reset_move_values()
        self.drone.takeoff()
        self.is_flying = True
        self.drone.hover(1.0)
        self.drone.reset_move_values()
        self.calibrate_axis_signs()

    def read_snapshot(self, delay: float = 0.005) -> SensorSnapshot:
        position = self.drone.get_position_data(delay=delay)
        motion = self.drone.get_motion_data(delay=delay)
        return SensorSnapshot(
            timestamp=time.time(),
            x_mm=float(position[1]) * 1000.0,
            y_mm=float(position[2]) * 1000.0,
            z_mm=float(position[3]) * 1000.0,
            roll_deg=float(motion[7]),
            pitch_deg=float(motion[8]),
            yaw_deg=float(motion[9]),
            accel_x=float(motion[1]),
            accel_y=float(motion[2]),
            accel_z=float(motion[3]),
            gyro_roll=float(motion[4]),
            gyro_pitch=float(motion[5]),
            gyro_yaw=float(motion[6]),
        )

    def stop_and_hover(self) -> None:
        self.drone.reset_move_values()
        self.drone.hover(0.2)

    def _average_snapshot(self, samples: int = 3, pause: float = 0.03) -> SensorSnapshot:
        snapshots = [self.read_snapshot()]
        for _ in range(samples - 1):
            time.sleep(pause)
            snapshots.append(self.read_snapshot())
        return SensorSnapshot(
            timestamp=snapshots[-1].timestamp,
            x_mm=fmean(snapshot.x_mm for snapshot in snapshots),
            y_mm=fmean(snapshot.y_mm for snapshot in snapshots),
            z_mm=fmean(snapshot.z_mm for snapshot in snapshots),
            roll_deg=fmean(snapshot.roll_deg for snapshot in snapshots),
            pitch_deg=fmean(snapshot.pitch_deg for snapshot in snapshots),
            yaw_deg=fmean(snapshot.yaw_deg for snapshot in snapshots),
            accel_x=fmean(snapshot.accel_x for snapshot in snapshots),
            accel_y=fmean(snapshot.accel_y for snapshot in snapshots),
            accel_z=fmean(snapshot.accel_z for snapshot in snapshots),
            gyro_roll=fmean(snapshot.gyro_roll for snapshot in snapshots),
            gyro_pitch=fmean(snapshot.gyro_pitch for snapshot in snapshots),
            gyro_yaw=fmean(snapshot.gyro_yaw for snapshot in snapshots),
        )

    def _apply_control_pulse(self, roll: int = 0, pitch: int = 0, throttle: int = 0, yaw: int = 0, duration: float = 0.15) -> None:
        self.drone.set_roll(roll)
        self.drone.set_pitch(pitch)
        self.drone.set_throttle(throttle)
        self.drone.set_yaw(yaw)
        self.drone.move(duration)
        self.drone.reset_move_values()
        self.drone.hover(0.25)

    def _probe_axis_direction(self, axis: str, power: int = 12, duration: float = 0.18) -> float | None:
        def axis_value(snapshot: SensorSnapshot) -> float:
            if axis == "pitch":
                return snapshot.x_mm
            if axis == "roll":
                return snapshot.y_mm
            if axis == "throttle":
                return snapshot.z_mm
            raise ValueError(f"unsupported axis probe: {axis}")

        positive_kwargs = {"pitch": 0, "roll": 0, "throttle": 0, "yaw": 0}
        negative_kwargs = {"pitch": 0, "roll": 0, "throttle": 0, "yaw": 0}
        positive_kwargs[axis] = power
        negative_kwargs[axis] = -power

        before_positive = self._average_snapshot()
        self._apply_control_pulse(duration=duration, **positive_kwargs)
        after_positive = self._average_snapshot()
        delta_positive = axis_value(after_positive) - axis_value(before_positive)

        self.drone.hover(0.25)
        before_negative = self._average_snapshot()
        self._apply_control_pulse(duration=duration, **negative_kwargs)
        after_negative = self._average_snapshot()
        delta_negative = axis_value(after_negative) - axis_value(before_negative)

        response = delta_positive - delta_negative
        self.logger.info(
            "%s軸の符号検査 delta_pos=%.1fmm delta_neg=%.1fmm response=%.1fmm",
            axis,
            delta_positive,
            delta_negative,
            response,
        )
        if abs(response) < 12.0:
            self.logger.warning("%s符号検査の応答が小さすぎます response=%.1fmm", axis, response)
            return None
        return 1.0 if response > 0.0 else -1.0

    def calibrate_axis_signs(self) -> None:
        self.logger.info(
            "固定符号を使用します pitch=%.0f roll=%.0f throttle=%.0f yaw=%.0f",
            self.pitch_sign,
            self.roll_sign,
            self.throttle_sign,
            self.yaw_sign,
        )

    def _sync_sign_from_monitor(self, monitor: AxisSignMonitor, attr_name: str, pid: PIDController) -> None:
        current_sign = float(getattr(self, attr_name))
        if not monitor.is_confident() or monitor.sign == 0.0 or monitor.sign == current_sign:
            return

        setattr(self, attr_name, monitor.sign)
        pid.reset()
        self.profile.setdefault("signs", {})
        self.profile["signs"][monitor.axis_name] = monitor.sign
        self.logger.warning(
            "%s軸の実測応答から符号を補正しました: %.0f -> %.0f confidence=%.2f samples=%d",
            monitor.axis_name,
            current_sign,
            monitor.sign,
            monitor.confidence,
            monitor.samples,
        )

    def _update_axis_sign_monitors(
        self,
        prev_snapshot: SensorSnapshot,
        snapshot: SensorSnapshot,
        previous_commands: dict[str, int],
    ) -> None:
        delta_x = snapshot.x_mm - prev_snapshot.x_mm
        delta_y = snapshot.y_mm - prev_snapshot.y_mm
        delta_z = snapshot.z_mm - prev_snapshot.z_mm

        self.pitch_monitor.observe(previous_commands["pitch"], delta_x)
        self.roll_monitor.observe(previous_commands["roll"], delta_y)
        # Vertical motion is usually subtler than lateral motion, so we accept
        # smaller commands and smaller deltas here to let the monitor converge.
        self.throttle_monitor.observe(previous_commands["throttle"], delta_z, min_command=3, min_delta_mm=2.0)

        self._sync_sign_from_monitor(self.pitch_monitor, "pitch_sign", self.pitch_pid)
        self._sync_sign_from_monitor(self.roll_monitor, "roll_sign", self.roll_pid)
        self._sync_sign_from_monitor(self.throttle_monitor, "throttle_sign", self.throttle_pid)

    def _compute_pitch_profile(
        self,
        error_x: float,
        motion_phase_ratio: float,
        x_velocity: float,
        error_y: float,
        error_z: float,
    ) -> tuple[float, float]:
        abs_x_error = abs(error_x)
        phase_ratio = _clamp(motion_phase_ratio, 0.0, 1.0)

        if phase_ratio < 0.30:
            self.pitch_pid.set_gains(0.135, 0.00035, 0.020)
            pitch_limit = 62.0
            pitch_min = 22.0
        elif phase_ratio < 0.70:
            self.pitch_pid.set_gains(0.120, 0.00040, 0.022)
            pitch_limit = 52.0
            pitch_min = 16.0
        elif abs_x_error > 80.0:
            self.pitch_pid.set_gains(0.105, 0.00045, 0.024)
            pitch_limit = 40.0
            pitch_min = 10.0
        elif abs_x_error > 25.0:
            self.pitch_pid.set_gains(0.090, 0.00045, 0.022)
            pitch_limit = 28.0
            pitch_min = 6.0
        else:
            self.pitch_pid.set_gains(0.065, 0.00040, 0.018)
            pitch_limit = 16.0
            pitch_min = 0.0

        if abs_x_error > 50.0 and abs(x_velocity) < 35.0:
            pitch_min = max(pitch_min, 14.0)

        if abs(error_y) > 260.0 or abs(error_z) > 120.0:
            pitch_limit = min(pitch_limit, 12.0)
            pitch_min = 0.0
        elif abs(error_y) > 140.0 or abs(error_z) > 60.0:
            pitch_limit = min(pitch_limit, 22.0)
            pitch_min = min(pitch_min, 6.0)

        self.pitch_pid.output_limit = pitch_limit
        return pitch_limit, pitch_min

    def _compute_roll_profile(self, error_y: float, y_velocity: float) -> tuple[float, float]:
        abs_y_error = abs(error_y)
        if abs_y_error > 120.0:
            self.roll_pid.set_gains(0.110, 0.00030, 0.042)
            roll_limit = 42.0
        elif abs_y_error > 45.0:
            self.roll_pid.set_gains(0.118, 0.00032, 0.046)
            roll_limit = 38.0
        elif abs_y_error > 15.0:
            self.roll_pid.set_gains(0.126, 0.00035, 0.050)
            roll_limit = 30.0
        else:
            self.roll_pid.set_gains(0.134, 0.00038, 0.054)
            roll_limit = 24.0

        roll_min = 4.0 if abs_y_error > 30.0 and abs(y_velocity) < 20.0 else 0.0
        self.roll_pid.output_limit = roll_limit
        return roll_limit, roll_min

    def _compute_throttle_profile(self, error_z: float, z_velocity: float) -> tuple[float, float]:
        abs_z_error = abs(error_z)
        if abs_z_error > 120.0:
            self.throttle_pid.set_gains(0.145, 0.00430, 0.038)
            throttle_limit = 34.0
        elif abs_z_error > 45.0:
            self.throttle_pid.set_gains(0.155, 0.00440, 0.040)
            throttle_limit = 32.0
        elif abs_z_error > 18.0:
            self.throttle_pid.set_gains(0.166, 0.00450, 0.042)
            throttle_limit = 28.0
        else:
            self.throttle_pid.set_gains(0.176, 0.00460, 0.045)
            throttle_limit = 24.0

        throttle_min = 4.0 if abs_z_error > 25.0 and abs(z_velocity) < 15.0 else 0.0
        self.throttle_pid.output_limit = throttle_limit
        return throttle_limit, throttle_min

    def _command_axis(
        self,
        pid: PIDController,
        target: float,
        current: float,
        dt: float,
        min_output: float,
        sign: float = 1.0,
    ) -> int:
        pid.min_output = min_output
        return int(round(pid.compute(target=target, current=current, dt=dt) * sign))


    def move_relative_mm(
        self,
        target_distance_mm: float,
        lateral_distance_mm: float = 0.0,
        vertical_distance_mm: float = 0.0,
        timeout_sec: float = 20.0,
        loop_period_sec: float = 0.05,
        settle_tolerance_mm: float = 10.0,
        settle_time_sec: float = 0.4,
    ) -> FlightSummary | None:
        if self.log_writer is None:
            self.open_log()

        start = self.read_snapshot()
        target_x_mm = start.x_mm + target_distance_mm
        target_y_mm = start.y_mm + lateral_distance_mm
        target_z_mm = start.z_mm + vertical_distance_mm
        target_yaw_deg = start.yaw_deg

        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.throttle_pid.reset()
        self.yaw_pid.reset()

        samples: list[ControlSample] = []
        start_perf = time.perf_counter()
        prev_sample_perf = start_perf
        prev_snapshot = start
        stable_since = None
        step = 0
        lateral_fault_count = 0
        move_direction = _sign(target_distance_mm) or 1.0
        target_span = max(abs(target_distance_mm), 1.0)
        motion_phase_ratio = 0.0

        self.logger.info(
            "移動開始 target=(%.1f, %.1f, %.1f)mm start=(%.1f, %.1f, %.1f) yaw=%.1f",
            target_distance_mm,
            lateral_distance_mm,
            vertical_distance_mm,
            start.x_mm,
            start.y_mm,
            start.z_mm,
            start.yaw_deg,
        )

        while True:
            loop_start_perf = time.perf_counter()
            elapsed = loop_start_perf - start_perf
            if elapsed > timeout_sec:
                self.logger.warning("タイムアウトしました")
                break

            sleep_for = loop_period_sec - (loop_start_perf - prev_sample_perf)
            if sleep_for > 0:
                time.sleep(sleep_for)

            snapshot = self.read_snapshot()
            sample_perf = time.perf_counter()
            dt = max(sample_perf - prev_sample_perf, 0.001)

            error_x = target_x_mm - snapshot.x_mm
            error_y = target_y_mm - snapshot.y_mm
            error_z = target_z_mm - snapshot.z_mm
            error_yaw = target_yaw_deg - snapshot.yaw_deg

            x_velocity = (snapshot.x_mm - prev_snapshot.x_mm) / dt
            y_velocity = (snapshot.y_mm - prev_snapshot.y_mm) / dt
            z_velocity = (snapshot.z_mm - prev_snapshot.z_mm) / dt

            traveled_mm = max(0.0, (snapshot.x_mm - start.x_mm) * move_direction)
            current_phase_ratio = traveled_mm / target_span
            motion_phase_ratio = max(motion_phase_ratio, current_phase_ratio)
            _, pitch_min = self._compute_pitch_profile(error_x, motion_phase_ratio, x_velocity, error_y, error_z)
            _, roll_min = self._compute_roll_profile(error_y, y_velocity)
            _, throttle_min = self._compute_throttle_profile(error_z, z_velocity)

            self.yaw_pid.output_limit = 10.0 if abs(error_yaw) > 8.0 else 6.0

            pitch_cmd = self._command_axis(self.pitch_pid, target_x_mm, snapshot.x_mm, dt, pitch_min, self.pitch_sign)
            roll_cmd = self._command_axis(self.roll_pid, target_y_mm, snapshot.y_mm, dt, roll_min, self.roll_sign)
            throttle_cmd = self._command_axis(self.throttle_pid, target_z_mm, snapshot.z_mm, dt, throttle_min, self.throttle_sign)
            yaw_cmd = self._command_axis(self.yaw_pid, target_yaw_deg, snapshot.yaw_deg, dt, 0.0, self.yaw_sign)

            if motion_phase_ratio < 0.55:
                roll_cmd = 0 if abs(error_y) < 80.0 else int(_clamp(roll_cmd, 6.0))
                throttle_cmd = 0 if abs(error_z) < 60.0 else int(_clamp(throttle_cmd, 6.0))
            elif motion_phase_ratio < 0.85:
                roll_cmd = 0 if abs(error_y) < 35.0 else int(_clamp(roll_cmd, 10.0))
                throttle_cmd = 0 if abs(error_z) < 25.0 else int(_clamp(throttle_cmd, 10.0))
            else:
                roll_cmd = int(_clamp(roll_cmd, 18.0))
                throttle_cmd = int(_clamp(throttle_cmd, 16.0))

            if abs(error_y) > 260.0 or abs(error_z) > 120.0:
                pitch_cmd = 0
            elif abs(error_y) > 140.0 or abs(error_z) > 60.0:
                pitch_cmd = int(_clamp(pitch_cmd, 12.0))

            if abs(error_y) > 280.0:
                roll_cmd = int(_clamp(roll_cmd, 30.0))

            moving_away_laterally = _sign(error_y) == _sign(y_velocity) and abs(y_velocity) > 90.0
            if abs(error_y) > 180.0 and moving_away_laterally:
                lateral_fault_count += 1
            else:
                lateral_fault_count = 0

            if abs(error_y) > 320.0 or lateral_fault_count >= 5:
                self.logger.error(
                    "横方向ドリフトを検知したため停止します: error_y=%.1f y_velocity=%.1f roll_cmd=%d",
                    error_y,
                    y_velocity,
                    roll_cmd,
                )
                break

            sample = ControlSample(
                step=step,
                timestamp=snapshot.timestamp,
                elapsed_sec=elapsed,
                dt_sec=dt,
                target_x_mm=target_x_mm,
                target_y_mm=target_y_mm,
                target_z_mm=target_z_mm,
                target_yaw_deg=target_yaw_deg,
                x_mm=snapshot.x_mm,
                y_mm=snapshot.y_mm,
                z_mm=snapshot.z_mm,
                yaw_deg=snapshot.yaw_deg,
                error_x_mm=error_x,
                error_y_mm=error_y,
                error_z_mm=error_z,
                error_yaw_deg=error_yaw,
                pitch_cmd=pitch_cmd,
                roll_cmd=roll_cmd,
                throttle_cmd=throttle_cmd,
                yaw_cmd=yaw_cmd,
                x_velocity_mm_s=x_velocity,
                y_velocity_mm_s=y_velocity,
                z_velocity_mm_s=z_velocity,
                yaw_rate_deg_s=snapshot.gyro_yaw,
            )
            self._record_sample(sample)
            samples.append(sample)

            self.logger.info(
                "step=%03d t=%5.2fs x=%8.1f/%8.1f err=%7.1f y=%7.1f z=%7.1f yaw=%6.1f cmd[p=%3d r=%3d t=%3d y=%3d]",
                step,
                elapsed,
                snapshot.x_mm,
                target_x_mm,
                error_x,
                snapshot.y_mm,
                snapshot.z_mm,
                snapshot.yaw_deg,
                pitch_cmd,
                roll_cmd,
                throttle_cmd,
                yaw_cmd,
            )

            self.drone.set_roll(roll_cmd)
            self.drone.set_pitch(pitch_cmd)
            self.drone.set_throttle(throttle_cmd)
            self.drone.set_yaw(yaw_cmd)
            self.drone.move()

            stable = is_settled_state(
                error_x_mm=error_x,
                error_y_mm=error_y,
                error_z_mm=error_z,
                error_yaw_deg=error_yaw,
                x_velocity_mm_s=x_velocity,
                y_velocity_mm_s=y_velocity,
                z_velocity_mm_s=z_velocity,
            ) and abs(error_x) <= settle_tolerance_mm

            if stable:
                if stable_since is None:
                    stable_since = elapsed
                elif elapsed - stable_since >= settle_time_sec:
                    self.logger.info("目標位置に安定しました")
                    break
            else:
                stable_since = None

            prev_snapshot = snapshot
            prev_sample_perf = sample_perf
            step += 1

        self.stop_and_hover()
        summary = FlightLogAnalyzer.summarize(samples)
        if summary is not None:
            self._record_summary(summary)
            self.profile.setdefault("signs", {})
            self.profile["signs"]["pitch"] = self.pitch_sign
            self.profile["signs"]["roll"] = self.roll_sign
            self.profile["signs"]["throttle"] = self.throttle_sign
            self.profile["signs"]["yaw"] = self.yaw_sign
            self.logger.info(
                "summary samples=%d duration=%.2fs final_error=%.1fmm max_error=%.1fmm osc=%d settle=%s",
                summary.samples,
                summary.duration_sec,
                summary.final_error_x_mm,
                summary.max_abs_error_x_mm,
                summary.oscillation_count,
                "n/a" if summary.settling_time_sec is None else f"{summary.settling_time_sec:.2f}s",
            )
        return summary

    def _record_sample(self, sample: ControlSample) -> None:
        if self.log_writer is not None:
            self.log_writer.write("sample", asdict(sample))

    def _record_summary(self, summary: FlightSummary) -> None:
        if self.log_writer is not None:
            self.log_writer.write("summary", asdict(summary))

    def safe_land_and_close(self) -> None:
        try:
            if self.is_flying:
                self.logger.info("着陸します")
                self.stop_and_hover()
                self.drone.land()
        finally:
            if self.log_writer is not None:
                self.log_writer.close()
            self.logger.info("通信を切断します")
            self.drone.close()


def _setup_logging() -> logging.Logger:
    logger = logging.getLogger("precision_drone")
    if not logger.handlers:
        logger.setLevel(logging.INFO)
        formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")

        stream = logging.StreamHandler()
        stream.setFormatter(formatter)
        logger.addHandler(stream)

        file_handler = logging.FileHandler(LOG_DIR / "latest_run.log", encoding="utf-8")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def run_flight(
    target_distance_mm: float = 2000.0,
    lateral_distance_mm: float = 0.0,
    vertical_distance_mm: float = 0.0,
    timeout_sec: float = 20.0,
) -> FlightSummary | None:
    logger = _setup_logging()
    profile = load_pid_profile(target_distance_mm)
    controller = PrecisionDroneController(logger=logger, pid_profile=profile)

    try:
        with controller.open_log():
            controller.initialize_and_takeoff()
            summary = controller.move_relative_mm(
                target_distance_mm=target_distance_mm,
                lateral_distance_mm=lateral_distance_mm,
                vertical_distance_mm=vertical_distance_mm,
                timeout_sec=timeout_sec,
            )
            if summary is not None:
                tuned_profile = adjust_pid_profile(controller.profile, summary)
                save_pid_profile_for_distance(tuned_profile, target_distance_mm)
            return summary
    except KeyboardInterrupt:
        logger.warning("ユーザーにより中断されました")
        return None
    finally:
        controller.safe_land_and_close()


def main() -> None:
    run_flight(target_distance_mm=2000.0, lateral_distance_mm=0.0, vertical_distance_mm=0.0, timeout_sec=20.0)
