from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import fmean, median

DOCS_URL = "https://docs.robolink.com/docs/CoDroneEDU/Python/Drone-Function-Documentation"
LOG_DIR = Path(__file__).resolve().parent / "flight_logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
PID_PROFILE_PATH = LOG_DIR / "last_pid_profile.json"
HISTORY_ANALYSIS_PATH = LOG_DIR / "last_history_analysis.json"
FIXED_CONTROL_SIGNS = {"pitch": 1.0, "roll": -1.0, "throttle": 1.0, "yaw": 1.0}
DEFAULT_MAX_PITCH = 100
DEFAULT_BRAKE_PITCH = 100
DEFAULT_MAX_ROLL = 100
DEFAULT_BRAKE_ROLL = 100
DEFAULT_MAX_THROTTLE = 80
DEFAULT_BRAKE_THROTTLE = 80
DEFAULT_BRAKE_ACCEL_MM_S2 = 90.0
DEFAULT_TIMEOUT_SEC = 35.0
BRAKE_MARGIN_MM = 35.0
BRAKE_RELEASE_SPEED_MM_S = 25.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _signed_limit(value: float, limit: float) -> float:
    return _clamp(value, -abs(limit), abs(limit))


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _wrap_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def calculate_stopping_distance_mm(
    speed_mm_s: float,
    brake_accel_mm_s2: float,
    sensor_latency_sec: float,
    margin_mm: float = BRAKE_MARGIN_MM,
) -> float:
    speed = max(0.0, speed_mm_s)
    acceleration = max(brake_accel_mm_s2, 1.0)
    latency = max(sensor_latency_sec, 0.0)
    return (speed * speed) / (2.0 * acceleration) + (speed * latency) + margin_mm


def _import_drone_class():
    try:
        from codrone_edu.drone import Drone as drone_class

        return drone_class
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "codrone_edu が見つかりません。.venv を有効化してから実行するか、"
            "CoDrone EDU の Python ライブラリをインストールしてください。"
        ) from exc


def profile_key_for_distance(target_distance_mm: float) -> str:
    rounded_mm = max(0, int(round(abs(target_distance_mm))))
    return f"{rounded_mm}mm"


def pid_profile_path_for_key(profile_key: str) -> Path:
    return LOG_DIR / f"last_pid_profile_{profile_key}.json"


def _list_distance_profile_paths() -> list[Path]:
    return sorted(
        path
        for path in LOG_DIR.glob("last_pid_profile_*mm.json")
        if path.name not in {"last_pid_profile.json"}
    )


def _distance_from_profile_path(path: Path) -> float | None:
    stem = path.stem.removeprefix("last_pid_profile_").removesuffix("mm")
    try:
        return float(stem)
    except ValueError:
        return None


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
        abs(error_x_mm) <= 8.0
        and abs(error_y_mm) <= 12.0
        and abs(error_z_mm) <= 12.0
        and abs(error_yaw_deg) <= 5.0
        and abs(x_velocity_mm_s) <= 25.0
        and abs(y_velocity_mm_s) <= 25.0
        and abs(z_velocity_mm_s) <= 20.0
    )


def default_pid_profile() -> dict:
    return {
        "pitch": {
            "kp": 0.110,
            "ki": 0.00030,
            "kd": 0.010,
            "kv": 0.080,
            "output_limit": 68.0,
            "integral_limit": 600.0,
            "min_output": 6.0,
            "min_output_error": 30.0,
        },
        "roll": {
            "kp": 0.165,
            "ki": 0.00070,
            "kd": 0.012,
            "kv": 0.120,
            "output_limit": 52.0,
            "integral_limit": 500.0,
            "min_output": 7.0,
            "min_output_error": 28.0,
        },
        "throttle": {
            "kp": 0.205,
            "ki": 0.00520,
            "kd": 0.014,
            "kv": 0.180,
            "output_limit": 42.0,
            "integral_limit": 450.0,
            "min_output": 5.0,
            "min_output_error": 18.0,
        },
        "yaw": {
            "kp": 0.520,
            "ki": 0.00020,
            "kd": 0.030,
            "kv": 0.050,
            "output_limit": 18.0,
            "integral_limit": 60.0,
            "min_output": 0.0,
            "min_output_error": 0.0,
        },
        "signs": dict(FIXED_CONTROL_SIGNS),
    }


def load_pid_profile(target_distance_mm: float) -> dict:
    profile = default_pid_profile()
    target_magnitude = abs(target_distance_mm)
    exact_path = pid_profile_path_for_key(profile_key_for_distance(target_distance_mm))

    candidate_paths: list[Path] = []
    if exact_path.exists():
        candidate_paths.append(exact_path)

    nearest_paths = sorted(
        (
            (abs(distance_mm - target_magnitude), path)
            for path in _list_distance_profile_paths()
            for distance_mm in [_distance_from_profile_path(path)]
            if distance_mm is not None and path != exact_path
        ),
        key=lambda item: item[0],
    )
    candidate_paths.extend(path for _, path in nearest_paths[:3])
    candidate_paths.append(PID_PROFILE_PATH)

    raw = None
    for path in candidate_paths:
        if not path.exists():
            continue
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
            break
        except Exception:
            continue

    if raw is None:
        return profile

    for axis_name, config in profile.items():
        if axis_name == "signs":
            continue
        raw_config = raw.get(axis_name)
        if not isinstance(raw_config, dict):
            continue
        for key in config:
            if key in raw_config:
                try:
                    config[key] = float(raw_config[key])
                except (TypeError, ValueError):
                    pass

    # Control-axis polarity is fixed for this hardware and should not drift by profile.
    profile["signs"] = dict(FIXED_CONTROL_SIGNS)
    profile["throttle"]["min_output"] = 0.0

    # Old learned profiles can contain limits that permit severe wind-up. Keep
    # learned gains, but normalize safety-critical bounds on every load.
    profile["pitch"]["integral_limit"] = min(profile["pitch"]["integral_limit"], 600.0)
    profile["roll"]["integral_limit"] = min(profile["roll"]["integral_limit"], 500.0)
    profile["throttle"]["integral_limit"] = min(profile["throttle"]["integral_limit"], 450.0)
    profile["yaw"]["integral_limit"] = min(profile["yaw"]["integral_limit"], 60.0)
    profile["pitch"]["output_limit"] = max(profile["pitch"]["output_limit"], 55.0)
    profile["roll"]["kp"] = max(profile["roll"]["kp"], 0.165)
    profile["roll"]["ki"] = max(profile["roll"]["ki"], 0.00070)
    profile["roll"]["output_limit"] = max(profile["roll"]["output_limit"], 52.0)
    profile["roll"]["min_output"] = max(profile["roll"]["min_output"], 7.0)
    profile["roll"]["min_output_error"] = max(profile["roll"].get("min_output_error", 0.0), 28.0)
    profile["throttle"]["kp"] = max(profile["throttle"]["kp"], 0.205)
    profile["throttle"]["ki"] = max(profile["throttle"]["ki"], 0.00520)
    profile["throttle"]["output_limit"] = max(profile["throttle"]["output_limit"], 42.0)
    profile["throttle"]["min_output"] = max(profile["throttle"]["min_output"], 5.0)
    profile["throttle"]["min_output_error"] = max(profile["throttle"].get("min_output_error", 0.0), 18.0)
    profile["throttle"]["kd"] = min(profile["throttle"]["kd"], 0.025)
    profile["throttle"]["kv"] = min(profile["throttle"]["kv"], 0.18)

    return profile


def save_pid_profile(profile: dict) -> None:
    PID_PROFILE_PATH.write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")


def save_pid_profile_for_distance(profile: dict, target_distance_mm: float) -> None:
    profile_key = profile_key_for_distance(target_distance_mm)
    pid_profile_path_for_key(profile_key).write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")
    save_pid_profile(profile)


@dataclass
class SensorSnapshot:
    wall_time: float
    pos_age_sec: float
    motion_age_sec: float
    pos_sensor_time: float
    motion_sensor_time: float
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
class StateEstimate:
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_deg: float
    x_velocity_mm_s: float
    y_velocity_mm_s: float
    z_velocity_mm_s: float
    yaw_rate_deg_s: float
    pos_fresh: bool
    motion_fresh: bool
    stale_cycles: int
    stale_duration_sec: float
    last_position_update_wall_time: float


@dataclass
class ControlSample:
    step: int
    wall_time: float
    elapsed_sec: float
    dt_sec: float
    pos_fresh: bool
    motion_fresh: bool
    stale_cycles: int
    stale_duration_sec: float
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
    x_velocity_mm_s: float
    y_velocity_mm_s: float
    z_velocity_mm_s: float
    yaw_rate_deg_s: float
    pitch_cmd: int
    roll_cmd: int
    throttle_cmd: int
    yaw_cmd: int
    pitch_limit: float
    roll_limit: float
    throttle_limit: float
    yaw_limit: float
    pitch_integral: float
    roll_integral: float
    throttle_integral: float
    yaw_integral: float
    control_phase: str
    predicted_remaining_x_mm: float
    stopping_distance_x_mm: float
    control_phase_y: str = "settle"
    control_phase_z: str = "settle"
    predicted_remaining_y_mm: float = 0.0
    predicted_remaining_z_mm: float = 0.0
    stopping_distance_y_mm: float = BRAKE_MARGIN_MM
    stopping_distance_z_mm: float = BRAKE_MARGIN_MM


@dataclass
class FlightSummary:
    samples: int
    duration_sec: float
    target_x_mm: float
    target_y_mm: float
    target_z_mm: float
    final_error_x_mm: float
    final_error_y_mm: float
    final_error_z_mm: float
    final_error_yaw_deg: float
    max_abs_error_x_mm: float
    max_abs_error_y_mm: float
    max_abs_error_z_mm: float
    mean_abs_error_x_mm: float
    mean_abs_error_y_mm: float
    mean_abs_error_z_mm: float
    peak_speed_x_mm_s: float
    peak_speed_y_mm_s: float
    peak_speed_z_mm_s: float
    stale_ratio: float
    oscillation_count_x: int
    oscillation_count_y: int
    oscillation_count_z: int
    settling_time_sec: float | None
    termination_reason: str
    brake_start_error_x_mm: float | None
    brake_start_speed_x_mm_s: float | None
    peak_abs_pitch_cmd: int


@dataclass
class HistoryAnalysis:
    runs_analyzed: int
    median_final_error_x_mm: float
    median_final_error_y_mm: float
    median_final_error_z_mm: float
    median_duration_sec: float
    median_stale_ratio: float


@dataclass
class FlightCommand:
    target_x_mm: float = 0.0
    target_y_mm: float = 0.0
    target_z_mm: float = 0.0
    target_yaw_deg: float = 0.0
    timeout_sec: float = DEFAULT_TIMEOUT_SEC
    loop_period_sec: float = 0.06
    settle_time_sec: float = 0.5
    history_limit: int = 8
    auto_tune: bool = True
    profile_distance_mm: float | None = None
    max_pitch: int = DEFAULT_MAX_PITCH
    brake_pitch: int = DEFAULT_BRAKE_PITCH
    max_roll: int = DEFAULT_MAX_ROLL
    brake_roll: int = DEFAULT_BRAKE_ROLL
    max_throttle: int = DEFAULT_MAX_THROTTLE
    brake_throttle: int = DEFAULT_BRAKE_THROTTLE
    brake_accel_mm_s2: float = DEFAULT_BRAKE_ACCEL_MM_S2

    @property
    def profile_target_distance_mm(self) -> float:
        if self.profile_distance_mm is not None:
            return self.profile_distance_mm
        return self.target_x_mm


@dataclass
class AxisRapidState:
    axis_name: str
    command_name: str
    direction: float
    initial_abs_error: float
    min_rapid_distance_mm: float
    phase: str
    last_fresh_speed_toward: float = 0.0
    predicted_remaining_mm: float = 0.0
    stopping_distance_mm: float = BRAKE_MARGIN_MM
    best_abs_error_mm: float = 0.0
    runaway_count: int = 0
    reverse_fresh_count: int = 0
    rapid_passes: int = 1
    guard_grace_cycles: int = 0


class PIDController:
    def __init__(
        self,
        *,
        kp: float,
        ki: float,
        kd: float,
        kv: float,
        output_limit: float,
        integral_limit: float,
        min_output: float,
    ) -> None:
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd
        self.base_kv = kv
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.min_output = min_output
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_measurement_rate = 0.0
        self._has_prev = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_measurement_rate = 0.0
        self._has_prev = False

    def compute(
        self,
        *,
        error: float,
        measurement: float,
        measurement_rate: float,
        dt: float,
        output_limit: float,
        min_output: float,
        min_output_error: float = 0.0,
        gain_scale: float = 1.0,
        allow_integral: bool = True,
    ) -> float:
        if dt <= 0.0:
            return 0.0

        kp = self.base_kp * gain_scale
        ki = self.base_ki * gain_scale
        kd = self.base_kd * gain_scale
        kv = self.base_kv * gain_scale

        error_rate = 0.0 if not self._has_prev else (error - self.prev_error) / dt
        measurement_accel = 0.0 if not self._has_prev else (measurement_rate - self.prev_measurement_rate) / dt
        derivative_term = (0.60 * error_rate) - (0.40 * measurement_accel)

        if allow_integral:
            candidate_integral = _clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
            candidate_output = (
                (kp * error)
                + (ki * candidate_integral)
                + (kd * derivative_term)
                - (kv * measurement_rate)
            )
            # Integrate while unsaturated, or when the error would pull a
            # saturated command back toward the controllable range.
            if abs(candidate_output) <= output_limit or candidate_output * error < 0.0:
                self.integral = candidate_integral
        else:
            self.integral *= 0.92

        output = (kp * error) + (ki * self.integral) + (kd * derivative_term) - (kv * measurement_rate)
        output = _signed_limit(output, output_limit)

        if min_output > 0.0 and abs(error) >= min_output_error and abs(output) < min_output:
            output = _sign(error) * min_output

        self.prev_error = error
        self.prev_measurement = measurement
        self.prev_measurement_rate = measurement_rate
        self._has_prev = True
        self.output_limit = output_limit
        self.min_output = min_output
        return _signed_limit(output, output_limit)


class FlightLogWriter:
    def __init__(self, log_dir: Path) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = log_dir / f"pid_flight_{stamp}.jsonl"
        self._fh = self.log_path.open("w", encoding="utf-8")

    def write(self, event: str, payload: dict) -> None:
        self._fh.write(json.dumps({"event": event, "logged_at": time.time(), **payload}, ensure_ascii=False) + "\n")
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
    def summarize(
        samples: list[ControlSample],
        termination_reason: str = "unknown",
    ) -> FlightSummary | None:
        if not samples:
            return None

        last = samples[-1]
        brake_sample = next((sample for sample in samples if sample.control_phase == "brake"), None)
        return FlightSummary(
            samples=len(samples),
            duration_sec=last.elapsed_sec,
            target_x_mm=last.target_x_mm - samples[0].x_mm,
            target_y_mm=last.target_y_mm - samples[0].y_mm,
            target_z_mm=last.target_z_mm - samples[0].z_mm,
            final_error_x_mm=last.error_x_mm,
            final_error_y_mm=last.error_y_mm,
            final_error_z_mm=last.error_z_mm,
            final_error_yaw_deg=last.error_yaw_deg,
            max_abs_error_x_mm=max(abs(s.error_x_mm) for s in samples),
            max_abs_error_y_mm=max(abs(s.error_y_mm) for s in samples),
            max_abs_error_z_mm=max(abs(s.error_z_mm) for s in samples),
            mean_abs_error_x_mm=fmean(abs(s.error_x_mm) for s in samples),
            mean_abs_error_y_mm=fmean(abs(s.error_y_mm) for s in samples),
            mean_abs_error_z_mm=fmean(abs(s.error_z_mm) for s in samples),
            peak_speed_x_mm_s=max(abs(s.x_velocity_mm_s) for s in samples),
            peak_speed_y_mm_s=max(abs(s.y_velocity_mm_s) for s in samples),
            peak_speed_z_mm_s=max(abs(s.z_velocity_mm_s) for s in samples),
            stale_ratio=sum(1 for s in samples if not s.pos_fresh) / len(samples),
            oscillation_count_x=FlightLogAnalyzer._count_sign_changes([s.error_x_mm for s in samples]),
            oscillation_count_y=FlightLogAnalyzer._count_sign_changes([s.error_y_mm for s in samples]),
            oscillation_count_z=FlightLogAnalyzer._count_sign_changes([s.error_z_mm for s in samples]),
            settling_time_sec=FlightLogAnalyzer._estimate_settling_time(samples),
            termination_reason=termination_reason,
            brake_start_error_x_mm=None if brake_sample is None else brake_sample.error_x_mm,
            brake_start_speed_x_mm_s=None if brake_sample is None else brake_sample.x_velocity_mm_s,
            peak_abs_pitch_cmd=max(abs(sample.pitch_cmd) for sample in samples),
        )

    @staticmethod
    def _count_sign_changes(values: list[float]) -> int:
        sign_changes = 0
        previous = _sign(values[0])
        for value in values[1:]:
            current = _sign(value)
            if previous != 0.0 and current != 0.0 and previous != current:
                sign_changes += 1
            if current != 0.0:
                previous = current
        return sign_changes

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
                elif sample.elapsed_sec - stable_since >= 0.5:
                    return stable_since
            else:
                stable_since = None
        return None

    @staticmethod
    def analyze_history(
        log_dir: Path,
        limit: int = 8,
        target_distance_mm: float | None = None,
    ) -> HistoryAnalysis | None:
        summaries: list[dict] = []
        for path in reversed(sorted(log_dir.glob("pid_flight_*.jsonl"))):
            summary = None
            with path.open(encoding="utf-8") as fh:
                for line in fh:
                    record = json.loads(line)
                    if record.get("event") == "summary":
                        summary = record
            if summary is not None:
                if target_distance_mm is not None:
                    logged_target = abs(float(summary.get("target_x_mm", 0.0)))
                    match_tolerance = max(25.0, abs(target_distance_mm) * 0.10)
                    if abs(logged_target - abs(target_distance_mm)) > match_tolerance:
                        continue
                summaries.append(summary)
                if len(summaries) >= limit:
                    break

        if not summaries:
            return None

        return HistoryAnalysis(
            runs_analyzed=len(summaries),
            median_final_error_x_mm=median(float(item["final_error_x_mm"]) for item in summaries),
            median_final_error_y_mm=median(float(item["final_error_y_mm"]) for item in summaries),
            median_final_error_z_mm=median(float(item["final_error_z_mm"]) for item in summaries),
            median_duration_sec=median(float(item["duration_sec"]) for item in summaries),
            median_stale_ratio=median(float(item.get("stale_ratio", 0.0)) for item in summaries),
        )


def adjust_pid_profile(profile: dict, summary: FlightSummary | None) -> dict:
    if summary is None:
        return profile

    tuned = json.loads(json.dumps(profile))
    pitch = tuned["pitch"]
    roll = tuned["roll"]
    throttle = tuned["throttle"]
    yaw = tuned["yaw"]

    if abs(summary.final_error_x_mm) > 25.0:
        pitch["kp"] *= 1.05
        pitch["kv"] *= 0.98
    if summary.oscillation_count_x >= 3:
        pitch["kp"] *= 0.94
        pitch["kd"] *= 1.08
        pitch["kv"] *= 1.06

    if abs(summary.final_error_y_mm) > 18.0:
        roll["kp"] *= 1.04
        roll["ki"] *= 1.03
    if summary.oscillation_count_y >= 3:
        roll["kp"] *= 0.95
        roll["kd"] *= 1.07
        roll["kv"] *= 1.06

    if abs(summary.final_error_z_mm) > 15.0:
        if summary.peak_speed_z_mm_s > 55.0:
            # CoDrone altitude feedback has enough delay that stronger D/velocity
            # feedback causes command reversal and sustained hunting.
            throttle["kp"] *= 0.96
            throttle["ki"] *= 0.85
            throttle["kv"] *= 0.94
            throttle["kd"] *= 0.90
        else:
            throttle["kp"] *= 1.02
            throttle["ki"] *= 1.01
    if summary.oscillation_count_z >= 3:
        throttle["kp"] *= 0.95
        throttle["kd"] *= 1.08
        throttle["kv"] *= 1.06

    if abs(summary.final_error_yaw_deg) > 5.0:
        yaw["kp"] *= 1.03
        yaw["kd"] *= 1.03

    for axis_name, kp_range, ki_range, kd_range, kv_range in (
        ("pitch", (0.05, 0.22), (0.0, 0.004), (0.0, 0.08), (0.02, 0.25)),
        ("roll", (0.06, 0.24), (0.0, 0.006), (0.0, 0.08), (0.03, 0.30)),
        ("throttle", (0.08, 0.28), (0.0, 0.012), (0.0, 0.025), (0.04, 0.18)),
        ("yaw", (0.20, 0.90), (0.0, 0.01), (0.0, 0.12), (0.01, 0.20)),
    ):
        config = tuned[axis_name]
        config["kp"] = _clamp(config["kp"], *kp_range)
        config["ki"] = _clamp(config["ki"], *ki_range)
        config["kd"] = _clamp(config["kd"], *kd_range)
        config["kv"] = _clamp(config["kv"], *kv_range)

    return tuned


class PrecisionDroneController:
    def __init__(self, logger: logging.Logger | None = None, pid_profile: dict | None = None) -> None:
        self.logger = logger or logging.getLogger("precision_drone")
        self.drone = _import_drone_class()()
        self.is_flying = False
        self.log_writer: FlightLogWriter | None = None
        self.profile = pid_profile or default_pid_profile()

        self.pitch_pid = self._make_pid(self.profile["pitch"])
        self.roll_pid = self._make_pid(self.profile["roll"])
        self.throttle_pid = self._make_pid(self.profile["throttle"])
        self.yaw_pid = self._make_pid(self.profile["yaw"])

        self.pitch_sign = float(FIXED_CONTROL_SIGNS["pitch"])
        self.roll_sign = float(FIXED_CONTROL_SIGNS["roll"])
        self.throttle_sign = float(FIXED_CONTROL_SIGNS["throttle"])
        self.yaw_sign = float(FIXED_CONTROL_SIGNS["yaw"])

    @staticmethod
    def _make_pid(config: dict) -> PIDController:
        return PIDController(
            kp=float(config["kp"]),
            ki=float(config["ki"]),
            kd=float(config.get("kd", 0.0)),
            kv=float(config.get("kv", 0.0)),
            output_limit=float(config["output_limit"]),
            integral_limit=float(config["integral_limit"]),
            min_output=float(config.get("min_output", 0.0)),
        )

    def open_log(self) -> FlightLogWriter:
        self.log_writer = FlightLogWriter(LOG_DIR)
        self.log_writer.write("metadata", {"docs_url": DOCS_URL, "profile": self.profile})
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
        self.logger.info("使用ドキュメント: %s", DOCS_URL)

    def read_snapshot(self, delay: float = 0.01) -> SensorSnapshot:
        position = self.drone.get_position_data(delay=delay)
        motion = self.drone.get_motion_data(delay=delay)
        now = time.time()
        return SensorSnapshot(
            wall_time=now,
            pos_age_sec=max(0.0, float(position[0])),
            motion_age_sec=max(0.0, float(motion[0])),
            pos_sensor_time=float(position[0]),
            motion_sensor_time=float(motion[0]),
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

    def acquire_reference_snapshot(
        self,
        *,
        minimum_altitude_mm: float = 150.0,
        timeout_sec: float = 3.0,
        required_fresh_samples: int = 3,
    ) -> SensorSnapshot:
        deadline = time.perf_counter() + timeout_sec
        fresh_snapshots: list[SensorSnapshot] = []
        latest_snapshot = self.read_snapshot()

        while time.perf_counter() < deadline:
            snapshot = self.read_snapshot()
            latest_snapshot = snapshot

            position_looks_valid = snapshot.z_mm >= minimum_altitude_mm or (
                abs(snapshot.x_mm) > 1.0 or abs(snapshot.y_mm) > 1.0
            )
            if position_looks_valid:
                fresh_snapshots.append(snapshot)
                if len(fresh_snapshots) >= required_fresh_samples:
                    break
            time.sleep(0.03)

        if not fresh_snapshots:
            self.logger.warning(
                "基準位置の新鮮な取得に失敗したため、最後のスナップショットを使います x=%.1f y=%.1f z=%.1f",
                latest_snapshot.x_mm,
                latest_snapshot.y_mm,
                latest_snapshot.z_mm,
            )
            return latest_snapshot

        return SensorSnapshot(
            wall_time=fresh_snapshots[-1].wall_time,
            pos_age_sec=fmean(item.pos_age_sec for item in fresh_snapshots),
            motion_age_sec=fmean(item.motion_age_sec for item in fresh_snapshots),
            pos_sensor_time=fresh_snapshots[-1].pos_sensor_time,
            motion_sensor_time=fresh_snapshots[-1].motion_sensor_time,
            x_mm=fmean(item.x_mm for item in fresh_snapshots),
            y_mm=fmean(item.y_mm for item in fresh_snapshots),
            z_mm=fmean(item.z_mm for item in fresh_snapshots),
            roll_deg=fmean(item.roll_deg for item in fresh_snapshots),
            pitch_deg=fmean(item.pitch_deg for item in fresh_snapshots),
            yaw_deg=fmean(item.yaw_deg for item in fresh_snapshots),
            accel_x=fmean(item.accel_x for item in fresh_snapshots),
            accel_y=fmean(item.accel_y for item in fresh_snapshots),
            accel_z=fmean(item.accel_z for item in fresh_snapshots),
            gyro_roll=fmean(item.gyro_roll for item in fresh_snapshots),
            gyro_pitch=fmean(item.gyro_pitch for item in fresh_snapshots),
            gyro_yaw=fmean(item.gyro_yaw for item in fresh_snapshots),
        )

    def stop_and_hover(self, duration: float = 0.25) -> None:
        self.drone.reset_move_values()
        self.drone.hover(duration)

    def _compute_state_estimate(
        self,
        snapshot: SensorSnapshot,
        previous_snapshot: SensorSnapshot | None,
        previous_state: StateEstimate | None,
    ) -> StateEstimate:
        pos_fresh = previous_snapshot is None or snapshot.pos_sensor_time != previous_snapshot.pos_sensor_time
        motion_fresh = previous_snapshot is None or snapshot.motion_sensor_time != previous_snapshot.motion_sensor_time

        if previous_state is None or previous_snapshot is None:
            return StateEstimate(
                x_mm=snapshot.x_mm,
                y_mm=snapshot.y_mm,
                z_mm=snapshot.z_mm,
                yaw_deg=snapshot.yaw_deg,
                x_velocity_mm_s=0.0,
                y_velocity_mm_s=0.0,
                z_velocity_mm_s=0.0,
                yaw_rate_deg_s=snapshot.gyro_yaw,
                pos_fresh=pos_fresh,
                motion_fresh=motion_fresh,
                stale_cycles=0,
                stale_duration_sec=0.0,
                last_position_update_wall_time=snapshot.wall_time,
            )

        if pos_fresh:
            # Position packets often arrive slower than the control loop. Use
            # the interval since the last fresh packet, otherwise velocity is
            # exaggerated after every stale sequence.
            dt = max(snapshot.wall_time - previous_state.last_position_update_wall_time, 0.001)
            raw_vx = (snapshot.x_mm - previous_state.x_mm) / dt
            raw_vy = (snapshot.y_mm - previous_state.y_mm) / dt
            raw_vz = (snapshot.z_mm - previous_state.z_mm) / dt
            alpha = 0.40
            x_velocity = (alpha * raw_vx) + ((1.0 - alpha) * previous_state.x_velocity_mm_s)
            y_velocity = (alpha * raw_vy) + ((1.0 - alpha) * previous_state.y_velocity_mm_s)
            z_velocity = (alpha * raw_vz) + ((1.0 - alpha) * previous_state.z_velocity_mm_s)
            stale_cycles = 0
            stale_duration_sec = 0.0
            last_position_update_wall_time = snapshot.wall_time
        else:
            x_velocity = previous_state.x_velocity_mm_s * 0.92
            y_velocity = previous_state.y_velocity_mm_s * 0.92
            z_velocity = previous_state.z_velocity_mm_s * 0.92
            stale_cycles = previous_state.stale_cycles + 1
            last_position_update_wall_time = previous_state.last_position_update_wall_time
            stale_duration_sec = max(snapshot.wall_time - last_position_update_wall_time, 0.0)

        return StateEstimate(
            x_mm=snapshot.x_mm,
            y_mm=snapshot.y_mm,
            z_mm=snapshot.z_mm,
            yaw_deg=snapshot.yaw_deg,
            x_velocity_mm_s=x_velocity,
            y_velocity_mm_s=y_velocity,
            z_velocity_mm_s=z_velocity,
            yaw_rate_deg_s=snapshot.gyro_yaw,
            pos_fresh=pos_fresh,
            motion_fresh=motion_fresh,
            stale_cycles=stale_cycles,
            stale_duration_sec=stale_duration_sec,
            last_position_update_wall_time=last_position_update_wall_time,
        )

    @staticmethod
    def _command_limit(error_mm: float, base_limit: float, near_limit: float, near_band_mm: float) -> float:
        magnitude = abs(error_mm)
        if magnitude >= near_band_mm:
            return base_limit
        ratio = magnitude / max(near_band_mm, 1.0)
        return near_limit + ((base_limit - near_limit) * math.sqrt(max(ratio, 0.0)))

    def _axis_command(
        self,
        *,
        pid: PIDController,
        config: dict,
        error: float,
        measurement: float,
        measurement_rate: float,
        dt: float,
        sign: float,
        output_limit: float,
        gain_scale: float,
        allow_integral: bool,
    ) -> int:
        output = pid.compute(
            error=error,
            measurement=measurement,
            measurement_rate=measurement_rate,
            dt=dt,
            output_limit=output_limit,
            min_output=float(config.get("min_output", 0.0)),
            min_output_error=float(config.get("min_output_error", 0.0)),
            gain_scale=gain_scale,
            allow_integral=allow_integral,
        )
        return int(round(output * sign))

    def _build_rapid_state(
        self,
        *,
        axis_name: str,
        command_name: str,
        target_mm: float,
        min_rapid_distance_mm: float,
    ) -> AxisRapidState:
        initial_abs_error = abs(target_mm)
        phase = "acquire" if initial_abs_error >= min_rapid_distance_mm else "settle"
        if _sign(target_mm) == 0.0:
            phase = "settle"
        return AxisRapidState(
            axis_name=axis_name,
            command_name=command_name,
            direction=_sign(target_mm),
            initial_abs_error=initial_abs_error,
            min_rapid_distance_mm=min_rapid_distance_mm,
            phase=phase,
            predicted_remaining_mm=initial_abs_error,
            best_abs_error_mm=initial_abs_error,
        )

    def _apply_rapid_axis(
        self,
        *,
        rapid: AxisRapidState,
        pid: PIDController,
        error_mm: float,
        velocity_mm_s: float,
        pos_fresh: bool,
        stale_duration_sec: float,
        elapsed_sec: float,
        position_interval_sec: float,
        brake_accel_mm_s2: float,
        current_cmd: int,
        sign: float,
        max_power: int,
        brake_power: int,
        recovery_power: int = 35,
        release_speed_mm_s: float = BRAKE_RELEASE_SPEED_MM_S,
        allow_fast: bool = True,
    ) -> tuple[int, bool, float]:
        if rapid.direction == 0.0 or rapid.initial_abs_error < rapid.min_rapid_distance_mm:
            rapid.phase = "settle"
            rapid.predicted_remaining_mm = max(0.0, abs(error_mm))
            rapid.stopping_distance_mm = BRAKE_MARGIN_MM
            return current_cmd, False, 0.0

        speed_toward_target = rapid.direction * velocity_mm_s
        if pos_fresh:
            rapid.last_fresh_speed_toward = speed_toward_target

        planning_speed_toward = max(
            speed_toward_target,
            rapid.last_fresh_speed_toward if not pos_fresh else speed_toward_target,
            0.0,
        )
        rapid.predicted_remaining_mm = max(
            0.0,
            (rapid.direction * error_mm) - (planning_speed_toward * stale_duration_sec),
        )
        braking_latency_sec = max(0.12, position_interval_sec, stale_duration_sec)
        rapid.stopping_distance_mm = calculate_stopping_distance_mm(
            speed_mm_s=planning_speed_toward,
            brake_accel_mm_s2=brake_accel_mm_s2,
            sensor_latency_sec=braking_latency_sec,
        )

        if rapid.phase == "acquire":
            current_cmd = 0
            if pos_fresh:
                rapid.phase = "accelerate"
                self.logger.info(
                    "%s軸高速移動: 最大加速を開始します %s=%d",
                    rapid.axis_name,
                    rapid.command_name,
                    max_power,
                )

        if rapid.phase == "accelerate":
            if not allow_fast:
                rapid.phase = "settle"
                pid.reset()
                self.logger.warning("%s軸高速移動: 軸間誤差を優先して高速移動を解除します", rapid.axis_name)
            elif rapid.predicted_remaining_mm <= rapid.stopping_distance_mm:
                rapid.phase = "brake"
                self.logger.info(
                    "%s軸高速移動: 最大制動を開始します remaining=%.1f stop=%.1f speed=%.1f",
                    rapid.axis_name,
                    rapid.predicted_remaining_mm,
                    rapid.stopping_distance_mm,
                    speed_toward_target,
                )
            if rapid.phase == "accelerate":
                current_cmd = int(rapid.direction * sign * max_power)

        suppress_runaway_guard = False
        if rapid.phase == "brake":
            current_cmd = int(-rapid.direction * sign * brake_power)
            if pos_fresh and speed_toward_target <= release_speed_mm_s:
                pid.reset()
                resume_threshold = max(120.0, rapid.stopping_distance_mm * 1.5)
                max_rapid_passes = 1 if rapid.initial_abs_error <= 400.0 else 3
                can_resume = (
                    rapid.rapid_passes < max_rapid_passes
                    and rapid.predicted_remaining_mm > resume_threshold
                    and allow_fast
                )
                if can_resume:
                    rapid.rapid_passes += 1
                    rapid.phase = "accelerate"
                    suppress_runaway_guard = True
                    rapid.guard_grace_cycles = 2
                    rapid.reverse_fresh_count = 0
                    current_cmd = int(rapid.direction * sign * max_power)
                    self.logger.info(
                        "%s軸高速移動: 早期停止のため再加速します pass=%d remaining=%.1f",
                        rapid.axis_name,
                        rapid.rapid_passes,
                        rapid.predicted_remaining_mm,
                    )
                else:
                    rapid.phase = "settle"
                    current_cmd = 0
                    self.logger.info(
                        "%s軸高速移動: 制動完了、PID精密停止へ移行します speed=%.1f err=%.1f",
                        rapid.axis_name,
                        speed_toward_target,
                        error_mm,
                    )

        if rapid.phase == "settle" and abs(error_mm) > 30.0 and speed_toward_target < -10.0:
            if speed_toward_target < -80.0:
                recovery = min(max_power, max(recovery_power, abs(speed_toward_target) * 0.45))
            else:
                recovery = min(recovery_power, max_power)
            current_cmd = int(rapid.direction * sign * recovery)

        abort_after_sample = False
        if pos_fresh and elapsed_sec >= 0.50:
            guard_grace_active = rapid.guard_grace_cycles > 0
            if guard_grace_active:
                rapid.guard_grace_cycles -= 1
                rapid.reverse_fresh_count = 0

            still_closer_than_start = abs(error_mm) <= max(rapid.initial_abs_error * 0.90, rapid.initial_abs_error - 40.0)
            if rapid.phase == "accelerate" and not guard_grace_active:
                if speed_toward_target < -35.0:
                    rapid.reverse_fresh_count += 1
                elif speed_toward_target > -15.0:
                    rapid.reverse_fresh_count = 0

                if rapid.reverse_fresh_count >= 2 and not still_closer_than_start:
                    current_cmd = 0
                    abort_after_sample = True
                    self.logger.error(
                        "%s軸で有効な位置更新2回の逆走を検知したため停止します speed=%.1f err=%.1f",
                        rapid.axis_name,
                        speed_toward_target,
                        error_mm,
                    )

            moving_away = abs(error_mm) - rapid.best_abs_error_mm
            runaway_threshold = max(25.0, rapid.initial_abs_error * 0.05)
            settle_abort_error = max(60.0, rapid.initial_abs_error * 0.20)
            moving_toward_target = error_mm * velocity_mm_s > 0.0
            runaway_guard_active = (
                not suppress_runaway_guard
                and not guard_grace_active
                and not moving_toward_target
                and not still_closer_than_start
                and (rapid.phase != "settle" or abs(error_mm) >= settle_abort_error)
            )
            if abs(error_mm) < rapid.best_abs_error_mm:
                rapid.best_abs_error_mm = abs(error_mm)
                rapid.runaway_count = 0
            elif rapid.initial_abs_error >= 60.0 and runaway_guard_active and moving_away >= runaway_threshold:
                rapid.runaway_count += 1
            else:
                rapid.runaway_count = 0

            runaway_count_limit = 4 if rapid.phase == "settle" else 2
            if rapid.runaway_count >= runaway_count_limit:
                current_cmd = 0
                abort_after_sample = True
                self.logger.error(
                    "%s軸が最良到達点から逆方向へ離脱したため安全停止します err=%.1f best=%.1f runaway=%.1f",
                    rapid.axis_name,
                    error_mm,
                    rapid.best_abs_error_mm,
                    moving_away,
                )

        return current_cmd, abort_after_sample, speed_toward_target

    @staticmethod
    def _scale_horizontal_commands(pitch_cmd: int, roll_cmd: int, max_vector_power: float = 100.0) -> tuple[int, int]:
        magnitude = math.hypot(pitch_cmd, roll_cmd)
        if magnitude <= max_vector_power or magnitude <= 0.0:
            return pitch_cmd, roll_cmd
        scale = max_vector_power / magnitude
        return int(round(pitch_cmd * scale)), int(round(roll_cmd * scale))

    def move_relative_mm(
        self,
        *,
        target_x_mm: float,
        target_y_mm: float = 0.0,
        target_z_mm: float = 0.0,
        target_yaw_deg: float = 0.0,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        loop_period_sec: float = 0.06,
        settle_time_sec: float = 0.5,
        max_pitch: int = DEFAULT_MAX_PITCH,
        brake_pitch: int = DEFAULT_BRAKE_PITCH,
        max_roll: int = DEFAULT_MAX_ROLL,
        brake_roll: int = DEFAULT_BRAKE_ROLL,
        max_throttle: int = DEFAULT_MAX_THROTTLE,
        brake_throttle: int = DEFAULT_BRAKE_THROTTLE,
        brake_accel_mm_s2: float = DEFAULT_BRAKE_ACCEL_MM_S2,
    ) -> FlightSummary | None:
        if self.log_writer is None:
            self.open_log()

        start_snapshot = self.acquire_reference_snapshot()
        start_state = self._compute_state_estimate(start_snapshot, None, None)
        absolute_target_x = start_state.x_mm + target_x_mm
        absolute_target_y = start_state.y_mm + target_y_mm
        absolute_target_z = start_state.z_mm + target_z_mm
        absolute_target_yaw = _wrap_angle_deg(start_state.yaw_deg + target_yaw_deg)

        for controller in (self.pitch_pid, self.roll_pid, self.throttle_pid, self.yaw_pid):
            controller.reset()

        self.logger.info(
            "移動開始 target=(%.1f, %.1f, %.1f, yaw %.1f) start=(%.1f, %.1f, %.1f, yaw %.1f)",
            target_x_mm,
            target_y_mm,
            target_z_mm,
            target_yaw_deg,
            start_state.x_mm,
            start_state.y_mm,
            start_state.z_mm,
            start_state.yaw_deg,
        )

        samples: list[ControlSample] = []
        stable_since = None
        start_perf = time.perf_counter()
        previous_perf = start_perf
        previous_snapshot = start_snapshot
        previous_state = start_state
        rapid_x = self._build_rapid_state(
            axis_name="x",
            command_name="pitch",
            target_mm=target_x_mm,
            min_rapid_distance_mm=500.0,
        )
        rapid_y = self._build_rapid_state(
            axis_name="y",
            command_name="roll",
            target_mm=target_y_mm,
            min_rapid_distance_mm=500.0,
        )
        rapid_z = self._build_rapid_state(
            axis_name="z",
            command_name="throttle",
            target_mm=target_z_mm,
            min_rapid_distance_mm=300.0,
        )
        termination_reason = "timeout"
        position_interval_sec = 0.12
        step = 0

        while True:
            now_perf = time.perf_counter()
            elapsed = now_perf - start_perf
            if elapsed > timeout_sec:
                self.logger.warning("タイムアウトしました")
                termination_reason = "timeout"
                break

            sleep_for = loop_period_sec - (now_perf - previous_perf)
            if sleep_for > 0.0:
                time.sleep(sleep_for)

            snapshot = self.read_snapshot()
            current_perf = time.perf_counter()
            dt = max(current_perf - previous_perf, 0.001)
            state = self._compute_state_estimate(snapshot, previous_snapshot, previous_state)

            if state.pos_fresh:
                fresh_interval = max(snapshot.wall_time - previous_state.last_position_update_wall_time, dt)
                position_interval_sec = max(
                    min(fresh_interval, 0.80),
                    position_interval_sec * 0.85,
                )

            error_x = absolute_target_x - state.x_mm
            error_y = absolute_target_y - state.y_mm
            error_z = absolute_target_z - state.z_mm
            error_yaw = _wrap_angle_deg(absolute_target_yaw - state.yaw_deg)

            gain_scale_pitch = 1.12 if abs(error_x) > 300.0 else 1.0
            gain_scale_roll = 1.08 if abs(error_y) > 120.0 else 1.0
            gain_scale_throttle = 1.12 if abs(error_z) > 60.0 else 1.0
            if abs(error_z) < 18.0:
                gain_scale_throttle *= 0.88

            pitch_limit = self._command_limit(error_x, self.profile["pitch"]["output_limit"], 16.0, 140.0)
            roll_limit = self._command_limit(error_y, self.profile["roll"]["output_limit"], 12.0, 90.0)
            throttle_limit = self._command_limit(error_z, self.profile["throttle"]["output_limit"], 10.0, 80.0)
            yaw_limit = self._command_limit(error_yaw, self.profile["yaw"]["output_limit"], 6.0, 20.0)

            if abs(error_z) < 18.0:
                throttle_limit = min(throttle_limit, 10.0)
            if abs(error_z) < 10.0:
                throttle_limit = min(throttle_limit, 5.0)

            sensor_healthy = state.stale_duration_sec < 0.30
            pitch_cmd = self._axis_command(
                pid=self.pitch_pid,
                config=self.profile["pitch"],
                error=error_x,
                measurement=state.x_mm,
                measurement_rate=state.x_velocity_mm_s,
                dt=dt,
                sign=self.pitch_sign,
                output_limit=pitch_limit,
                gain_scale=gain_scale_pitch,
                allow_integral=(
                    sensor_healthy
                    and 20.0 < abs(error_x) < 250.0
                    and abs(state.x_velocity_mm_s) < 120.0
                ),
            )
            roll_cmd = self._axis_command(
                pid=self.roll_pid,
                config=self.profile["roll"],
                error=error_y,
                measurement=state.y_mm,
                measurement_rate=state.y_velocity_mm_s,
                dt=dt,
                sign=self.roll_sign,
                output_limit=roll_limit,
                gain_scale=gain_scale_roll,
                allow_integral=(
                    sensor_healthy
                    and 18.0 < abs(error_y) < 140.0
                    and abs(state.y_velocity_mm_s) < 90.0
                ),
            )
            throttle_cmd = self._axis_command(
                pid=self.throttle_pid,
                config=self.profile["throttle"],
                error=error_z,
                measurement=state.z_mm,
                measurement_rate=state.z_velocity_mm_s,
                dt=dt,
                sign=self.throttle_sign,
                output_limit=throttle_limit,
                gain_scale=gain_scale_throttle,
                allow_integral=(
                    sensor_healthy
                    and 15.0 < abs(error_z) < 90.0
                    and abs(state.z_velocity_mm_s) < 40.0
                ),
            )
            yaw_cmd = self._axis_command(
                pid=self.yaw_pid,
                config=self.profile["yaw"],
                error=error_yaw,
                measurement=state.yaw_deg,
                measurement_rate=state.yaw_rate_deg_s,
                dt=dt,
                sign=self.yaw_sign,
                output_limit=yaw_limit,
                gain_scale=1.0,
                allow_integral=sensor_healthy and 3.0 < abs(error_yaw) < 45.0,
            )

            pitch_cmd, abort_x, speed_toward_target_x = self._apply_rapid_axis(
                rapid=rapid_x,
                pid=self.pitch_pid,
                error_mm=error_x,
                velocity_mm_s=state.x_velocity_mm_s,
                pos_fresh=state.pos_fresh,
                stale_duration_sec=state.stale_duration_sec,
                elapsed_sec=elapsed,
                position_interval_sec=position_interval_sec,
                brake_accel_mm_s2=brake_accel_mm_s2,
                current_cmd=pitch_cmd,
                sign=self.pitch_sign,
                max_power=max_pitch,
                brake_power=brake_pitch,
                allow_fast=True,
            )
            roll_cmd, abort_y, _speed_toward_target_y = self._apply_rapid_axis(
                rapid=rapid_y,
                pid=self.roll_pid,
                error_mm=error_y,
                velocity_mm_s=state.y_velocity_mm_s,
                pos_fresh=state.pos_fresh,
                stale_duration_sec=state.stale_duration_sec,
                elapsed_sec=elapsed,
                position_interval_sec=position_interval_sec,
                brake_accel_mm_s2=brake_accel_mm_s2,
                current_cmd=roll_cmd,
                sign=self.roll_sign,
                max_power=max_roll,
                brake_power=brake_roll,
                allow_fast=True,
            )
            throttle_cmd, abort_z, _speed_toward_target_z = self._apply_rapid_axis(
                rapid=rapid_z,
                pid=self.throttle_pid,
                error_mm=error_z,
                velocity_mm_s=state.z_velocity_mm_s,
                pos_fresh=state.pos_fresh,
                stale_duration_sec=state.stale_duration_sec,
                elapsed_sec=elapsed,
                position_interval_sec=position_interval_sec,
                brake_accel_mm_s2=brake_accel_mm_s2,
                current_cmd=throttle_cmd,
                sign=self.throttle_sign,
                max_power=max_throttle,
                brake_power=brake_throttle,
                recovery_power=24,
                allow_fast=state.z_mm > 120.0 or rapid_z.direction > 0.0,
            )
            abort_after_sample = abort_x or abort_y or abort_z
            if abort_after_sample:
                termination_reason = "runaway"

            pitch_cmd, roll_cmd = self._scale_horizontal_commands(pitch_cmd, roll_cmd)

            if abs(error_z) < 60.0 and (error_z * state.z_velocity_mm_s) > 220.0:
                brake_cmd = -_sign(state.z_velocity_mm_s) * min(10.0, abs(state.z_velocity_mm_s) * 0.18)
                throttle_cmd = int(round((throttle_cmd + brake_cmd) * 0.5))

            if abs(error_x) < 30.0:
                self.pitch_pid.integral *= 0.82
            if abs(error_y) < 25.0:
                self.roll_pid.integral *= 0.82
            if abs(error_z) < 20.0:
                self.throttle_pid.integral *= 0.90
            if abs(error_z) < 12.0 and abs(state.z_velocity_mm_s) < 18.0:
                throttle_cmd = 0

            if not sensor_healthy:
                if rapid_x.phase not in {"accelerate", "brake"}:
                    pitch_cmd = int(_signed_limit(pitch_cmd, 12.0))
                if rapid_y.phase not in {"accelerate", "brake"}:
                    roll_cmd = int(_signed_limit(roll_cmd, 10.0))
                if rapid_z.phase not in {"accelerate", "brake"}:
                    throttle_cmd = int(_signed_limit(throttle_cmd, 10.0))
                yaw_cmd = int(_signed_limit(yaw_cmd, 5.0))

            sample = ControlSample(
                step=step,
                wall_time=snapshot.wall_time,
                elapsed_sec=elapsed,
                dt_sec=dt,
                pos_fresh=state.pos_fresh,
                motion_fresh=state.motion_fresh,
                stale_cycles=state.stale_cycles,
                stale_duration_sec=state.stale_duration_sec,
                target_x_mm=absolute_target_x,
                target_y_mm=absolute_target_y,
                target_z_mm=absolute_target_z,
                target_yaw_deg=absolute_target_yaw,
                x_mm=state.x_mm,
                y_mm=state.y_mm,
                z_mm=state.z_mm,
                yaw_deg=state.yaw_deg,
                error_x_mm=error_x,
                error_y_mm=error_y,
                error_z_mm=error_z,
                error_yaw_deg=error_yaw,
                x_velocity_mm_s=state.x_velocity_mm_s,
                y_velocity_mm_s=state.y_velocity_mm_s,
                z_velocity_mm_s=state.z_velocity_mm_s,
                yaw_rate_deg_s=state.yaw_rate_deg_s,
                pitch_cmd=pitch_cmd,
                roll_cmd=roll_cmd,
                throttle_cmd=throttle_cmd,
                yaw_cmd=yaw_cmd,
                pitch_limit=pitch_limit,
                roll_limit=roll_limit,
                throttle_limit=throttle_limit,
                yaw_limit=yaw_limit,
                pitch_integral=self.pitch_pid.integral,
                roll_integral=self.roll_pid.integral,
                throttle_integral=self.throttle_pid.integral,
                yaw_integral=self.yaw_pid.integral,
                control_phase=rapid_x.phase,
                predicted_remaining_x_mm=rapid_x.predicted_remaining_mm,
                stopping_distance_x_mm=rapid_x.stopping_distance_mm,
                control_phase_y=rapid_y.phase,
                control_phase_z=rapid_z.phase,
                predicted_remaining_y_mm=rapid_y.predicted_remaining_mm,
                predicted_remaining_z_mm=rapid_z.predicted_remaining_mm,
                stopping_distance_y_mm=rapid_y.stopping_distance_mm,
                stopping_distance_z_mm=rapid_z.stopping_distance_mm,
            )
            self._record_sample(sample)
            samples.append(sample)

            self.logger.info(
                "step=%03d t=%5.2fs phase=(%s,%s,%s) err=(%7.1f,%7.1f,%7.1f,y%5.1f) vel=(%7.1f,%7.1f,%7.1f) cmd[p=%3d r=%3d t=%3d y=%3d] fresh=%s/%s stale=%d %.3fs stop=(%.1f/%.1f,%.1f/%.1f,%.1f/%.1f)",
                step,
                elapsed,
                rapid_x.phase,
                rapid_y.phase,
                rapid_z.phase,
                error_x,
                error_y,
                error_z,
                error_yaw,
                state.x_velocity_mm_s,
                state.y_velocity_mm_s,
                state.z_velocity_mm_s,
                pitch_cmd,
                roll_cmd,
                throttle_cmd,
                yaw_cmd,
                int(state.pos_fresh),
                int(state.motion_fresh),
                state.stale_cycles,
                state.stale_duration_sec,
                rapid_x.predicted_remaining_mm,
                rapid_x.stopping_distance_mm,
                rapid_y.predicted_remaining_mm,
                rapid_y.stopping_distance_mm,
                rapid_z.predicted_remaining_mm,
                rapid_z.stopping_distance_mm,
            )

            self.drone.set_roll(roll_cmd)
            self.drone.set_pitch(pitch_cmd)
            self.drone.set_throttle(throttle_cmd)
            self.drone.set_yaw(yaw_cmd)
            self.drone.move()

            if abort_after_sample:
                break

            stable = is_settled_state(
                error_x_mm=error_x,
                error_y_mm=error_y,
                error_z_mm=error_z,
                error_yaw_deg=error_yaw,
                x_velocity_mm_s=state.x_velocity_mm_s,
                y_velocity_mm_s=state.y_velocity_mm_s,
                z_velocity_mm_s=state.z_velocity_mm_s,
            )
            if stable:
                if stable_since is None:
                    stable_since = elapsed
                elif elapsed - stable_since >= settle_time_sec:
                    self.logger.info("目標位置に安定しました")
                    termination_reason = "settled"
                    break
            else:
                stable_since = None

            if state.stale_duration_sec >= 1.20 and elapsed >= 1.0:
                termination_reason = "sensor_timeout"
                self.logger.error("位置センサ更新が %.2fs 停止したため安全停止します", state.stale_duration_sec)
                break

            previous_perf = current_perf
            previous_snapshot = snapshot
            previous_state = state
            step += 1

        self.stop_and_hover()
        summary = FlightLogAnalyzer.summarize(samples, termination_reason=termination_reason)
        if summary is not None:
            self._record_summary(summary)
            self.profile["signs"] = dict(FIXED_CONTROL_SIGNS)
            self.logger.info(
                "summary result=%s samples=%d duration=%.2fs final_err=(%.1f, %.1f, %.1f, yaw %.1f) stale=%.2f settle=%s brake=(%s,%s)",
                summary.termination_reason,
                summary.samples,
                summary.duration_sec,
                summary.final_error_x_mm,
                summary.final_error_y_mm,
                summary.final_error_z_mm,
                summary.final_error_yaw_deg,
                summary.stale_ratio,
                "n/a" if summary.settling_time_sec is None else f"{summary.settling_time_sec:.2f}s",
                "n/a" if summary.brake_start_error_x_mm is None else f"{summary.brake_start_error_x_mm:.1f}mm",
                "n/a" if summary.brake_start_speed_x_mm_s is None else f"{summary.brake_start_speed_x_mm_s:.1f}mm/s",
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


def analyze_recent_logs(
    limit: int = 8,
    target_distance_mm: float | None = None,
) -> HistoryAnalysis | None:
    analysis = FlightLogAnalyzer.analyze_history(
        LOG_DIR,
        limit=limit,
        target_distance_mm=target_distance_mm,
    )
    if analysis is not None:
        HISTORY_ANALYSIS_PATH.write_text(json.dumps(asdict(analysis), ensure_ascii=False, indent=2), encoding="utf-8")
    return analysis


def run_flight(
    target_distance_mm: float = 0.0,
    lateral_distance_mm: float = 0.0,
    vertical_distance_mm: float = 0.0,
    yaw_deg: float = 0.0,
    timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    loop_period_sec: float = 0.06,
    settle_time_sec: float = 0.5,
    history_limit: int = 8,
    auto_tune: bool = True,
    profile_distance_mm: float | None = None,
    max_pitch: int = DEFAULT_MAX_PITCH,
    brake_pitch: int = DEFAULT_BRAKE_PITCH,
    max_roll: int = DEFAULT_MAX_ROLL,
    brake_roll: int = DEFAULT_BRAKE_ROLL,
    max_throttle: int = DEFAULT_MAX_THROTTLE,
    brake_throttle: int = DEFAULT_BRAKE_THROTTLE,
    brake_accel_mm_s2: float = DEFAULT_BRAKE_ACCEL_MM_S2,
) -> FlightSummary | None:
    logger = _setup_logging()
    history = analyze_recent_logs(limit=history_limit, target_distance_mm=target_distance_mm)
    if history is not None:
        logger.info(
            "最近%d回の中央値 final_err=(%.1f, %.1f, %.1f) duration=%.2fs stale=%.2f",
            history.runs_analyzed,
            history.median_final_error_x_mm,
            history.median_final_error_y_mm,
            history.median_final_error_z_mm,
            history.median_duration_sec,
            history.median_stale_ratio,
        )

    profile_target_distance_mm = target_distance_mm if profile_distance_mm is None else profile_distance_mm
    profile = load_pid_profile(profile_target_distance_mm)
    controller = PrecisionDroneController(logger=logger, pid_profile=profile)

    try:
        with controller.open_log():
            controller.initialize_and_takeoff()
            summary = controller.move_relative_mm(
                target_x_mm=target_distance_mm,
                target_y_mm=lateral_distance_mm,
                target_z_mm=vertical_distance_mm,
                target_yaw_deg=yaw_deg,
                timeout_sec=timeout_sec,
                loop_period_sec=loop_period_sec,
                settle_time_sec=settle_time_sec,
                max_pitch=max_pitch,
                brake_pitch=brake_pitch,
                max_roll=max_roll,
                brake_roll=brake_roll,
                max_throttle=max_throttle,
                brake_throttle=brake_throttle,
                brake_accel_mm_s2=brake_accel_mm_s2,
            )
            safe_to_tune = (
                summary is not None
                and summary.termination_reason in {"settled", "timeout"}
                and abs(summary.final_error_x_mm) <= max(abs(summary.target_x_mm) * 1.25, 50.0)
                and abs(summary.final_error_y_mm) <= max(abs(summary.target_y_mm) * 1.25, 50.0)
                and abs(summary.final_error_z_mm) <= max(abs(summary.target_z_mm) * 1.25, 50.0)
            )
            if safe_to_tune and auto_tune:
                tuned_profile = adjust_pid_profile(controller.profile, summary)
                save_pid_profile_for_distance(tuned_profile, profile_target_distance_mm)
            elif summary is not None and auto_tune:
                logger.warning("異常終了または誤差増大のため PID 自動更新を破棄します")
            return summary
    except KeyboardInterrupt:
        logger.warning("ユーザーにより中断されました")
        return None
    finally:
        controller.safe_land_and_close()


def _non_negative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError("0 以上の値を指定してください")
    return parsed


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("0 より大きい値を指定してください")
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("1 以上の整数を指定してください")
    return parsed


def _control_power(value: str) -> int:
    parsed = int(value)
    if not 1 <= parsed <= 100:
        raise argparse.ArgumentTypeError("1 から 100 の整数を指定してください")
    return parsed


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CoDrone EDU の多軸 PID 制御フライト実行ツール")
    parser.add_argument("--x-mm", type=float, default=0.0, help="前後方向の目標移動量[mm]")
    parser.add_argument("--y-mm", type=float, default=0.0, help="左右方向の目標移動量[mm]")
    parser.add_argument("--z-mm", type=float, default=0.0, help="上下方向の目標移動量[mm]")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="ヨー方向の目標回転量[deg]")
    parser.add_argument("--timeout-sec", type=_positive_float, default=DEFAULT_TIMEOUT_SEC, help="全体タイムアウト[sec]")
    parser.add_argument("--loop-period-sec", type=_positive_float, default=0.06, help="制御周期[sec]")
    parser.add_argument("--settle-time-sec", type=_positive_float, default=0.5, help="整定判定に必要な連続安定時間[sec]")
    parser.add_argument("--history-limit", type=_positive_int, default=8, help="履歴解析に使う直近ログ数")
    parser.add_argument("--max-pitch", type=_control_power, default=DEFAULT_MAX_PITCH, help="高速移動時の最大加速ピッチ[1-100]")
    parser.add_argument("--brake-pitch", type=_control_power, default=DEFAULT_BRAKE_PITCH, help="高速移動時の最大制動ピッチ[1-100]")
    parser.add_argument("--max-roll", type=_control_power, default=DEFAULT_MAX_ROLL, help="高速移動時の最大加速ロール[1-100]")
    parser.add_argument("--brake-roll", type=_control_power, default=DEFAULT_BRAKE_ROLL, help="高速移動時の最大制動ロール[1-100]")
    parser.add_argument("--max-throttle", type=_control_power, default=DEFAULT_MAX_THROTTLE, help="高速移動時の最大加速スロットル[1-100]")
    parser.add_argument("--brake-throttle", type=_control_power, default=DEFAULT_BRAKE_THROTTLE, help="高速移動時の最大制動スロットル[1-100]")
    parser.add_argument(
        "--brake-accel-mm-s2",
        type=_positive_float,
        default=DEFAULT_BRAKE_ACCEL_MM_S2,
        help="制動距離予測に使う減速度[mm/s^2]",
    )
    parser.add_argument(
        "--profile-distance-mm",
        type=_non_negative_float,
        default=None,
        help="PID プロファイル選択用の距離[mm]。省略時は x 目標距離を使う",
    )
    parser.add_argument("--print-profile", action="store_true", help="実行前に使用プロファイルを表示する")
    parser.add_argument("--analyze-only", action="store_true", help="飛行せず履歴解析だけ実行する")
    parser.add_argument("--no-auto-tune", action="store_true", help="飛行後の自動 PID 更新を無効化する")
    return parser


def _summary_to_pretty_json(summary: FlightSummary | HistoryAnalysis | dict | None) -> str:
    if summary is None:
        return "null"
    payload = asdict(summary) if hasattr(summary, "__dataclass_fields__") else summary
    return json.dumps(payload, ensure_ascii=False, indent=2)


def _command_from_args(args: argparse.Namespace) -> FlightCommand:
    return FlightCommand(
        target_x_mm=args.x_mm,
        target_y_mm=args.y_mm,
        target_z_mm=args.z_mm,
        target_yaw_deg=args.yaw_deg,
        timeout_sec=args.timeout_sec,
        loop_period_sec=args.loop_period_sec,
        settle_time_sec=args.settle_time_sec,
        history_limit=args.history_limit,
        auto_tune=not args.no_auto_tune,
        profile_distance_mm=args.profile_distance_mm,
        max_pitch=args.max_pitch,
        brake_pitch=args.brake_pitch,
        max_roll=args.max_roll,
        brake_roll=args.brake_roll,
        max_throttle=args.max_throttle,
        brake_throttle=args.brake_throttle,
        brake_accel_mm_s2=args.brake_accel_mm_s2,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    command = _command_from_args(args)

    if args.print_profile:
        print(_summary_to_pretty_json(load_pid_profile(command.profile_target_distance_mm)))

    if args.analyze_only:
        analysis = analyze_recent_logs(limit=command.history_limit)
        print(_summary_to_pretty_json(analysis))
        return 0

    try:
        summary = run_flight(
            target_distance_mm=command.target_x_mm,
            lateral_distance_mm=command.target_y_mm,
            vertical_distance_mm=command.target_z_mm,
            yaw_deg=command.target_yaw_deg,
            timeout_sec=command.timeout_sec,
            loop_period_sec=command.loop_period_sec,
            settle_time_sec=command.settle_time_sec,
            history_limit=command.history_limit,
            auto_tune=command.auto_tune,
            profile_distance_mm=command.profile_distance_mm,
            max_pitch=command.max_pitch,
            brake_pitch=command.brake_pitch,
            max_roll=command.max_roll,
            brake_roll=command.brake_roll,
            max_throttle=command.max_throttle,
            brake_throttle=command.brake_throttle,
            brake_accel_mm_s2=command.brake_accel_mm_s2,
        )
    except Exception as exc:
        print(f"flight failed: {exc}", file=sys.stderr)
        return 1

    print(_summary_to_pretty_json(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
