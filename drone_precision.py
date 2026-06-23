from __future__ import annotations

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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _sign(value: float) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 0.0


def default_pid_profile() -> dict:
    return {
        "pitch": {"kp": 0.110, "ki": 0.00040, "kd": 0.030, "output_limit": 55.0, "integral_limit": 3000.0},
        "roll": {"kp": 0.090, "ki": 0.00030, "kd": 0.040, "output_limit": 40.0, "integral_limit": 2200.0},
        "throttle": {"kp": 0.135, "ki": 0.00450, "kd": 0.040, "output_limit": 32.0, "integral_limit": 1500.0},
        "yaw": {"kp": 0.500, "ki": 0.00000, "kd": 0.080, "output_limit": 16.0, "integral_limit": 120.0},
    }


def load_pid_profile() -> dict:
    profile = default_pid_profile()
    if not PID_PROFILE_PATH.exists():
        return profile

    try:
        raw = json.loads(PID_PROFILE_PATH.read_text(encoding="utf-8"))
    except Exception:
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
    return profile


def save_pid_profile(profile: dict) -> None:
    PID_PROFILE_PATH.write_text(json.dumps(profile, ensure_ascii=False, indent=2), encoding="utf-8")


def adjust_pid_profile(profile: dict, summary: FlightSummary | None) -> dict:
    if summary is None:
        return profile

    next_profile = json.loads(json.dumps(profile))
    clamp = lambda v, lo, hi: max(lo, min(hi, v))

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

    if abs(summary.final_error_z_mm) > 15.0:
        throttle["kp"] *= 1.05
        throttle["kd"] *= 0.98

    if abs(summary.final_error_yaw_deg) > 6.0:
        yaw["kp"] *= 1.04
        yaw["kd"] *= 1.02

    pitch["kp"] = clamp(pitch["kp"], 0.05, 0.22)
    pitch["ki"] = clamp(pitch["ki"], 0.0, 0.004)
    pitch["kd"] = clamp(pitch["kd"], 0.01, 0.08)

    roll["kp"] = clamp(roll["kp"], 0.04, 0.18)
    roll["ki"] = clamp(roll["ki"], 0.0, 0.003)
    roll["kd"] = clamp(roll["kd"], 0.02, 0.08)

    throttle["kp"] = clamp(throttle["kp"], 0.08, 0.28)
    throttle["ki"] = clamp(throttle["ki"], 0.0, 0.01)
    throttle["kd"] = clamp(throttle["kd"], 0.02, 0.08)

    yaw["kp"] = clamp(yaw["kp"], 0.2, 0.9)
    yaw["ki"] = clamp(yaw["ki"], 0.0, 0.01)
    yaw["kd"] = clamp(yaw["kd"], 0.02, 0.16)

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
        self.integral = _clamp(self.integral + error * dt, self.integral_limit)

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
        output = _clamp(output, self.output_limit)

        if self.min_output > 0.0 and abs(error) > 1e-6 and abs(output) < self.min_output:
            output = _sign(error) * self.min_output

        return _clamp(output, self.output_limit)


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
            stable = (
                abs(sample.error_x_mm) <= 10.0
                and abs(sample.error_y_mm) <= 15.0
                and abs(sample.error_z_mm) <= 15.0
                and abs(sample.error_yaw_deg) <= 6.0
                and abs(sample.x_velocity_mm_s) <= 25.0
                and abs(sample.y_velocity_mm_s) <= 25.0
                and abs(sample.z_velocity_mm_s) <= 25.0
            )
            if stable:
                if stable_since is None:
                    stable_since = sample.elapsed_sec
                elif sample.elapsed_sec - stable_since >= 0.8:
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

    def _command_axis(
        self,
        pid: PIDController,
        target: float,
        current: float,
        dt: float,
        min_output: float,
        invert: bool = False,
    ) -> int:
        pid.min_output = min_output
        command = pid.compute(target=target, current=current, dt=dt)
        if invert:
            command = -command
        return int(round(command))

    def move_relative_mm(
        self,
        target_distance_mm: float,
        timeout_sec: float = 20.0,
        loop_period_sec: float = 0.05,
        settle_tolerance_mm: float = 10.0,
        settle_time_sec: float = 0.8,
    ) -> FlightSummary | None:
        if self.log_writer is None:
            self.open_log()

        start = self.read_snapshot()
        target_x_mm = start.x_mm + target_distance_mm
        target_y_mm = start.y_mm
        target_z_mm = start.z_mm
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

        self.logger.info(
            "移動開始 target=%.1fmm start=(%.1f, %.1f, %.1f) yaw=%.1f",
            target_distance_mm,
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

            abs_x_error = abs(error_x)
            if abs_x_error > max(250.0, abs(target_distance_mm) * 0.50):
                self.pitch_pid.set_gains(0.120, 0.00040, 0.025)
                pitch_limit = 55.0
                pitch_min = 16.0
            elif abs_x_error > max(120.0, abs(target_distance_mm) * 0.24):
                self.pitch_pid.set_gains(0.110, 0.00045, 0.024)
                pitch_limit = 48.0
                pitch_min = 14.0
            elif abs_x_error > max(50.0, abs(target_distance_mm) * 0.10):
                self.pitch_pid.set_gains(0.100, 0.00050, 0.020)
                pitch_limit = 36.0
                pitch_min = 12.0
            elif abs_x_error > 20.0:
                self.pitch_pid.set_gains(0.085, 0.00055, 0.018)
                pitch_limit = 24.0
                pitch_min = 8.0
            else:
                self.pitch_pid.set_gains(0.060, 0.00050, 0.015)
                pitch_limit = 14.0
                pitch_min = 0.0
            if abs_x_error > 40.0 and abs(x_velocity) < 25.0:
                pitch_min = max(pitch_min, 12.0)
                pitch_limit = max(pitch_limit, 30.0)
            self.pitch_pid.output_limit = pitch_limit

            self.roll_pid.output_limit = 30.0 if abs(error_y) > 20.0 else 16.0
            self.throttle_pid.output_limit = 24.0 if abs(error_z) > 18.0 else 12.0
            self.yaw_pid.output_limit = 10.0 if abs(error_yaw) > 8.0 else 6.0

            pitch_cmd = self._command_axis(self.pitch_pid, target_x_mm, snapshot.x_mm, dt, pitch_min)
            # CoDrone EDU の roll は、ログ上の y 軸と向きが逆だったため反転する。
            roll_cmd = self._command_axis(self.roll_pid, target_y_mm, snapshot.y_mm, dt, 0.0, invert=True)
            throttle_cmd = self._command_axis(self.throttle_pid, target_z_mm, snapshot.z_mm, dt, 0.0)
            yaw_cmd = self._command_axis(self.yaw_pid, target_yaw_deg, snapshot.yaw_deg, dt, 0.0)

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

            stable = (
                abs(error_x) <= settle_tolerance_mm
                and abs(error_y) <= 15.0
                and abs(error_z) <= 15.0
                and abs(error_yaw) <= 6.0
                and abs(x_velocity) <= 25.0
                and abs(y_velocity) <= 25.0
                and abs(z_velocity) <= 25.0
            )

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


def run_flight(target_distance_mm: float = 500.0, timeout_sec: float = 20.0) -> FlightSummary | None:
    logger = _setup_logging()
    profile = load_pid_profile()
    controller = PrecisionDroneController(logger=logger, pid_profile=profile)
    controller.open_log()

    try:
        controller.initialize_and_takeoff()
        summary = controller.move_relative_mm(target_distance_mm=target_distance_mm, timeout_sec=timeout_sec)
        if summary is not None:
            save_pid_profile(adjust_pid_profile(profile, summary))
        return summary
    except KeyboardInterrupt:
        logger.warning("ユーザーにより中断されました")
        return None
    finally:
        controller.safe_land_and_close()


def main() -> None:
    run_flight(target_distance_mm=500.0, timeout_sec=20.0)
