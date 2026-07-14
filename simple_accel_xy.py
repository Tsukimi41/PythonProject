from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import fmean


LOG_DIR = Path(__file__).resolve().parent / "accel_position_logs"
ACCEL_RAW_TO_CM_S2 = 10.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def rotate_body_to_world(forward: float, left: float, yaw_deg: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = forward * cos_yaw - left * sin_yaw
    world_y = forward * sin_yaw + left * cos_yaw
    return world_x, world_y


def rotate_world_to_body(world_x: float, world_y: float, yaw_deg: float) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    forward = world_x * cos_yaw + world_y * sin_yaw
    left = -world_x * sin_yaw + world_y * cos_yaw
    return forward, left


@dataclass
class Config:
    target_x_cm: float
    target_y_cm: float
    timeout_sec: float = 12.0
    command_dt_sec: float = 0.05
    calibrate_sec: float = 1.5
    max_power: int = 35
    min_power: int = 18
    tolerance_cm: float = 8.0
    accel_deadband_cm_s2: float = 12.0
    accel_lpf_alpha: float = 0.30
    velocity_leak_per_sec: float = 0.18
    zupt_accel_cm_s2: float = 6.0
    zupt_gyro_deg_s: float = 12.0
    zupt_command_power: int = 5
    accel_x_sign: float = -1.0
    accel_y_sign: float = 1.0
    pitch_sign: float = 1.0
    roll_sign: float = -1.0
    kp: float = 0.85
    kd: float = 0.20


@dataclass
class Motion:
    raw_ax: float
    raw_ay: float
    raw_az: float
    gyro_roll: float
    gyro_pitch: float
    gyro_yaw: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    bottom_range_cm: float


@dataclass
class Bias:
    ax: float
    ay: float
    az: float
    yaw_deg: float


@dataclass
class Estimate:
    x_cm: float = 0.0
    y_cm: float = 0.0
    vx_cm_s: float = 0.0
    vy_cm_s: float = 0.0
    filtered_ax_body_cm_s2: float = 0.0
    filtered_ay_body_cm_s2: float = 0.0


@dataclass
class Sample:
    elapsed_sec: float
    raw_ax: float
    raw_ay: float
    raw_az: float
    gyro_roll: float
    gyro_pitch: float
    gyro_yaw: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    yaw_from_start_deg: float
    bottom_range_cm: float
    ax_body_cm_s2: float
    ay_body_cm_s2: float
    ax_world_cm_s2: float
    ay_world_cm_s2: float
    x_cm: float
    y_cm: float
    vx_cm_s: float
    vy_cm_s: float
    target_x_cm: float
    target_y_cm: float
    error_x_cm: float
    error_y_cm: float
    pitch: int
    roll: int
    zupt_applied: bool


def read_motion(drone) -> Motion:
    motion = drone.get_motion_data()
    return Motion(
        raw_ax=float(motion[1]),
        raw_ay=float(motion[2]),
        raw_az=float(motion[3]),
        gyro_roll=float(motion[4]),
        gyro_pitch=float(motion[5]),
        gyro_yaw=float(motion[6]),
        roll_deg=float(motion[7]),
        pitch_deg=float(motion[8]),
        yaw_deg=float(motion[9]),
        bottom_range_cm=float(drone.get_bottom_range("cm")),
    )


def calibrate(drone, config: Config) -> Bias:
    ax_values: list[float] = []
    ay_values: list[float] = []
    az_values: list[float] = []
    yaw_values: list[float] = []
    end_time = time.perf_counter() + config.calibrate_sec
    while time.perf_counter() < end_time:
        motion = read_motion(drone)
        ax_values.append(motion.raw_ax)
        ay_values.append(motion.raw_ay)
        az_values.append(motion.raw_az)
        yaw_values.append(motion.yaw_deg)
        time.sleep(config.command_dt_sec)
    return Bias(
        ax=fmean(ax_values),
        ay=fmean(ay_values),
        az=fmean(az_values),
        yaw_deg=fmean(yaw_values),
    )


def remove_deadband(value: float, deadband: float) -> float:
    if abs(value) <= deadband:
        return 0.0
    return value - deadband if value > 0.0 else value + deadband


def low_pass(previous: float, current: float, alpha: float) -> float:
    return alpha * current + (1.0 - alpha) * previous


def leak_velocity(value: float, leak_per_sec: float, dt: float) -> float:
    return value * math.exp(-leak_per_sec * dt)


def update_estimate(
    estimate: Estimate,
    motion: Motion,
    bias: Bias,
    config: Config,
    dt: float,
) -> tuple[Estimate, float, float, float, float]:
    ax_body = (motion.raw_ax - bias.ax) * ACCEL_RAW_TO_CM_S2 * config.accel_x_sign
    ay_body = (motion.raw_ay - bias.ay) * ACCEL_RAW_TO_CM_S2 * config.accel_y_sign

    filtered_ax_body = low_pass(
        estimate.filtered_ax_body_cm_s2,
        ax_body,
        config.accel_lpf_alpha,
    )
    filtered_ay_body = low_pass(
        estimate.filtered_ay_body_cm_s2,
        ay_body,
        config.accel_lpf_alpha,
    )

    filtered_ax_body = remove_deadband(filtered_ax_body, config.accel_deadband_cm_s2)
    filtered_ay_body = remove_deadband(filtered_ay_body, config.accel_deadband_cm_s2)

    yaw_from_start = wrap_angle_deg(motion.yaw_deg - bias.yaw_deg)
    ax_world, ay_world = rotate_body_to_world(filtered_ax_body, filtered_ay_body, yaw_from_start)

    next_vx = leak_velocity(estimate.vx_cm_s + ax_world * dt, config.velocity_leak_per_sec, dt)
    next_vy = leak_velocity(estimate.vy_cm_s + ay_world * dt, config.velocity_leak_per_sec, dt)
    next_x = estimate.x_cm + (estimate.vx_cm_s + next_vx) * 0.5 * dt
    next_y = estimate.y_cm + (estimate.vy_cm_s + next_vy) * 0.5 * dt

    return (
        Estimate(
            x_cm=next_x,
            y_cm=next_y,
            vx_cm_s=next_vx,
            vy_cm_s=next_vy,
            filtered_ax_body_cm_s2=filtered_ax_body,
            filtered_ay_body_cm_s2=filtered_ay_body,
        ),
        filtered_ax_body,
        filtered_ay_body,
        ax_world,
        ay_world,
    )


def axis_power(error_cm: float, velocity_cm_s: float, config: Config) -> int:
    power = config.kp * error_cm - config.kd * velocity_cm_s
    if abs(error_cm) > config.tolerance_cm and abs(power) < config.min_power:
        power = math.copysign(config.min_power, power if power != 0.0 else error_cm)
    return int(round(clamp(power, -config.max_power, config.max_power)))


def command_from_estimate(
    estimate: Estimate,
    motion: Motion,
    bias: Bias,
    config: Config,
) -> tuple[int, int, float, float]:
    error_x = config.target_x_cm - estimate.x_cm
    error_y = config.target_y_cm - estimate.y_cm
    yaw_from_start = wrap_angle_deg(motion.yaw_deg - bias.yaw_deg)
    body_error_forward, body_error_left = rotate_world_to_body(error_x, error_y, yaw_from_start)
    body_velocity_forward, body_velocity_left = rotate_world_to_body(
        estimate.vx_cm_s,
        estimate.vy_cm_s,
        yaw_from_start,
    )

    pitch = axis_power(body_error_forward, body_velocity_forward, config) * config.pitch_sign
    roll = axis_power(body_error_left, body_velocity_left, config) * config.roll_sign
    return int(pitch), int(roll), error_x, error_y


def apply_zupt_if_stopped(
    estimate: Estimate,
    ax_world: float,
    ay_world: float,
    motion: Motion,
    pitch: int,
    roll: int,
    config: Config,
) -> tuple[Estimate, bool]:
    accel_is_small = abs(ax_world) < config.zupt_accel_cm_s2 and abs(ay_world) < config.zupt_accel_cm_s2
    gyro_is_small = (
        abs(motion.gyro_roll) < config.zupt_gyro_deg_s
        and abs(motion.gyro_pitch) < config.zupt_gyro_deg_s
        and abs(motion.gyro_yaw) < config.zupt_gyro_deg_s
    )
    command_is_small = abs(pitch) < config.zupt_command_power and abs(roll) < config.zupt_command_power

    if accel_is_small and gyro_is_small and command_is_small:
        return (
            Estimate(
                x_cm=estimate.x_cm,
                y_cm=estimate.y_cm,
                vx_cm_s=0.0,
                vy_cm_s=0.0,
                filtered_ax_body_cm_s2=estimate.filtered_ax_body_cm_s2,
                filtered_ay_body_cm_s2=estimate.filtered_ay_body_cm_s2,
            ),
            True,
        )
    return estimate, False


def make_log_writer() -> tuple[csv.DictWriter, object, Path]:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = LOG_DIR / f"inertial_xy_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=list(Sample.__dataclass_fields__.keys()))
    writer.writeheader()
    return writer, handle, path


def move_xy(drone, config: Config) -> Path:
    writer, log_handle, log_path = make_log_writer()
    estimate = Estimate()

    try:
        print("takeoff")
        drone.takeoff()
        time.sleep(1.0)

        print("calibrate")
        bias = calibrate(drone, config)

        last_time = time.perf_counter()
        control_start = last_time

        while True:
            now = time.perf_counter()
            elapsed = now - control_start
            if elapsed >= config.timeout_sec:
                print("timeout")
                break

            dt = clamp(now - last_time, 0.001, 0.20)
            last_time = now
            motion = read_motion(drone)
            estimate, ax_body, ay_body, ax_world, ay_world = update_estimate(
                estimate,
                motion,
                bias,
                config,
                dt,
            )
            pitch, roll, error_x, error_y = command_from_estimate(estimate, motion, bias, config)
            estimate, zupt_applied = apply_zupt_if_stopped(
                estimate,
                ax_world,
                ay_world,
                motion,
                pitch,
                roll,
                config,
            )

            drone.set_pitch(pitch)
            drone.set_roll(roll)
            drone.set_yaw(0)
            drone.set_throttle(0)
            drone.move(config.command_dt_sec)

            yaw_from_start = wrap_angle_deg(motion.yaw_deg - bias.yaw_deg)
            writer.writerow(asdict(Sample(
                elapsed_sec=elapsed,
                raw_ax=motion.raw_ax,
                raw_ay=motion.raw_ay,
                raw_az=motion.raw_az,
                gyro_roll=motion.gyro_roll,
                gyro_pitch=motion.gyro_pitch,
                gyro_yaw=motion.gyro_yaw,
                roll_deg=motion.roll_deg,
                pitch_deg=motion.pitch_deg,
                yaw_deg=motion.yaw_deg,
                yaw_from_start_deg=yaw_from_start,
                bottom_range_cm=motion.bottom_range_cm,
                ax_body_cm_s2=ax_body,
                ay_body_cm_s2=ay_body,
                ax_world_cm_s2=ax_world,
                ay_world_cm_s2=ay_world,
                x_cm=estimate.x_cm,
                y_cm=estimate.y_cm,
                vx_cm_s=estimate.vx_cm_s,
                vy_cm_s=estimate.vy_cm_s,
                target_x_cm=config.target_x_cm,
                target_y_cm=config.target_y_cm,
                error_x_cm=error_x,
                error_y_cm=error_y,
                pitch=pitch,
                roll=roll,
                zupt_applied=zupt_applied,
            )))

            print(
                f"x={estimate.x_cm:7.1f} y={estimate.y_cm:7.1f} "
                f"vx={estimate.vx_cm_s:7.1f} vy={estimate.vy_cm_s:7.1f} "
                f"ex={error_x:7.1f} ey={error_y:7.1f} "
                f"yaw={yaw_from_start:6.1f} pitch={pitch:4d} roll={roll:4d}"
            )

            if abs(error_x) <= config.tolerance_cm and abs(error_y) <= config.tolerance_cm:
                print("target reached by estimate")
                break

        return log_path
    finally:
        drone.set_pitch(0)
        drone.set_roll(0)
        drone.set_yaw(0)
        drone.set_throttle(0)
        drone.move(0.05)
        drone.land()
        log_handle.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CoDrone EDU inertial x/y move without optical flow.")
    parser.add_argument("--x-cm", type=float, required=True)
    parser.add_argument("--y-cm", type=float, required=True)
    parser.add_argument("--timeout-sec", type=float, default=12.0)
    parser.add_argument("--max-power", type=int, default=35)
    parser.add_argument("--min-power", type=int, default=18)
    parser.add_argument("--tolerance-cm", type=float, default=8.0)
    parser.add_argument("--accel-x-sign", type=float, choices=[-1.0, 1.0], default=-1.0)
    parser.add_argument("--accel-y-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--pitch-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--roll-sign", type=float, choices=[-1.0, 1.0], default=-1.0)
    return parser


def main() -> int:
    from codrone_edu.drone import Drone

    args = build_parser().parse_args()
    config = Config(
        target_x_cm=args.x_cm,
        target_y_cm=args.y_cm,
        timeout_sec=args.timeout_sec,
        max_power=args.max_power,
        min_power=args.min_power,
        tolerance_cm=args.tolerance_cm,
        accel_x_sign=args.accel_x_sign,
        accel_y_sign=args.accel_y_sign,
        pitch_sign=args.pitch_sign,
        roll_sign=args.roll_sign,
    )

    drone = Drone()
    try:
        drone.pair()
        log_path = move_xy(drone, config)
        print(f"log: {log_path}")
        return 0
    finally:
        drone.close()


if __name__ == "__main__":
    raise SystemExit(main())
