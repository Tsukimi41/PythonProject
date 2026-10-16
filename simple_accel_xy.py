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
    calibrate_sec: float = 2.0
    post_takeoff_wait_sec: float = 0.25
    max_power: int = 24
    min_power: int = 6
    tolerance_cm: float = 5.0
    final_slow_radius_cm: float = 18.0
    final_max_power: int = 10
    max_speed_cm_s: float = 28.0
    final_max_speed_cm_s: float = 10.0
    accel_deadband_cm_s2: float = 12.0
    accel_lpf_alpha: float = 0.30
    velocity_leak_per_sec: float = 0.75
    zupt_accel_cm_s2: float = 6.0
    zupt_gyro_deg_s: float = 12.0
    zupt_command_power: int = 5
    accel_source: str = "attitude"
    imu_weight: float = 0.25
    attitude_accel_gain: float = 1.0
    accel_x_sign: float = -1.0
    accel_y_sign: float = 1.0
    pitch_sign: float = 1.0
    roll_sign: float = -1.0
    position_to_speed_gain: float = 1.15
    velocity_gain: float = 0.45
    yaw_kp: float = 1.0
    yaw_limit: int = 12


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
    roll_deg: float
    pitch_deg: float
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
    imu_ax_body_cm_s2: float
    imu_ay_body_cm_s2: float
    attitude_ax_body_cm_s2: float
    attitude_ay_body_cm_s2: float
    used_ax_body_cm_s2: float
    used_ay_body_cm_s2: float
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
    desired_vx_cm_s: float
    desired_vy_cm_s: float
    pitch: int
    roll: int
    yaw: int
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
    roll_values: list[float] = []
    pitch_values: list[float] = []
    end_time = time.perf_counter() + config.calibrate_sec
    while time.perf_counter() < end_time:
        motion = read_motion(drone)
        ax_values.append(motion.raw_ax)
        ay_values.append(motion.raw_ay)
        az_values.append(motion.raw_az)
        roll_values.append(motion.roll_deg)
        pitch_values.append(motion.pitch_deg)
        yaw_values.append(motion.yaw_deg)
        time.sleep(config.command_dt_sec)
    return Bias(
        ax=fmean(ax_values),
        ay=fmean(ay_values),
        az=fmean(az_values),
        roll_deg=fmean(roll_values),
        pitch_deg=fmean(pitch_values),
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
) -> tuple[Estimate, float, float, float, float, float, float, float, float]:
    imu_ax_body = (motion.raw_ax - bias.ax) * ACCEL_RAW_TO_CM_S2 * config.accel_x_sign
    imu_ay_body = (motion.raw_ay - bias.ay) * ACCEL_RAW_TO_CM_S2 * config.accel_y_sign
    attitude_ax_body = (
        math.sin(math.radians(motion.pitch_deg - bias.pitch_deg))
        * 980.0
        * config.attitude_accel_gain
    )
    attitude_ay_body = (
        -math.sin(math.radians(motion.roll_deg - bias.roll_deg))
        * 980.0
        * config.attitude_accel_gain
    )

    if config.accel_source == "imu":
        ax_body = imu_ax_body
        ay_body = imu_ay_body
    elif config.accel_source == "blend":
        imu_weight = clamp(config.imu_weight, 0.0, 1.0)
        ax_body = imu_weight * imu_ax_body + (1.0 - imu_weight) * attitude_ax_body
        ay_body = imu_weight * imu_ay_body + (1.0 - imu_weight) * attitude_ay_body
    else:
        ax_body = attitude_ax_body
        ay_body = attitude_ay_body

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
        imu_ax_body,
        imu_ay_body,
        attitude_ax_body,
        attitude_ay_body,
    )


def desired_speed_for_error(error_cm: float, config: Config) -> float:
    abs_error = abs(error_cm)
    if abs_error <= config.tolerance_cm:
        return 0.0
    speed_limit = config.final_max_speed_cm_s if abs_error <= config.final_slow_radius_cm else config.max_speed_cm_s
    desired = config.position_to_speed_gain * error_cm
    return clamp(desired, -speed_limit, speed_limit)


def axis_power_from_velocity(
    error_cm: float,
    velocity_cm_s: float,
    desired_velocity_cm_s: float,
    config: Config,
) -> int:
    abs_error = abs(error_cm)
    power_limit = config.final_max_power if abs_error <= config.final_slow_radius_cm else config.max_power
    power = config.velocity_gain * (desired_velocity_cm_s - velocity_cm_s)
    if abs_error > config.tolerance_cm and abs(power) < config.min_power:
        power = math.copysign(config.min_power, desired_velocity_cm_s if desired_velocity_cm_s != 0.0 else error_cm)
    return int(round(clamp(power, -power_limit, power_limit)))


def command_from_estimate(
    estimate: Estimate,
    motion: Motion,
    bias: Bias,
    config: Config,
) -> tuple[int, int, int, float, float, float, float]:
    error_x = config.target_x_cm - estimate.x_cm
    error_y = config.target_y_cm - estimate.y_cm
    yaw_from_start = wrap_angle_deg(motion.yaw_deg - bias.yaw_deg)
    body_error_forward, body_error_left = rotate_world_to_body(error_x, error_y, yaw_from_start)
    body_velocity_forward, body_velocity_left = rotate_world_to_body(
        estimate.vx_cm_s,
        estimate.vy_cm_s,
        yaw_from_start,
    )
    desired_vx = desired_speed_for_error(error_x, config)
    desired_vy = desired_speed_for_error(error_y, config)
    body_desired_vx, body_desired_vy = rotate_world_to_body(desired_vx, desired_vy, yaw_from_start)

    pitch = axis_power_from_velocity(
        body_error_forward,
        body_velocity_forward,
        body_desired_vx,
        config,
    ) * config.pitch_sign
    roll = axis_power_from_velocity(
        body_error_left,
        body_velocity_left,
        body_desired_vy,
        config,
    ) * config.roll_sign
    yaw = int(round(clamp(-config.yaw_kp * yaw_from_start, -config.yaw_limit, config.yaw_limit)))
    return int(pitch), int(roll), yaw, error_x, error_y, desired_vx, desired_vy


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
        print("reset_gyro")
        drone.reset_gyro()
        time.sleep(0.2)

        print("calibrate")
        bias = calibrate(drone, config)

        print("takeoff")
        drone.takeoff()
        time.sleep(config.post_takeoff_wait_sec)

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
            (
                estimate,
                used_ax_body,
                used_ay_body,
                ax_world,
                ay_world,
                imu_ax_body,
                imu_ay_body,
                attitude_ax_body,
                attitude_ay_body,
            ) = update_estimate(
                estimate,
                motion,
                bias,
                config,
                dt,
            )
            pitch, roll, yaw, error_x, error_y, desired_vx, desired_vy = command_from_estimate(
                estimate,
                motion,
                bias,
                config,
            )
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
            drone.set_yaw(yaw)
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
                imu_ax_body_cm_s2=imu_ax_body,
                imu_ay_body_cm_s2=imu_ay_body,
                attitude_ax_body_cm_s2=attitude_ax_body,
                attitude_ay_body_cm_s2=attitude_ay_body,
                used_ax_body_cm_s2=used_ax_body,
                used_ay_body_cm_s2=used_ay_body,
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
                desired_vx_cm_s=desired_vx,
                desired_vy_cm_s=desired_vy,
                pitch=pitch,
                roll=roll,
                yaw=yaw,
                zupt_applied=zupt_applied,
            )))

            print(
                f"x={estimate.x_cm:7.1f} y={estimate.y_cm:7.1f} "
                f"vx={estimate.vx_cm_s:7.1f}/{desired_vx:5.1f} "
                f"vy={estimate.vy_cm_s:7.1f}/{desired_vy:5.1f} "
                f"ex={error_x:7.1f} ey={error_y:7.1f} "
                f"yaw={yaw_from_start:6.1f} cmd=({pitch:4d},{roll:4d},{yaw:4d})"
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
    parser.add_argument("--max-power", type=int, default=24)
    parser.add_argument("--min-power", type=int, default=6)
    parser.add_argument("--tolerance-cm", type=float, default=5.0)
    parser.add_argument("--calibrate-sec", type=float, default=2.0)
    parser.add_argument("--post-takeoff-wait-sec", type=float, default=0.25)
    parser.add_argument("--final-slow-radius-cm", type=float, default=18.0)
    parser.add_argument("--final-max-power", type=int, default=10)
    parser.add_argument("--max-speed-cm-s", type=float, default=28.0)
    parser.add_argument("--final-max-speed-cm-s", type=float, default=10.0)
    parser.add_argument("--position-to-speed-gain", type=float, default=1.15)
    parser.add_argument("--velocity-gain", type=float, default=0.45)
    parser.add_argument("--yaw-kp", type=float, default=1.0)
    parser.add_argument("--yaw-limit", type=int, default=12)
    parser.add_argument("--accel-source", choices=["attitude", "imu", "blend"], default="attitude")
    parser.add_argument("--imu-weight", type=float, default=0.25)
    parser.add_argument("--attitude-accel-gain", type=float, default=1.0)
    parser.add_argument("--velocity-leak", type=float, default=0.60)
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
        calibrate_sec=args.calibrate_sec,
        post_takeoff_wait_sec=args.post_takeoff_wait_sec,
        max_power=args.max_power,
        min_power=args.min_power,
        tolerance_cm=args.tolerance_cm,
        final_slow_radius_cm=args.final_slow_radius_cm,
        final_max_power=args.final_max_power,
        max_speed_cm_s=args.max_speed_cm_s,
        final_max_speed_cm_s=args.final_max_speed_cm_s,
        position_to_speed_gain=args.position_to_speed_gain,
        velocity_gain=args.velocity_gain,
        yaw_kp=args.yaw_kp,
        yaw_limit=args.yaw_limit,
        accel_source=args.accel_source,
        imu_weight=args.imu_weight,
        attitude_accel_gain=args.attitude_accel_gain,
        velocity_leak_per_sec=args.velocity_leak,
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
