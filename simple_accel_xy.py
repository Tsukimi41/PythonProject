from __future__ import annotations

import argparse
import csv
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import fmean


LOG_DIR = Path(__file__).resolve().parent / "accel_position_logs"
ACCEL_UNIT_TO_CM_S2 = 10.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class Config:
    target_x_cm: float
    target_y_cm: float
    timeout_sec: float = 12.0
    dt_sec: float = 0.05
    calibrate_sec: float = 1.5
    max_power: int = 35
    min_power: int = 18
    tolerance_cm: float = 8.0
    accel_deadband_cm_s2: float = 12.0
    accel_x_sign: float = -1.0
    accel_y_sign: float = 1.0
    pitch_sign: float = 1.0
    roll_sign: float = -1.0
    kp: float = 0.9
    kd: float = 0.18


@dataclass
class Bias:
    x: float
    y: float
    z: float


@dataclass
class State:
    x_cm: float = 0.0
    y_cm: float = 0.0
    vx_cm_s: float = 0.0
    vy_cm_s: float = 0.0


@dataclass
class Sample:
    elapsed_sec: float
    raw_ax: float
    raw_ay: float
    raw_az: float
    ax_cm_s2: float
    ay_cm_s2: float
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


def read_accel(drone) -> tuple[float, float, float]:
    return (
        float(drone.get_accel_x()),
        float(drone.get_accel_y()),
        float(drone.get_accel_z()),
    )


def calibrate_accel(drone, seconds: float, dt_sec: float) -> Bias:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    end_time = time.perf_counter() + seconds
    while time.perf_counter() < end_time:
        ax, ay, az = read_accel(drone)
        xs.append(ax)
        ys.append(ay)
        zs.append(az)
        time.sleep(dt_sec)
    return Bias(fmean(xs), fmean(ys), fmean(zs))


def remove_deadband(value: float, deadband: float) -> float:
    if abs(value) <= deadband:
        return 0.0
    return value - deadband if value > 0.0 else value + deadband


def update_state(state: State, raw_accel: tuple[float, float, float], bias: Bias, config: Config, dt: float) -> tuple[State, float, float]:
    raw_ax, raw_ay, _raw_az = raw_accel
    ax = (raw_ax - bias.x) * ACCEL_UNIT_TO_CM_S2 * config.accel_x_sign
    ay = (raw_ay - bias.y) * ACCEL_UNIT_TO_CM_S2 * config.accel_y_sign
    ax = remove_deadband(ax, config.accel_deadband_cm_s2)
    ay = remove_deadband(ay, config.accel_deadband_cm_s2)

    next_vx = state.vx_cm_s + ax * dt
    next_vy = state.vy_cm_s + ay * dt
    next_x = state.x_cm + (state.vx_cm_s + next_vx) * 0.5 * dt
    next_y = state.y_cm + (state.vy_cm_s + next_vy) * 0.5 * dt
    return State(next_x, next_y, next_vx, next_vy), ax, ay


def axis_power(error_cm: float, velocity_cm_s: float, config: Config) -> int:
    power = config.kp * error_cm - config.kd * velocity_cm_s
    if abs(error_cm) > config.tolerance_cm and abs(power) < config.min_power:
        power = config.min_power if power >= 0.0 else -config.min_power
    return int(round(clamp(power, -config.max_power, config.max_power)))


def command_from_state(state: State, config: Config) -> tuple[int, int, float, float]:
    error_x = config.target_x_cm - state.x_cm
    error_y = config.target_y_cm - state.y_cm
    pitch = axis_power(error_x, state.vx_cm_s, config) * config.pitch_sign
    roll = axis_power(error_y, state.vy_cm_s, config) * config.roll_sign
    return int(pitch), int(roll), error_x, error_y


def make_log_writer() -> tuple[csv.DictWriter, object, Path]:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = LOG_DIR / f"simple_accel_xy_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=list(Sample.__dataclass_fields__.keys()))
    writer.writeheader()
    return writer, handle, path


def move_xy(drone, config: Config) -> Path:
    writer, log_handle, log_path = make_log_writer()
    state = State()
    start = time.perf_counter()

    try:
        print("takeoff")
        drone.takeoff()
        time.sleep(1.0)

        print("calibrate")
        bias = calibrate_accel(drone, config.calibrate_sec, config.dt_sec)
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
            raw_accel = read_accel(drone)
            state, ax, ay = update_state(state, raw_accel, bias, config, dt)
            pitch, roll, error_x, error_y = command_from_state(state, config)

            drone.set_pitch(pitch)
            drone.set_roll(roll)
            drone.set_yaw(0)
            drone.set_throttle(0)
            drone.move(config.dt_sec)

            raw_ax, raw_ay, raw_az = raw_accel
            writer.writerow(asdict(Sample(
                elapsed_sec=elapsed,
                raw_ax=raw_ax,
                raw_ay=raw_ay,
                raw_az=raw_az,
                ax_cm_s2=ax,
                ay_cm_s2=ay,
                x_cm=state.x_cm,
                y_cm=state.y_cm,
                vx_cm_s=state.vx_cm_s,
                vy_cm_s=state.vy_cm_s,
                target_x_cm=config.target_x_cm,
                target_y_cm=config.target_y_cm,
                error_x_cm=error_x,
                error_y_cm=error_y,
                pitch=pitch,
                roll=roll,
            )))

            print(
                f"x={state.x_cm:7.1f} y={state.y_cm:7.1f} "
                f"ex={error_x:7.1f} ey={error_y:7.1f} "
                f"pitch={pitch:4d} roll={roll:4d}"
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
    parser = argparse.ArgumentParser(description="Simple CoDrone EDU accel-only x/y move.")
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
