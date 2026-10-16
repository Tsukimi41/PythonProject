from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path


CONFIG_PATH = Path(__file__).resolve().parent / "time_calibrated_xy.json"


@dataclass(frozen=True)
class DirectionCalibration:
    power: int
    speed_cm_s: float


@dataclass(frozen=True)
class Calibration:
    forward: DirectionCalibration
    back: DirectionCalibration
    left: DirectionCalibration
    right: DirectionCalibration
    trim_roll: int = 0
    trim_pitch: int = 0
    landing_roll: int = 0
    landing_pitch: int = 0
    landing_throttle: int = -12
    landing_duration_sec: float = 1.2
    min_duration_sec: float = 0.15
    max_segment_sec: float = 2.0
    settle_sec: float = 0.25

    @classmethod
    def default(cls) -> "Calibration":
        return cls(
            forward=DirectionCalibration(power=22, speed_cm_s=24.0),
            back=DirectionCalibration(power=-22, speed_cm_s=22.0),
            left=DirectionCalibration(power=-22, speed_cm_s=22.0),
            right=DirectionCalibration(power=22, speed_cm_s=22.0),
        )


@dataclass(frozen=True)
class Segment:
    axis: str
    direction: str
    distance_cm: float
    power: int
    duration_sec: float


def load_calibration(path: Path = CONFIG_PATH) -> Calibration:
    if not path.exists():
        return Calibration.default()

    data = json.loads(path.read_text(encoding="utf-8"))
    return Calibration(
        forward=DirectionCalibration(**data["forward"]),
        back=DirectionCalibration(**data["back"]),
        left=DirectionCalibration(**data["left"]),
        right=DirectionCalibration(**data["right"]),
        trim_roll=int(data.get("trim_roll", 0)),
        trim_pitch=int(data.get("trim_pitch", 0)),
        landing_roll=int(data.get("landing_roll", data.get("trim_roll", 0))),
        landing_pitch=int(data.get("landing_pitch", data.get("trim_pitch", 0))),
        landing_throttle=int(data.get("landing_throttle", -12)),
        landing_duration_sec=float(data.get("landing_duration_sec", 1.2)),
        min_duration_sec=float(data.get("min_duration_sec", 0.15)),
        max_segment_sec=float(data.get("max_segment_sec", 2.0)),
        settle_sec=float(data.get("settle_sec", 0.25)),
    )


def save_calibration(calibration: Calibration, path: Path = CONFIG_PATH) -> None:
    path.write_text(
        json.dumps(asdict(calibration), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def update_direction(
    calibration: Calibration,
    direction: str,
    measured_cm: float,
    duration_sec: float,
    power: int | None = None,
) -> Calibration:
    if measured_cm <= 0:
        raise ValueError("measured_cm must be positive")
    if duration_sec <= 0:
        raise ValueError("duration_sec must be positive")

    current = getattr(calibration, direction)
    updated = DirectionCalibration(
        power=current.power if power is None else signed_power_for_direction(direction, power),
        speed_cm_s=measured_cm / duration_sec,
    )
    values = asdict(calibration)
    values[direction] = asdict(updated)
    return Calibration(
        forward=DirectionCalibration(**values["forward"]),
        back=DirectionCalibration(**values["back"]),
        left=DirectionCalibration(**values["left"]),
        right=DirectionCalibration(**values["right"]),
        trim_roll=values["trim_roll"],
        trim_pitch=values["trim_pitch"],
        landing_roll=values["landing_roll"],
        landing_pitch=values["landing_pitch"],
        landing_throttle=values["landing_throttle"],
        landing_duration_sec=values["landing_duration_sec"],
        min_duration_sec=values["min_duration_sec"],
        max_segment_sec=values["max_segment_sec"],
        settle_sec=values["settle_sec"],
    )


def update_trim(calibration: Calibration, roll: int, pitch: int) -> Calibration:
    return Calibration(
        forward=calibration.forward,
        back=calibration.back,
        left=calibration.left,
        right=calibration.right,
        trim_roll=clamp_int(roll, -100, 100),
        trim_pitch=clamp_int(pitch, -100, 100),
        landing_roll=calibration.landing_roll,
        landing_pitch=calibration.landing_pitch,
        landing_throttle=calibration.landing_throttle,
        landing_duration_sec=calibration.landing_duration_sec,
        min_duration_sec=calibration.min_duration_sec,
        max_segment_sec=calibration.max_segment_sec,
        settle_sec=calibration.settle_sec,
    )


def update_landing(
    calibration: Calibration,
    roll: int,
    pitch: int,
    throttle: int,
    duration_sec: float,
) -> Calibration:
    if duration_sec <= 0:
        raise ValueError("duration_sec must be positive")
    if throttle >= 0:
        raise ValueError("landing throttle should be negative")

    return Calibration(
        forward=calibration.forward,
        back=calibration.back,
        left=calibration.left,
        right=calibration.right,
        trim_roll=calibration.trim_roll,
        trim_pitch=calibration.trim_pitch,
        landing_roll=clamp_int(roll, -100, 100),
        landing_pitch=clamp_int(pitch, -100, 100),
        landing_throttle=clamp_int(throttle, -100, -1),
        landing_duration_sec=duration_sec,
        min_duration_sec=calibration.min_duration_sec,
        max_segment_sec=calibration.max_segment_sec,
        settle_sec=calibration.settle_sec,
    )


def clamp_int(value: int, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def signed_power_for_direction(direction: str, absolute_power: int) -> int:
    power = abs(absolute_power)
    if direction in {"forward", "right"}:
        return power
    if direction in {"back", "left"}:
        return -power
    raise ValueError(f"Unknown direction: {direction}")


def plan_move(x_cm: float, y_cm: float, calibration: Calibration) -> list[Segment]:
    segments: list[Segment] = []
    if x_cm > 0:
        segments.extend(plan_axis("x", "forward", x_cm, calibration.forward, calibration))
    elif x_cm < 0:
        segments.extend(plan_axis("x", "back", abs(x_cm), calibration.back, calibration))

    if y_cm > 0:
        segments.extend(plan_axis("y", "left", y_cm, calibration.left, calibration))
    elif y_cm < 0:
        segments.extend(plan_axis("y", "right", abs(y_cm), calibration.right, calibration))

    return segments


def plan_axis(
    axis: str,
    direction: str,
    distance_cm: float,
    direction_calibration: DirectionCalibration,
    calibration: Calibration,
) -> list[Segment]:
    if direction_calibration.speed_cm_s <= 0:
        raise ValueError(f"{direction} speed_cm_s must be positive")

    remaining = distance_cm
    segments: list[Segment] = []
    while remaining > 0:
        segment_distance = min(
            remaining,
            direction_calibration.speed_cm_s * calibration.max_segment_sec,
        )
        duration_sec = segment_distance / direction_calibration.speed_cm_s
        if duration_sec < calibration.min_duration_sec:
            duration_sec = calibration.min_duration_sec
        segments.append(
            Segment(
                axis=axis,
                direction=direction,
                distance_cm=segment_distance,
                power=direction_calibration.power,
                duration_sec=duration_sec,
            )
        )
        remaining -= segment_distance
    return segments


def execute_segments(drone, segments: list[Segment], settle_sec: float) -> None:
    for segment in segments:
        print(
            f"{segment.direction}: {segment.distance_cm:.1f} cm, "
            f"power={segment.power}, duration={segment.duration_sec:.2f} sec"
        )
        if segment.axis == "x":
            drone.set_pitch(segment.power)
            drone.set_roll(0)
        else:
            drone.set_pitch(0)
            drone.set_roll(segment.power)

        drone.move(segment.duration_sec)
        stop_xy(drone, settle_sec)


def stop_xy(drone, settle_sec: float = 0.25) -> None:
    drone.set_pitch(0)
    drone.set_roll(0)
    drone.set_yaw(0)
    drone.set_throttle(0)
    drone.move(settle_sec)


def prepare_takeoff(drone, calibration: Calibration, reset_gyro: bool = True) -> None:
    drone.reset_move_values()
    if reset_gyro:
        print("Resetting gyro. Keep the drone flat and still.")
        drone.reset_gyro()
    print(f"Applying trim: roll={calibration.trim_roll}, pitch={calibration.trim_pitch}")
    drone.set_trim(calibration.trim_roll, calibration.trim_pitch)


def land_drone(drone, calibration: Calibration, mode: str) -> None:
    if mode == "auto":
        drone.land()
        return
    if mode != "controlled":
        raise ValueError("landing mode must be 'controlled' or 'auto'")

    print(
        f"Controlled landing: roll={calibration.landing_roll}, "
        f"pitch={calibration.landing_pitch}, throttle={calibration.landing_throttle}, "
        f"duration={calibration.landing_duration_sec:.2f} sec"
    )
    drone.set_roll(calibration.landing_roll)
    drone.set_pitch(calibration.landing_pitch)
    drone.set_yaw(0)
    drone.set_throttle(calibration.landing_throttle)
    drone.move(calibration.landing_duration_sec)
    stop_xy(drone, calibration.settle_sec)
    drone.land()


def make_drone():
    from codrone_edu.drone import Drone

    drone = Drone()
    drone.pair()
    return drone


def run_fly(args: argparse.Namespace) -> None:
    calibration = load_calibration(Path(args.config))
    segments = plan_move(args.x_cm, args.y_cm, calibration)

    if args.dry_run:
        print_plan(segments)
        return

    drone = make_drone()
    try:
        prepare_takeoff(drone, calibration, reset_gyro=not args.no_reset_gyro)
        drone.takeoff()
        time.sleep(args.takeoff_wait)
        execute_segments(drone, segments, calibration.settle_sec)
    finally:
        try:
            stop_xy(drone, calibration.settle_sec)
            land_drone(drone, calibration, args.land_mode)
        finally:
            drone.close()


def run_calibrate(args: argparse.Namespace) -> None:
    calibration = load_calibration(Path(args.config))
    direction = args.direction
    power = signed_power_for_direction(direction, args.power)

    segment = Segment(
        axis="x" if direction in {"forward", "back"} else "y",
        direction=direction,
        distance_cm=0.0,
        power=power,
        duration_sec=args.duration,
    )

    if args.dry_run:
        print_plan([segment])
    else:
        drone = make_drone()
        try:
            prepare_takeoff(drone, calibration, reset_gyro=not args.no_reset_gyro)
            drone.takeoff()
            time.sleep(args.takeoff_wait)
            execute_segments(drone, [segment], calibration.settle_sec)
        finally:
            try:
                stop_xy(drone, calibration.settle_sec)
                land_drone(drone, calibration, args.land_mode)
            finally:
                drone.close()

    measured_cm = args.measured_cm
    if measured_cm is None:
        value = input("Measured distance in cm. Leave empty to skip saving: ").strip()
        if not value:
            return
        measured_cm = float(value)

    updated = update_direction(
        calibration=calibration,
        direction=direction,
        measured_cm=measured_cm,
        duration_sec=args.duration,
        power=args.power,
    )
    save_calibration(updated, Path(args.config))
    print(f"Saved {direction}: power={power}, speed={measured_cm / args.duration:.2f} cm/s")


def run_trim(args: argparse.Namespace) -> None:
    path = Path(args.config)
    calibration = update_trim(load_calibration(path), roll=args.roll, pitch=args.pitch)
    save_calibration(calibration, path)
    print(f"Saved trim: roll={calibration.trim_roll}, pitch={calibration.trim_pitch}")


def run_landing(args: argparse.Namespace) -> None:
    path = Path(args.config)
    calibration = update_landing(
        load_calibration(path),
        roll=args.roll,
        pitch=args.pitch,
        throttle=args.throttle,
        duration_sec=args.duration,
    )
    save_calibration(calibration, path)
    print(
        f"Saved landing: roll={calibration.landing_roll}, "
        f"pitch={calibration.landing_pitch}, throttle={calibration.landing_throttle}, "
        f"duration={calibration.landing_duration_sec:.2f} sec"
    )


def print_plan(segments: list[Segment]) -> None:
    if not segments:
        print("No movement.")
        return
    for segment in segments:
        print(
            f"{segment.axis} {segment.direction}: "
            f"power={segment.power}, duration={segment.duration_sec:.2f} sec"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move CoDrone EDU by calibrated command duration only. "
            "This program does not call optical-flow position APIs."
        )
    )
    parser.add_argument("--config", default=str(CONFIG_PATH), help="Calibration JSON path.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    fly = subparsers.add_parser("fly", help="Fly x/y cm using saved calibration.")
    fly.add_argument("--x-cm", type=float, default=0.0, help="+X is forward.")
    fly.add_argument("--y-cm", type=float, default=0.0, help="+Y is left.")
    fly.add_argument("--takeoff-wait", type=float, default=1.5)
    fly.add_argument("--land-mode", choices=["controlled", "auto"], default="controlled")
    fly.add_argument("--no-reset-gyro", action="store_true")
    fly.add_argument("--dry-run", action="store_true")
    fly.set_defaults(func=run_fly)

    calibrate = subparsers.add_parser("calibrate", help="Run one timed calibration move.")
    calibrate.add_argument("direction", choices=["forward", "back", "left", "right"])
    calibrate.add_argument("--power", type=int, default=22, help="Absolute pitch/roll power.")
    calibrate.add_argument("--duration", type=float, default=1.0)
    calibrate.add_argument("--measured-cm", type=float, default=None)
    calibrate.add_argument("--takeoff-wait", type=float, default=1.5)
    calibrate.add_argument("--land-mode", choices=["controlled", "auto"], default="controlled")
    calibrate.add_argument("--no-reset-gyro", action="store_true")
    calibrate.add_argument("--dry-run", action="store_true")
    calibrate.set_defaults(func=run_calibrate)

    trim = subparsers.add_parser("trim", help="Save takeoff/hover trim values.")
    trim.add_argument("--roll", type=int, required=True, help="Negative corrects right drift; positive corrects left drift.")
    trim.add_argument("--pitch", type=int, required=True, help="Negative corrects forward drift; positive corrects backward drift.")
    trim.set_defaults(func=run_trim)

    landing = subparsers.add_parser("landing", help="Save controlled landing correction values.")
    landing.add_argument("--roll", type=int, required=True, help="Negative corrects right drift; positive corrects left drift.")
    landing.add_argument("--pitch", type=int, required=True, help="Negative corrects forward drift; positive corrects backward drift.")
    landing.add_argument("--throttle", type=int, default=-12, help="Negative descent throttle.")
    landing.add_argument("--duration", type=float, default=1.2, help="Manual descent duration before auto land.")
    landing.set_defaults(func=run_landing)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
