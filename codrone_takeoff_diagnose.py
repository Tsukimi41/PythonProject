from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any


LOG_DIR = Path(__file__).resolve().parent / "takeoff_diagnosis_logs"

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

FLIGHT_STATES = {
    0: "None",
    16: "Ready",
    17: "Start",
    18: "TakeOff",
    19: "Flight",
    20: "Landing",
    21: "Flip",
    22: "Reverse",
    32: "Stop",
    48: "Accident",
    49: "Error",
    64: "Test",
    65: "EndOfType",
}


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


@dataclass
class Snapshot:
    label: str
    elapsed_sec: float
    battery_percent: Any
    state_data: Any
    error_data: Any
    count_data: Any
    height_cm: Any
    bottom_range_cm: Any
    accel_x: Any
    accel_y: Any
    accel_z: Any
    decoded: dict[str, Any]


def _read_or_error(label: str, func):
    try:
        return func()
    except Exception as exc:
        return {"read_error": label, "type": type(exc).__name__, "message": str(exc)}


def _import_drone_class():
    try:
        from codrone_edu.drone import Drone

        return Drone
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "codrone_edu が見つかりません。.venv の Python で実行してください。"
        ) from exc


def decode_bit_flags(value: Any, names_by_bit: dict[int, str]) -> list[str]:
    if not isinstance(value, int):
        return []
    return [name for bit, name in names_by_bit.items() if value & bit]


def decode_snapshot_fields(state_data: Any, error_data: Any) -> dict[str, Any]:
    flight_state = state_data[2] if isinstance(state_data, list) and len(state_data) > 2 else None
    sensor_error_flags = error_data[1] if isinstance(error_data, list) and len(error_data) > 1 else None
    state_error_flags = error_data[2] if isinstance(error_data, list) and len(error_data) > 2 else None
    return {
        "flight_state_name": FLIGHT_STATES.get(flight_state, f"unknown:{flight_state}"),
        "sensor_error_names": decode_bit_flags(sensor_error_flags, SENSOR_ERROR_FLAGS),
        "state_error_names": decode_bit_flags(state_error_flags, STATE_ERROR_FLAGS),
    }


def read_snapshot(drone: object, label: str, started_at: float) -> Snapshot:
    state_data = _read_or_error("state_data", drone.get_state_data)
    error_data = _read_or_error(
        "error_data",
        lambda: drone.get_error_data(print_error=False),
    )
    return Snapshot(
        label=label,
        elapsed_sec=time.perf_counter() - started_at,
        battery_percent=_read_or_error("battery", drone.get_battery),
        state_data=state_data,
        error_data=error_data,
        count_data=_read_or_error("count_data", drone.get_count_data),
        height_cm=_read_or_error("height_cm", lambda: drone.get_height("cm")),
        bottom_range_cm=_read_or_error(
            "bottom_range_cm",
            lambda: drone.get_bottom_range("cm"),
        ),
        accel_x=_read_or_error("accel_x", drone.get_accel_x),
        accel_y=_read_or_error("accel_y", drone.get_accel_y),
        accel_z=_read_or_error("accel_z", drone.get_accel_z),
        decoded=decode_snapshot_fields(state_data, error_data),
    )


def print_snapshot(snapshot: Snapshot) -> None:
    state = snapshot.state_data
    error = snapshot.error_data
    count = snapshot.count_data
    flight_state = state[2] if isinstance(state, list) and len(state) > 2 else None
    sensor_error_flags = error[1] if isinstance(error, list) and len(error) > 1 else None
    state_error_flags = error[2] if isinstance(error, list) and len(error) > 2 else None
    takeoff_count = count[2] if isinstance(count, list) and len(count) > 2 else None
    landing_count = count[3] if isinstance(count, list) and len(count) > 3 else None

    print(
        "{label:>14} | t={elapsed:5.2f}s | battery={battery} | "
        "flight_state={flight_state}({flight_name}) | "
        "sensor_errors={sensor_errors} | state_errors={state_errors} | "
        "height={height}cm | takeoff_count={takeoff_count} | landing_count={landing_count}".format(
            label=snapshot.label,
            elapsed=snapshot.elapsed_sec,
            battery=snapshot.battery_percent,
            flight_state=flight_state,
            flight_name=snapshot.decoded.get("flight_state_name"),
            sensor_errors=snapshot.decoded.get("sensor_error_names") or sensor_error_flags,
            state_errors=snapshot.decoded.get("state_error_names") or state_error_flags,
            height=snapshot.height_cm,
            takeoff_count=takeoff_count,
            landing_count=landing_count,
        )
    )


def run_diagnosis(hold_sec: float, skip_land: bool) -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"takeoff_diagnosis_{stamp}.json"
    Drone = _import_drone_class()
    drone = Drone()
    started_at = time.perf_counter()
    snapshots: list[Snapshot] = []

    try:
        print("pairing")
        drone.pair()
        snapshots.append(read_snapshot(drone, "after_pair", started_at))
        print_snapshot(snapshots[-1])

        print("reset_move_values")
        drone.reset_move_values()
        time.sleep(0.3)
        snapshots.append(read_snapshot(drone, "after_reset", started_at))
        print_snapshot(snapshots[-1])

        print("takeoff")
        takeoff_return = _read_or_error("takeoff", drone.takeoff)
        snapshots.append(read_snapshot(drone, "after_takeoff", started_at))
        print_snapshot(snapshots[-1])

        end_time = time.perf_counter() + max(0.0, hold_sec)
        sample_index = 0
        while time.perf_counter() < end_time:
            time.sleep(0.5)
            snapshot = read_snapshot(drone, f"hold_{sample_index}", started_at)
            snapshots.append(snapshot)
            print_snapshot(snapshot)
            sample_index += 1

        diagnosis = {
            "takeoff_return": takeoff_return,
            "snapshots": [asdict(snapshot) for snapshot in snapshots],
            "notes": [
                "state_data[2] is the CoDrone flight-state value.",
                "error_data[2] is the state error flags bitmask.",
                "count_data[2] is takeoff count; count_data[3] is landing count.",
            ],
        }
        log_path.write_text(
            json.dumps(_json_safe(diagnosis), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return log_path
    finally:
        if not skip_land:
            try:
                print("land")
                drone.land()
            finally:
                drone.close()
        else:
            drone.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Diagnose CoDrone EDU takeoff failures.")
    parser.add_argument("--hold-sec", type=float, default=3.0, help="Seconds to keep reading state after takeoff.")
    parser.add_argument("--skip-land", action="store_true", help="Do not send land at the end.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    log_path = run_diagnosis(hold_sec=args.hold_sec, skip_land=args.skip_land)
    print(f"log: {log_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
