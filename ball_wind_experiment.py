from __future__ import annotations

import argparse
import csv
import json
import logging
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any


LOG_DIR = Path(__file__).resolve().parent / "ball_wind_logs"
LATEST_LOG_PATH = LOG_DIR / "latest_run.log"
MEASUREMENTS_CSV_PATH = LOG_DIR / "measurements.csv"


@dataclass(frozen=True)
class ExperimentConfig:
    """Tune only this block between experiments unless a safety rule is wrong."""

    ball_description: str = "直径約7cmの穴あきプラスチック製pickleball"
    floor_material: str = "グレーのフローリング"
    target_ball_move_cm: float = 30.0
    target_front_clearance_cm: float = 10.0
    placement_reference: str = "離陸前にドローン本体先端から球の手前端まで"
    target_front_range_cm: float = 10.0
    front_range_tolerance_cm: float = 2.0
    preflight_ball_check_enabled: bool = True
    preflight_front_range_min_cm: float = 6.0
    preflight_front_range_max_cm: float = 18.0

    minimum_battery_percent: int = 40
    minimum_safe_height_cm: float = 25.0
    pre_hover_sec: float = 2.0
    post_blow_hover_sec: float = 1.0
    sample_period_sec: float = 0.10

    # Main tuning variables. Increase/decrease these first.
    blow_pitch_power: int = 10
    blow_throttle_power: int = 0
    blow_duration_sec: float = 0.50
    blow_repeats: int = 1
    repeat_pause_sec: float = 0.60

    # Safety and repeatability.
    keep_height_guard_enabled: bool = True
    land_after_trial: bool = True


@dataclass(frozen=True)
class TrialMetadata:
    trial_id: str
    config: dict[str, Any]
    notes: str


def clamp_power(value: int) -> int:
    return max(-100, min(100, int(value)))


class JsonlLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, event: str, payload: dict[str, Any]) -> None:
        row = {
            "event": event,
            "wall_time": time.time(),
            **payload,
        }
        self._file.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        self._file.flush()

    def __enter__(self) -> "JsonlLogger":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def close(self) -> None:
        self._file.close()


def import_drone_class():
    try:
        from codrone_edu.drone import Drone

        return Drone
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "codrone_edu が見つかりません。.venv を有効化してから実行してください。"
        ) from exc


def setup_text_logger() -> logging.Logger:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("ball_wind_experiment")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    file_handler = logging.FileHandler(LATEST_LOG_PATH, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    return logger


def read_sensors(drone: Any) -> dict[str, Any]:
    snapshot: dict[str, Any] = {}
    for key, getter in (
        ("front_range_cm", lambda: drone.get_front_range("cm")),
        ("height_cm", lambda: drone.get_height("cm")),
        ("battery_percent", drone.get_battery),
    ):
        try:
            snapshot[key] = getter()
        except Exception as exc:
            snapshot[key] = None
            snapshot[f"{key}_error"] = repr(exc)
    return snapshot


def stop_motion(drone: Any) -> None:
    drone.set_pitch(0)
    drone.set_roll(0)
    drone.set_yaw(0)
    drone.set_throttle(0)
    drone.move(0.05)


def ensure_preflight_ball_position(config: ExperimentConfig, front_range_cm: float | None) -> None:
    if not config.preflight_ball_check_enabled:
        return
    if front_range_cm is None:
        raise RuntimeError("離陸前の前方距離センサを読めません。球の配置確認ができないため中止します。")
    if front_range_cm >= 900.0:
        raise RuntimeError("離陸前に前方の球を検出できません。球を本体先端から前方10cmに置いてください。")
    if front_range_cm < config.preflight_front_range_min_cm:
        raise RuntimeError(
            f"離陸前の球が近すぎます: センサ値 {front_range_cm:.1f}cm。"
            f"{config.preflight_front_range_min_cm:.1f}cm 以上にしてください。"
        )
    if front_range_cm > config.preflight_front_range_max_cm:
        raise RuntimeError(
            f"離陸前の球が遠すぎます: センサ値 {front_range_cm:.1f}cm。"
            f"{config.preflight_front_range_max_cm:.1f}cm 以下にしてください。"
        )


def ensure_height_is_safe(config: ExperimentConfig, height_cm: float | None) -> None:
    if not config.keep_height_guard_enabled:
        return
    if height_cm is None:
        raise RuntimeError("高度センサを読めません。接触禁止条件を確認できないため中止します。")
    if height_cm < config.minimum_safe_height_cm:
        raise RuntimeError(
            f"高度が低すぎます: {height_cm:.1f}cm。"
            f"{config.minimum_safe_height_cm:.1f}cm 以上を安全条件にしています。"
        )


def log_sensor_samples(
    *,
    drone: Any,
    config: ExperimentConfig,
    log: JsonlLogger,
    phase: str,
    duration_sec: float,
    sample_period_sec: float,
    command: dict[str, int],
) -> None:
    end_time = time.monotonic() + duration_sec
    while time.monotonic() < end_time:
        sensors = read_sensors(drone)
        log.write(
            "sample",
            {
                "phase": phase,
                "command": command,
                "sensors": sensors,
            },
        )
        ensure_height_is_safe(config, sensors.get("height_cm"))
        time.sleep(sample_period_sec)


def apply_wind_burst(
    *,
    drone: Any,
    config: ExperimentConfig,
    log: JsonlLogger,
    logger: logging.Logger,
) -> None:
    pitch = clamp_power(config.blow_pitch_power)
    throttle = clamp_power(config.blow_throttle_power)
    command = {"pitch": pitch, "roll": 0, "yaw": 0, "throttle": throttle}

    for burst_index in range(1, config.blow_repeats + 1):
        logger.info(
            "風パルス %d/%d: pitch=%d throttle=%d duration=%.2fs",
            burst_index,
            config.blow_repeats,
            pitch,
            throttle,
            config.blow_duration_sec,
        )
        log.write(
            "wind_burst_start",
            {
                "burst_index": burst_index,
                "command": command,
                "duration_sec": config.blow_duration_sec,
            },
        )

        drone.set_pitch(pitch)
        drone.set_roll(0)
        drone.set_yaw(0)
        drone.set_throttle(throttle)

        burst_end_time = time.monotonic() + config.blow_duration_sec
        while time.monotonic() < burst_end_time:
            chunk_sec = min(config.sample_period_sec, burst_end_time - time.monotonic())
            if chunk_sec <= 0:
                break
            drone.move(chunk_sec)
            sensors = read_sensors(drone)
            log.write(
                "sample",
                {
                    "phase": "wind_burst",
                    "burst_index": burst_index,
                    "command": command,
                    "sensors": sensors,
                },
            )
            ensure_height_is_safe(config, sensors.get("height_cm"))

        stop_motion(drone)

        log.write(
            "wind_burst_end",
            {
                "burst_index": burst_index,
                "command": {"pitch": 0, "roll": 0, "yaw": 0, "throttle": 0},
                "sensors": read_sensors(drone),
            },
        )

        if burst_index < config.blow_repeats:
            log_sensor_samples(
                drone=drone,
                config=config,
                log=log,
                phase="between_bursts",
                duration_sec=config.repeat_pause_sec,
                sample_period_sec=config.sample_period_sec,
                command={"pitch": 0, "roll": 0, "yaw": 0, "throttle": 0},
            )


def run_trial(config: ExperimentConfig, notes: str = "") -> Path:
    logger = setup_text_logger()
    trial_id = time.strftime("ball_wind_%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"{trial_id}.jsonl"
    log = JsonlLogger(log_path)
    Drone = import_drone_class()
    drone = Drone()
    is_flying = False

    try:
        metadata = TrialMetadata(trial_id=trial_id, config=asdict(config), notes=notes)
        log.write("metadata", asdict(metadata))
        logger.info("ペアリングします")
        drone.pair()

        initial_sensors = read_sensors(drone)
        log.write("preflight", {"sensors": initial_sensors})
        battery = initial_sensors.get("battery_percent")
        if battery is not None and battery < config.minimum_battery_percent:
            raise RuntimeError(
                f"バッテリー不足です: {battery}%。"
                f"{config.minimum_battery_percent}% 以上で実験してください。"
            )
        ensure_preflight_ball_position(config, initial_sensors.get("front_range_cm"))
        front_range = initial_sensors.get("front_range_cm")
        if front_range is not None and abs(front_range - config.target_front_range_cm) > config.front_range_tolerance_cm:
            logger.warning(
                "離陸前の前方距離センサ値 %.1fcm は目標 %.1fcm から外れています。"
                "本体先端から10cmの配置は目視基準なので、センサ値は参考として続行します。",
                front_range,
                config.target_front_range_cm,
            )

        logger.info(
            "離陸します。球は%s %.1fcm に置いてください",
            config.placement_reference,
            config.target_front_clearance_cm,
        )
        drone.takeoff()
        is_flying = True
        stop_motion(drone)

        log_sensor_samples(
            drone=drone,
            config=config,
            log=log,
            phase="pre_hover",
            duration_sec=config.pre_hover_sec,
            sample_period_sec=config.sample_period_sec,
            command={"pitch": 0, "roll": 0, "yaw": 0, "throttle": 0},
        )

        before_blow = read_sensors(drone)
        front_range = before_blow.get("front_range_cm")
        log.write("before_blow", {"sensors": before_blow})
        ensure_height_is_safe(config, before_blow.get("height_cm"))
        if front_range is not None and abs(front_range - config.target_front_range_cm) > config.front_range_tolerance_cm:
            logger.warning(
                "前方距離センサ値 %.1fcm は目標 %.1fcm から外れています。床上のpickleballでは安全判定に使わず、ログに残します。",
                front_range,
                config.target_front_range_cm,
            )

        apply_wind_burst(drone=drone, config=config, log=log, logger=logger)

        log_sensor_samples(
            drone=drone,
            config=config,
            log=log,
            phase="post_blow_hover",
            duration_sec=config.post_blow_hover_sec,
            sample_period_sec=config.sample_period_sec,
            command={"pitch": 0, "roll": 0, "yaw": 0, "throttle": 0},
        )

        summary = {
            "trial_id": trial_id,
            "target_ball_move_cm": config.target_ball_move_cm,
            "actual_ball_move_cm": None,
            "result_error_cm": None,
            "log_path": str(log_path),
            "tune_next": suggest_next_config(config, actual_ball_move_cm=None),
        }
        log.write("summary", summary)
        logger.info("実験ログ: %s", log_path)
        return log_path
    except KeyboardInterrupt:
        logger.warning("中断されました")
        log.write("aborted", {"reason": "keyboard_interrupt"})
        return log_path
    except Exception as exc:
        logger.exception("実験を中止します")
        log.write("error", {"message": str(exc), "type": type(exc).__name__})
        raise
    finally:
        try:
            stop_motion(drone)
            if is_flying and config.land_after_trial:
                logger.info("着陸します")
                drone.land()
        finally:
            log.close()
            drone.close()


def suggest_next_config(config: ExperimentConfig, actual_ball_move_cm: float | None) -> dict[str, Any]:
    if actual_ball_move_cm is None:
        return {
            "first_variable_to_change": "actual_ball_move_cm を記録してから判断",
            "config": asdict(config),
        }

    error_cm = config.target_ball_move_cm - actual_ball_move_cm
    next_config = config
    reason = ""

    if abs(error_cm) <= 3.0:
        reason = "目標30cmに十分近い。再現性確認を優先する。"
    elif actual_ball_move_cm < config.target_ball_move_cm:
        if config.blow_duration_sec < 1.5:
            next_config = replace(config, blow_duration_sec=round(config.blow_duration_sec + 0.15, 2))
            reason = "移動不足。まず風を当てる時間を少し増やす。"
        else:
            next_config = replace(config, blow_pitch_power=min(config.blow_pitch_power + 2, 30))
            reason = "移動不足で時間は長め。次はpitchを少し上げる。"
    else:
        if config.blow_duration_sec > 0.35:
            next_config = replace(config, blow_duration_sec=round(config.blow_duration_sec - 0.10, 2))
            reason = "動きすぎ。まず風を当てる時間を減らす。"
        else:
            next_config = replace(config, blow_pitch_power=max(config.blow_pitch_power - 2, 4))
            reason = "動きすぎで時間は短め。次はpitchを下げる。"

    return {
        "reason": reason,
        "error_cm": round(error_cm, 2),
        "config": asdict(next_config),
        "edit_here": {
            "main_knobs": [
                "ExperimentConfig.blow_duration_sec",
                "ExperimentConfig.blow_pitch_power",
                "ExperimentConfig.blow_repeats",
                "ExperimentConfig.target_front_clearance_cm",
            ]
        },
    }


def config_from_metadata(raw_config: dict[str, Any]) -> ExperimentConfig:
    defaults = asdict(ExperimentConfig())
    defaults.update({key: value for key, value in raw_config.items() if key in defaults})
    return ExperimentConfig(**defaults)


def append_measurement_csv(
    *,
    log_path: Path,
    trial_id: str,
    config: ExperimentConfig,
    actual_ball_move_cm: float,
    result_error_cm: float,
    notes: str,
    wind_burst_count: int,
    measurement_status: str,
) -> None:
    MEASUREMENTS_CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    csv_exists = MEASUREMENTS_CSV_PATH.exists()
    fieldnames = [
        "recorded_at",
        "trial_id",
        "log_path",
        "ball_description",
        "floor_material",
        "target_ball_move_cm",
        "actual_ball_move_cm",
        "result_error_cm",
        "target_front_clearance_cm",
        "placement_reference",
        "preflight_front_range_min_cm",
        "preflight_front_range_max_cm",
        "blow_pitch_power",
        "blow_throttle_power",
        "blow_duration_sec",
        "blow_repeats",
        "minimum_safe_height_cm",
        "wind_burst_count",
        "measurement_status",
        "notes",
    ]
    with MEASUREMENTS_CSV_PATH.open("a", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if not csv_exists:
            writer.writeheader()
        writer.writerow(
            {
                "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "trial_id": trial_id,
                "log_path": str(log_path),
                "ball_description": config.ball_description,
                "floor_material": config.floor_material,
                "target_ball_move_cm": config.target_ball_move_cm,
                "actual_ball_move_cm": actual_ball_move_cm,
                "result_error_cm": result_error_cm,
                "target_front_clearance_cm": config.target_front_clearance_cm,
                "placement_reference": config.placement_reference,
                "preflight_front_range_min_cm": config.preflight_front_range_min_cm,
                "preflight_front_range_max_cm": config.preflight_front_range_max_cm,
                "blow_pitch_power": config.blow_pitch_power,
                "blow_throttle_power": config.blow_throttle_power,
                "blow_duration_sec": config.blow_duration_sec,
                "blow_repeats": config.blow_repeats,
                "minimum_safe_height_cm": config.minimum_safe_height_cm,
                "wind_burst_count": wind_burst_count,
                "measurement_status": measurement_status,
                "notes": notes,
            }
        )


def append_measurement(log_path: Path, actual_ball_move_cm: float, notes: str = "") -> None:
    rows = []
    with log_path.open("r", encoding="utf-8") as src:
        for line in src:
            if line.strip():
                rows.append(json.loads(line))

    metadata = next((row for row in rows if row.get("event") == "metadata"), None)
    if metadata is None:
        raise RuntimeError(f"metadata が見つかりません: {log_path}")

    config = config_from_metadata(metadata["config"])
    trial_id = metadata.get("trial_id", log_path.stem)
    wind_burst_count = sum(1 for row in rows if row.get("event") == "wind_burst_start")
    measurement_status = "valid" if wind_burst_count > 0 else "invalid_no_wind_burst"
    error_cm = config.target_ball_move_cm - actual_ball_move_cm
    next_config_suggestion = (
        suggest_next_config(config, actual_ball_move_cm)
        if measurement_status == "valid"
        else {"reason": "風パルス前に中止されたログなので、この実測値ではチューニングしない。"}
    )
    with JsonlLogger(log_path) as as_log:
        as_log.write(
            "manual_measurement",
            {
                "actual_ball_move_cm": actual_ball_move_cm,
                "target_ball_move_cm": config.target_ball_move_cm,
                "result_error_cm": error_cm,
                "wind_burst_count": wind_burst_count,
                "measurement_status": measurement_status,
                "notes": notes,
                "next_config_suggestion": next_config_suggestion,
            },
        )
    append_measurement_csv(
        log_path=log_path,
        trial_id=trial_id,
        config=config,
        actual_ball_move_cm=actual_ball_move_cm,
        result_error_cm=error_cm,
        notes=notes,
        wind_burst_count=wind_burst_count,
        measurement_status=measurement_status,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CoDrone EDUで前方の球を風で動かす実験ツール")
    subparsers = parser.add_subparsers(dest="command")

    run_parser = subparsers.add_parser("run", help="1回の風実験を実行する")
    run_parser.add_argument("--notes", default="", help="実験条件メモ")
    run_parser.add_argument("--pitch", type=int, default=None, help="ExperimentConfig.blow_pitch_power を一時上書き")
    run_parser.add_argument("--throttle", type=int, default=None, help="ExperimentConfig.blow_throttle_power を一時上書き")
    run_parser.add_argument("--duration-sec", type=float, default=None, help="ExperimentConfig.blow_duration_sec を一時上書き")
    run_parser.add_argument("--repeats", type=int, default=None, help="ExperimentConfig.blow_repeats を一時上書き")
    run_parser.add_argument(
        "--front-clearance-cm",
        type=float,
        default=None,
        help="ExperimentConfig.target_front_clearance_cm を一時上書き",
    )
    run_parser.add_argument(
        "--minimum-safe-height-cm",
        type=float,
        default=None,
        help="ExperimentConfig.minimum_safe_height_cm を一時上書き",
    )

    measure_parser = subparsers.add_parser("measure", help="実測した球の移動量を既存ログへ追記する")
    measure_parser.add_argument("log_path", type=Path, help="ball_wind_logs/*.jsonl")
    measure_parser.add_argument("--actual-cm", type=float, required=True, help="あなたが測った球の移動量[cm]")
    measure_parser.add_argument("--notes", default="", help="測定メモ")

    return parser


def config_from_args(args: argparse.Namespace) -> ExperimentConfig:
    config = ExperimentConfig()
    updates: dict[str, Any] = {}
    if getattr(args, "pitch", None) is not None:
        updates["blow_pitch_power"] = args.pitch
    if getattr(args, "throttle", None) is not None:
        updates["blow_throttle_power"] = args.throttle
    if getattr(args, "duration_sec", None) is not None:
        updates["blow_duration_sec"] = args.duration_sec
    if getattr(args, "repeats", None) is not None:
        updates["blow_repeats"] = args.repeats
    if getattr(args, "front_clearance_cm", None) is not None:
        updates["target_front_clearance_cm"] = args.front_clearance_cm
        updates["target_front_range_cm"] = args.front_clearance_cm
    if getattr(args, "minimum_safe_height_cm", None) is not None:
        updates["minimum_safe_height_cm"] = args.minimum_safe_height_cm
    return replace(config, **updates)


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.command == "measure":
        append_measurement(args.log_path, args.actual_cm, args.notes)
        print(f"measurement appended: {args.log_path}")
        return 0

    if args.command in {None, "run"}:
        log_path = run_trial(config_from_args(args), notes=getattr(args, "notes", ""))
        print(f"log_path={log_path}")
        return 0

    parser.error("unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
