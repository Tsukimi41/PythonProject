import time
import logging
from typing import Optional
from codrone_edu.drone import Drone

# ==========================================
# 設定とロギングの初期化
# ==========================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


class PIDController:
    """
    独立したPIDコントローラークラス（保守性と再利用性を高めるための分離設計）
    """

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        # 最高速度（2.5m/s）に応じた出力制限
        # CoDrone EDUのモーター出力は -100 〜 100 の範囲
        self.output_limit: float = output_limit

        self.integral: float = 0.0
        self.prev_error: float = 0.0

    def compute(self, setpoint: float, current_value: float, dt: float) -> float:
        """
        PIDの計算を行い、操作量（モーターのピッチ/ロール出力）を返す
        """
        # ゼロ除算の防止（堅牢性）
        if dt <= 0.0:
            return 0.0

        error = setpoint - current_value
        self.integral += error * dt

        # アンチワインドアップ (Integral Windup) 対策
        # 積分項が大きくなりすぎてオーバーシュートするのを防ぐ（54.8gの軽量機体には必須）
        if self.ki > 0:
            max_integral = self.output_limit / self.ki
            self.integral = max(min(self.integral, max_integral), -max_integral)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID出力の計算
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # 出力のクランプ（制限）
        return max(min(output, self.output_limit), -self.output_limit)


class PrecisionDroneController:
    """
    CoDrone EDUを安全かつ精密に制御するためのラッパークラス
    """

    def __init__(self):
        self.drone = Drone()
        self.is_flying = False

    def initialize_and_takeoff(self) -> None:
        """
        ドローンのペアリング、安全確認、および離陸処理
        """
        logging.info("ドローンに接続しています...")
        self.drone.pair()

        # フライト前のバッテリーチェック（堅牢性）
        battery_level = self.drone.get_battery()
        logging.info(f"現在のバッテリー残量: {battery_level}%")
        if battery_level < 20:
            logging.error("バッテリー残量が少なすぎます。飛行を中止します。")
            raise RuntimeError("Low Battery Warning")

        logging.info("離陸します。")
        self.drone.takeoff()
        self.is_flying = True
        time.sleep(1)  # 姿勢安定のためのウェイト

    def move_forward_mm_pid(self, target_mm: float, timeout_sec: float = 10.0) -> None:
        """
        指定されたミリメートル(mm)分、PID制御を用いて前進するメソッド
        """
        logging.info(f"目標移動距離: {target_mm} mm のPID制御移動を開始します。")

        # CoDrone EDUのオプティカルフローセンサーは標準でcm単位の絶対位置を返すため、
        # mmに変換して扱う
        initial_y_cm = self.drone.get_position_y()
        start_y_mm = initial_y_cm * 10.0
        target_pos_mm = start_y_mm + target_mm

        # 軽量機体 (54.8g) とコアレスDCモーターの特性に合わせたPIDゲインの初期値
        # ※実際の床材の摩擦や空気抵抗に合わせて微調整(チューニング)してください
        pid = PIDController(kp=0.4, ki=0.02, kd=0.15, output_limit=60.0)

        start_time = time.time()
        prev_time = start_time

        # 許容誤差 (mm) - この範囲内に入れば到達とみなす
        tolerance_mm = 15.0

        while True:
            current_time = time.time()
            dt = current_time - prev_time

            # 安全第一: タイムアウトによる無限ループ・暴走の防止（堅牢性）
            if (current_time - start_time) > timeout_sec:
                logging.warning("タイムアウトしました。目標地点に到達する前にループを抜けます。")
                self.drone.set_pitch(0)
                self.drone.move(0.1)
                break

            # 制御ループの周期を安定させる (例: 約20Hz = 0.05秒間隔)
            if dt < 0.05:
                time.sleep(0.05 - dt)
                current_time = time.time()
                dt = current_time - prev_time

            # 現在位置の取得 (cm -> mmに変換)
            current_y_mm = self.drone.get_position_y() * 10.0
            error_mm = target_pos_mm - current_y_mm

            # 目標地点付近での停止判定
            if abs(error_mm) <= tolerance_mm:
                logging.info(f"目標地点に到達しました。最終誤差: {error_mm:.2f} mm")
                self.drone.set_pitch(0)
                self.drone.move(0.1)
                break

            # PID出力の計算 (出力値は -100 〜 100 の範囲になる)
            pitch_power = int(pid.compute(setpoint=target_pos_mm, current_value=current_y_mm, dt=dt))

            # ドローンにピッチ(前進/後退)のパワーを設定してごく短時間(dt)移動を継続させる
            self.drone.set_pitch(pitch_power)
            self.drone.move(0.05)  # 短い移動コマンドを連続送信

            prev_time = current_time

    def safe_land_and_close(self) -> None:
        """
        必ず安全に着陸し、通信を閉じるための終了処理
        """
        try:
            if self.is_flying:
                logging.info("着陸態勢に入ります...")
                self.drone.land()
        finally:
            logging.info("ドローンとの通信を切断します。")
            self.drone.close()


# ==========================================
# メイン実行ブロック
# ==========================================
if __name__ == "__main__":
    controller = PrecisionDroneController()

    try:
        # 初期化と離陸
        controller.initialize_and_takeoff()

        # ==========================================
        # 変数の設定: 動かしたい距離を mm 単位で指定
        # ==========================================
        TARGET_DISTANCE_MM = 500.0  # 例: 50cm (500mm) 前進する

        # PID制御による精密移動の実行
        controller.move_forward_mm_pid(target_mm=TARGET_DISTANCE_MM, timeout_sec=8.0)

    except KeyboardInterrupt:
        # Ctrl+Cで中断された場合も安全に着陸させる（堅牢性）
        logging.warning("ユーザーによってプログラムが中断されました。緊急着陸します。")
    except Exception as e:
        # 予期せぬエラー発生時の補足（保守性）
        logging.error(f"実行中にエラーが発生しました: {e}")
    finally:
        # どのような終了経路をたどっても、確実に着陸してポートを閉じる
        controller.safe_land_and_close()