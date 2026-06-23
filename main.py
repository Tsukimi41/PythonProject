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
    独立したPIDコントローラークラス。
    ドローンの制御ロジックを分離することで、保守性と再利用性を高めます。
    """

    def __init__(self, kp: float, ki: float, kd: float, output_limit: float):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        # モーター出力制限 (CoDrone EDUの操作量は通常 -100 〜 100)
        self.output_limit: float = output_limit

        self.integral: float = 0.0
        self.prev_error: float = 0.0

    def compute(self, target: float, current: float, dt: float) -> float:
        """
        現在の状態と目標値からPID演算を行い、操作量を返します。
        """
        # ゼロ除算の防止 (堅牢性)
        if dt <= 0.0:
            return 0.0

        error = target - current
        self.integral += error * dt

        # アンチワインドアップ (Integral Windup) 対策
        # 積分項が大きくなりすぎて目標値を大幅に行き過ぎるオーバーシュートを防ぐ（54.8gの軽量機体には必須）
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
    CoDrone EDU のハードウェア制御とPIDによる精密移動を行うラッパークラス。
    """

    def __init__(self):
        self.drone = Drone()
        self.is_flying = False

    def initialize_and_takeoff(self) -> None:
        """
        ペアリング、安全確認（バッテリー）、および離陸処理を行います。
        """
        logging.info("ドローンに接続しています...")
        self.drone.pair()

        # 飛行前のバッテリーチェック (堅牢性)
        battery_level = self.drone.get_battery()
        logging.info(f"現在のバッテリー残量: {battery_level}%")
        if battery_level < 15:
            logging.error("バッテリー残量が少なすぎます。飛行を中止します。")
            raise RuntimeError("Low Battery Warning")

        logging.info("離陸します...")
        self.drone.takeoff()
        self.is_flying = True

        # 離陸直後の姿勢安定のためのウェイト
        self.drone.hover(1)

        time.sleep(2)

    def move_forward_mm_pid(self, target_distance_mm: float, timeout_sec: float = 10.0) -> None:
        """
        オプティカルフローセンサーを利用し、現在の位置から指定された距離(mm)だけ前進します。
        ゲイン調整用に、X, Y, Zの現在位置とPIDの内部状態を連続出力します。
        """
        logging.info(f"目標距離: {target_distance_mm} mm のPID制御移動を開始します。")

        start_x_mm = self.drone.get_pos_x(unit="mm")
        target_pos_mm = start_x_mm + target_distance_mm

        # チューニング対象のPIDゲイン
        pid = PIDController(kp=0.31, ki=0.15, kd=0.04, output_limit=90.0)

        start_time = time.time()
        prev_time = start_time
        tolerance_mm = 10.0

        # 前回のモーター出力を記憶する変数
        prev_pitch_power = 0


        # 整定の確認のための変数
        settling_start_time = None
        required_settling_time = 1.0  # 許容誤差内にこの時間留まれば到達とみなす

        # ログのヘッダーを出力（見やすくするため）
        print(
            f"{'Time':>6} | {'TargetX':>7} | {'Curr_X':>7} | {'Curr_Y':>7} | {'Curr_Z':>7} | {'Error':>7} | {'Power':>5}")
        print("-" * 75)

        while True:
            current_time = time.time()
            dt = current_time - prev_time

            if (current_time - start_time) > timeout_sec:
                logging.warning("\nタイムアウトしました。")
                self.stop_and_hover()
                break

            if dt < 0.1:
                time.sleep(0.1 - dt)
                current_time = time.time()
                dt = current_time - prev_time

            # 1. センサーから現在のX, Y, Z位置を取得 (全てmm単位)
            current_x_mm = self.drone.get_pos_x(unit="mm")
            current_y_mm = self.drone.get_pos_y(unit="mm")  # 左右のズレ確認用
            current_z_mm = self.drone.get_pos_z(unit="mm")  # 高度維持の確認用

            error_mm = target_pos_mm - current_x_mm

            # 目標地点付近での停止判定
            if abs(error_mm) <= tolerance_mm:
                if settling_start_time is None:
                    # 初めて許容範囲に入った時間を記録
                    settling_start_time = current_time
                elif (current_time - settling_start_time) >= required_settling_time:
                    # 指定時間、許容範囲内に留まり続けたので「完全に到達」と判定
                    print("-" * 75)
                    logging.info(f"目標地点で安定しました。最終誤差: {error_mm:.2f} mm")
                    self.stop_and_hover()
                    break
            else:
                # もし勢い余って許容範囲から外れてしまったら、タイマーをリセットする
                settling_start_time = None

            # 2. PID出力の計算
            pitch_power = int(pid.compute(target=target_pos_mm, current=current_x_mm, dt=dt))

            # ==========================================
            # 3. ここに printf (print) を挿入！
            # ==========================================
            # 経過時間、目標X、現在X/Y/Z、誤差、モーター出力をフォーマットして出力
            elapsed = current_time - start_time
            print(
                f"{elapsed:>6.2f} | {target_pos_mm:>7.1f} | {current_x_mm:>7.1f} | {current_y_mm:>7.1f} | {current_z_mm:>7.1f} | {error_mm:>7.1f} | {pitch_power:>5}")

            # 4. ドローンへの移動コマンド送信(パワーが変化した時だけコマンドを送信する)
            if pitch_power != prev_pitch_power:
                self.drone.set_pitch(pitch_power)
                self.drone.move()
                prev_pitch_power = pitch_power  # 前回の値を更新


            prev_time = current_time

    def stop_and_hover(self) -> None:
        """
        移動を即座に停止し、その場でホバリングして機体を安定させます。
        """
        self.drone.set_pitch(0)
        self.drone.set_roll(0)
        self.drone.set_yaw(0)
        self.drone.set_throttle(0)
        self.drone.hover(1)

    def safe_land_and_close(self) -> None:
        """
        例外発生時も含め、必ず安全に着陸し通信を閉じるための終了処理。
        """
        try:
            if self.is_flying:
                logging.info("着陸態勢に入ります...")
                self.drone.land()
        except Exception as e:
            logging.error(f"着陸処理中にエラーが発生しました: {e}")
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
        TARGET_DISTANCE_MM = 500.0  # 例: 500mm (50cm) 前進する

        # PID制御による精密移動の実行
        controller.move_forward_mm_pid(target_distance_mm=TARGET_DISTANCE_MM, timeout_sec=20.0)

    except KeyboardInterrupt:
        # Ctrl+C で中断された場合のフェイルセーフ
        logging.warning("ユーザーによってプログラムが中断されました。緊急着陸します。")
    except Exception as e:
        logging.error(f"実行中にエラーが発生しました: {e}")
    finally:
        # 正常・異常に関わらず確実に着陸とポート切断を行う
        controller.safe_land_and_close()