from codrone_edu.drone import *
import time
import keyboard

# ==========================================
# PIDコントローラーのクラス化
# ==========================================
class PIDController:
    def __init__(self, kp, ki, kd, max_out=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out  # ドローンの最大傾き（パワー）を制限
        self.integral = 0
        self.prev_error = 0

    def calculate(self, target, current, dt):
        if dt <= 0:
            return 0

        error = target - current

        # 積分 (I)
        self.integral += error * dt
        self.integral = max(-30, min(30, self.integral))  # ワインドアップ防止

        # 微分 (D)
        derivative = (error - self.prev_error) / dt

        # 出力計算
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error

        # パワーが大きすぎると機体がひっくり返るため制限する
        return max(-self.max_out, min(self.max_out, int(output)))


# ==========================================
# メイン処理
# ==========================================
drone = Drone()
drone.pair()

# X(前後), Y(左右) のPIDゲイン
# ※風に打ち勝つにはPとIを少し強めにしますが、上げすぎると暴走します
pid_x = PIDController(kp=2.0, ki=0.1, kd=0.5)
pid_y = PIDController(kp=2.0, ki=0.1, kd=0.5)

# 目標座標の初期値（離陸した場所を 0, 0 とする）
target_x = 0.0
target_y = 0.0
MOVE_STEP = 5.0  # 1回のキー入力で移動させる目標距離(cm)

print("離陸します...")
drone.takeoff()
time.sleep(2)
drone.reset_sensor()  # 現在地を(0,0)にリセット

print("=== 座標ベースPID制御 開始 ===")
print("[W/S] 前後移動 (Target X 変更)")
print("[A/D] 左右移動 (Target Y 変更)")
print("[Q]   終了して着陸")

prev_time = time.time()

try:
    while True:
        current_time = time.time()
        dt = current_time - prev_time

        # ------------------------------------------------
        # 1. PCキーボード入力による「目標座標」の更新
        # ------------------------------------------------
        if keyboard.is_pressed('q'):
            print("終了コマンドを受信しました。")
            break
        elif keyboard.is_pressed('w'):
            target_x += MOVE_STEP
            time.sleep(0.001)  # キーの連続入力を少し防ぐ
        elif keyboard.is_pressed('s'):
            target_x -= MOVE_STEP
            time.sleep(0.001)
        elif keyboard.is_pressed('a'):
            target_y += MOVE_STEP  # 左がYのプラス
            time.sleep(0.001)
        elif keyboard.is_pressed('d'):
            target_y -= MOVE_STEP  # 右がYのマイナス
            time.sleep(0.001)

        # ------------------------------------------------
        # 2. センサ値の読み取り
        # ------------------------------------------------
        current_x = drone.get_pos_x()
        current_y = drone.get_pos_y()

        # ------------------------------------------------
        # 3. PID計算 (目標位置に留まるためのパワーを算出)
        # ------------------------------------------------
        # X軸（前後）のズレは、Pitch（前後傾き）で補正する
        pitch_power = pid_x.calculate(target_x, current_x, dt)

        # Y軸（左右）のズレは、Roll（左右傾き）で補正する
        roll_power = pid_y.calculate(target_y, current_y, dt)

        # ------------------------------------------------
        # 4. ドローンへコマンド送信
        # ------------------------------------------------
        drone.set_pitch(pitch_power)
        drone.set_roll(roll_power)
        drone.move(0.03)  # 0.05秒だけ適用してすぐ次のループへ

        # デバッグ表示（現在地と、計算された対抗パワーを確認）
        print(
            f"Target({target_x},{target_y}) | Current({current_x},{current_y}) | Power(P:{pitch_power}, R:{roll_power})")

        prev_time = current_time

except Exception as e:
    print(f"エラー発生: {e}")

finally:
    # 安全のための終了処理
    print("着陸します。")
    drone.set_pitch(0)
    drone.set_roll(0)
    drone.land()
    drone.close()