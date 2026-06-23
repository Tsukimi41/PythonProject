from codrone_edu.drone import *
import time
# ドローンの初期設定
drone = Drone()
drone.pair()

# ==========================================
# PIDチューニング用ゲイン（係数）
# ※機体の状態に合わせてこの3つの数値を調整します
# ==========================================
Kp = 6.00  # Pゲイン: 誤差に対する反応の強さ Pのみ:6.0
Ki = 0.00  # Iゲイン: 蓄積された誤差の補正（小さめに設定）
Kd = 0.00 # Dゲイン: 振動を抑えるブレーキの強さ


target_z = 80  # 目標の高さ (80cm)

# PID計算用の変数
integral = 0
prev_error = 0
prev_time = time.time()

print("離陸します。")
drone.takeoff()
time.sleep(2)  # 離陸直後の姿勢安定待ち

print(f"目標高さ {target_z}cm でのPID制御を開始します。(Ctrl+Cで停止)")

try:
    while True:
        # 現在の時刻と、前回のループからの経過時間(dt)を取得
        current_time = time.time()
        dt = current_time - prev_time
        if dt <= 0:
            continue

        # ------------------------------------------------
        # 1. センサ値の読み取り
        # ------------------------------------------------
        x = drone.get_pos_x()
        y = drone.get_pos_y()
        z = drone.get_pos_z()  # 現在の高さ(cm)

        ax = drone.get_accel_x()
        ay = drone.get_accel_y()
        az = drone.get_accel_z()  # Z方向の加速度

        # ------------------------------------------------
        # 2. PIDの計算
        # ------------------------------------------------
        # 誤差 (Error)
        error = target_z - z

        # 積分 (Integral) : 誤差を経過時間で掛けて足し合わせる
        integral += error * dt
        # ワインドアップ対策: 積分値が大きくなりすぎて暴走するのを防ぐため制限
        integral = max(-50, min(50, integral))

        # 微分 (Derivative) : 誤差の変化率を計算 (または加速度 az を補助として使うことも可能)
        derivative = (error-prev_error)/dt

        # 出力スロットルの計算
        throttle = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # スロットル値をCoDroneの許容範囲(-100 〜 100)に制限
        throttle = max(-100, min(100, int(throttle)))

        # ------------------------------------------------
        # 3. ドローンへのコマンド送信
        # ------------------------------------------------
        drone.set_throttle(throttle)


        # 0.03秒間実行して次のループへ (ループを高速に回すことで滑らかに制御)
        drone.move(0.03)

        # デバッグ表示（調整時に数値を観察してください）
        print(f"Z:{z}cm | 誤差:{error} | スロットル:{throttle} (az:{az})")

        # 次のループのための準備
        prev_error = error
        prev_time = current_time

except KeyboardInterrupt:
    # ユーザーがプログラムを停止(Ctrl+C)したときの安全処理
    print("\n制御を中断し、着陸します。")
    drone.set_throttle(0)
    drone.land()
    drone.close()