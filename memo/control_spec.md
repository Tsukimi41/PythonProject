# CoDrone EDU 内蔵センサ精密移動 制御仕様書

## 最終制御方式

CoDrone EDU単体で最適な方式は純粋なPIDではない。

採用方式:

```text
状態機械
+ 内蔵センサ信頼度判定
+ yaw補正つき世界座標->機体座標変換
+ 距離別bang-bang加速/制動
+ 残距離ベース速度サーボ
+ 水平ドリフト抑制
+ 高度回復ガード
+ 低速最終補正
```

PIDは補助として使う。主役は速度サーボ、制動距離、状態遷移である。

## なぜPID単独ではないか

CoDrone EDUは水平移動時にpitch/rollで傾く。傾くと下向きoptical flowとToFの測定が悪化する。

さらにyawがずれると、世界座標のx/y方向と機体のpitch/roll方向が一致しない。

この状態で世界座標誤差をそのままPIDに入れると、補正方向が回転して円運動や発散が起きる。

## 座標変換

位置センサのx/yは世界座標、pitch/roll指令は機体座標である。

水平制御前に必ず現在yawで変換する。

```text
body_x = world_x * cos(yaw) + world_y * sin(yaw)
body_y = -world_x * sin(yaw) + world_y * cos(yaw)
```

- `body_x` を pitch 制御へ渡す。
- `body_y` を roll 制御へ渡す。

## yaw制御

実機ログから、yaw指令の符号は次で固定する。

```python
FIXED_CONTROL_SIGNS["yaw"] = -1.0
```

yaw誤差が大きい場合:

- 8deg超: pitch/rollを段階的に縮小する。
- 22deg超: pitch/rollを最大8に制限する。
- 55deg超: `attitude_fault` として安全停止する。

## 制御フェーズ

### acquire

離陸後、複数サンプルを平均して基準位置を取得する。

### accelerate

長距離のみ最大加速する。

- X/Y: 500mm以上。
- Z: 300mm以上。

300mm級のX/Yでは使わない。短距離で最高速加速すると、制動遅れとセンサ誤差で精度が落ちる。

### brake

制動距離に入ったら最大制動する。

```text
stopping_distance = v^2 / (2a) + v * sensor_latency + margin
```

### settle

残距離から許容速度を計算し、その速度へ追従する。

```text
reachable_speed = sqrt(2 * brake_accel * remaining_distance)
desired_speed = min(max_speed, reachable_speed * 0.82)
command = velocity_gain * (desired_velocity - current_velocity)
        + position_gain * position_error
```

制動距離内で速く接近している場合は、加速指令よりブレーキを優先する。

### final hold

位置誤差と速度が許容範囲内に一定時間入ったら完了する。

## 水平ドリフト抑制

内蔵センサは時間経過で誤差が増える場合がある。

そのため、直近の水平誤差履歴を監視する。

抑制条件:

- 最近の最良誤差から大きく悪化している。
- x/y誤差の符号反転が多い。
- 直前区間より平均誤差が増えている。

抑制時:

- pitch/rollを0.55または0.35倍にする。
- pitch/rollの積分を0.45倍へ減衰する。
- ログに警告を出す。

## 高度ガード

水平移動中に高度が急落したら水平制御を捨てる。

```text
pitch = 0
roll = 0
yaw = 0
throttle = 80
```

目的:

- optical flow/ToFが破綻する前に高度を戻す。
- 傾いたまま床付近へ落ちるのを防ぐ。

## センサ異常

### sensor_timeout

位置更新が0.65秒以上止まった場合。

### altitude_fault

目標高度が十分高いにもかかわらず、実高度が危険域に落ちた場合。

### attitude_fault

yaw誤差が55度を超えた場合。

### runaway

高速フェーズで目標から離れる挙動が続いた場合。

## ログ

各sampleに記録する。

- 世界座標位置/誤差/速度。
- 機体座標誤差/速度。
- pitch/roll/throttle/yaw指令。
- PID積分値。
- 制御フェーズ。
- 制動距離。
- stale状態。

## 最終結論

CoDrone EDU単体では、センサが信用できる範囲での最速・精密・堅牢な相対移動を目指す。

真の絶対位置厳密一致は保証しない。外部測位なしでは観測できないためである。
