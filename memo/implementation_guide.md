# CoDrone EDU 内蔵センサ精密移動 実装手順書

## 1. 前提

外部測位は使わない。

使用するセンサ:

- optical flow position sensor
- bottom ToF
- front ToF
- barometer
- accelerometer
- gyroscope
- yaw

実装方針:

- 内蔵センサが信用できる範囲で最大限精密にする。
- センサ破綻時は追い続けず安全停止する。
- 長距離は最高速加速/急制動。
- 短距離は速度サーボ。
- yawと座標変換を最優先で正しく扱う。

## 2. 基本実行

```powershell
python main.py --x-mm 300 --no-auto-tune
python main.py --y-mm 300 --no-auto-tune
python main.py --z-mm 300 --no-auto-tune
```

複数軸:

```powershell
python main.py --x-mm 300 --y-mm 200 --z-mm 100 --no-auto-tune
```

長距離:

```powershell
python main.py --x-mm 600 --no-auto-tune
python main.py --y-mm 600 --no-auto-tune
```

## 3. ファイル構成

- `drone_precision.py`: 制御本体。
- `main.py`: CLI入口。
- `test_drone_precision.py`: ユニットテスト。
- `flight_logs/`: 飛行ログ。
- `experiment_logs/`: 実験解析。
- `memo/requirements.md`: 要件定義。
- `memo/control_spec.md`: 制御仕様。
- `memo/implementation_guide.md`: 実装手順。

## 4. 制御ループ

1. ペアリング。
2. バッテリー確認。
3. gyro reset。
4. takeoff。
5. hoverして基準位置を複数サンプルで取得。
6. 目標絶対位置を計算。
7. 位置/モーションセンサを読む。
8. 世界座標誤差を計算。
9. yawで機体座標誤差へ変換。
10. 距離に応じて高速フェーズまたは速度サーボを選ぶ。
11. yaw誤差が大きければ水平出力を抑える。
12. 水平誤差が増幅していればドリフトガードをかける。
13. 高度が落ちたら水平制御を捨てて上昇回復する。
14. 整定したらhover。
15. 異常なら安全停止。

## 5. 実験手順

### 300mm単軸

```powershell
python main.py --x-mm 300 --no-auto-tune
python main.py --y-mm 300 --no-auto-tune
python main.py --z-mm 300 --no-auto-tune
```

見る項目:

- `termination_reason`
- `final_error_x_mm`
- `final_error_y_mm`
- `final_error_z_mm`
- `final_error_yaw_deg`
- `peak_speed_x/y/z`
- `oscillation_count_x/y/z`
- `body_error_x/y`
- `stale_ratio`

### 600mm高速

```powershell
python main.py --x-mm 600 --no-auto-tune
python main.py --y-mm 600 --no-auto-tune
```

見る項目:

- `accelerate`
- `brake`
- `settle`
- 最大加速指令
- 最大制動指令
- 制動後の発散有無

## 6. チューニング順序

1. yaw符号とyaw保持を確認する。
2. body_errorとpitch/rollの向きを確認する。
3. 高度が落ちていないか確認する。
4. stale_ratioを見る。
5. 水平誤差が時間で増幅していないか確認する。
6. 速度サーボの `max_speed`, `velocity_gain`, `position_gain` を調整する。
7. bang-bang開始距離を調整する。

## 7. 変更してはいけないもの

不用意に変更禁止:

- pitch/roll/yaw/throttle の符号。
- 世界座標から機体座標への変換。
- sensor_timeout。
- altitude_fault。
- attitude_fault。
- 高度回復ガード。

これらは安全性と発散防止に直結する。

## 8. テスト

```powershell
python -m unittest -v test_drone_precision.py
```

構文チェック:

```powershell
python -B -c "import ast, pathlib; [ast.parse(pathlib.Path(p).read_text(encoding='utf-8'), filename=p) for p in ('drone_precision.py','test_drone_precision.py','main.py')]; print('syntax ok')"
```

## 9. 完成判断

CoDrone EDU単体での完成判断:

- 300mm x/y/z が危険な発散を起こさない。
- yaw drift時に円運動へ入りにくい。
- 時間経過で水平誤差が増幅した場合、出力抑制が入る。
- 高度低下時に上昇回復へ移る。
- センサ停止時に早く止まる。
- ログから原因を追える。

## 10. 最終結論

外部測位なしのCoDrone EDUで最適な制御は、PID単独ではなく、状態機械、座標変換、速度サーボ、制動距離、センサ信頼度判定、ドリフト抑制を組み合わせたハイブリッド制御である。

厳密な絶対位置一致は保証しない。内蔵センサで観測できる範囲の中で、最速・精密・堅牢な相対移動を実現する。
