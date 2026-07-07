CoDrone EDU (Robolink)

Weight  54.8 grams (with battery) 
Motors  4 x coreless DC motors 
Maximum Speed  2.5 meters/second (9km/hr) 
138.8 mm x 138.5 mm x 34.8 mm 

Communication Protocol  2.4 GHz Radio Frequency (Not Wi-Fi or Bluetooth compatible) 
Communication Chip  Essemi HW2000B 2.4G Transceiver 
Range  50 meters (line-of-sight)
Sensors ●Barometer 
●3-axis accelerometer 
●3-axis gyroscope 
●Temperature sensor
●2 x RGB color sensors (bottom) 
●Time-of-flight front range sensor (up to 1.5m) 
●Time-of-flight bottom range sensor (up to 1.5m) 
●Optical flow position sensor

## Final Goal

CoDrone EDUをPID制御で指定量だけ精密に移動させる。最終的にはx, y, zの各軸に対して移動指令値を与え、指令された相対移動量だけ正確に移動して停止する制御系を構築する。

風、電波干渉、センサー誤差、物理的な妨害などの外乱を受けても、外乱後に目標位置へ復帰し、最終停止位置を可能な限り厳密に一致させる。単に一定時間だけ速度指令を出すのではなく、センサー値から現在位置と速度を推定し、目標位置との誤差が許容範囲内になるまで閉ループで補正する。

最優先事項は最速である。理論上可能な範囲で最短時間移動を目指し、加速区間では安全に制御可能な最大速度へ素早く到達し、停止直前では急減速して目標位置に収束させる。最高速移動と急ブレーキ停止を前提にしつつ、オーバーシュート、振動、積分飽和、センサー遅延、モーター応答遅れを抑える。

## Control Requirements

- x, y, zの各軸に独立した指令値を送れること。
- PID制御を基本とし、必要に応じて速度制限、加速度制限、フィードフォワード、デッドバンド、積分リセット、アンチワインドアップを組み合わせること。
- 位置誤差、速度、加速度、姿勢、センサー信頼度を監視し、外乱を検知した場合でも目標位置への再収束を継続すること。
- 最速移動のため、目標距離に応じて加速、最高速巡航、急減速、精密停止のフェーズを切り替えること。
- 最終位置合わせでは速度を落として微調整し、許容誤差内に一定時間収まった場合のみ到達完了と判定すること。
- 物理的な妨害や測位不能が続く場合は、無理に出力を上げ続けず、安全停止または再試行状態へ遷移すること。

## Design Priorities

1. 最速性: 制御可能な範囲で最高速移動と最短停止を目指す。
2. 精密性: 最終位置誤差を最小化し、停止後のドリフトも補正する。
3. 堅牢性: 風、干渉、センサー誤差、外部妨害を前提に復帰できる構成にする。
4. 保守性: PIDゲイン、速度制限、許容誤差、安全閾値を設定値として分離し、軸ごとの制御ロジックを共通化する。
