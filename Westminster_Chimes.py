from codrone_edu.drone import *

drone = Drone()
drone.pair()
drone.set_drone_LED(255,255,255,255)
# 【第1フレーズ】キーン・コーン・カーン・コーン
drone.drone_buzzer(Note.A4, 1000)
drone.drone_buzzer(Note.F4, 1000)
drone.drone_buzzer(Note.G4, 1000)
drone.drone_buzzer(Note.C4, 2000)

# 1秒待機（余韻のための隙間を空けたい場合は、音の鳴らない待機処理を入れると自然です）
# time.sleep(1)

# 【第2フレーズ】キーン・コーン・カーン・コーン
drone.drone_buzzer(Note.C4, 1000)
drone.drone_buzzer(Note.G4, 1000)
drone.drone_buzzer(Note.A4, 1000)
drone.drone_buzzer(Note.F4, 2000)

# 【第3フレーズ】キーン・コーン・カーン・コーン
drone.drone_buzzer(Note.A4, 1000)
drone.drone_buzzer(Note.G4, 1000)
drone.drone_buzzer(Note.F4, 1000)
drone.drone_buzzer(Note.C4, 2000)

# 【第4フレーズ】キーン・コーン・カーン・コーン
drone.drone_buzzer(Note.C4, 1000)
drone.drone_buzzer(Note.G4, 1000)
drone.drone_buzzer(Note.A4, 1000)
drone.drone_buzzer(Note.F4, 3000) # 最後は一番長く響かせる
drone.close()