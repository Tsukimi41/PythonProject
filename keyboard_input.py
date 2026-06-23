import time
from codrone_edu.drone import *
drone = Drone()
drone.pair()
#ユーザ入力
print("操作コマンド一覧")
print("w:up")
print("s:down")
print("a:turn left")
print("d:turn right")
print("i:front")
print("l:right")
print("j:left")
print("k:back")
print("q:quit")
drone.takeoff() #必要ならトリムを微調整する
power = 40
while True:
    drone.set_throttle(0)
    drone.set_roll(0)
    drone.set_yaw(0)
    drone.set_pitch(0)
    direction = input("")
    if direction == "w":
        drone.set_throttle(power)
        drone.move(1)
    elif direction == "s":
        drone.set_throttle(-power)
        drone.move(1)
    elif direction == "a":
        drone.set_yaw(-power)
        drone.move(1)
    elif direction == "d":
        drone.set_yaw(power)
        drone.move(1)
    elif direction == "i":
        drone.set_pitch(power)
        drone.move(1)
    elif direction == "k":
        drone.set_pitch(-power)
        drone.move(1)
    elif direction == "j":
        drone.set_roll(-power)
        drone.move(1)
    elif direction == "l":
        drone.set_roll(power)
        drone.move(1)
    elif direction == "q":
        time.sleep(1)
        drone.land()
        break
    else:
        print("無効なコマンド")

print("終了")
drone.land()
drone.close()



