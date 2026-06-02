import time
from codrone_edu.drone import *
drone = Drone()
drone.pair()
#回転数をユーザ入力
num_spins = int(input("ドローンを何回転させますか: "))
drone.takeoff()
drone.hover(1)
drone.set_yaw(50) # 回転速度
i = 0
while i < num_spins: # 0からカウント
    drone.move(5)
    i += 1
drone.land()