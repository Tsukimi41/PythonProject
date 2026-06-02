from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
drone.takeoff()

start_time = time.time()
end_time = start_time + 30 # 30ûß~f{2 ̄Û

drone.set_pitch(50)
drone.set_yaw(50)
while start_time < end_time:
    drone.move()
    start_time = time.time() # ÿf{2öõ

drone.land()
print("飛行時間:", time.time() - (end_time - 30), "秒")
drone.close()


## move()の処理時間によるずれ、time()の取得タイミングのずれ、Pythonのループ処理によるわずかなずれにより
# タイミングの精度が必要な時には、注意が必要
