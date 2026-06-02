from codrone_edu.drone import *
drone = Drone()
drone.pair()
#¿ ̄ ̄Û
power = 50
move_time = 2 # _2fÿ¿uÿ
yaw_power = -50 # ÿïÞÿ-ÿÿ1+ÿó
turn_time = 0.8 # 72þïÞ{ßzfÿ}ÿwÿ¿u|ß
drone.takeoff()
#Þ¿5Þ~ûü÷
for i in range(5):
    drone.set_pitch(power)
    drone.move(move_time)
    drone.set_pitch(0)
    drone.set_yaw(yaw_power)
    drone.move(turn_time)
    drone.set_yaw(0)
drone.land()
drone.close()