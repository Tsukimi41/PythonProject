from codrone_edu.drone import *
import time

drone = Drone()
drone.pair()

def square():
    drone.takeoff()
    for i in range(4): # 4
        drone.set_yaw(0)
        drone.set_pitch(30)
        drone.move(1) # _2
        drone.set_pitch(0)
        drone.set_yaw(20)
        drone.move(2) # ïÞÿ¿2ö|»
    drone.land()


#関数の呼び出し
square()
drone.close()