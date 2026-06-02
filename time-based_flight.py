from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
#--------関数の定義---------
def fly(seconds):
    start_time = time.time()
    while time.time() - start_time < seconds:
        drone.move()
#--------------------------------
drone.takeoff()
drone.set_pitch(30) # _2y» ̧v{ ̄Û
fly(2) # 2ûpq_2
drone.land()
drone.close()