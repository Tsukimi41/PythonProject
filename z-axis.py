from codrone_edu.drone import *
drone = Drone()
drone.pair()
drone.takeoff()
for i in 10:
    drone.set_throttle(100)  # 1ýóøût225{ ̄Ûÿ¶» ́{{Ng
    drone.move(1)  # 1û11ýóøûí_2ß ̄
    drone.set_throttle(-100)
    drone.move(1)
drone.land()
drone.close()