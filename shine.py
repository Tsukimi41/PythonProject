from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
#¿ÿRed
for i in range(0, 256):
    drone.set_drone_LED(i, 255 - i, i, 100)
    drone.set_controller_LED(i, 255 - i, i, 100)
    time.sleep(0.01)
drone.close()