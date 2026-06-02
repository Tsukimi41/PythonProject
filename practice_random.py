from codrone_edu.drone import *
import time
import random

drone = Drone()
drone.pair()
for i in range(3):
    red = random.randint(0, 255)
    blue = random.randint(0, 255)
    green = random.randint(0, 255)
    drone.takeoff()
    time.sleep(0.05)
    drone.set_drone_LED(red, blue, green, 100)
    drone.land()
drone.close()