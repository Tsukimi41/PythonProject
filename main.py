from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
print("Paired!")
drone.takeoff()
print("In the air!")
#drone.hover(5)
time.sleep(5)
print("Hovering for 5 seconds!")
print("Landing")
drone.land()
drone.close()
print("Program complete")