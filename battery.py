# from codrone_edu.drone import *
# drone = Drone()
# drone.pair()
# battery = drone.get_battery()
# print("Battery Level: ", battery)
# drone.close()

from codrone_edu.drone import *
drone = Drone()
drone.pair()
battery = drone.get_battery()
print("Before: ", battery)
drone.takeoff()
drone.hover(10)
drone.land()
battery = drone.get_battery()
print("After: ", battery)
drone.close()