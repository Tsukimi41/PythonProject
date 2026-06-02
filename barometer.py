from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
# #地上
# pressure_1 = drone.get_pressure()
# #Level2
# drone.takeoff()
# pressure_2 = drone.get_pressure()
# #Lv.3
# drone.set_throttle(30)
# drone.move(1)
# pressure_3 = drone.get_pressure()
# #Lv.3
# drone.move(1)
# pressure_4 = drone.get_pressure()
# #着陸
# drone.land()
# #結果
# print("On the Ground: ", pressure_1)
# print("Level 1: ", pressure_2)
# print("Level 2: ", pressure_3)
# print("Level 3: ", pressure_4)
# drone.close()


while True:
    print(drone.get_pressure())
    sleep(0.1)
drone.close()