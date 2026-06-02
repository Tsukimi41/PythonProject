from codrone_edu.drone import *
drone = Drone()
drone.pair()
while True:
    temperature = drone.get_drone_temperature()
    print(temperature) # [Ofÿ#
drone.close()