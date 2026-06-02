from codrone_edu.drone import *
drone = Drone()
drone.pair()
while True:
    height = drone.get_height()
    print(height)
drone.land()
drone.close()


#距離センサーの限界は、150cmくらい。