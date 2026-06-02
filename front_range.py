from codrone_edu.drone import *
drone = Drone()
drone.pair()
while True:
    front = drone.get_front_range()
    print(front)
drone.land()
drone.close()


#front_rangeの限界は、3.0~210cmくらい。
