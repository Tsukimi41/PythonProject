from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
drone.drone_buzzer(Note.C4, 500)
drone.drone_buzzer(Note.C4, 10000)

drone.close()