from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
start_time = time.time() # ¿ ̄Ûf{23
drone.takeoff()
drone.land()
end_time = time.time() # ¿ ̄Îf{23
elapsed_time = end_time - start_time # ¿ ̄f2ÿ
print(elapsed_time) # ¿ ̄f2û?
drone.close()