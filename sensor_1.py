from codrone_edu.drone import *
import time
drone = Drone()
drone.pair()
i=1
while True:
    #print("Temperature is: ", drone.get_drone_temperature())
    #print("Pressure is: ", drone.get_pressure())
    #print("Battery is: ", drone.get_battery())
    #print(drone.get_pos_x(), drone.get_pos_y(), drone.get_pos_z())
    colors = drone.get_colors(kind = "rgb")
    print( {i: colors})
    i += 1
    time.sleep(0.1)


#drone.close()