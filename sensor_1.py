from codrone_edu.drone import *
drone = Drone()
drone.pair()
while True:
    #print("Temperature is: ", drone.get_drone_temperature())
    #print("Pressure is: ", drone.get_pressure())
    #print("Battery is: ", drone.get_battery())
    print("Angle is: ", drone.get_angle_x(), drone.get_angle_y(), drone.get_angle_z())
    sleep(0.1)
drone.close()
