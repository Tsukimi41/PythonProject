from codrone_edu.drone import *
drone = Drone()
drone.pair()
while True:
    x_angle = drone.get_angle_x() # ýüûÿXþ
    y_angle = drone.get_angle_y() # ôóñÿYþ
    z_angle = drone.get_angle_z() # øüÿZþ
    print(x_angle)
    print(y_angle)
    print(z_angle)
drone.close()