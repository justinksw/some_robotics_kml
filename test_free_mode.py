from time import sleep

import kinematics
from pymycobot.mycobot import MyCobot


mycobot = MyCobot('COM3')
mycobot.set_color(r=255, g=0, b=0)
mycobot.set_free_mode()
sleep(5)

# mycobot.send_angles([0, 0, 0, 0, 0, 0], 50)
# sleep(5)

try:
    while True:

        coords = mycobot.get_coords()
        angles = mycobot.get_angles()

        print('coords', coords)
        print('angles', angles)

        # kinematics.is_valid(coords)

        sleep(3)

except Exception as e:
    print("Error:" + str(e))

# Home
# coords = [48.6, -64.6, 411.1, -15.7, 0.0, -15.7]
# angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
