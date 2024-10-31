import queue
import threading
import time
from time import sleep
from urllib.request import urlopen

import cv2
import numpy as np

import kinematics
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot


class ReadStream(threading.Thread):
    def __init__(self, t_name, queue, stream):
        threading.Thread.__init__(self, name=t_name)
        self.data = queue
        self.stream_copy = stream
        self.stream = stream

    def run(self):
        CAMERA_BUFFRER_SIZE = 4096
        bts = b''
        while True:
            try:
                bts += self.stream.read(CAMERA_BUFFRER_SIZE)
                jpghead = bts.find(b'\xff\xd8')
                jpgend = bts.find(b'\xff\xd9')
                if jpghead > -1 and jpgend > -1:
                    jpg = bts[jpghead:jpgend+2]
                    bts = bts[jpgend+2:]
                    img = cv2.imdecode(np.frombuffer(
                        jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

                    cv2.imshow('Stream', img)
                    self.data.put(img)
                k = cv2.waitKey(1)

            except Exception as e:
                print("Error:" + str(e))
                bts = b''
                self.stream = self.stream_copy
                continue

            if k & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
        print("%s finished!" % self.getName())


class MoveRobot(threading.Thread):
    def __init__(self, t_name, queue, robot, thread1):
        threading.Thread.__init__(self, name=t_name)
        self.data = queue
        self.robot = robot
        self.thread1 = thread1

    def run(self):
        self.robot.set_color(r=0, g=255, b=0)
        # self.robot.set_free_mode()

        self.robot.send_coords(
            [-93.6, -5.5, 312.5, -21.15, -0.25, -22.29], 75, 0)
        sleep(5)

        angles = self.robot.get_angles()

        c = 0
        while True and self.thread1.is_alive():
            # coords = self.robot.get_coords()
            # angles = self.robot.get_angles()
            # print(f'coords: {coords}')
            # print(f'angles: {angles}')

            for i in range(-10, 20, 5):
                i = i * kinematics.D2M
                self.robot.send_angle(Angle.J5.value, angles[4]+i, 100)
                sleep(3)

                for j in range(-10, 20, 5):
                    j = j * kinematics.D2M
                    self.robot.send_angle(Angle.J4.value, angles[3]+j, 100)
                    sleep(3)

                    img = self.data.get()

                    cv2.imwrite('data/p4_{}.jpg'.format(c), img)

                    print('Saved picture')
                    c += 1

            break
        self.robot.set_color(r=255, g=0, b=0)
        print("%s finished!" % self.getName())


def main():
    URL = 'http://192.168.4.1/'
    try:
        stream = urlopen(URL)
        mycobot = MyCobot('COM3')
    except Exception as e:
        print("Error:" + str(e))
        return False

    q = queue.LifoQueue()
    producer = ReadStream('Cam Stream.', q, stream)
    consumer = MoveRobot('Robot', q, mycobot, producer)
    producer.start()
    consumer.start()
    q.join()
    producer.join()
    consumer.join()
    print('All threads terminate!')


if __name__ == '__main__':
    main()


# # set2
# coords: [-7.2, -226.4, 160.3, -21.32, 0.29, -15.83]
# angles: [14.98, 6.18, -14.17, -5.72, -10.06, 14.11]

    # while getattr(self, 'do_run') is True:
    #     print('coords: ', self.robot.get_coords())
    #     print('angles: ', self.robot.get_angles())
    #     # sleep(1)

    #     # elif k & 0xFF == ord('w'):
    #     #     coords = self.robot.get_coords()
    #     #     x = coords[0] + 5
    #     #     self.robot.send_coord(Coord.X.value, x, 75)
    #     #     sleep(1)

    #     # elif k & 0xFF == ord('a'):
    #     #     coords = self.robot.get_coords()
    #     #     y = coords[1] + 5
    #     #     self.robot.send_coord(Coord.Y.value, y, 75)
    #     #     sleep(1)

    #     # elif k & 0xFF == ord('s'):
    #     #     coords = self.robot.get_coords()
    #     #     x = coords[0] - 5
    #     #     self.robot.send_coord(Coord.X.value, x, 75)
    #     #     sleep(1)

    #     # elif k & 0xFF == ord('d'):
    #     #     coords = self.robot.get_coords()
    #     #     y = coords[1] - 5
    #     #     self.robot.send_coord(Coord.Y.value, y, 75)
    #     #     sleep(1)

    #     # elif k & 0xFF == ord('r'):
    #     #     coords = self.robot.get_coords()
    #     #     z = coords[3] + 5
    #     #     self.robot.send_coord(Coord.Z.value, z, 75)
    #     #     sleep(1)

    #     # elif k & 0xFF == ord('f'):
    #     #     coords = self.robot.get_coords()
    #     #     z = coords[3] - 5
    #     #     self.robot.send_coord(Coord.Z.value, z, 75)
    #     #     sleep(1)


#

    #

    # delta_x = 5
    # delta_y = 5
    # coords = self.robot.get_coords()
    # if posi[0] > 0:
    #     self.robot.send_coord(Coord.X.value, coords[0] + delta_x)
    #     sleep(5)
    # elif posi[0] < 0:
    #     self.robot.send_coord(Coord.X.value, coords[0] - delta_x)
    #     sleep(5)
    # else:
    #     pass
    # if posi[1] > 0:
    #     self.robot.send_coord(Coord.Y.value, coords[1] + delta_y)
    #     sleep(5)
    # elif posi[1] < 0:
    #     self.robot.send_coord(Coord.Y.value, coords[1] - delta_y)
    #     sleep(5)
    # else:
    #     pass


#

# Approach 3

# Start Pose

# End Pose
