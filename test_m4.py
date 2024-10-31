import queue
import threading
from time import sleep
from urllib.request import urlopen

import cv2
import numpy as np

import detection
import kinematics
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot

IM_CENTER = (400, 300)

IM_SHAPE = (800, 600)
codec = cv2.VideoWriter_fourcc(*'MJPG')
VideoWriter = cv2.VideoWriter('result.avi', codec, 5, IM_SHAPE)


def run_trajectory(robot):

    speed = 90
    delay = 3

    # # Home
    # robot.send_angles([0, 0, 0, 0, 0, 0], speed)
    # sleep(delay)

    # # Pick card 1
    # robot.send_coords(
    #     [15.0, 170.0, 180.0, 30.0, 0.0, 15.0], speed, 0)
    # sleep(delay)

    # # Pick card 2
    # robot.send_coords(
    #     [15.0, 164.0, 79.0, 30.0, 0.0, 15.0], speed, 0)
    # sleep(delay)

    # robot.enable_pump()
    # sleep(1)

    # # Pick card 3
    # robot.send_coords(
    #     [15.0, 180.0, 250.0, 30.0, 0.0, 15.0], speed, 0)
    # sleep(delay)

    # Home
    robot.send_angles([0, 0, 0, 0, 0, 0], speed)
    sleep(delay)

    # # Local reader
    # robot.send_coords(
    #     [-170.0, 20.0, 135.0, -30.0, 0.0, 20.0], speed, 0)
    # sleep(delay)

    # Position 1
    robot.send_coords(
        [-93.6, -5.5, 312.5, -21.15, -0.25, -22.29], speed, 0)
    sleep(delay)


class StreamParse(threading.Thread):
    def __init__(self, t_name, q, url, stream):
        threading.Thread.__init__(self, name=t_name)
        self.data = q
        self.url = url
        self.stream = stream
        self.do_run = True

    def run(self):
        CAMERA_BUFFRER_SIZE = 4096
        bts = b''
        while getattr(self, 'do_run') is True:
            try:
                bts += self.stream.read(CAMERA_BUFFRER_SIZE)
                jpghead = bts.find(b'\xff\xd8')
                jpgend = bts.find(b'\xff\xd9')
                if jpghead > -1 and jpgend > -1:
                    jpg = bts[jpghead:jpgend+2]
                    bts = bts[jpgend+2:]
                    img = cv2.imdecode(np.frombuffer(
                        jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

                    self.data.put(img)
            except:
                bts = b''
                self.stream = urlopen(self.url)
                continue
        print("%s finished!" % self.getName())


class Detection(threading.Thread):
    def __init__(self, t_name, q, model, box_area, box_posi, box_ratio):
        threading.Thread.__init__(self, name=t_name)
        self.data = q
        self.do_run = True
        self.model = model

        self.box_area = box_area
        self.box_posi = box_posi
        self.box_ratio = box_ratio

    def run(self):
        while getattr(self, 'do_run') is True:
            img = self.data.get()

            box = self.model.detect(img)

            if box is not False:
                box_center = ((box[0][0]+box[1][0])/2, (box[0][1]+box[1][1])/2)

                ##
                relative_center = detection.get_relative_center(
                    box_center, IM_CENTER)
                box_area = detection.get_box_area(box)
                box_ratio = detection.get_box_hw_ratio(box)

                self.box_posi.put(relative_center)
                self.box_area.put(box_area)
                self.box_ratio.put(box_ratio)

                cv2.putText(img,
                            str((int(box_area), round(box_ratio, 2))),  # text
                            (int(box[0][0]), int(box[1][1])),  # coord
                            cv2.FONT_HERSHEY_SIMPLEX,  # font
                            1,  # font scale
                            (0, 0, 255),
                            1,  # line thickness
                            cv2.LINE_AA)

                cv2.putText(img,
                            str((round(relative_center[0], 1), round(
                                relative_center[1], 1))),
                            # + str(int(math.sqrt(relative_center[0]**2+relative_center[1]**2))),  # text
                            (int(box_center[0]), int(box_center[1])),  # coord
                            cv2.FONT_HERSHEY_SIMPLEX,  # font
                            1,  # font scale
                            (255, 0, 0),
                            1,  # line thickness
                            cv2.LINE_AA)

                line = detection.get_center_difference_line(
                    box_center, IM_CENTER)

                cv2.rectangle(img, (int(box[0][0]), int(box[0][1])),
                              (int(box[1][0]), int(box[1][1])), (0, 255, 0), 2)

                cv2.line(img, line[0], line[1], (255, 0, 0), 2)

            else:
                pass
            cv2.imshow('', img)

            # VideoWriter.write(img)

            self.data.task_done()

            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
        print("%s finished!" % self.getName())


class RobotControl(threading.Thread):
    def __init__(self, t_name, robot, box_area, box_posi, box_ratio):
        threading.Thread.__init__(self, name=t_name)
        self.robot = robot
        self.do_run = True

        self.box_area = box_area
        self.box_posi = box_posi
        self.box_ratio = box_ratio

    def run(self):
        self.robot.set_color(r=0, g=255, b=0)

        run_trajectory(self.robot)

        rot_x = 5 * kinematics.D2M
        rot_y = 5 * kinematics.D2M

        # 1 ~ 15
        # 2 ~ 30
        # 5 ~ 60~80

        rot_speed = 100
        rot_delay = 1

        boundary_x = 30
        boundary_y = 40

        while getattr(self, 'do_run') is True:
            posi = self.box_posi.get()

            if abs(posi[0]) > 80:
                rot_x = 5 * kinematics.D2M
            else:
                rot_x = 2 * kinematics.D2M

            if abs(posi[1]) > 80:
                rot_y = 4 * kinematics.D2M
            else:
                rot_y = 2 * kinematics.D2M

            # center_distance = math.sqrt(posi[0]**2 + posi[1]**2)

            # print(f'posi ({round(posi[0], 1)}, {round(posi[1], 1)})')
            # print('center distance', round(center_distance, 1))

            sleep(1)
            angles = self.robot.get_angles()

            if angles:
                if posi[0] < -boundary_x or posi[0] > boundary_x:
                    if posi[0] < 0:
                        self.robot.send_angle(
                            Angle.J5.value, angles[4]-rot_x, rot_speed)
                        sleep(rot_delay)
                    elif posi[0] > 0:
                        self.robot.send_angle(
                            Angle.J5.value, angles[4]+rot_x, rot_speed)
                        sleep(rot_delay)

                elif posi[1] < -boundary_y or posi[1] > 0:
                    if posi[1] < 0:
                        self.robot.send_angle(
                            Angle.J4.value, angles[3]+rot_y, rot_speed)
                        sleep(rot_delay)
                    elif posi[1] > 0:
                        self.robot.send_angle(
                            Angle.J4.value, angles[3]-rot_y, rot_speed)
                        sleep(rot_delay)
                else:
                    break

        if getattr(self, 'do_run') is True:
            area = self.box_area.get()
            ratio = self.box_ratio.get()
            print('Area', int(area))
            print('Ratio', round(ratio, 2))

            # forward = round(-0.005 * area +
            #                 -50 * ratio + 580, 1)
            forward = round(-0.005 * area +
                            -50 * ratio + 600, 1)
            print('forward', forward)

            # forward = 378
            # print(f'{int(area)},{round(ratio, 2)},{forward}')

            move = [0, 0, forward]
            coords = self.robot.get_coords()
            coords_n = kinematics.get_coords_new(coords, move)

            print('calculated coord_n', coords_n)

            coords_n[2] = 113  # set Z coordinate
            coords_n[3] = -30
            coords_n[4] = 0
            coords_n[5] = -22
            print('tuned coords_n', coords_n)

            kinematics.is_valid(coords_n)

            self.robot.send_coords(coords_n, 75, 0)
            sleep(5)

        self.robot.stop()
        self.robot.disable_pump()
        self.robot.set_color(r=255, g=0, b=0)
        print("%s finished!" % self.getName())


def main():
    stream_url = 'http://192.168.4.1/'
    robot_com = 'COM3'
    robot = MyCobot(robot_com)

    if not robot.get_angles() or not robot.get_coords():
        print("False: Can't get robot's current pose.")
        return False
    else:
        # print(robot.get_angles())
        # print(robot.get_coords())
        pass

    try:
        stream = urlopen(stream_url)
        model = detection.DetectionModel()

    except Exception as e:
        print("Error:" + str(e))
        return False

    img_q = queue.LifoQueue()

    box_area_q = queue.LifoQueue()
    box_posi_q = queue.LifoQueue()
    box_ratio_q = queue.LifoQueue()

    stream_parse_t = StreamParse(
        'StreamParse', img_q, stream_url, stream)
    detection_t = Detection(
        'Detection', img_q, model, box_area_q, box_posi_q, box_ratio_q)
    robot_control_t = RobotControl(
        'Robot Control', robot, box_area_q, box_posi_q, box_ratio_q)

    stream_parse_t.start()
    detection_t.start()
    robot_control_t.start()

    img_q.join()
    box_area_q.join()
    box_posi_q.join()
    box_ratio_q.join()

    detection_t.join()

    robot_control_t.do_run = False
    robot_control_t.join()

    stream_parse_t.do_run = False
    stream_parse_t.join()

    print('All threads terminate!')


if __name__ == '__main__':
    main()
