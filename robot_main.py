from time import sleep
import numpy as np
import detection
import kinematics
from get_image import get_img
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot
import cv2

ROBOT_COM = 'COM3'

SPEED = 90  # robot's moving speed, max. is 100
DELAY = 3  # set delay for robot to move

POSE = {
    'Home': [48.6, -64.6, 411.1, -15.7, 0.0, -15.7],
    'Pick_Card_1': [15.0, 170.0, 180.0, 30.0, 0.0, 15.0],
    'Pick_Card_2': [15.0, 164.0, 79.0, 30.0, 0.0, 15.0],
    'Pick_Card_3': [15.0, 180.0, 250.0, 30.0, 0.0, 15.0],
    'Local_Reader': [-170.0, 20.0, 135.0, -30.0, 0.0, 20.0],
    'Position_1': [-93.6, -5.5, 312.5, -21.15, -0.25, -22.29]
}

TRAJECTORY_TO_P1 = [
    POSE['Home'],
    POSE['Pick_Card_1'],
    POSE['Pick_Card_2'],
    POSE['Pick_Card_3'],
    POSE['Home'],
    POSE['Local_Reader'],
    POSE['Position_1']
]

IM_SHAPE = (800, 600)
IM_CENTER = (400, 300)


def setup():
    try:
        robot = MyCobot(ROBOT_COM)
    except Exception as e:
        print("Error:" + str(e))
        return False

    if not robot.get_angles() or not robot.get_coords():
        print("False: Can't get robot's current pose.")
        return False

    if get_img() is False:
        print("False: Can't get image.")
        return False

    try:
        model = detection.DetectionModel()
    except Exception as e:
        print("Error:" + str(e))
        return False

    return robot, model


def run_trajectory_to_p1(robot):
    for i in range(len(TRAJECTORY_TO_P1)):
        robot.send_coords(TRAJECTORY_TO_P1[i], SPEED, 0)
        sleep(DELAY)
        if i == 2:
            robot.enable_pump()
            sleep(1)


def tune_pose(robot, model):
    rot_x = 2 * kinematics.D2M
    rot_y = 2 * kinematics.D2M

    rot_speed = 100
    rot_delay = 1

    boundary_x = (-30, 30)
    boundary_y = (-30, 0)

    while True:
        img = get_img()
        box = model.detect(img)

        if box is not False:
            box_center = ((box[0][0]+box[1][0])/2, (box[0][1]+box[1][1])/2)
            box_posi = detection.get_relative_center(box_center, IM_CENTER)

            if abs(box_posi[0]) > 80:
                rot_x = 5 * kinematics.D2M
            else:
                rot_x = 2 * kinematics.D2M

            if abs(box_posi[1]) > 80:
                rot_y = 4 * kinematics.D2M
            else:
                rot_y = 2 * kinematics.D2M

            print('rot_x', rot_x * kinematics.M2D)
            print('rot_y', rot_y * kinematics.M2D)

            angles = robot.get_angles()

            if angles:
                if box_posi[0] < boundary_x[0] or box_posi[0] > boundary_x[1]:
                    if box_posi[0] < 0:
                        robot.send_angle(
                            Angle.J5.value, angles[4]-rot_x, rot_speed)
                        sleep(rot_delay)
                    elif box_posi[0] > 0:
                        robot.send_angle(
                            Angle.J5.value, angles[4]+rot_x, rot_speed)
                        sleep(rot_delay)

                elif box_posi[1] < boundary_y[0] or box_posi[1] > boundary_y[1]:
                    if box_posi[1] < 0:
                        robot.send_angle(
                            Angle.J4.value, angles[3]+rot_y, rot_speed)
                        sleep(rot_delay)
                    elif box_posi[1] > 0:
                        robot.send_angle(
                            Angle.J4.value, angles[3]-rot_y, rot_speed)
                        sleep(rot_delay)
                else:
                    break
        else:
            print('no detection')
    return box


def robot_control(robot, model):
    robot.set_color(r=0, g=255, b=0)

    run_trajectory_to_p1(robot)
    box = tune_pose(robot, model)

    print('tune pose ok')

    box_area = detection.get_box_area(box)
    box_ratio = detection.get_box_hw_ratio(box)
    forward = round(-0.005 * box_area + -50 * box_ratio + 600, 1)

    print('Area', int(box_area))
    print('Ratio', round(box_ratio, 2))
    print('forward', forward)

    box_center = ((box[0][0]+box[1][0])/2, (box[0][1]+box[1][1])/2)
    box_posi = detection.get_relative_center(box_center, IM_CENTER)
    print('box_posi', box_posi)

    move = [0, 0, forward]
    coords = robot.get_coords()
    coords_n = kinematics.get_coords_new(coords, move)

    print('coords_n', coords_n)
    kinematics.is_valid(coords_n)

    robot.send_coords(coords_n, SPEED, 0)
    sleep(5)

    robot.send_coords(POSE['Home'], SPEED, 0)
    sleep(DELAY)

    robot.send_coords(POSE['Local_Reader'], SPEED, 0)
    sleep(5)

    robot.stop()
    robot.disable_pump()
    robot.set_color(r=255, g=0, b=0)


def main():
    if setup() is False:
        return False
    else:
        robot, model = setup()
    robot_control(robot, model)


if __name__ == '__main__':
    main()
