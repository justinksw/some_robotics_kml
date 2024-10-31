import numpy as np
from transforms3d import euler
from transforms3d import quaternions as quat

D2R = np.pi / 180  # degree_to_radian
R2D = 180 / np.pi  # radian_to_degree

D2M = D2R * 10  # degree_to_mycobot
M2D = R2D / 10  # mycobot_to_degree

R2M = 10  # radian_to_mycobot
M2R = 1/10  # mycobot_to_radian


def is_valid(coords):
    x_B = np.array(coords[0:3])
    offset = np.array([0, 0, 130])  # offset joint-2 to base

    x_j2 = x_B - offset
    x_j2_norm = np.linalg.norm(x_j2)

    if x_j2_norm < 280:  # handbook states 280
        print('whithin workspace', x_j2_norm)
        return True
    else:
        print('out of workspace', x_j2_norm)
        return False


def get_coords_new(coords_cur, move_vec):
    """
    Args:
        coords (list): current coordinate of end-effector wrt Base
        move_vec (list): moving vector wrt end-effector

    Returns:
        list: 1x6: new coordinate of end-effector wrt Base
    """
    r_f_e = move_vec

    r_e_B = coords_cur[0:3]
    eulers_e_B = coords_cur[3:6]  # from mycobot

    q_e_B = euler.euler2quat(
        eulers_e_B[0]*M2R,
        eulers_e_B[1]*M2R,
        eulers_e_B[2]*M2R, 'sxyz')  # !!! Angles in RADIAN !!!

    while 1:
        r_f_B = quat.rotate_vector(r_f_e, q_e_B)

        x_B = r_e_B + r_f_B

        coords_new = [x_B[0], x_B[1], x_B[2],
                      eulers_e_B[0], eulers_e_B[1], eulers_e_B[2]]
        coords_new = [round(i, 2) for i in coords_new]

        # set final z coordinate
        coords_new[2] = 114
        # set final orientation
        coords_new[3] = -30
        coords_new[4] = 0
        coords_new[5] = -22

        if is_valid(coords_new):
            break
        else:
            # print('invalid coords_new', coords_new)
            r_f_e[2] -= 5

    return coords_new


# def get_move_vec(box_posi, step):
#     """get move vector base on current detection box position in image

#     Args:
#         posi (tuple): center position of detection box in image
#         step (int/float): moving step

#     Returns:
#         numpy.ndarray: moving vector
#     """
#     move_vec = box_posi / np.linalg.norm(box_posi) * step
#     move_vec = np.append(move_vec, 0)

#     mat = np.array([[1, 0, 0],
#                     [0, -1, 0],
#                     [0, 0, -1]])

#     q = quat.mat2quat(mat)
#     move_vec = quat.rotate_vector(move_vec, q)
#     return move_vec


# def move_robot(robot, coords_new):
#     coords_old = robot.get_coords()

#     robot.send_coords(coords_new, 75, 0)
#     sleep(5)

#     if robot.get_coords() == coords_old:
#         print('in')
#         robot.send_coord(Coord.X.value, coords_new[0], 75)
#         sleep(2)
#         robot.send_coord(Coord.Y.value, coords_new[1], 75)
#         sleep(2)
#         robot.send_coord(Coord.Z.value, coords_new[2], 75)
#         sleep(2)


# if __name__ == '__main__':
#     mycobot = MyCobot('COM3')
#     mycobot.set_color(r=0, g=255, b=0)

#     mycobot.send_coords(
#         [47.4, -118.3, 338.3, -25.31, 0.93, -22.06], 50, 0)
#     sleep(5)
#     mycobot.send_coords(
#         [-87.6, -30.6, 310.2, -21.74, -0.31, -21.08], 50, 0)
#     sleep(5)

#     coords = mycobot.get_coords()
#     print('coords', coords)

#     move = [0, -20, 0]

#     coords_n = get_coords_new(coords, move)
#     print('coords_n', coords_n)

#     move_robot(mycobot, coords_n)

#     mycobot.stop()
