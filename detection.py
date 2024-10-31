import math
import os
import time

import cv2
import numpy as np
from object_detection.builders import model_builder
from object_detection.utils import config_util
from transforms3d import quaternions as quat

import tensorflow as tf


class DetectionModel:
    def __init__(self):
        CUSTOM_MODEL_NAME = 'my_ssd_mobnet'
        CHECKPOINT_PATH = os.path.join(
            'Tensorflow', 'workspace', 'models', CUSTOM_MODEL_NAME, 'ckpt-3')
        PIPELINE_CONFIG_PATH = os.path.join(
            'Tensorflow', 'workspace', 'models', CUSTOM_MODEL_NAME, 'pipeline.config')

        # Load pipeline config and build a detection model
        configs = config_util.get_configs_from_pipeline_file(
            PIPELINE_CONFIG_PATH)

        self.model = model_builder.build(
            model_config=configs['model'], is_training=False)

        # Restore checkpoint
        ckpt = tf.compat.v2.train.Checkpoint(model=self.model)
        ckpt.restore(CHECKPOINT_PATH).expect_partial()

    @tf.autograph.experimental.do_not_convert
    def detect(self, image):
        """
        Args:
            image (numpy.ndarray)

        Returns:
            tuple: 2x2: box: (pt1, pt2)
        """
        try:
            im_shape = (image.shape[0], image.shape[1])
            image = tf.convert_to_tensor(
                np.expand_dims(image, 0), dtype=tf.float32)

            image, shapes = self.model.preprocess(image)
            prediction_dict = self.model.predict(image, shapes)
            detections = self.model.postprocess(prediction_dict, shapes)

            detection_scores = detections['detection_scores'].numpy()
            highest_score = np.amax(detection_scores)

            if highest_score >= 0.8:
                detection_boxes = detections['detection_boxes'].numpy()
                box_ratio = detection_boxes[0][0]  # y_min, x_min, y_max, x_max

                box = ((box_ratio[1]*im_shape[1], box_ratio[0]*im_shape[0]),  # (x_min, y_min)
                       (box_ratio[3]*im_shape[1], box_ratio[2]*im_shape[0]))  # (x_max, y_max)

                return box
            else:
                # print('no detection')
                return False
        except Exception as e:
            print(e)
            return False


def get_box_area(box):
    """get_box_area

    Args:
        box (tuple): 2x2: ((x_min, y_min), (x_max, y_max))

    Returns:
        float: area of box
    """
    return abs(box[1][0] - box[0][0]) * abs(box[1][1] - box[0][1])


def get_box_hw_ratio(box):
    """get box height to weight ratio

    Args:
        box (tuple): 2x2: ((x_min, y_min), (x_max, y_max))

    Returns:
        float: ratio of height to weight
    """
    return abs(box[1][0] - box[0][0]) / abs(box[1][1] - box[0][1])


def get_relative_center(box_center, img_center):
    """get center coord of box wrt to center of image

    Args:
        box_center (tuple): 1x2: (x, y)
        img_center (tuple): 1x2: (x, y)

    Returns:
        tuple: 1x2: (x, y)
    """

    r_img_center_wrt_img = np.array([img_center[0], img_center[1]])
    r_box_center_wrt_img = np.array([box_center[0], box_center[1]])

    r_box_to_img_center_wrt_img = -1 * r_img_center_wrt_img + r_box_center_wrt_img

    mat = np.array([[1, 0],
                    [0, -1]])

    r_box_to_img_center_wrt_img_center = mat.dot(r_box_to_img_center_wrt_img)

    return tuple(r_box_to_img_center_wrt_img_center)


def get_center_difference(box_center, img_center):
    """get_center_difference

    Args:
        box_center (tuple): 1x2: (x, y)
        img_center (tuple): 1x2: (x, y)

    Returns:
        float: absolute difference between two center
    """
    difference = math.sqrt(
        (box_center[0]-img_center[0])**2 + (box_center[1]-img_center[1])**2)
    return difference


def get_center_difference_line(box_center, img_center):
    """get line indicating center difference

    Args:
        box_center (tuple): 1x2: (x, y)
        img_center (tuple): 1x2: (x, y)

    Returns:
        tuple: 2x2: (pt1, pt2)
    """
    img_center = [int(i) for i in img_center]
    box_center = [int(i) for i in box_center]
    line = (tuple(img_center), tuple(box_center))
    return line


def get_center_difference_line_midpt(box_center, img_center):
    """get mid-pt of line indicating center difference

    Args:
        box_center (tuple): 1x2: (x, y)
        img_center (tuple): 1x2: (x, y)

    Returns:
        tuple: 1x2: (x, y)
    """
    line_midpt = (int((box_center[0]+img_center[0])/2),
                  int((box_center[1]+img_center[1])/2))
    return line_midpt


# # test
# if __name__ == '__main__':

#     model = DetectionModel()

#     IM_CENTER = (400, 300)

#     IMAGE_PATH = os.path.join(
#         'Tensorflow', 'workspace', 'images', 'test', '5.jpg')
#     im = cv2.imread(IMAGE_PATH)
#     im = np.array(im)

#     bo = model.detect(im)

#     box_center = ((bo[0][0]+bo[1][0])/2, (bo[0][1]+bo[1][1])/2)

#     ##
#     relative_center = get_relative_center(box_center, IM_CENTER)

#     box_area = get_box_area(bo)

#     cv2.putText(im,
#                 str(round(box_area/1000, 1))+'k',  # text
#                 (int(bo[0][0]), int(bo[1][1])),  # coord
#                 cv2.FONT_HERSHEY_SIMPLEX,  # font
#                 0.5,  # font scale
#                 (0, 0, 255),
#                 2,  # line thickness
#                 cv2.LINE_AA)

#     cv2.putText(im,
#                 str((round(relative_center[0], 1), round(
#                     relative_center[1], 1))),  # text
#                 (int(box_center[0]), int(box_center[1])),  # coord
#                 cv2.FONT_HERSHEY_SIMPLEX,  # font
#                 0.5,  # font scale
#                 (255, 0, 0),
#                 2,  # line thickness
#                 cv2.LINE_AA)

#     line = get_center_difference_line(box_center, IM_CENTER)

#     cv2.rectangle(im, (int(bo[0][0]), int(bo[0][1])),
#                   (int(bo[1][0]), int(bo[1][1])), (0, 255, 0), 2)

#     cv2.line(im, line[0], line[1], (255, 0, 0), 2)

#     # center_difference = get_center_difference(box_center, IM_CENTER)
#     # print('center_difference', center_difference)

#     # line_midpt = get_center_difference_line_midpt(box_center, IM_CENTER)

#     # cv2.putText(im,
#     #             str(round(center_difference, 1)),  # text
#     #             (line_midpt[0], line_midpt[1]),  # coord
#     #             cv2.FONT_HERSHEY_SIMPLEX,  # font
#     #             0.5,  # font scale
#     #             (255, 0, 0),
#     #             2,  # line thickness
#     #             cv2.LINE_AA)

#     cv2.imshow('', im)
#     cv2.waitKey(0)

# if __name__ == '__main__':
#     model = DetectionModel()

#     im = cv2.imread('./data/p1_21.jpg')

#     print('start')
#     start = time.time()
#     det = model.detect(im)
#     end = time.time()
#     print('Detection time: {}s'.format(round(end-start, 2)))
