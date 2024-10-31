from urllib.request import urlopen

import cv2
import numpy as np

# img.shape: (row, column, channel)
# width = column num = img.shape[1]
# height = row num = img.shape[0]
# img's resolution = width, height


def get_img(url='http://192.168.4.1/'):
    try:
        stream = urlopen(url)
    except Exception as e:
        print('Error:', e)
        return False

    CAMERA_BUFFRER_SIZE = 4096
    bts = b''
    while True:
        try:
            bts += stream.read(CAMERA_BUFFRER_SIZE)
            jpghead = bts.find(b'\xff\xd8')
            jpgend = bts.find(b'\xff\xd9')
            if jpghead > -1 and jpgend > -1:
                jpg = bts[jpghead:jpgend+2]
                bts = bts[jpgend+2:]
                img = cv2.imdecode(np.frombuffer(
                    jpg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                break
        except:
            bts = b''
            stream = urlopen(url)
            continue
    # print(img.shape)  # (600, 800, 3)
    # print(type(img))  # <class 'numpy.ndarray'>
    return img


# if __name__ == '__main__':
#     im = get_img()
#     if im is not False:
#         cv2.imshow('', im)
#         cv2.waitKey(0)
