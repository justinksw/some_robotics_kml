from urllib.request import urlopen

import cv2 as cv
import numpy as np

# img.shape: (row, column, channel)
# width = column num = img.shape[1]
# height = row num = img.shape[0]
# img's resolution = width, height


# video_out = 'video_out.avi'
# codec = cv.VideoWriter_fourcc(*'MJPG')
# fps = 5
# resolution = (800, 600)
# video_writer = cv.VideoWriter(video_out, codec, fps, resolution)

# change to your ESP32-CAM ip
url = "http://192.168.4.1/"

CAMERA_BUFFRER_SIZE = 4096
stream = urlopen(url)
bts = b''
i = 2000
while True:
    try:
        bts += stream.read(CAMERA_BUFFRER_SIZE)
        jpghead = bts.find(b'\xff\xd8')
        jpgend = bts.find(b'\xff\xd9')
        if jpghead > -1 and jpgend > -1:
            jpg = bts[jpghead:jpgend+2]
            bts = bts[jpgend+2:]
            img = cv.imdecode(np.frombuffer(
                jpg, dtype=np.uint8), cv.IMREAD_UNCHANGED)
            # img=cv.flip(img, 0)  # >0: 垂直翻轉, 0: 水平翻轉, <0: 垂直水平翻轉
            # print(f'img.shape: {img.shape}')
            # img = cv.resize(img, (80, 60))
            cv.imshow("Stream", img)
            # video_writer.write(img)

        k = cv.waitKey(1)

    except Exception as e:
        print("Error:" + str(e))
        bts = b''
        stream = urlopen(url)
        continue

    k = cv.waitKey(1)
    # 按a拍照存檔
    if k & 0xFF == ord('a'):
        cv.imwrite(f'./photos/{i}.jpg', img)
        i = i+1
    # 按q離開
    if k & 0xFF == ord('q'):
        break

# video_writer.release()
cv.destroyAllWindows()
