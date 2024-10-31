import queue
import threading
import time
from time import sleep
from urllib.request import urlopen

import cv2
import numpy as np

from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot


video_out = 'video.avi'
codec = cv2.VideoWriter_fourcc(*'MJPG')
fps = 5
resolution = (800, 600)
video_writer = cv2.VideoWriter(video_out, codec, fps, resolution)


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
                    video_writer.write(img)

                    self.data.put(img)

                k = cv2.waitKey(1)

            except Exception as e:
                print("Error:" + str(e))
                bts = b''
                self.stream = self.stream_copy
                continue

            if k & 0xFF == ord('a'):
                cv2.imwrite('./photos/c.jpg', img)
                continue

            if k & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
        print("%s finished!" % self.getName())


class MoveRobot(threading.Thread):
    def __init__(self, t_name, queue, thread1):
        threading.Thread.__init__(self, name=t_name)
        self.data = queue
        self.thread1 = thread1

    def run(self):
        c = 1
        while True and self.thread1.is_alive():
            img = self.data.get()

            cv2.imwrite(f'photos/{c}.jpg', img)
            print(f'{c} saved picture')

            self.data.task_done()
            sleep(2)
            c += 1
        print("%s finished!" % self.getName())


def main():
    URL = 'http://192.168.4.1/'
    try:
        stream = urlopen(URL)
    except Exception as e:
        print("Error:" + str(e))
        return False

    q = queue.LifoQueue()
    producer = ReadStream('Cam Stream.', q, stream)
    consumer = MoveRobot('Robot', q, producer)
    producer.start()
    consumer.start()
    q.join()
    producer.join()
    consumer.join()
    print('All threads terminate!')


if __name__ == '__main__':
    main()
