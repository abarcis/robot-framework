#! /usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
from datetime import datetime


class Camera:
    def __init__(self, path="/home/pi/log/imgs"):
        self.CAM_FPS = 10

        CAPTURED_IMG_SIZE = (640, 480)
        # CAPTURED_IMG_SIZE = (800, 600)
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FPS, self.CAM_FPS)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURED_IMG_SIZE[0])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURED_IMG_SIZE[1])
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cam.read()

        self.path = path

    def take_picture(self):
        for i in range(16):
            start = datetime.now()
            self.cam.grab()
            end = datetime.now()
            if (end - start).total_seconds() >= 1 / self.CAM_FPS:
                # buffer is empty
                break
        frame = self.cam.retrieve()[1]
        if frame is None:
            raise Exception('Image cannot be retrieved. Disconnected camera?')
        # frame = cv2.resize(frame, (320, 240))
        # jpg = bytearray(cv2.imencode('.jpg', frame)[1])
        return frame

    def save_picture(self, image):
        cv2.imwrite(f'{self.path}/{datetime.now()}.jpg', image)
