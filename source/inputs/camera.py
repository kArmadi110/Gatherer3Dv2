from inputs.g3d_input import G3DInput
import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2


class G3DCameraIn(G3DInput):
    def __init__(self, input_name: str, width: int, height: int, exposure: int, fps: int, gpio: int, gpio_state: int):
        G3DInput.__init__(self, input_name)
        self._counter = 0
        self._width = width
        self._height = height
        self.lastFrameState = True

        # Legacy
        # self.cap = cv2.VideoCapture(self._input_name, apiPreference=cv2.CAP_V4L, params=[
        #     cv2.CAP_PROP_FRAME_WIDTH, width,
        #     cv2.CAP_PROP_FRAME_HEIGHT, height])
        # cv2.CAP_PROP_FPS, fps
        # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exposure)

        # GSTREAM
        # self.cap = cv2.VideoCapture(f" libcamerasrc ! video/x-raw, width=(int){self._width}, height=(int){self._height}" "," +
        #                             f" framerate=(fraction){fps}/1 !" +
        #                             " videoconvert ! videoscale !" +
        #                             " video/x-raw, width=(int){self._width}, height=(int){self._height} ! appsink", cv2.CAP_GSTREAMER)

        self.cap = Picamera2()
        self.cap.configure(self.cap.create_preview_configuration(main={"format": 'XRGB8888', "size": (width, height)}))
        self.cap.start()

        self._gpio = gpio_state
        if gpio_state:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._gpio, GPIO.OUT)
            GPIO.output(self._gpio, 1)

    def deinit(self):
        return

        self.cap.release()
        GPIO.cleanup()

    def read(self) -> np.array:
        im = self.cap.capture_array()
        return im

        self.lastFrameState, frame = self.cap.read()

        if self.lastFrameState:
            cv2.imwrite(f"test{self._counter}.png", frame)

        return frame

    def isOpen(self) -> bool:
        #        return self.cap.isOpened() and self.lastFrameState
        return True
