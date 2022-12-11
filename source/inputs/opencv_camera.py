import numpy as np
import cv2

from core.config_types import Config

from core.g3d_input import G3DInput

# TODO: test with legacy camera


class CV2Camera(G3DInput):
    def __init__(self, cfg: Config):
        G3DInput.__init__(self, cfg)

        self._counter = 0
        self._stream_state = True

        # GSTREAM
        # self._cap = cv2.VideoCapture(f" libcamerasrc ! video/x-raw, width=(int){self._cfg.resolution[0]}, height=(int){self._cfg.resolution[1]}" "," +
        #                             f" framerate=(fraction){self._cfg.input_fps}/1 !" +
        #                             " videoconvert ! videoscale !" +
        #                             " video/x-raw, width=(int){self._cfg.resolution[0]}, height=(int){self._cfg.resolution[1]} ! appsink",
        #                             cv2.CAP_GSTREAMER)

        self._cap = cv2.VideoCapture(int(self._cfg.input_name), apiPreference=cv2.CAP_V4L, params=[
            cv2.CAP_PROP_FRAME_WIDTH, self._cfg.resolution[0],
            cv2.CAP_PROP_FRAME_HEIGHT, self._cfg.resolution[1],
            cv2.CAP_PROP_FPS, self._cfg.input_fps])

        if self._cfg.input_exposure > 0:
            self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self._cap.set(cv2.CAP_PROP_EXPOSURE, self._cfg.input_exposure)

    def deinit(self):
        G3DInput.deinit(self)
        self._cap.release()

    def read(self) -> np.array:
        self._stream_state, frame = self._cap.read()

        return frame

    def is_open(self) -> bool:
        return self._cap.isOpened() and self._stream_state
