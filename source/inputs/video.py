import numpy as np
import cv2

from core.config_types import Config

from core.g3d_input import G3DInput


class Video(G3DInput):
    def __init__(self, cfg: Config):
        G3DInput.__init__(self, cfg)

        self._cap = cv2.VideoCapture(self._cfg.input_folder+self._cfg.input_name)
        self._stream_state = True

    def read(self) -> np.array:
        self._stream_state, frame = self._cap.read()

        return frame

    def deinit(self):
        self._cap.release()

    def is_open(self) -> bool:
        return self._cap.isOpened() and self._stream_state
