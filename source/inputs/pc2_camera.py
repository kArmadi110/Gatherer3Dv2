import numpy as np
from picamera2 import Picamera2

from core.config_types import Config

from core.g3d_input import G3DInput


class PC2Camera(G3DInput):
    def __init__(self, cfg: Config):
        G3DInput.__init__(self, cfg)
        self._counter = 0
        self._stream_state = True

        self._cap = Picamera2()
        self._cap.configure(self._cap.create_video_configuration(main={"format": 'BGR888', "size": (self._cfg.resolution[0], self._cfg.resolution[1])}))
        self._cap.framerate = self._cfg.input_fps
        self._cap.sensor_mode = 4

        self._cap.start()

    def deinit(self):
        G3DInput.deinit(self)

        self._cap.stop()

    def read(self) -> np.array:
        frame = self._cap.capture_array()

        if frame is None:
            self._stream_state = False

        return frame

    def is_open(self) -> bool:
        return self._stream_state
