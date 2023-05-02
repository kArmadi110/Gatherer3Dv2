from os import listdir
from os.path import isfile, join

import numpy as np
import cv2

from core.config_types import Config

from core.g3d_io import G3DInput


class Images(G3DInput):
    """Reads images from a folder."""

    def __init__(self, cfg: Config):
        G3DInput.__init__(self, cfg)

        # Generate the frame list in adwance.
        # This is quite useful when we export a bunch of images for calibration
        # but wee only want to use the best of the best.
        self._frame_list = [join(self._cfg.input_folder, f) for f in listdir(self._cfg.input_folder)
                            if self._cfg.input_name in f and
                            isfile(join(self._cfg.input_folder, f))]
        self._frame_id = 0
        self._stream_state = True

    def read(self) -> np.array:
        frame = cv2.imread(self._frame_list[self._frame_id])

        self._frame_id += 1

        if frame is None:
            self._stream_state = False

        return frame

    def deinit(self):
        pass

    def is_open(self) -> bool:
        return self._stream_state
