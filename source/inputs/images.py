from os import listdir
from os.path import isfile, join

import numpy as np
import cv2

from core.config_types import Config

from inputs.g3d_input import G3DInput


class G3DImagesIn(G3DInput):
    def __init__(self, cfg: Config):
        G3DInput.__init__(cfg)

        self._frame_list = [f for f in listdir(self._cfg.input_folder)
                            if f.contains(self._cfg.input_name) and
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
