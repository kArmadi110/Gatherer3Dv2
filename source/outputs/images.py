import numpy as np
import cv2

from config_types import Config

from outputs.g3d_output import G3DOutput


class G3DImagesOut(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._counter = 0
        self._output_path = self._cfg.output_folder + self._cfg.output_name + "{}.png"

    def process_frame(self, frame: np.array):
        if not frame.empty():
            raise Exception("Empty frame received")

        cv2.imwrite(self._output_path.format(self._counter), frame)
        self._counter += 1

    def deinit(self):
        pass
