import numpy as np
from outputs.g3d_output import G3DOutput
import cv2


class G3DImagesOut(G3DOutput):
    def __init__(self, output_name: str):
        G3DOutput.__init__(self, output_name)
        self._counter = 0

    def process_frame(self, frame: np.array):
        if not frame.empty():
            raise Exception("Empty frame received")

        cv2.imwrite(self._output_name.format(self._counter), frame)
        self._counter += 1

    def deinit(self):
        pass
