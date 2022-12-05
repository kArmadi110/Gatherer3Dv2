from inputs.g3d_input import G3DInput
import numpy as np
import cv2


class G3DImagesIn(G3DInput):
    def __init__(self, input_name: str):
        G3DInput.__init__(input_name)
        self.counter = 0

    def read(self) -> np.array:
        image = cv2.imread(self.input_name.format(self.counter))
        self.counter = self.counter + 1

        if not result:
            raise Exception("Empty frame")

        return image

    def deinit(self):
        pass

    def isOpen(self) -> bool:
        raise NotImplementedError
