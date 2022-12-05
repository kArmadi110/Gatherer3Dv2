from inputs.g3d_input import G3DInput
import numpy as np
import cv2


class G3DVideoIn(G3DInput):
    def __init__(self, input_name: str):
        G3DInput.__init__(self, input_name)
        print(self._input_name)
        self.cap = cv2.VideoCapture(self._input_name)
        self.lastFrameState = True

    def read(self) -> np.array:
        self.lastFrameState, frame = self.cap.read()
        return frame

    def deinit(self):
        self.cap.release()

    def isOpen(self) -> bool:
        return self.cap.isOpened() and self.lastFrameState
