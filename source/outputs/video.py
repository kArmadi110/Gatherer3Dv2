
import numpy as np
import cv2

from outputs.g3d_output import G3DOutput


class G3DVidoOut(G3DOutput):
    def __init__(self, output_name: str, width: int, height: int):
        G3DOutput.__init__(self, output_name)
        # self.output_video = cv2.VideoWriter(
        #     self._output_name+".avi", cv2.VideoWriter_fourcc(*'png '), 30, (width, height))
        #TODO: FPS
        self.output_video = cv2.VideoWriter(self._output_name+'.mp4', cv2.VideoWriter_fourcc(*'avc1'), 20, (width, height))

    def process_frame(self, frame: np.array):
        self.output_video.write(frame)

    def deinit(self):
        self.output_video.release()
