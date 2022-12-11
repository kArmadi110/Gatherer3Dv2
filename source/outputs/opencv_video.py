import numpy as np
import cv2

from core.config_types import Config

from core.g3d_output import G3DOutput


class CV2Video(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._output_video = cv2.VideoWriter(self._cfg.output_folder+self._cfg.output_name+'.mkv',  # mp4, mkv
                                             cv2.VideoWriter_fourcc(*'x264'),  # avc1, mp4v, x264, mjpg, XVID, H264
                                             self._cfg.input_fps,
                                             self._cfg.resolution)

        self._counter = 0

    def process_frame(self, frame: np.array) -> bool:
        self._output_video.write(frame)
        self._counter += 1

    def deinit(self):
        self._output_video.release()
