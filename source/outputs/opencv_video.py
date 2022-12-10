import numpy as np
import cv2

from core.config_types import Config

from outputs.g3d_output import G3DOutput


class CV2Video(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._output_video = cv2.VideoWriter(self._cfg.output_folder+self._cfg.output_name+'.mp4',
                                             cv2.VideoWriter_fourcc(*'x264'),  # avc1
                                             self._cfg.input_fps,
                                             self._cfg.resolution)

    def process_frame(self, frame: np.array) -> bool:
        self._output_video.write(frame)

    def deinit(self):
        self._output_video.release()
