import subprocess

import numpy as np
import cv2

from core.config_types import Config

from core.g3d_output import G3DOutput


class FFMPEGStream(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._stream = subprocess.Popen(['ffmpeg',
                                         '-y',
                                         '-f', 'rawvideo',
                                         '-vcodec', 'rawvideo',
                                         '-pix_fmt', 'bgr24',
                                         '-s', f"{self._cfg.streamer_res[0]}x{self._cfg.streamer_res[1]}",
                                         '-r', str(self._cfg.input_fps),
                                         '-i', '-',
                                         '-c:v', 'libx264',
                                         '-pix_fmt', 'yuv420p',
                                         '-preset', 'ultrafast',
                                         '-f', 'rtsp',
                                         '-rtsp_transport', 'tcp',
                                         f'{self._cfg.streaner_path}'], stdin=subprocess.PIPE)

    def process_frame(self, frame: np.array) -> bool:
        self._stream.stdin.write(cv2.resize(frame, self._cfg.streamer_res).tobytes())

    def deinit(self):
        self._stream.stdin.close()
        self._stream.wait()
