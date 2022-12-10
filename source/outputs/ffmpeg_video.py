
import subprocess

import numpy as np

from core.config_types import Config

from core.g3d_output import G3DOutput


class FFMPEGVideo(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._ffmpeg_output = subprocess.Popen(['ffmpeg',
                                                '-y',
                                               '-f', 'rawvideo',
                                                '-vcodec', 'rawvideo',
                                                "-framerate", "30",
                                                '-pix_fmt', 'bgr24',
                                                '-s', f"{self._cfg.resolution[0]}x{self._cfg.resolution[1]}",
                                                '-r', str(self._cfg.input_fps),
                                                '-i', '-',
                                                '-c:v', 'h264_v4l2m2m',
                                                f'{self._cfg.output_folder + self._cfg.output_name}.mp4'],
                                               stdin=subprocess.PIPE,
                                               bufsize=2*3*self._cfg.resolution[0]*self._cfg.resolution[1])

    def process_frame(self, frame: np.array) -> bool:
        self._ffmpeg_output.stdin.write(frame.tobytes())

    def deinit(self):
        self._ffmpeg_output.stdin.close()
        self._ffmpeg_output.wait()
