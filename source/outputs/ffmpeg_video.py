
import subprocess

import numpy as np

from core.config_types import Config

from core.g3d_io import G3DOutput


class FFMPEGVideo(G3DOutput):
    def __init__(self, cfg: Config):
        """
        Writes frames to a video file on a separate process.
        FFMPEG buffer error with 1640x1232.
        Quality is also quite low.
        """
        G3DOutput.__init__(self, cfg)

        self._ffmpeg_output = subprocess.Popen(['ffmpeg',
                                                '-y',
                                               '-f', 'rawvideo',
                                                '-vcodec', 'rawvideo',
                                                "-framerate", f"{self._cfg.input_fps}",
                                                '-pix_fmt', 'bgr24',  # yuv420p, nv12, bgr24
                                                '-s', f"{self._cfg.resolution[0]}x{self._cfg.resolution[1]}",
                                                '-r', f"{self._cfg.input_fps}",
                                                '-i', '-',
                                                # '-b:v', '30k',
                                                # h264_v4l2m2m hw acceleration. Other: h263_v4l2m2m mpeg4_v4l2m2m vp8_v4l2m2m
                                                '-c:v', 'h264_v4l2m2m',
                                                # '-c:v', 'h264_omx', # not working on 64 bit os
                                                # '-flush_packets', '1',
                                                f'{self._cfg.output_folder + self._cfg.output_name}.mp4'],
                                               stdin=subprocess.PIPE,
                                               bufsize=2*3*self._cfg.resolution[0]*self._cfg.resolution[1])
        self._counter = 0

    def process_frame(self, frame: np.array) -> bool:
        self._ffmpeg_output.stdin.write(frame.tobytes())
        self._counter += 1

    def deinit(self):
        self._ffmpeg_output.stdin.close()
        self._ffmpeg_output.wait()
