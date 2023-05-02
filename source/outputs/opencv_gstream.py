import numpy as np
import cv2

from core.config_types import Config

from core.g3d_io import G3DOutput


class CV2Gstream(G3DOutput):
    """Writes frames to a gstreamer output."""

    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._stream = cv2.VideoWriter('appsrc ! videoconvert' +
                                       ' ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=900000 key-int-max=40' +
                                       f' ! rtspclientsink location={self._cfg.streaner_path}', cv2.CAP_GSTREAMER, 0,
                                       fps=self._cfg.input_fps, frameSize=self._cfg.streamer_res)

        if not self._stream.isOpened():
            raise Exception("Output Streaming error from opencv!")

    def process_frame(self, frame: np.array) -> bool:
        self._stream.write(cv2.resize(frame, self._cfg.streamer_res))

    def deinit(self):
        self._stream.release()
