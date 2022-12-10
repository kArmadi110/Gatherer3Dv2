import numpy as np
import cv2

from core.config_types import Config

from outputs.g3d_output import G3DOutput


class CV2Gstream(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._stream = cv2.VideoWriter('appsrc ! videoconvert' +
                                       ' ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=900000 key-int-max=40' +
                                       f' ! rtspclientsink location={self._cfg.output_folder+self._cfg.output_name}', cv2.CAP_GSTREAMER, 0,
                                       fps=self._cfg.input_fps, frameSize=self._cfg.resolution)

        if not self._stream.isOpened():
            raise Exception("Streaming error!")

    def process_frame(self, frame: np.array) -> bool:
        self._stream.write(cv2.resize(frame, self._cfg.resolution))

    def deinit(self):
        self._stream.release()
