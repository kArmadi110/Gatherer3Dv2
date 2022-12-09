import numpy as np

from config_types import Config


class G3DOutput():
    def __init__(self, cfg: Config):
        self._cfg = cfg

    def deinit(self):
        raise NotImplementedError

    def process_frame(frame: np.array) -> bool:
        raise NotImplementedError
