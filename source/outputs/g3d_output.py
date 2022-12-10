import numpy as np

from core.config_types import Config
from core.base_channel import BaseChannel


class G3DOutput(BaseChannel):
    def __init__(self, cfg: Config):
        self._cfg = cfg

    def deinit(self):
        raise NotImplementedError

    def process_frame(self, frame: np.array) -> bool:
        raise NotImplementedError
