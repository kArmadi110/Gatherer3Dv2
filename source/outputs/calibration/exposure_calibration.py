import numpy as np

from core.config_types import Config

from outputs.calibration.base import Base


class ExposureCalibration(Base):
    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

    def load(self):
        pass  # TODO:

    def process_frame(self, frame: np.array):
        pass  # TODO:

    def calibrate(self):
        pass  # TODO:
