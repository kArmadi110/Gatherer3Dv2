import numpy as np

from core.config_types import Config

from outputs.calibration.base import Base


class ExposureCalibration(Base):
    """
    Calibrates the exposure of the camera.
    Not implemented yet. The config should provide this input.
    """

    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

    def load(self):
        pass  # TODO:

    def process_frame(self, frame: np.array):
        pass  # TODO:

    def calibrate(self):
        pass  # TODO:
