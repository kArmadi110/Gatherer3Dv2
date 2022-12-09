import numpy as np
import RPi.GPIO as GPIO

from config_types import Config


class G3DInput():
    def __init__(self, cfg: Config):
        self._cfg = cfg

        if self._cfg.laser_gpio > 0:
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._cfg.laser_gpio, GPIO.OUT)
            GPIO.output(self._cfg.laser_gpio, 1)

    def read(self) -> np.array:
        raise NotImplementedError

    def deinit(self):
        if self._cfg.laser_gpio > 0:
            GPIO.output(self._cfg.laser_gpio, 0)
            GPIO.cleanup()

    def isOpen(self) -> bool:
        raise NotImplementedError
