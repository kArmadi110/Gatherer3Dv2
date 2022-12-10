import numpy as np
import RPi.GPIO

from core.config_types import Config


class G3DInput():
    def __init__(self, cfg: Config):
        self._cfg = cfg

        if self._cfg.laser_gpio > 0:
            RPi.GPIO.cleanup()
            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setup(self._cfg.laser_gpio, RPi.GPIO.OUT)
            RPi.GPIO.output(self._cfg.laser_gpio, 1)

    def read(self) -> np.array:
        raise NotImplementedError

    def deinit(self):
        if self._cfg.laser_gpio > 0:
            RPi.GPIO.output(self._cfg.laser_gpio, 0)
            RPi.GPIO.cleanup()

    def is_open(self) -> bool:
        raise NotImplementedError
