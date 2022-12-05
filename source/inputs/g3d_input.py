import numpy as np


class G3DInput():
    def __init__(self, input_name: str):
        self._input_name = input_name

    def read(self) -> np.array:
        raise NotImplementedError

    def deinit(self):
        raise NotImplementedError

    def isOpen(self) -> bool:
        raise NotImplementedError
