import numpy as np


class G3DOutput():
    def __init__(self, output_name: str, ):
        self._output_name = output_name

    def deinit(self):
        raise NotImplementedError

    def process_frame(frame: np.array) -> bool:
        raise NotImplementedError
