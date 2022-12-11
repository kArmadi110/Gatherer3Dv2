from queue import Queue
from threading import Thread
import numpy as np

from core.config_types import Config

from core.g3d_output import G3DOutput


class G3DAsyncOutput(G3DOutput):
    def __init__(self, cfg: Config, core: G3DOutput):
        G3DOutput.__init__(self, cfg)

        self._core = core

        self._input_queue = Queue(maxsize=self._cfg.async_q_limit)
        self._thread = Thread(target=self.loop, args=(), daemon=True)

        self._thread_ctrl = True
        self._thread.start()

    def deinit(self):
        self._input_queue.join()

        self._thread_ctrl = False
        self._input_queue.put(None)
        self._thread.join()

        self._core.deinit()

    def process_frame(self, frame: np.array) -> bool:
        if self._input_queue.full():
            _ = self._input_queue.get()
            self._input_queue.task_done()

        self._input_queue.put(frame)

    def loop(self):
        while self._thread_ctrl:
            frame = self._input_queue.get()

            if frame is not None:
                self._core.process_frame(frame)
                self._input_queue.task_done()
