import time
from queue import Queue
from threading import Thread
import numpy as np

import RPi.GPIO

from core.config_types import Config


class G3DIOInterface():
    """Base class for all IO interfaces."""
    def __init__(self, cfg: Config):
        self._cfg = cfg

    def deinit(self):
        raise NotImplementedError


class G3DInput(G3DIOInterface):
    """Base class for all input interfaces."""
    def __init__(self, cfg: Config):
        G3DIOInterface.__init__(self, cfg)

        # handle laser
        if self._cfg.laser_gpio > 0:
            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setup(self._cfg.laser_gpio, RPi.GPIO.OUT)
            RPi.GPIO.output(self._cfg.laser_gpio, 1)

    def deinit(self):
        if self._cfg.laser_gpio > 0:
            RPi.GPIO.output(self._cfg.laser_gpio, 0)
            RPi.GPIO.cleanup()

    def read(self) -> np.array:
        raise NotImplementedError

    def is_open(self) -> bool:
        """Returns true if the input is open without an error."""
        raise NotImplementedError


class G3DOutput(G3DIOInterface):
    """Base class for all output interfaces."""
    def process_frame(self, frame: np.array) -> bool:
        raise NotImplementedError


class G3DAsyncIO(G3DIOInterface):
    """Base class for all async IO interfaces.
       Receives an input or output interface and creates a thread that handles the input or output asynchronously.
       Provides the same interface as the GPIOInterface.
       Processes the input or output in a separate thread from a queue through the injected sync io.
       The received IO should not be provided to any other AsyncIO
       """

    def __init__(self, cfg: Config, core: G3DIOInterface):
        """Creates a thread."""
        G3DIOInterface.__init__(self, cfg)

        self._core = core

        self._input_queue = Queue(maxsize=self._cfg.async_q_limit)
        self._thread = Thread(target=self.loop, args=(), daemon=True)

        self._thread_ctrl = True
        self._thread.start()

    def deinit(self):
        """Kill the thread and deinit the core IO."""
        self._input_queue.join()

        self._thread_ctrl = False
        self._input_queue.put(None)
        self._thread.join()

        self._core.deinit()

    def loop(self):
        raise NotImplementedError


class G3DAsyncInput(G3DInput, G3DAsyncIO):
    """Async input interface."""

    def __init__(self, cfg: Config, core: G3DInput):
        G3DInput.__init__(self, cfg=cfg)
        G3DAsyncIO.__init__(self, cfg=cfg, core=core)

    def is_open(self) -> bool:
        return self._core.is_open()

    def read(self) -> np.array:
        """Reads from the queue."""
        frame = None
        loop_control = frame is None and self.is_open()

        while loop_control:
            frame = self._input_queue.get()
            loop_control = frame is None and self.is_open()

            if loop_control:
                time.sleep(.01)

        return frame

    def loop(self):
        """Reads from the core Input and puts it into the queue."""
        while self._thread_ctrl and self.is_open():
            frame = self._core.read()

            if frame is not None:
                if self._input_queue.full():
                    _ = self._input_queue.get()
                    self._input_queue.task_done()

                self._input_queue.put(frame)


class G3DAsyncOutput(G3DOutput, G3DAsyncIO):
    """Async output interface."""
    def __init__(self, cfg: Config, core: G3DOutput):
        G3DOutput.__init__(self, cfg=cfg)
        G3DAsyncIO.__init__(self, cfg=cfg, core=core)
        self.throwaway = 0
        self.fps = 0

    def deinit(self):
        G3DAsyncIO.deinit(self)

    def process_frame(self, frame: np.array) -> bool:
        """Puts the frame into the queue."""
        if self._input_queue.full():
            _ = self._input_queue.get()
            self._input_queue.task_done()
            self.throwaway += 1

        self._input_queue.put(frame)

    def loop(self):
        """Processes the frame from the queue."""
        while self._thread_ctrl:
            frame = self._input_queue.get()

            if frame is not None:
                self._core.process_frame(frame)
                self._input_queue.task_done()
                self.fps += 1
