import signal
from core.g3d_input import G3DInput
from core.g3d_output import G3DOutput
import time


class ProcessLoop():
    def __init__(self, input_stream: G3DInput, output_streams: G3DOutput):
        if not input_stream or not output_streams:
            raise RuntimeError("input or output stream is empty")

        signal.signal(signal.SIGINT, self.sigint_handler)

        self._input_stream = input_stream
        self._output_streams = output_streams

        self._thread_ctrl = self._input_stream.is_open()
        self._counter = 0

    def sigint_handler(self, sig, frame):  # pylint: disable=unused-argument
        self._thread_ctrl = False
        print("Please wait for shutdown!")

    def deinit(self):
        self._input_stream.deinit()
        for ostream in self._output_streams:
            ostream.deinit()

    def run(self):
        frame = self._input_stream.read()
        start = time.time()

        while (self._thread_ctrl and self._input_stream.is_open()):
            self._counter += 1

            for i in self._output_streams:
                i.process_frame(frame)

            frame = self._input_stream.read()
            # if time.time()-start > 120:
            # break

        self.deinit()
