import signal
from inputs import G3DInput
from outputs import G3DOutput


class ProcessLoop():
    def __init__(self, input_stream: G3DInput, output_streams: G3DOutput):
        if not input_stream or not output_streams:
            raise RuntimeError("input or output stream is empty")

        signal.signal(signal.SIGINT, self.sigint_handler)

        self._input_stream = input_stream
        self._output_streams = output_streams

        self._threadCtrl = self._input_stream.isOpen()
        self._counter = 0

    def sigint_handler(self, sig, frame):
        self._threadCtrl = False
        print("Please wait for shutdown!")

    def deinit(self):
        self._input_stream.deinit()
        for os in self._output_streams:
            os.deinit()

    def run(self):
        frame = self._input_stream.read()

        while (self._threadCtrl and self._input_stream.isOpen()):
            for i in self._output_streams:
                i.process_frame(frame)
            frame = self._input_stream.read()

        self.deinit()
