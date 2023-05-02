import signal

from core.g3d_io import G3DInput, G3DOutput


class ProcessLoop():
    """ProcessLoop class is used to process frames from input stream and send them to output streams."""

    def __init__(self, input_stream: G3DInput, output_streams: G3DOutput):
        if not input_stream or not output_streams:
            raise RuntimeError("input or output stream is empty")

        # handle sigint to kill threads gracefully
        signal.signal(signal.SIGINT, self.sigint_handler)

        self._input_stream = input_stream
        self._output_streams = output_streams

        # main thread control
        self._thread_ctrl = self._input_stream.is_open()
        self._counter = 0

    def sigint_handler(self, sig, frame):  # pylint: disable=unused-argument
        self._thread_ctrl = False
        print("Please wait for shutdown!")

    def deinit(self):
        """Deinitializes the input and output streams."""
        self._input_stream.deinit()
        for ostream in self._output_streams:
            ostream.deinit()

    def run(self):
        """Runs the main thread process loop."""

        frame = self._input_stream.read()

        while (self._thread_ctrl and self._input_stream.is_open()):
            self._counter += 1

            for i in self._output_streams:
                i.process_frame(frame)

            frame = self._input_stream.read()

        self.deinit()
