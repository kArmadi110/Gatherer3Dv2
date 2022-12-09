import cv2
import numpy as np
import time
import ffmpeg
import subprocess

from outputs.g3d_output import G3DOutput


class G3DStreamOut(G3DOutput):
    def __init__(self, output_name: str, width: int, height: int, streaming_fps: int):
        G3DOutput.__init__(self, output_name)

        self._framesize = (width, height)
        self._streaming_fps = streaming_fps

        self.stream = cv2.VideoWriter('appsrc ! videoconvert' +
                                      ' ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=900000 key-int-max=40' +
                                      f' ! rtspclientsink location={self._output_name}', cv2.CAP_GSTREAMER, 0,
                                      fps=self._streaming_fps, frameSize=self._framesize)

        # self.stream = cv2.VideoWriter(f"appsrc ! video/x-raw,format=BGR ! queue ! videoconvert ! x264enc insert-vui=1 ! rtspclientsink location={self._output_name}",
        #                               cv2.CAP_GSTREAMER, 0, float(self._streaming_fps), (int(width), int(height)))

        # streaming_process = subprocess.Popen(['ffmpeg',
        #    '-y',
        #    '-f', 'rawvideo',
        #    '-vcodec', 'rawvideo',
        #    '-pix_fmt', 'bgr24',
        #    '-s', f"{self._framesize[0]}x{self._framesize[1]}",
        #    '-r', str(self._streaming_fps),
        #    '-i', '-',
        #    '-c:v', 'libx264',
        #    '-pix_fmt', 'yuv420p',
        #    '-preset', 'ultrafast',
        #    '-f', 'flv',
        #    f'{self._output_name}'], stdin=subprocess.PIPE) # rtmp://localhost:1935/stream/Gatherer3D

        #
        if not self.stream.isOpened():
            raise Exception("Streaming error!")

    def process_frame(self, frame: np.array):
        self.stream.write(cv2.resize(frame, self._framesize))

        # streaming_process.stdin.write(frame.tobytes())

    def deinit(self):
        self.stream.release()

        # streaming_process.stdin.close()
        # streaming_process.wait()
