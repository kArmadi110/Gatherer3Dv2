
import numpy as np
import cv2
import subprocess

from outputs.g3d_output import G3DOutput
import ffmpeg
import time


class G3DVidoOut(G3DOutput):
    def __init__(self, output_name: str, width: int, height: int):
        G3DOutput.__init__(self, output_name)
        # self.output_video = cv2.VideoWriter(
        #     self._output_name+".avi", cv2.VideoWriter_fourcc(*'png '), 30, (width, height))
        #TODO: FPS

        # self.output_video = cv2.VideoWriter(self._output_name+'.mp4', cv2.VideoWriter_fourcc(*'avc1'), 30, (width, height))
        # self.output_video = cv2.VideoWriter(self._output_name+'.mp4', cv2.VideoWriter_fourcc(*'x264'), 15, (width, height))

        # self.ffmpeg_output = subprocess.Popen(['ffmpeg',
        #                                       '-y',
        #                                        '-f', 'rawvideo',
        #                                        '-vcodec', 'rawvideo',
        #                                        "-framerate", "30",
        #                                        '-pix_fmt', 'bgr24',
        #                                        '-s', f"{width}x{height}",
        #                                        '-r', str(30),
        #                                        '-i', '-',
        #                                        '-c:v', 'h264_v4l2m2m',
        #                                        f'{self._output_name}.mp4'], stdin=subprocess.PIPE, bufsize=3*width*height*5)

    def process_frame(self, frame: np.array):
        pass
        # self.output_video.write(frame)
        # self.ffmpeg_output.stdin.write(frame.tobytes())
        # self.ffmpeg_output.stdin.write(frame.tobytes())
        # time.sleep(0.5)
        # tempresult = self.ffmpeg_output.communicate()
        # asd = frame.tobytes()
        # self.ffmpeg_output.communicate(asd)

    def deinit(self):
        # self.output_video.release()
        time.sleep(1)
        # self.ffmpeg_output.stdin.close()
        # self.ffmpeg_output.wait()
