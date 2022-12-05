from enum import Enum
from dataclasses import dataclass
from dataclasses import dataclass, field
from typing import List
from typing import Tuple


class InputMode(Enum):
    CAMERA = 0
    IMAGES = 1
    VIDEO = 2


class OutputMode(Enum):
    CALIB_BOARD = 0
    CALIBRATION = 1
    MESH = 2
    STREAM = 3
    VIDEO = 4
    IMAGES = 5


class CalibrationType(Enum):
    CAMERA = 0
    LASER = 1
    EXPOSURE = 2
    FULL = 3
    IMPORT = 4


@dataclass
class BoardDescriptor():
    printSizeX: int = 2200
    printSizeY: int = 2200
    boardDimX: int = 11
    boardDimY: float = 11
    squareSize: int = 0.02
    markerSize: int = 0.01


# @dataclass
# class Config():
#     input_mode: InputMode = InputMode.VIDEO
#     output_mode: List = field(default_factory=lambda: [OutputMode.IMAGES, OutputMode.STREAM])
#     calib_mode: CalibrationType = CalibrationType.FULL

#     input_folder: str = "./02_out/"
#     output_path: str = "./02_out/temp/"
#     output_file: str = "out_{}.png"
#     input_name: str = "out_.mp4"
#     calib_folder: str = "./02_out/calib/"

#     resolution: Tuple[int, int] = (1632, 1232)
#     input_fps: int = 30
#     input_exposure: int = 100
#     certanty: int = 0.8

#     streamer_fps: int = 30
#     streamer_res: Tuple[int, int] = (640, 480)
#     streaner_path: str = "rtsp://localhost:8554/G3D"

#     laser_gpio: int = 14

# @dataclass
# class Config():
#     input_mode: InputMode = InputMode.VIDEO
#     output_mode: List = field(default_factory=lambda: [OutputMode.MESH])
#     # input_mode: InputMode = InputMode.CAMERA
#     # output_mode: List = field(default_factory=lambda: [OutputMode.VIDEO])

#     calib_mode: CalibrationType = CalibrationType.IMPORT

#     input_folder: str = "./02_out/"
#     output_path: str = "./02_out/"
#     output_file: str = "08_YODA"
#     input_name: str = "08_YODA.mp4"

#     minimal_campos_precision: float = 0.1

#     # input_name: str = "01_RUBIC5.mp4"
#     # input_name: str = "01_RUBICX.mp4"
#     # input_name: str = "02_benchy.mp4"
#     # input_name: str = "03_penguin.mp4"
#     # input_name: str = "04_yoda.mp4"
#     # input_name: str = "04_yoda2.mp4"
#     # input_name: str = "05_key.mp4"
#     # input_name: str = "0"

#     # output_file: str = "calibration2"
#     # input_name: str = "calibration.mp4"

#     resolution: Tuple[int, int] = (1632, 1232)
#     input_fps: int = 15
#     input_exposure: int = 100
#     certanty: int = 0.8

#     streamer_fps: int = 8
#     streamer_res: Tuple[int, int] = (640, 480)
#     streaner_path: str = "rtsp://localhost:8554/G3D"

#     laser_gpio: int = 14
#     calib_board: BoardDescriptor = BoardDescriptor()

#     export_cage: bool = True


@dataclass
class Config():
    input_mode: InputMode = InputMode.CAMERA
    output_mode: List = field(default_factory=lambda: [OutputMode.IMAGES])

    calib_mode: CalibrationType = CalibrationType.IMPORT

    input_folder: str = "./02_out/"
    output_path: str = "./02_out/"
    output_file: str = "09_temp"
    input_name: str = "/dev/video0"

    # output_file: str = "calibration2"
    # input_name: str = "calibration.mp4"

    # resolution: Tuple[int, int] = (1920, 1080)
    resolution: Tuple[int, int] = (640, 480)
    input_fps: int = 15
    input_exposure: int = 100
    certanty: int = 0.8

    streamer_fps: int = 8
    streamer_res: Tuple[int, int] = (640, 480)
    streaner_path: str = "rtsp://localhost:8554/G3D"

    laser_gpio: int = 14
    calib_board: BoardDescriptor = BoardDescriptor()


# calibration

# @dataclass
# class Config():
#     input_mode: InputMode = InputMode.VIDEO
#     output_mode: List = field(default_factory=lambda: [OutputMode.CALIBRATION])

#     calib_mode: CalibrationType = CalibrationType.LASER

#     input_folder: str = "./02_out/"
#     output_path: str = "./02_out/"
#     # output_file: str = "RECORDING3"
#     # input_name: str = "RECORDING2.mp4"
#     # input_name: str = "0"

#     output_file: str = "calibration3"
#     input_name: str = "calibration.mp4"

#     resolution: Tuple[int, int] = (1632, 1232)
#     input_fps: int = 15
#     input_exposure: int = 100
#     certanty: int = 0.8

#     streamer_fps: int = 8
#     streamer_res: Tuple[int, int] = (640, 480)
#     streaner_path: str = "rtsp://localhost:8554/G3D"

#     laser_gpio: int = 14
#     calib_board: BoardDescriptor = BoardDescriptor()
