from enum import Enum
from dataclasses import dataclass
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


class CalibMode(Enum):
    CAMERA = 0
    LASER = 1
    EXPOSURE = 2
    FULL = 3
    IMPORT = 4


class SegmentationMode(Enum):
    MIDLE = 0
    INTENSITY = 1
    SUBPIXEL = 2


@dataclass
class BoardDescriptor():
    print_size_x: int = 2200
    print_size_x: int = 2200
    board_dim_x: int = 11
    board_dim_y: float = 11
    square_size: int = 0.02
    marker_size: int = 0.01


@dataclass
class Config():
    is_async_mode: bool
    async_q_limit: bool

    input_mode: InputMode
    output_mode: List
    calib_mode: CalibMode

    input_folder: str
    output_folder: str
    output_name: str
    input_name: str

    resolution: Tuple[int, int]
    input_fps: int
    input_exposure: int

    streamer_res: Tuple[int, int]
    streaner_path: str

    laser_gpio: int

    pos_certainty_th: int
    calib_board: BoardDescriptor = BoardDescriptor()
