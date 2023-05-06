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
    IMPORT = 3
    NONE = 3


class SegmentationMode(Enum):
    MIDLE = 0
    INTENSITY = 1
    SUBPIXEL = 2


class FilterMode(Enum):
    NO_FILTER = 0
    RADIUS = 1
    STATISTICAL = 2


@dataclass
class BoardDescriptor():
    print_size_x: int = 2200
    print_size_y: int = 2200
    board_dim_x: int = 11
    board_dim_y: float = 11
    square_size: int = 0.02
    marker_size: int = 0.01


@dataclass
class Config():
    """Main configuration class"""

    # determines is async mode is used (True for multithreading)
    is_async_mode: bool
    # determines the maximum size of the async queue
    async_q_limit: bool

    # input mode (camera, images, video)
    input_mode: InputMode
    # output mode (calibration, mesh, stream)
    output_mode: List[OutputMode]
    # calibration mode (camera, laser, exposure, import). Only used if CALIBRATION is in output_mode
    calib_mode: CalibMode

    # input and output folders and names
    input_folder: str
    output_folder: str
    output_name: str
    input_name: str

    resolution: Tuple[int, int]
    input_fps: int
    input_exposure: int

    # streamer resolution and path. Only used if STREAM is in output_mode
    streamer_res: Tuple[int, int]
    streaner_path: str

    laser_gpio: int
    # experimental color mdoe
    color_mode: bool
    segmentation_mode: SegmentationMode

    # confidence threshold percentage for position detection
    pos_confidence_th: int
    # calibration board descriptor
    calib_board: BoardDescriptor = BoardDescriptor()

    # True if only the calib_board size cage should be exported
    export_cage: bool = True

    # laser threshold for segmentation
    # laser_calib_th *width*height is the number of pixels that should be allowed
    # above that we have a faulty segmentation
    laser_calib_th: float = 0.02

    # filter mode and parameters for outlier filtering for the mesh
    filter_mode: FilterMode = FilterMode.NO_FILTER
    filter_param_1: int = 0
    filter_param_2: float = 0

    debug_mode: bool = False
    debug_log: bool = True
