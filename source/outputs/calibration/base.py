from typing import Tuple, List
import numpy as np
import cv2

from core.config_types import Config, CalibMode, SegmentationMode
from core.pickleable import Pickleable

from core.g3d_output import G3DOutput


class _PickleableBase(Pickleable):
    def __init__(self, cfg: Config):
        self._name = "_PickleableBase.pickle"
        self._cfg = cfg
        self._pickle_path = self._cfg.output_folder+f"/pickles/{self._name}"

        self.camera_matrix = None
        self.distortion_coefficients = None
        self.plane = []

    def load(self):  # pylint: disable=arguments-differ
        Pickleable.load(self, self._pickle_path)

    def save(self):  # pylint: disable=arguments-differ
        Pickleable.save(self, self._pickle_path)


class Base(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg)

        self._members = _PickleableBase(cfg)

        self._aruco_dict = None
        self._charuco_board = None

        self._all_corners = []
        self._all_ids = []
        self._rotation_vectors = []
        self._translation_vectors = []

        self._laser_points = []
        self._colors = []

        if self._cfg.calib_mode == CalibMode.IMPORT:
            self.load()

        self._generate_charuco_plane()

    def deinit(self):
        if self._cfg.calib_mode != CalibMode.IMPORT:
            self._members.save()

    def load(self):
        self._members.load()

    def get_laser_points(self, frame: np.array, mask_corners: List = None):
        if mask_corners:
            x_array = [i[0][0][0] for i in mask_corners]
            y_array = [i[0][0][1] for i in mask_corners]

            min_x = round(min(x_array))
            min_y = round(min(y_array))

            max_x = round(max(x_array))
            max_y = round(max(y_array))

            masked = np.zeros(frame.shape, np.uint8)
            masked[min_y:max_y, min_x:max_x] = frame[min_y:max_y, min_x:max_x]
            return self.segment_laser(masked)
        else:
            return self.segment_laser(frame)

    def _generate_charuco_plane(self):
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self._charuco_board = cv2.aruco.CharucoBoard_create(
            self._cfg.calib_board.boardDimX, self._cfg.calib_board.boardDimY,
            self._cfg.calib_board.squareSize, self._cfg.calib_board.markerSize,
            self._aruco_dict)

    def generate_charuco_plane(self):
        imboard = self._charuco_board.draw((self._cfg.calib_board.printSizeX, self._cfg.calib_board.printSizeY))
        cv2.imwrite(self._cfg.output_folder + f"{self._cfg.output_name}.png", imboard)

    def generate_checker_plane(self):
        result = np.zeros([self._cfg.calib_board.printSizeX, self._cfg.calib_board.printSizeY, 1], dtype=np.uint8)
        square_size = int(self._cfg.calib_board.printSizeX/self._cfg.calib_board.boardDimX*2)

        for pos_x in range(0, int(self._cfg.calib_board.boardDimX/2)):
            for pos_y in range(0, int(self._cfg.calib_board.boardDimY/2)):
                if pos_x % 2 == pos_y % 2:
                    for i in range(0, square_size):
                        for j in range(0, square_size):
                            result[pos_x*square_size + i, pos_y*square_size + j] = 255

        cv2.imwrite(self._cfg.output_folder + f"{self._cfg.output_name}.png", result)

    def segment_laser(self, frame: np.array) -> Tuple[List, List]:
        if self._cfg.segmentation_mode == SegmentationMode.MIDLE:
            return self._segment_laser_midle(frame)
        elif self._cfg.segmentation_mode == SegmentationMode.INTENSITY:
            return self._segment_laser_intensity(frame)
        elif self._cfg.segmentation_mode == SegmentationMode.SUBPIXEL:
            return self._segment_laser_intensity(frame, True)

    def _determine_pixel_color(self, frame: np.array, mask: np.array, pos_x: int, pos_y: int):
        color1 = frame[pos_x][pos_y]
        color2 = frame[pos_x][pos_y]

        laser_edge_shift = 5

        for i in range(self._cfg.resolution[0]-pos_x):
            if mask[pos_x-i][pos_y] == 0:
                if pos_x-i-laser_edge_shift >= 0:
                    color1 = frame[pos_x-i-laser_edge_shift][pos_y]
                else:
                    color1 = frame[pos_x-i][pos_y]
                break

        for i in range(self._cfg.resolution[0]-pos_x):
            if mask[pos_x+i][pos_y] == 0:
                if pos_x+i+laser_edge_shift >= 0:
                    color2 = frame[pos_x+i+laser_edge_shift][pos_y]
                else:
                    color2 = frame[pos_x+i][pos_y]
                break

        return (color2+color1)/2.0

    def _segment_laser_midle(self, frame: np.array) -> Tuple[List, List]:
        result = []
        colors = []

        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        _, thrs = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        last_non_zero = 0
        nonzero = np.nonzero(thrs)

        for i in range(len(nonzero[0])):
            if nonzero[0][last_non_zero] != nonzero[0][i]:
                pos_x = nonzero[0][last_non_zero]
                pos_y = round(nonzero[1][last_non_zero] +
                              ((nonzero[1][i-1]-nonzero[1][last_non_zero]) /
                               2.0))

                result.append([pos_y, pos_x])

                if self._cfg.color_mode:
                    colors.append(self._determine_pixel_color(frame, thrs, pos_x, pos_y))

                last_non_zero = i

        return result, colors

    def _segment_laser_intensity(self, frame: np.array, subpixel: bool = False) -> Tuple[List, List]:
        result = []
        colors = []

        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        _, thrs = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        last_non_zero = 0
        nonzero = np.nonzero(thrs)

        max_val = 0
        max_x = 0

        for i in range(len(nonzero[0])):

            if img[nonzero[0][i]][nonzero[1][i]] > max_val:
                max_val = img[nonzero[0][i]][nonzero[1][i]]
                max_x = i

            if nonzero[0][last_non_zero] != nonzero[0][i]:
                pos_x = nonzero[0][max_x]
                pos_y = nonzero[1][max_x]

                if self._cfg.color_mode:
                    colors.append(self._determine_pixel_color(frame, thrs, pos_x, pos_y))

                if subpixel:
                    delta = (int(img[pos_x][pos_y+2])*2 +
                             int(img[pos_x][pos_y+1]) -
                             int(img[pos_x][pos_y-1]) -
                             int(img[pos_x][pos_y-2])*2) / \
                        (int(img[pos_x][pos_y-2]) +
                         int(img[pos_x][pos_y-1]) +
                         int(img[pos_x][pos_y]) +
                         int(img[pos_x][pos_y+1]) +
                         int(img[pos_x][pos_y+2]))

                    result.append([pos_y + delta, pos_x])
                else:
                    result.append([pos_y, pos_x])

                max_val = 0
                max_x = 0
                last_non_zero = i

        return result, colors
