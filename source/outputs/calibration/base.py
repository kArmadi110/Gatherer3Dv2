from typing import Tuple, List
import numpy as np
import cv2
import open3d as o3d

from core.config_types import Config, CalibMode, SegmentationMode, FilterMode
from core.pickleable import Pickleable

from core.g3d_io import G3DOutput


class _PickleableBase(Pickleable):
    """Pickleable base class for the Base class."""

    def __init__(self, cfg: Config):
        """
        Initializes the class.
        Contains all the configuration parameters exported previously.
        """

        self._name = cfg.input_name + "_PickleableBase.pickle"
        self._cfg = cfg
        self._pickle_path = self._cfg.input_folder+f"/pickles/{self._name}"

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

        self._success = 0
        self._frame_num = 0

        self.load()

        self._generate_charuco_plane()

    def deinit(self):
        if self._cfg.calib_mode != CalibMode.IMPORT:
            self._members.save()

        self.export_report()

    def load(self):
        self._members.load()

    def export_report(self):
        return NotImplementedError

    def process_frame(self, frame: np.array) -> bool:
        raise NotImplementedError

    def get_laser_points(self, frame: np.array, mask_corners: List = None):
        """
        Get laser points.
        if mask is set only the masked area will be processed.
        """

        if mask_corners:
            x_array = [i[0][0][0] for i in mask_corners]
            y_array = [i[0][0][1] for i in mask_corners]

            min_x = round(min(x_array))
            min_y = round(min(y_array))

            max_x = round(max(x_array))
            max_y = round(max(y_array))

            masked = np.zeros(frame.shape, np.uint8)
            masked[min_y:max_y, min_x:max_x] = frame[min_y:max_y, min_x:max_x]

            if self._cfg.debug_mode:
                cv2.imwrite(f"./bin/temp/{self._success}_masked.png", masked)
            return self.segment_laser(masked)
        else:
            return self.segment_laser(frame)

    def _generate_charuco_plane(self):
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self._charuco_board = cv2.aruco.CharucoBoard_create(
            self._cfg.calib_board.board_dim_x, self._cfg.calib_board.board_dim_y,
            self._cfg.calib_board.square_size, self._cfg.calib_board.marker_size,
            self._aruco_dict)

    def generate_charuco_plane(self):
        """Generate the charuco plane based on config."""

        imboard = self._charuco_board.draw((self._cfg.calib_board.print_size_x, self._cfg.calib_board.print_size_y))
        cv2.imwrite(self._cfg.output_folder + f"{self._cfg.output_name}.png", imboard)

    def generate_checker_plane(self):
        """Generate checker plane. Not used anymore."""

        result = np.zeros([self._cfg.calib_board.print_size_x, self._cfg.calib_board.print_size_y, 1], dtype=np.uint8)
        square_size = int(self._cfg.calib_board.print_size_x/self._cfg.calib_board.board_dim_x*2)

        for pos_x in range(0, int(self._cfg.calib_board.board_dim_x/2)):
            for pos_y in range(0, int(self._cfg.calib_board.board_dim_y/2)):
                if pos_x % 2 == pos_y % 2:
                    for i in range(0, square_size):
                        for j in range(0, square_size):
                            result[pos_x*square_size + i, pos_y*square_size + j] = 255

        cv2.imwrite(self._cfg.output_folder + f"{self._cfg.output_name}.png", result)

    def segment_laser(self, frame: np.array) -> Tuple[List, List]:
        """
        Segment laser line from frame.
        Will call the selected segmentation method.
        """

        if self._cfg.segmentation_mode == SegmentationMode.MIDLE:
            return self._segment_laser_midle(frame)
        elif self._cfg.segmentation_mode == SegmentationMode.INTENSITY:
            return self._segment_laser_intensity(frame)
        elif self._cfg.segmentation_mode == SegmentationMode.SUBPIXEL:
            return self._segment_laser_intensity(frame, True)

    def _determine_pixel_color(self, frame: np.array, mask: np.array, pos_x: int, pos_y: int):
        """Experimental pixel color determination around the laser line."""

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
        """Segment laser line from fame. Only use the midle of the masked area."""

        result = []
        colors = []

        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        _, thrs = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        last_non_zero = 0
        nonzero = np.nonzero(thrs)

        for i in range(len(nonzero[0])):
            if nonzero[0][last_non_zero] != nonzero[0][i]:
                pos_x = nonzero[0][last_non_zero]
                pos_y = round(nonzero[1][last_non_zero] +
                              ((nonzero[1][i-1]-nonzero[1][last_non_zero]) / 2.0))

                result.append([pos_y, pos_x])

                if self._cfg.color_mode:
                    colors.append(self._determine_pixel_color(frame, thrs, pos_x, pos_y))

                last_non_zero = i

        return result, colors

    def _segment_laser_intensity(self, frame: np.array, subpixel: bool = False) -> Tuple[List, List]:
        """
        Segment laser based on intensity.
        If subpixel is true, the segmentation will try a subpixel accuracy.
        """

        result = []
        colors = []

        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        _, thrs = cv2.threshold(img, 100, 255, cv2.THRESH_OTSU)

        nonzero = np.nonzero(thrs)

        if len(nonzero[0]) > img.shape[0]*img.shape[1]*self._cfg.laser_calib_th:
            return result, colors

        if self._cfg.debug_mode:
            cv2.imwrite(f"./bin/temp/{self._success}_threshold.png", thrs)

        last_non_zero = 0
        max_val = 0
        max_x = 0

        for i in range(len(nonzero[0])):

            if img[nonzero[0][i]][nonzero[1][i]] > max_val:
                max_val = img[nonzero[0][i]][nonzero[1][i]]
                max_x = i

            if nonzero[0][last_non_zero] != nonzero[0][i]:
                pos_x = nonzero[0][max_x]
                pos_y = nonzero[1][max_x]

                if pos_x+2 >= self._cfg.resolution[1] or pos_y+2 >= self._cfg.resolution[0]:
                    continue

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

    def export_pcd(self, points_3d, suffix: str = ""):
        """
        Export point cloud to file.
        """

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)

        if len(self._colors) > 0:
            pcd.colors = o3d.utility.Vector3dVector(np.array(self._colors).astype(np.float) / 255.0)

        if self._cfg.filter_mode == FilterMode.RADIUS:
            pcd, _ = pcd.remove_radius_outlier(nb_points=self._cfg.filter_param_1, radius=self._cfg.filter_param_2)
        elif self._cfg.filter_mode == FilterMode.STATISTICAL:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=self._cfg.filter_param_1, std_ratio=self._cfg.filter_param_2)

        o3d.io.write_point_cloud(self._cfg.output_folder + self._cfg.output_name + suffix + ".pcd", pcd)

    def _filter_points(self, points_3d):
        """
        Filter outlier cloud points.
        """

        if self._cfg.filter_mode == FilterMode.NO_FILTER:
            return points_3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)

        if len(self._colors) > 0:
            pcd.colors = o3d.utility.Vector3dVector(np.array(self._colors).astype(np.float) / 255.0)

        if self._cfg.filter_mode == FilterMode.RADIUS:
            pcd, _ = pcd.remove_radius_outlier(nb_points=self._cfg.filter_param_1, radius=self._cfg.filter_param_2)
        elif self._cfg.filter_mode == FilterMode.STATISTICAL:
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=self._cfg.filter_param_1, std_ratio=self._cfg.filter_param_2)

        return np.asarray(pcd.points)

    def export_stl(self, points_3d):
        """
        Export stl based on point cloud.
        This will only work after a proper cleanup and post processing.
        """

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)

        pcd.estimate_normals()

        poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, width=0, scale=1.1, linear_fit=False)[0]
        poisson_mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(poisson_mesh)

        bbox = pcd.get_axis_aligned_bounding_box()
        p_mesh_crop = poisson_mesh.crop(bbox)

        o3d.io.write_triangle_mesh(self._cfg.output_folder+self._cfg.output_name + ".stl", p_mesh_crop)

    def _draw_test_image(self, img_undist, rvec, tvec):
        """Draw test iamge with axis on charuco board"""

        newImage = img_undist.copy()

        imaxis = cv2.drawFrameAxes(newImage, self._members.camera_matrix,
                                   self._members.distortion_coefficients,
                                   rvec, tvec, 0.1)

        cv2.imwrite(f"./bin/temp/{self._success}_axis2.png", imaxis)
