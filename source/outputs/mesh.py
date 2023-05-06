import numpy as np
from outputs.calibration.base import Base
import cv2
from core.config_types import Config


class Mesh(Base):
    """Generate mesh from input frames."""

    _MIN_CORNER_NUMBER = 16

    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

        self._points_2d = []
        self._points_3d = []

        self.rotation_vectors = []
        self.translation_vectors = []

        self._max_sizes = (self._cfg.calib_board.square_size * self._cfg.calib_board.board_dim_x,
                           self._cfg.calib_board.square_size * self._cfg.calib_board.board_dim_y,
                           self._cfg.calib_board.square_size * (self._cfg.calib_board.board_dim_y
                                                                if self._cfg.calib_board.board_dim_y > self._cfg.calib_board.board_dim_x
                                                                else self._cfg.calib_board.board_dim_x))

    def process_frame(self, frame: np.array):
        # undistort image before processing
        img_undist = cv2.undistort(frame, self._members.camera_matrix, self._members.distortion_coefficients, None)
        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)

        # subpixel detection
        para = cv2.aruco.DetectorParameters_create()
        para.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=para)
        cam_inverse = np.linalg.inv(self._members.camera_matrix)

        if len(corners) > self._cfg.pos_confidence_th * len(self._charuco_board.chessboardCorners) and len(corners) > self._MIN_CORNER_NUMBER:
            # for corner in corners:
            #     cv2.cornerSubPix(gray, corner, winSize=(
            #         3, 3), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS +
            #                                             cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001))
            ret, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)

            if not ret:
                return

            # estimate charuco board pose
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(char_corners, char_ids, self._charuco_board, self._members.camera_matrix, self._members.distortion_coefficients,
                                                                 np.empty(1), np.empty(1))
            if not ret:
                return

            if self._cfg.debug_mode:
                self._draw_test_image(img_undist, rvec, tvec)

            # save corners
            self._all_corners.append(corners)
            self._all_ids.append(ids)

            temp_2d, colors = self.get_laser_points(img_undist)
            if len(temp_2d) == 0:
                return

            temp = np.array([0, 0, 0, 1])

            # rotation translation inverse matrix
            rot_trans = np.hstack((cv2.Rodrigues(rvec)[0], tvec))
            rot_trans = np.vstack((rot_trans, temp))
            rot_trans = np.linalg.inv(rot_trans)

            if self._cfg.debug_log:
                print(f"FOUND new imageId: {self._success} ")

            self._success += 1

            # calculate laser plane intersection with camera to pixel line
            for point in temp_2d:
                vector = [[point[0]],
                          [point[1]],
                          [1]]

                norm_point = cam_inverse.dot(vector)

                plane_translation = (-1*self._members.plane[2]) / \
                    (self._members.plane[0]*norm_point[0][0] + self._members.plane[1]*norm_point[1][0] - 1)

                result = [[plane_translation*norm_point[0][0]], [plane_translation*norm_point[1][0]], [plane_translation], [1]]

                result = rot_trans.dot(result)

                export = True

                # only export the charuco board sized cage
                if self._cfg.export_cage:
                    for i in range(len(self._max_sizes)):  # pylint: disable=consider-using-enumerate
                        if (result[i][0] > self._max_sizes[i] or result[i][0] < 0.001):
                            export = False
                            break

                if export:
                    self._points_3d.append([[result[0][0]], [result[1][0]], [result[2][0]]])

                    if self._cfg.color_mode:
                        self._colors.append(colors[i])

    def deinit(self):
        """Export filtered and unfiltered point cloud."""

        self.export_pcd(self._points_3d, "base")

        self._points_3d = self._filter_points(self._points_3d)

        self.export_pcd(self._points_3d, "filtered")

        # Turned off for now
        # self.export_stl(self._points_3d)
