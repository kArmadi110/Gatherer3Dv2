import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import Base


class LaserCalibration(Base):
    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

        self._residuals = 0
        self._rank = 0

    def _fit_plane(self, points):
        rows, _ = points.shape
        a_mtx = np.ones((rows, 3))

        a_mtx[:, 0] = points[:, 0]
        a_mtx[:, 1] = points[:, 1]
        z_mtx = points[:, 2]

        (x_mul, y_mul, constant), self._residuals, self._rank, _ = np.linalg.lstsq(a_mtx, z_mtx)
        self._members.plane = [x_mul, y_mul, constant]

    def process_frame(self, frame: np.array):
        img_undist = cv2.undistort(frame, self._members.camera_matrix, self._members.distortion_coefficients, None)

        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=parameters)
        self._frame_num += 1

        if len(corners) > self._cfg.pos_confidence_th*len(self._charuco_board.chessboardCorners):
            ret, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)

            if not ret:
                return

            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(char_corners, char_ids,
                                                                 self._charuco_board, self._members.camera_matrix,
                                                                 self._members.distortion_coefficients,
                                                                 np.empty(1), np.empty(1))
            if not ret:
                return

            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(
                    3, 3), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS +
                                                        cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001))

            self._all_corners.append(corners)
            self._all_ids.append(ids)
            self._rotation_vectors.append(rvec)
            self._translation_vectors.append(tvec)
            self._laser_points.append(self.get_laser_points(img_undist, corners)[0])
            self._success += 1

    def deinit(self):
        plane_points = []

        for i in range(len(self._laser_points)):  # pylint: disable=consider-using-enumerate
            rotation_translation = np.hstack((cv2.Rodrigues(self._rotation_vectors[i])[0], self._translation_vectors[i]))
            t_cam = self._members.camera_matrix.dot(rotation_translation)

            for laser_point in self._laser_points[i]:
                pos_x = laser_point[0]
                pos_y = laser_point[1]
                pos_z = 0

                pos_3d = np.linalg.inv(np.hstack((t_cam[:, 0:2], np.array(
                    [[-1*pos_x], [-1*pos_y], [-1]])))).dot((-pos_z*t_cam[:, 2]-t_cam[:, 3]))

                temp_result = rotation_translation.dot(np.array([[pos_3d[0]], [pos_3d[1]], [0], [1]]))

                plane_points.append(np.array([temp_result[0][0], temp_result[1][0], temp_result[2][0]]))

        self._fit_plane(np.array(plane_points))
        self.export_pcd(plane_points)

        Base.deinit(self)

    def export_report(self):
        with open(f"{self._cfg.output_folder}laser{self._cfg.output_name}.txt", "a", encoding="utf8") as outfile:
            outfile.write("--------------------------------------------------")
            outfile.write(f"Laser Calibration on {self._cfg.input_folder + self._cfg.input_name}")
            outfile.write(f"Camera matrix\n {self._members.camera_matrix}\n")
            outfile.write(f"Plane fitting lstsq residuals \n {len(self._residuals)}\n")
            outfile.write(f"Plane fitting lstsq rank \n {self._rank}\n")
            outfile.write(f"Number of input\n {self._frame_num}\n")
            outfile.write(f"Number of useful inputs\n {self._success}\n")
