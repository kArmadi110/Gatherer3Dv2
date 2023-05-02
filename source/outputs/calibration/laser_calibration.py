import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import Base


class LaserCalibration(Base):
    """Laser calibration."""

    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

        self._residuals = 0
        self._rank = 0
        self._rotation_vectors = []
        self._translation_vectors = []
        self._members.plane = []

    def _fit_plane(self, points):
        """Fit plane to the given points."""

        rows, _ = points.shape
        a_mtx = np.ones((rows, 3))

        a_mtx[:, 0] = points[:, 0]
        a_mtx[:, 1] = points[:, 1]
        z_mtx = points[:, 2]

        (x_mul, y_mul, constant), self._residuals, self._rank, _ = np.linalg.lstsq(a_mtx, z_mtx)
        self._members.plane = [x_mul, y_mul, constant]

    def process_frame(self, frame: np.array):
        """Process frame and collect laser points."""

        # undistort the image before laser detection
        img_undist = cv2.undistort(frame, self._members.camera_matrix, self._members.distortion_coefficients, None)

        # we will use the gray image for charuco detection
        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)

        # detect markers with subpixel accuracy
        para = cv2.aruco.DetectorParameters_create()
        para.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=para)

        self._frame_num += 1

        if len(corners) > self._cfg.pos_confidence_th*len(self._charuco_board.chessboardCorners):
            ret, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)

            if not ret:
                return

            # estimate board pose
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(char_corners, char_ids,
                                                                 self._charuco_board, self._members.camera_matrix,
                                                                 self._members.distortion_coefficients,
                                                                 np.empty(1), np.empty(1))
            if not ret:
                return

            # get the laser points
            temp = self.get_laser_points(img_undist, corners)[0]

            if len(temp) > 0:
                # save laser points and board position if there is any.
                self._all_corners.append(corners)
                self._all_ids.append(ids)
                self._rotation_vectors.append(rvec)
                self._translation_vectors.append(tvec)
                self._laser_points.append(temp)

                print(f"FOUND{self._success} ")

                # DEBUG: write image
                # cv2.imwrite(f"./bin/temp/{self._success}_original.png", img_undist)
                self._success += 1

    def deinit(self):
        """Calibrate laser plane equation."""

        plane_points = []

        for i in range(len(self._laser_points)):  # pylint: disable=consider-using-enumerate
            # rotation|translation matrix
            rotation_translation = np.hstack((cv2.Rodrigues(self._rotation_vectors[i])[0], self._translation_vectors[i]))

            # rotation matrix inverse
            rot_mi = np.linalg.inv(cv2.Rodrigues(self._rotation_vectors[i])[0])

            # translation vector
            tran_vec = np.array(self._translation_vectors[i])

            for laser_point in self._laser_points[i]:

                # detected laser point
                pos_x = laser_point[0]
                pos_y = laser_point[1]
                # for laser calibration every pointis on the same plane
                # it's on the charuco board.
                pos_z = 0

                # left side of our equation
                left = rot_mi.dot(np.linalg.inv(self._members.camera_matrix)).dot(np.array([[pos_x], [pos_y], [1]]))

                # right side of our equation
                right = rot_mi.dot(tran_vec)

                # reverse the camera projection to the charuco calibration plane.
                pos_3d = rot_mi.dot((pos_z+right[2])/left[2] *
                                    np.linalg.inv(self._members.camera_matrix).dot(
                                        np.array([[pos_x], [pos_y], [1]])) -
                                    tran_vec)

                # move the point to the camera position
                temp_result = rotation_translation.dot(np.array([[pos_3d[0]], [pos_3d[1]], [0], [1]]))

                plane_points.append(np.array([temp_result[0][0], temp_result[1][0], temp_result[2][0]]))

        self.export_pcd(plane_points, "base")
        plane_points = self._filter_points(plane_points)

        self.export_pcd(plane_points, "filtered")

        self._fit_plane(np.array(plane_points))

        Base.deinit(self)

    def export_report(self):
        """Export laser calibration report."""

        with open(f"{self._cfg.output_folder}laser{self._cfg.output_name}.txt", "a", encoding="utf8") as outfile:
            outfile.write("--------------------------------------------------")
            outfile.write(f"Laser Calibration on {self._cfg.input_folder + self._cfg.input_name}")
            outfile.write(f"Camera matrix\n {self._members.camera_matrix}\n")
            outfile.write(f"Plane fitting lstsq residuals \n {len(self._residuals)}\n")
            outfile.write(f"Plane fitting lstsq rank \n {self._rank}\n")
            outfile.write(f"Number of input\n {self._frame_num}\n")
            outfile.write(f"Number of useful inputs\n {self._success}\n")
