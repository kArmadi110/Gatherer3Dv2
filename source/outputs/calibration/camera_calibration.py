import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import Base


class CameraCalibration(Base):
    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)
        self._std_dev_intr = 0
        self._std_dev_extr = 0
        self._per_view_errors = 0

        self._success = 0
        self._frame_num = 0

    def process_frame(self, frame: np.array):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict)
        self._frame_num += 1

        if len(corners) > self._cfg.pos_confidence_th*len(self._charuco_board.chessboardCorners):
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001))
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)

            if res2[1] is not None and res2[2] is not None:
                self._all_corners.append(res2[1])
                self._all_ids.append(res2[2])
                self._success += 1

    def deinit(self):
        dist_coeffs_init = np.zeros((5, 1))

        camera_mtx_init = np.array([[1000.,       0., self._cfg.resolution[0]/2.],
                                    [0.,       1000., self._cfg.resolution[1]/2.],
                                    [0.,          0.,                         1.]])

        flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                 cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
        (ret, self._members.camera_matrix, self._members.distortion_coefficients,
         self._rotation_vectors, self._translation_vectors,
         self._std_dev_intr, self._std_dev_extr, self._per_view_errors) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=self._all_corners,
            charucoIds=self._all_ids,
            board=self._charuco_board,
            imageSize=self._cfg.resolution,
            cameraMatrix=camera_mtx_init,
            distCoeffs=dist_coeffs_init,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

        if not ret:
            raise RuntimeError("Failed to calibrate the camera")

        Base.deinit(self)

    def export_report(self):
        with open(f"{self._cfg.output_folder}camera{self._cfg.output_name}.txt", "a", encoding="utf8") as outfile:
            outfile.write("--------------------------------------------------\n")
            outfile.write(f"Camera Calibration on {self._cfg.input_folder + self._cfg.input_name}\n")
            outfile.write(f"Camera matrix\n {self._members.camera_matrix}\n")
            outfile.write(f"std deviation intrinsics\n {self._std_dev_intr}\n")
            outfile.write(f"std deviation extrinsics\n {self._std_dev_extr}\n")
            outfile.write(f"Re-projection error \n {self._per_view_errors}\n")
            outfile.write(f"Avarage reprojection error\n {np.average(self._per_view_errors)}\n")
            outfile.write(f"Standard deviation of reprojection error\n {np.std(self._per_view_errors)}\n")
            outfile.write(f"Number of  found corner\n {len(self._all_corners)}\n")
            outfile.write(f"Number of frames\n {self._frame_num}\n")
            outfile.write(f"Number of useful frames\n {self._success}\n")
