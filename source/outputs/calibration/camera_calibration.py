import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import Base


class CameraCalibration(Base):
    def __init__(self, cfg: Config):
        Base.__init__(self, cfg)

    def process_frame(self, frame: np.array):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self._aruco_dict)

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

    def deinit(self):
        dist_coeffs_init = np.zeros((5, 1))

        camera_mtx_init = np.array([[1000.,       0., self._cfg.resolution[0]/2.],
                                    [0.,       1000., self._cfg.resolution[1]/2.],
                                    [0.,          0.,                         1.]])

        flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                 cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
        (ret, self._members.camera_matrix, self._members.distortion_coefficients,
         self._rotation_vectors, self._translation_vectors,
         _, _, _) = cv2.aruco.calibrateCameraCharucoExtended(
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
