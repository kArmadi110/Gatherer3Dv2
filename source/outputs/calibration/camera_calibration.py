import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import CalibrationBase


class CameraCalibration(CalibrationBase):
    def __init__(self, cfg: Config):
        CalibrationBase.__init__(self, cfg)

    def process_frame(self, frame: np.array):
        self._read_chessboards(frame)

    def calibrate(self):
        """
        Calibrates the camera using the dected corners.
        """
        print("CAMERA CALIBRATION")
        print(f"{self.allCorners} {self.allIds}")
        print(f"{len(self.allCorners)} {len(self.allIds)}")

        distCoeffsInit = np.zeros((5, 1))
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                 cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
        # flags = (cv2.CALIB_RATIONAL_MODEL)
        (ret, self.camera_matrix, self.distortion_coefficients0,
         self.rotation_vectors, self.translation_vectors,
         self.stdDeviationsIntrinsics, self.stdDeviationsExtrinsics,
         self.perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=self.allCorners,
            charucoIds=self.allIds,
            board=self._charuco_board,
            imageSize=self._cfg.resolution,
            cameraMatrix=self.cameraMatrixInit,
            distCoeffs=distCoeffsInit,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
        # self.testWithAxes(self.camera_matrix, self.distortion_coefficients0, self.rotation_vectors, self.translation_vectors)

    def _read_chessboards(self, frame):
        """
        Charuco base pose estimation.
        """
        self.counter += 1
        print(self.counter)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
            gray, self._aruco_dict)

        if len(corners) > 0.15*len(self._charuco_board.chessboardCorners):
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001))
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)

            if res2[1] is not None and res2[2] is not None:

                if (self.selectedFrame is None):
                    self.selectedFrame = frame

                self.allCorners.append(res2[1])
                self.allIds.append(res2[2])
                objPoints, imagepoints = aruco.getBoardObjectAndImagePoints(self._charuco_board, corners, ids)
                self.laserPoints.append(self.getLaserPoints(frame, imagepoints))
                cv2.imwrite(f"./02_out/temp/to_segment{self.counter}.png", frame)

        self.imsize = gray.shape
