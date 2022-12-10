import numpy as np
import cv2

from core.config_types import Config

from outputs.calibration.base import CalibrationBase


class LaserCalibration(CalibrationBase):
    def __init__(self, cfg: Config):
        CalibrationBase.__init__(self, cfg)

    def _fit_plane(self, points):
        rows, _ = points.shape
        A = np.ones((rows, 3))

        A[:, 0] = points[:, 0]
        A[:, 1] = points[:, 1]
        Z = points[:, 2]

        (a, b, c), _, _, _ = np.linalg.lstsq(A, Z)
        self.plane = [a, b, c]

    def process_frame(self, frame: np.array):
        self._read_for_laser(frame)

    def calibrate(self):
        results1 = []
        results2 = []
        results3 = []
        results4 = []
        results5 = []
        results6 = []

        temp = np.array([0, 0, 0, 1])
        LcamOriginal = np.hstack((cv2.Rodrigues(self.rotation_vectors[0])[0], self.translation_vectors[0]))

        for i in range(len(self.laserPoints)):
            # if (i == 0):
            #     Lcam = self.camera_matrix.dot(LcamOriginal)
            # else:
            # Lcam = self.camera_matrix.dot(np.hstack((cv2.Rodrigues(self.rotation_vectors[i])[0],
            #                                         self.translation_vectors[i])))
            # LcamTemp = np.dot(np.vstack((Lcam, temp)), np.linalg.inv(np.vstack((LcamOriginal, temp))))
            # Lcam = np.dot(Lcam, LcamTemp)
            # Lcam = np.delete(Lcam, 3, 0)

            # Lcam = self.camera_matrix.dot(LcamOriginal)
            Transf = np.hstack((cv2.Rodrigues(self.rotation_vectors[i])[0], self.translation_vectors[i]))
            # Lcam = np.vstack((Lcam, temp))
            # LcamTemp = np.dot(Lcam, np.linalg.inv(np.vstack((LcamOriginal, temp))))
            # Lcam = np.dot(Lcam, np.linalg.inv(LcamTemp))

            # Lcam = np.delete(Lcam, 3, 0)
            Lcam = self.camera_matrix.dot(Transf)

            # result = []
            for j in range(len(self.laserPoints[i])):
                px = self.laserPoints[i][j][0][0]
                py = self.laserPoints[i][j][0][1]
                Z = 0

                X = np.linalg.inv(np.hstack((Lcam[:, 0:2], np.array(
                    [[-1*px], [-1*py], [-1]])))).dot((-Z*Lcam[:, 2]-Lcam[:, 3]))

                resultCopy = np.array([[X[0]], [X[1]], [0], [1]])

                X1 = Lcam.dot(resultCopy)

                tempCam = np.linalg.inv(np.vstack((Lcam, temp)))
                X2 = tempCam.dot(resultCopy)

                tempCam = np.delete(tempCam, 3, 0)
                X3 = tempCam.dot(resultCopy)

                X4 = Transf.dot(resultCopy)

                tempCam = np.linalg.inv(np.vstack((Transf, temp)))
                X5 = tempCam.dot(resultCopy)

                tempCam = np.delete(tempCam, 3, 0)
                X6 = tempCam.dot(resultCopy)

                # result.append(X)
                results1.append(np.array([X1[0][0], X1[1][0], X1[2][0]]))
                results2.append(np.array([X2[0][0], X2[1][0], X2[2][0]]))
                results3.append(np.array([X3[0][0], X3[1][0], X3[2][0]]))
                results4.append(np.array([X4[0][0], X4[1][0], X4[2][0]]))
                results5.append(np.array([X5[0][0], X5[1][0], X5[2][0]]))
                results6.append(np.array([X6[0][0], X6[1][0], X6[2][0]]))

        # for i in range(len(results)):
        #     for j in range(len(results[i])):
        #         results[i][j][2] = 0

        # for i in range(0, len(results)):
        #     for j in range(len(results[i])):

        #         Lcam = np.hstack((cv2.Rodrigues(self.rotation_vectors[i])[0], self.translation_vectors[i]))
        #         # Lcam = self.camera_matrix.dot(Lcam)
        #         # Lcam = np.vstack((Lcam, temp))
        #         # LcamTemp = np.dot(Lcam, np.linalg.inv(np.vstack((LcamOriginal, temp))))
        #         # # Lcam = np.dot(Lcam, np.linalg.inv(LcamTemp))
        #         # # Lcam = LcamTemp
        #         # Lcam = np.linalg.inv(np.vstack((Lcam, temp)))
        #         # Lcam = np.delete(Lcam, 3, 0)

        #         resultCopy = np.array([[results[i][j][0]], [results[i][j][1]], [results[i][j][2]], [1]])
        #         # TODO: generate figure
        #         X = Lcam.dot(resultCopy)
        #         # X = np.linalg.inv(Lcam).dot(temp)

        #         results[i][j] = X

        # finalPoints = []
        # for i in len(results):
        #     for j in len(results[i]):
        #         X = transform
        #         finalPoints.append(X)

        with open("./02_out/laser1.txt", "w") as outfile:
            for i in range(len(results1)):
                # for j in range(len(results[i])):
                outfile.write(f"{results1[i][0]} {results1[i][1]} {results1[i][2]}\n")

        with open("./02_out/laser2.txt", "w") as outfile:
            for i in range(len(results2)):
                # for j in range(len(results[i])):
                outfile.write(f"{results2[i][0]} {results2[i][1]} {results2[i][2]}\n")

        with open("./02_out/laser3.txt", "w") as outfile:
            for i in range(len(results2)):
                # for j in range(len(results[i])):
                outfile.write(f"{results3[i][0]} {results3[i][1]} {results3[i][2]}\n")

        with open("./02_out/laser4.txt", "w") as outfile:
            for i in range(len(results2)):
                # for j in range(len(results[i])):
                outfile.write(f"{results4[i][0]} {results4[i][1]} {results4[i][2]}\n")

        with open("./02_out/laser5.txt", "w") as outfile:
            for i in range(len(results2)):
                # for j in range(len(results[i])):
                outfile.write(f"{results5[i][0]} {results5[i][1]} {results5[i][2]}\n")

        with open("./02_out/laser6.txt", "w") as outfile:
            for i in range(len(results2)):
                # for j in range(len(results[i])):
                outfile.write(f"{results6[i][0]} {results6[i][1]} {results6[i][2]}\n")

        self.fit_plane(np.array(results1))
        self.fit_plane(np.array(results2))
        self.fit_plane(np.array(results3))
        self.fit_plane(np.array(results4))

    def _read_for_laser(self, frame):
        img_undist = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients0, None)

        self.counter += 1
        print(self.counter)

        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict, parameters=parameters)

        if len(corners) > 0.4*len(self._charuco_board.chessboardCorners):
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)
            ret, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self._charuco_board, self.camera_matrix, self.distortion_coefficients0,
                                                             np.empty(1), np.empty(1))

            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(
                    3, 3), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS +
                                                        cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001))
            self.allCorners.append(corners)
            self.allIds.append(ids)
            self.rotation_vectors.append(rvec)
            self.translation_vectors.append(tvec)
            self.laserPoints.append(self.getLaserPoints(img_undist, corners))
