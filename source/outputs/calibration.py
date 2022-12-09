import numpy as np
import pickle
from outputs.g3d_output import G3DOutput
from config_types import *
import cv2
from cv2 import aruco
from scipy.linalg import lstsq


class G3DCalibration(G3DOutput):
    def __init__(self, cfg: Config):
        G3DOutput.__init__(self, cfg.output_path)
        self._cfg = cfg
        self.counter = 0

        self._aruco_dict = None
        self._charuco_board = None

        self.allCorners = []
        self.allIds = []
        self.camera_matrix = None
        self.distortion_coefficients0 = None
        self.rotation_vectors = []
        self.translation_vectors = []
        self.stdDeviationsIntrinsics = None
        self.stdDeviationsExtrinsics = None
        self.perViewErrors = None

        self.selectedFrame = None
        self.laserPoints = []
        self.colors = []

        self.cameraMatrixInit = np.array([[1000.,       0., self._cfg.resolution[0]/2.],
                                          [0.,       1000., self._cfg.resolution[1]/2.],
                                          [0.,          0.,                         1.]])

        self.plane = []

        # if (self._cfg.calib_mode == CalibrationType.IMPORT):
        self.load()
        self._generateChArucoPlane()

    def deinit(self):
        if (self._cfg.calib_mode == CalibrationType.IMPORT):
            return

        if (self._cfg.calib_mode == CalibrationType.CAMERA or self._cfg.calib_mode == CalibrationType.FULL):
            self.calibrateCamera()

        if (self._cfg.calib_mode == CalibrationType.EXPOSURE or self._cfg.calib_mode == CalibrationType.FULL):
            self.calibrateExposure()

        if (self._cfg.calib_mode == CalibrationType.LASER or self._cfg.calib_mode == CalibrationType.FULL):
            self.calibrateLaser()

        # with open("./02_out/pickles/allCorners.pickle", "wb") as outfile:
        #     pickle.dump(self.allCorners, outfile)

        # with open("./02_out/pickles/allIds.pickle", "wb") as outfile:
        #     pickle.dump(self.allIds, outfile)

        # # with open("./02_out/pickles/camera_matrix.pickle", "wb") as outfile:
        # #     pickle.dump(self.camera_matrix, outfile)

        # # with open("./02_out/pickles/distortion_coefficients0.pickle", "wb") as outfile:
        # #     pickle.dump(self.distortion_coefficients0, outfile)

        # with open("./02_out/pickles/rotation_vectors.pickle", "wb") as outfile:
        #     pickle.dump(self.rotation_vectors, outfile)

        # with open("./02_out/pickles/translation_vectors.pickle", "wb") as outfile:
        #     pickle.dump(self.translation_vectors, outfile)

        # # with open("./02_out/pickles/stdDeviationsIntrinsics.pickle", "wb") as outfile:
        # #     pickle.dump(self.stdDeviationsIntrinsics, outfile)

        # # with open("./02_out/pickles/stdDeviationsExtrinsics.pickle", "wb") as outfile:
        # #     pickle.dump(self.stdDeviationsExtrinsics, outfile)

        # # with open("./02_out/pickles/perViewErrors.pickle", "wb") as outfile:
        # #     pickle.dump(self.perViewErrors, outfile)

        # # with open("./02_out/pickles/selectedFrame.pickle", "wb") as outfile:
        # #     pickle.dump(self.selectedFrame, outfile)

        # with open("./02_out/pickles/laserPoints.pickle", "wb") as outfile:
        #     pickle.dump(self.laserPoints, outfile)

        # with open("./02_out/pickles/plane.pickle", "wb") as outfile:
        #     pickle.dump(self.plane, outfile)

    def load(self):
        with open("./02_out/pickles/allCorners.pickle", "rb") as infile:
            self.allCorners = pickle.load(infile)

        with open("./02_out/pickles/allIds.pickle", "rb") as infile:
            self.allIds = pickle.load(infile)

        with open("./02_out/pickles/camera_matrix.pickle", "rb") as infile:
            self.camera_matrix = pickle.load(infile)

        with open("./02_out/pickles/distortion_coefficients0.pickle", "rb") as infile:
            self.distortion_coefficients0 = pickle.load(infile)

        with open("./02_out/pickles/rotation_vectors.pickle", "rb") as infile:
            self.rotation_vectors = pickle.load(infile)

        with open("./02_out/pickles/translation_vectors.pickle", "rb") as infile:
            self.translation_vectors = pickle.load(infile)

        with open("./02_out/pickles/stdDeviationsIntrinsics.pickle", "rb") as infile:
            self.stdDeviationsIntrinsics = pickle.load(infile)

        with open("./02_out/pickles/stdDeviationsExtrinsics.pickle", "rb") as infile:
            self.stdDeviationsExtrinsics = pickle.load(infile)

        with open("./02_out/pickles/perViewErrors.pickle", "rb") as infile:
            self.perViewErrors = pickle.load(infile)

        with open("./02_out/pickles/selectedFrame.pickle", "rb") as infile:
            self.selectedFrame = pickle.load(infile)

        with open("./02_out/pickles/laserPoints.pickle", "rb") as infile:
            self.laserPoints = pickle.load(infile)

        with open("./02_out/pickles/plane.pickle", "rb") as infile:
            self.plane = pickle.load(infile)

        # self.plane = []
        # self.laserPoints = []
        # self.allCorners = []
        # self.allIds = []

    def process_frame(self, frame: np.array):
        #TODO: fix
        # self._read_chessboards(frame)
        # self._read_for_laser(frame)

        self.deinit()
        exit(0)
        pass

    def _generateChArucoPlane(self):
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self._charuco_board = aruco.CharucoBoard_create(
            self._cfg.calib_board.boardDimX, self._cfg.calib_board.boardDimY, self._cfg.calib_board.squareSize, self._cfg.calib_board.markerSize, self._aruco_dict)

    def generateChArucoPlane(self):
        imboard = self._charuco_board.draw((self._cfg.calib_board.printSizeX, self._cfg.calib_board.printSizeY))
        cv2.imwrite(self._output_name + "charuccoBoard1.png", imboard)
        self.deinit()

    def generateCheckerPlane(self):
        result = np.zeros([self._cfg.calib_board.printSizeX, self._cfg.calib_board.printSizeY, 1], dtype=np.uint8)
        square_size = int(self._cfg.calib_board.printSizeX/self._cfg.calib_board.boardDimX*2)

        for x in range(0, int(self._cfg.calib_board.boardDimX/2)):
            for y in range(0, int(self._cfg.calib_board.boardDimY/2)):
                if x % 2 == y % 2:
                    for i in range(0, square_size):
                        for j in range(0, square_size):
                            result[x*square_size + i, y*square_size + j] = 255

        cv2.imwrite("chessBoard.png", result)

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

    def segmentLaserMidle(self, frame):
        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        _, thrs = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        temp = 0
        nz = np.nonzero(thrs)

        colors = []
        result = []

        for i in range(len(nz[0])):
            if nz[0][temp] != nz[0][i]:
                x = nz[0][temp]
                y = round(nz[1][temp]+(nz[1][i-1]-nz[1][temp])/2.0)

                result.append([y, x])

                if (self._cfg.color_mode):
                    colors.append(self.determinePixelColor(frame, thrs, x, y))
                temp = i

        return result, colors

    def segmentLaserIntensity(self, frame):
        img = cv2.subtract(frame[:, :, 2], frame[:, :, 1])
        img = cv2.GaussianBlur(img, (11, 1), 0, 0)
        rows, cols = img.shape
        _, thrs = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        temp = 0
        nz = np.nonzero(thrs)

        colors = []
        result = []
        maxVal = 0
        maxX = 0

        for i in range(len(nz[0])):

            if img[nz[0][i]][nz[1][i]] > maxVal:
                maxVal = img[nz[0][i]][nz[1][i]]
                maxX = i

            if nz[0][temp] != nz[0][i]:
                x = nz[0][maxX]
                y = nz[1][maxX]

                if (self._cfg.color_mode):
                    colors.append(self.determinePixelColor(frame, thrs, x, y))

                delta = (2*int(img[x][y+2])+int(img[x][y+1])-int(img[x][y-1])-2*int(img[x][y-2]))/(int(img[x][y-2])+int(img[x][y-1])+int(img[x][y])+int(img[x][y+1])+int(img[x][y+2]))

                result.append([y+delta, x])

                maxVal = 0
                maxX = 0
                temp = i

        return result, colors

    def segmentLaserSubpixel(self, frame):
        pass

    def determinePixelColor(self, frame, mask, x, y):
        color1 = None
        color2 = None

        CUSTOM_CONST = 5

        for i in range(self._cfg.resolution[0]-x):
            if (mask[x-i][y] == 0):
                if x-i-CUSTOM_CONST >= 0:
                    color1 = frame[x-i-CUSTOM_CONST][y]
                else:
                    color1 = frame[x-i][y]
                break

        for i in range(self._cfg.resolution[0]-x):
            if (mask[x+i][y] == 0):
                if x+i+CUSTOM_CONST >= 0:
                    color2 = frame[x+i+CUSTOM_CONST][y]
                else:
                    color2 = frame[x+i][y]
                break
        colorResult = (color2+color1)/2.0

        return colorResult

    def getLaserPoints(self, frame, pointsForMask):
        cv2.imwrite(f"./02_out/temp/original/{self.counter}.png", frame)
        x_array = [i[0][0][0] for i in pointsForMask]
        y_array = [i[0][0][1] for i in pointsForMask]

        # x_array = [i[0][0] for i in pointsForMask]
        # y_array = [i[0][1] for i in pointsForMask]

        min_x = round(min(x_array))
        min_y = round(min(y_array))

        max_x = round(max(x_array))
        max_y = round(max(y_array))

        # masked = np.zeros(frame.shape, np.uint8)
        # masked[min_y:max_y, min_x:max_x] = frame[min_y:max_y, min_x:max_x]
        # cv2.imwrite(f"./02_out/temp/masked/{self.counter}.png", masked)

        result = []

        if (self._cfg.segmentation_mode == SegmentationMode.MIDLE):
            result, colors = self.segmentLaserMidle(frame)
        elif (self._cfg.segmentation_mode == SegmentationMode.INTENSITY):
            result, colors = self.segmentLaserIntensity(frame)
        elif (self._cfg.segmentation_mode == SegmentationMode.SUBPIXEL):
            result, colors = self.segmentLaserSubpixel(frame)

        # cv2.imwrite("./02_out/temp/laser.png", laser)
        # temp = cv2.findNonZero(laser)[0][0]

        return result, colors

    def calibrateCamera(self):
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

    def testWithAxes(self, mtx, dist, rvecs, tvecs):
        img_undist = cv2.undistort(self.selectedFrame, mtx, dist, None)

        cv2.imwrite("./02_out/_distorted.png", self.selectedFrame)
        cv2.imwrite("./02_out/_undistorted.png", img_undist)

        cv2.imwrite("./02_out/_distortedLaser.png", self.segmentLaser(self.selectedFrame))
        cv2.imwrite("./02_out/_undistortedLaser.png", self.segmentLaser(img_undist))

        gray = cv2.cvtColor(self.selectedFrame, cv2.COLOR_BGR2GRAY)
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict,
                                                              parameters=parameters)
        print(len(corners))
        print(len(ids))
        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

        frame_markers = aruco.drawDetectedMarkers(self.selectedFrame.copy(), corners, ids)
        cv2.imwrite("./02_out/frame_markers.png", frame_markers)

        size_of_marker = 0.02  # side lenght of the marker in meter
        length_of_axis = 0.1
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)
        print(rvecs, tvecs)

        imaxis = aruco.drawDetectedMarkers(self.selectedFrame.copy(), corners, ids)

        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

        cv2.imwrite("./02_out/markers.png", imaxis)

    def calibrateLaser(self):
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

    def fit_plane(self, points):
        # https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
        rows, _ = points.shape
        A = np.ones((rows, 3))

        A[:, 0] = points[:, 0]
        A[:, 1] = points[:, 1]

        Z = points[:, 2]

        (a, b, c), _, _, _ = np.linalg.lstsq(A, Z)
        self.plane = [a, b, c]  # TODO: export/import pickle plane

    def calibrateExposure(self):
        pass


#
#
#
#
#
#
#
#
#
#
#
#


    def LASERSTUFF(self, mtx, dist, rvecs, tvecs):
        self.selectedFrame = cv2.imread("./out/_distorted.png")
        img_undist = cv2.undistort(self.selectedFrame, mtx, dist, None)

        # print(rvecs)
        # print(tvecs)
        # return

        cv2.imwrite("./out/_distorted.png", self.selectedFrame)
        cv2.imwrite("./out/_undistorted.png", img_undist)

        gray = cv2.cvtColor(self.selectedFrame, cv2.COLOR_BGR2GRAY)
        # _aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict,
                                                              parameters=parameters)

        objPoints, imagepoints = aruco.getBoardObjectAndImagePoints(
            self._charuco_board, corners, ids)

        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize=(
                3, 3), zeroZone=(-1, -1), criteria=criteria)

        print("1")
        print(len(corners))
        print("2")
        print(corners)
        print("3")
        print(len(ids))
        print("4")
        print(objPoints)
        print("5")
        print(imagepoints)
        # return

        # px=1251.4963
        # py=858.8839
        px = 1278.6255
        py = 866.16364
        Z = 0
        Lcam = mtx.dot(np.hstack((cv2.Rodrigues(rvecs[0])[0], tvecs[0])))
        X = np.linalg.inv(np.hstack((Lcam[:, 0:2], np.array(
            [[-1*px], [-1*py], [-1]])))).dot((-Z*Lcam[:, 2]-Lcam[:, 3]))
        print(X)
        return

        # px=1251.4963
        # py=858.8839
        # px=1278.6255
        # py=866.16364
        # 1251.4963     858.8839           0.185      0.055      0.       0.18571468 0.05308852 0.35872269
        # 1278.6255     866.16364          0.19500001 0.055      0.       0.19525311 0.05365029 0.35786451

        frame_markers = aruco.drawDetectedMarkers(
            self.selectedFrame.copy(), corners, ids)
        cv2.imwrite("./out/frame_markers.png", frame_markers)

        size_of_marker = 0.02  # side lenght of the marker in meter
        length_of_axis = 0.1
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, size_of_marker, mtx, dist)
        # print(rvecs, tvecs)

        imaxis = aruco.drawDetectedMarkers(
            self.selectedFrame.copy(), corners, ids)

        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(
                imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

        cv2.imwrite("./out/markers.png", imaxis)
