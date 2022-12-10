import numpy as np
from numpy.linalg import inv
import open3d as o3d
from outputs.calibration import G3DCalibration
import cv2
from core.config_types import *

from cv2 import aruco


class G3dMesh(G3DCalibration):
    _MIN_CORNER_NUMBER = 16

    def __init__(self, cfg: Config):
        G3DCalibration.__init__(self, cfg)
        self.points_2d = []
        self.points_3d = []

        self.maxSizes = (self._cfg.calib_board.squareSize*self._cfg.calib_board.boardDimX,
                         self._cfg.calib_board.squareSize*self._cfg.calib_board.boardDimY,
                         self._cfg.calib_board.squareSize * (self._cfg.calib_board.boardDimY
                                                             if self._cfg.calib_board.boardDimY > self._cfg.calib_board.boardDimX
                                                             else self._cfg.calib_board.boardDimX))

    def process_frame(self, frame: np.array):
        self.counter += 1
        print(self.counter)
        img_undist = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients0, None)
        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict, parameters=parameters)
        cam_inverse = np.linalg.inv(self.camera_matrix)

        if len(corners) > self._cfg.minimal_campos_precision * len(self._charuco_board.chessboardCorners) and len(corners) > G3dMesh._MIN_CORNER_NUMBER:
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize=(
                    3, 3), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS +
                                                        cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001))
            charucoretval, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(corners, ids, gray, self._charuco_board)
            ret, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self._charuco_board, self.camera_matrix, self.distortion_coefficients0,
                                                             np.empty(1), np.empty(1))

            self.allCorners.append(corners)
            self.allIds.append(ids)
            self.rotation_vectors.append(rvec)
            self.translation_vectors.append(tvec)

            temp_2d, colors = self.getLaserPoints(img_undist, corners)
            temp_3d = []
            temp = np.array([0, 0, 0, 1])

            Lcam = np.hstack((cv2.Rodrigues(rvec)[0], tvec))
            Lcam = np.vstack((Lcam, temp))
            LcamInverse = np.linalg.inv(Lcam)

            for i in range(len(temp_2d)):
                vector = [[temp_2d[i][0]],
                          [temp_2d[i][1]],
                          [1]]

                norm_point = np.vstack((cam_inverse.dot(vector), [1]))
                # norm_point = Lcam.dot(norm_point)

                # part1 = (-1)
                # part2 = (self.plane[0]*norm_point[0][0]+self.plane[1]*norm_point[1][0]+self.plane[2])

                part1 = (-1*self.plane[2])
                part2 = (self.plane[0]*norm_point[0][0]+self.plane[1]*norm_point[1][0]-1)

                t = part1/part2

                result0 = [[t*norm_point[0][0]], [t*norm_point[1][0]], [t], [1]]

                # result1 = Lcam.dot(result0)
                result2 = LcamInverse.dot(result0)

                export = True

                if (self._cfg.export_cage):
                    for i in range(len(self.maxSizes)):
                        if (result2[i][0] > self.maxSizes[i] or result2[i][0] < 0):
                            export = False
                            break

                if export:
                    self.points_3d.append([[result2[0][0]], [result2[1][0]], [result2[2][0]]])
                    self.colors.append(colors[i])

    def deinit(self):
        self.exportToPCD()
        # self.exportToSTL()

    def exportToPCD(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points_3d)
        pcd.colors = o3d.utility.Vector3dVector(np.array(self.colors).astype(np.float) / 255.0)
        o3d.io.write_point_cloud(self._cfg.output_path + self._cfg.output_file + ".pcd", pcd)

    def exportToSTL(self):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)

        pcd.estimate_normals()

        poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, width=0, scale=1.1, linear_fit=False)[0]
        poisson_mesh = o3d.geometry.TriangleMesh.compute_triangle_normals(poisson_mesh)

        bbox = pcd.get_axis_aligned_bounding_box()
        p_mesh_crop = poisson_mesh.crop(bbox)

        o3d.io.write_triangle_mesh(self._output_name + ".stl", p_mesh_crop)

    def testPointClouds(self):
        points = []

        for i in range(11):
            for j in range(11):
                points.append([i/10.0, j/10.0, 0])

        for i in range(11):
            for j in range(11):
                points.append([i/10.0, j/10.0, 1])

        for i in range(11):
            for j in range(11):
                points.append([i/10.0, 0, j/10.0])

        for i in range(11):
            for j in range(11):
                points.append([0, i/10.0, j/10.0])

        for i in range(11):
            for j in range(11):
                points.append([i/10.0, 1, j/10.0])

        for i in range(11):
            for j in range(11):
                points.append([1, i/10.0, j/10.0])

        self.exportToPCD("./out/test1.pcd", points)
        self.exportToSTL("./out/test1.stl", points)
