# Installation
## Introduction
TODO: general stuff from the project
jupyter nbconvert --execute --to markdown README.ipynb

## Required models


```python
#TODO: show required frame
```

## Required hardware
TODO: 

## Required packages


```python
#TODO: python required packages + streaming + used tools
```

## Usage
### Calibration board 


```python
#TODO: generate two calibration boards
#TODO: calibration with axes on image

    # def _exportExamples(self, mtx, dist, rvecs, tvecs):
    #     self.selectedFrame = cv2.imread("./out/_distorted.png")
    #     img_undist = cv2.undistort(self.selectedFrame, mtx, dist, None)

    #     # print(rvecs)
    #     # print(tvecs)
    #     # return

    #     cv2.imwrite("./out/_distorted.png", self.selectedFrame)
    #     cv2.imwrite("./out/_undistorted.png", img_undist)

    #     gray = cv2.cvtColor(self.selectedFrame, cv2.COLOR_BGR2GRAY)
    #     # _aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    #     parameters = aruco.DetectorParameters_create()
    #     corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict,
    #                                                           parameters=parameters)

    #     objPoints, imagepoints = aruco.getBoardObjectAndImagePoints(
    #         self._charuco_board, corners, ids)

    #     # SUB PIXEL DETECTION
    #     criteria = (cv2.TERM_CRITERIA_EPS +
    #                 cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    #     for corner in corners:
    #         cv2.cornerSubPix(gray, corner, winSize=(
    #             3, 3), zeroZone=(-1, -1), criteria=criteria)

    #     print("1")
    #     print(len(corners))
    #     print("2")
    #     print(corners)
    #     print("3")
    #     print(len(ids))
    #     print("4")
    #     print(objPoints)
    #     print("5")
    #     print(imagepoints)
    #     # return

    #     # px=1251.4963
    #     # py=858.8839
    #     px = 1278.6255
    #     py = 866.16364
    #     Z = 0
    #     Lcam = mtx.dot(np.hstack((cv2.Rodrigues(rvecs[0])[0], tvecs[0])))
    #     X = np.linalg.inv(np.hstack((Lcam[:, 0:2], np.array(
    #         [[-1*px], [-1*py], [-1]])))).dot((-Z*Lcam[:, 2]-Lcam[:, 3]))
    #     print(X)
    #     return

    #     # px=1251.4963
    #     # py=858.8839
    #     # px=1278.6255
    #     # py=866.16364
    #     # 1251.4963     858.8839           0.185      0.055      0.       0.18571468 0.05308852 0.35872269
    #     # 1278.6255     866.16364          0.19500001 0.055      0.       0.19525311 0.05365029 0.35786451

    #     frame_markers = aruco.drawDetectedMarkers(
    #         self.selectedFrame.copy(), corners, ids)
    #     cv2.imwrite("./out/frame_markers.png", frame_markers)

    #     size_of_marker = 0.02  # side lenght of the marker in meter
    #     length_of_axis = 0.1
    #     rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
    #         corners, size_of_marker, mtx, dist)
    #     # print(rvecs, tvecs)

    #     imaxis = aruco.drawDetectedMarkers(
    #         self.selectedFrame.copy(), corners, ids)

    #     for i in range(len(tvecs)):
    #         imaxis = aruco.drawAxis(
    #             imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

    #     cv2.imwrite("./out/markers.png", imaxis)

    # def testWithAxes(self, mtx, dist, rvecs, tvecs):
    #     img_undist = cv2.undistort(self.selectedFrame, mtx, dist, None)

    #     cv2.imwrite("./02_out/_distorted.png", self.selectedFrame)
    #     cv2.imwrite("./02_out/_undistorted.png", img_undist)

    #     cv2.imwrite("./02_out/_distortedLaser.png", self.segmentLaser(self.selectedFrame))
    #     cv2.imwrite("./02_out/_undistortedLaser.png", self.segmentLaser(img_undist))

    #     gray = cv2.cvtColor(self.selectedFrame, cv2.COLOR_BGR2GRAY)
    #     # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    #     parameters = aruco.DetectorParameters_create()
    #     corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict,
    #                                                           parameters=parameters)
    #     print(len(corners))
    #     print(len(ids))
    #     # SUB PIXEL DETECTION
    #     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    #     for corner in corners:
    #         cv2.cornerSubPix(gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)

    #     frame_markers = aruco.drawDetectedMarkers(self.selectedFrame.copy(), corners, ids)
    #     cv2.imwrite("./02_out/frame_markers.png", frame_markers)

    #     size_of_marker = 0.02  # side lenght of the marker in meter
    #     length_of_axis = 0.1
    #     rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)
    #     print(rvecs, tvecs)

    #     imaxis = aruco.drawDetectedMarkers(self.selectedFrame.copy(), corners, ids)

    #     for i in range(len(tvecs)):
    #         imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)

    #     cv2.imwrite("./02_out/markers.png", imaxis)

```

## Configs
TODO: describe the configs with examples


```python
#TODO: describe the configs with examples
```

## Example run on a test video sequence


```python
#TODO: Example run on a test video sequence
```

# Code analysis

## Metrics
Source:

 - https://pypi.org/project/radon/
 - https://radon.readthedocs.io/en/latest/commandline.html


```python
!radon raw source > ./docs/analysis/raw.txt
!radon cc source > ./docs/analysis/cc.txt
!radon mi source > ./docs/analysis/mi.txt
```


```python
#!python -m cProfile ./source/main.py > ./docs/analysis/cProfile.txt
#!python -m cProfile -o ./docs/analysis/cProfile.prof ./source/main.py
#!python -m snakeviz ./docs/analysis/cProfile.prof
```

## Architecture
TODO: add architecture

# Figures


```python
# TODO: sheered plane on test image
# TODO: calibration plane and laser plane intersection
# TODO: laser degree calculation and figure
# TODO: demonstrational image with oriignal image + segmented line + 3d points gathered
# TODO: module structure
# TODO: cube example from mesh.py

    # def testPointClouds(self):
    #     points = []

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([i/10.0, j/10.0, 0])

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([i/10.0, j/10.0, 1])

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([i/10.0, 0, j/10.0])

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([0, i/10.0, j/10.0])

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([i/10.0, 1, j/10.0])

    #     for i in range(11):
    #         for j in range(11):
    #             points.append([1, i/10.0, j/10.0])

    #     self.exportToPCD("./out/test1.pcd", points)
    #     self.exportToSTL("./out/test1.stl", points)

```

# Tests


```python
#TODO: Test in dark
#TODO: Test in light
#TODO: Test with horizontal line
#TODO: Test with vertical line

```
