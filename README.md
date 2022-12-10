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
!echo ---------------------------------------------------------------
!echo ------------------------- Raw metrics -------------------------
!echo ---------------------------------------------------------------
!radon raw source

!echo ---------------------------------------------------------------
!echo -------------------- Cyclomatic Complexity --------------------
!echo ---------------------------------------------------------------
!radon cc source -a -nc

!echo ---------------------------------------------------------------
!echo -------------------- Maintainability Index --------------------
!echo ---------------------------------------------------------------
!radon mi source
```

    ---------------------------------------------------------------
    ------------------------- Raw metrics -------------------------
    ---------------------------------------------------------------
    source/main.py
        LOC: 19
        LLOC: 11
        SLOC: 11
        Comments: 1
        Single comments: 1
        Multi: 0
        Blank: 7
        - Comment Stats
            (C % L): 5%
            (C % S): 9%
            (C + M % L): 5%
    source/inputs/oprncv_camera.py
        LOC: 42
        LLOC: 21
        SLOC: 24
        Comments: 6
        Single comments: 6
        Multi: 0
        Blank: 12
        - Comment Stats
            (C % L): 14%
            (C % S): 25%
            (C + M % L): 14%
    source/inputs/__init__.py
        LOC: 6
        LLOC: 4
        SLOC: 4
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 2
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/inputs/video.py
        LOC: 25
        LLOC: 16
        SLOC: 16
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 9
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/inputs/pc2_camera.py
        LOC: 36
        LLOC: 25
        SLOC: 24
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 12
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/inputs/images.py
        LOC: 36
        LLOC: 22
        SLOC: 24
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 12
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/ffmpeg_video.py
        LOC: 34
        LLOC: 13
        SLOC: 25
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 9
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/ffmpeg_stream.py
        LOC: 34
        LLOC: 14
        SLOC: 26
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 8
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/opencv_gstream.py
        LOC: 25
        LLOC: 14
        SLOC: 17
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 8
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/__init__.py
        LOC: 13
        LLOC: 9
        SLOC: 9
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 4
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/mesh.py
        LOC: 111
        LLOC: 71
        SLOC: 80
        Comments: 4
        Single comments: 4
        Multi: 0
        Blank: 27
        - Comment Stats
            (C % L): 4%
            (C % S): 5%
            (C + M % L): 4%
    source/outputs/images.py
        LOC: 24
        LLOC: 16
        SLOC: 16
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 8
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/opencv_video.py
        LOC: 22
        LLOC: 12
        SLOC: 15
        Comments: 1
        Single comments: 0
        Multi: 0
        Blank: 7
        - Comment Stats
            (C % L): 5%
            (C % S): 7%
            (C + M % L): 5%
    source/outputs/calibration/__init__.py
        LOC: 5
        LLOC: 4
        SLOC: 4
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 1
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/outputs/calibration/laser_calibration.py
        LOC: 179
        LLOC: 94
        SLOC: 94
        Comments: 44
        Single comments: 44
        Multi: 0
        Blank: 41
        - Comment Stats
            (C % L): 25%
            (C % S): 47%
            (C + M % L): 25%
    source/outputs/calibration/camera_calibration.py
        LOC: 72
        LLOC: 36
        SLOC: 50
        Comments: 2
        Single comments: 2
        Multi: 6
        Blank: 14
        - Comment Stats
            (C % L): 3%
            (C % S): 4%
            (C + M % L): 11%
    source/outputs/calibration/exposure_calibration.py
        LOC: 19
        LLOC: 12
        SLOC: 12
        Comments: 3
        Single comments: 0
        Multi: 0
        Blank: 7
        - Comment Stats
            (C % L): 16%
            (C % S): 25%
            (C + M % L): 16%
    source/outputs/calibration/base.py
        LOC: 220
        LLOC: 152
        SLOC: 164
        Comments: 2
        Single comments: 0
        Multi: 0
        Blank: 56
        - Comment Stats
            (C % L): 1%
            (C % S): 1%
            (C + M % L): 1%
    source/core/process_loop.py
        LOC: 36
        LLOC: 26
        SLOC: 26
        Comments: 1
        Single comments: 0
        Multi: 0
        Blank: 10
        - Comment Stats
            (C % L): 3%
            (C % S): 4%
            (C + M % L): 3%
    source/core/builder.py
        LOC: 63
        LLOC: 39
        SLOC: 45
        Comments: 1
        Single comments: 1
        Multi: 0
        Blank: 17
        - Comment Stats
            (C % L): 2%
            (C % S): 2%
            (C + M % L): 2%
    source/core/g3d_input.py
        LOC: 26
        LLOC: 19
        SLOC: 19
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 7
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/pickleable.py
        LOC: 13
        LLOC: 9
        SLOC: 9
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 4
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/config_types.py
        LOC: 70
        LLOC: 75
        SLOC: 52
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 18
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/__init__.py
        LOC: 6
        LLOC: 4
        SLOC: 4
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 2
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/g3d_output.py
        LOC: 14
        LLOC: 9
        SLOC: 9
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 5
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/config_constants.py
        LOC: 103
        LLOC: 12
        SLOC: 76
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 27
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    source/core/g3d_async_output.py
        LOC: 41
        LLOC: 28
        SLOC: 28
        Comments: 0
        Single comments: 0
        Multi: 0
        Blank: 13
        - Comment Stats
            (C % L): 0%
            (C % S): 0%
            (C + M % L): 0%
    [0m---------------------------------------------------------------
    -------------------- Cyclomatic Complexity --------------------
    ---------------------------------------------------------------
    [0m---------------------------------------------------------------
    -------------------- Maintainability Index --------------------
    ---------------------------------------------------------------
    source/main.py - [32mA[0m
    source/inputs/oprncv_camera.py - [32mA[0m
    source/inputs/__init__.py - [32mA[0m
    source/inputs/video.py - [32mA[0m
    source/inputs/pc2_camera.py - [32mA[0m
    source/inputs/images.py - [32mA[0m
    source/outputs/ffmpeg_video.py - [32mA[0m
    source/outputs/ffmpeg_stream.py - [32mA[0m
    source/outputs/opencv_gstream.py - [32mA[0m
    source/outputs/__init__.py - [32mA[0m
    source/outputs/mesh.py - [32mA[0m
    source/outputs/images.py - [32mA[0m
    source/outputs/opencv_video.py - [32mA[0m
    source/outputs/calibration/__init__.py - [32mA[0m
    source/outputs/calibration/laser_calibration.py - [32mA[0m
    source/outputs/calibration/camera_calibration.py - [32mA[0m
    source/outputs/calibration/exposure_calibration.py - [32mA[0m
    source/outputs/calibration/base.py - [32mA[0m
    source/core/process_loop.py - [32mA[0m
    source/core/builder.py - [32mA[0m
    source/core/g3d_input.py - [32mA[0m
    source/core/pickleable.py - [32mA[0m
    source/core/config_types.py - [32mA[0m
    source/core/__init__.py - [32mA[0m
    source/core/g3d_output.py - [32mA[0m
    source/core/config_constants.py - [32mA[0m
    source/core/g3d_async_output.py - [32mA[0m
    [0m


```python
# import cProfile
# cProfile.run('foo()')
#TODO: do the actual profiling
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
