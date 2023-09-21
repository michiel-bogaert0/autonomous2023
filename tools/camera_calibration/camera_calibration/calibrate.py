#!/usr/bin/python
import glob
import random

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from tqdm import tqdm


class Calibrator:
    def __init__(self, size: str, square_length: int, marker_length: int):
        """
        Initializes the calibrator

        Args:
          size: string in the format {nr_hor_squares}x{nr_ver_squares} which corresponds to the chosen chessboard format.
                Example: '6x7'
          square_length: the actual length of a square. The unit that you choose here will also be the unit that gets returned in
                for example the PnP algorithm. Set to 1 and the relative position will be given in number of squares. Set to the actual
                size of a square in mm (for example, 21mm) and the relative position will be given in mm.
          marker_length: "The real length of a side of a marker in mm (only relevant when using charuco boards)"
        """

        self.nr_hor_squares = int(size.split("x")[0])
        self.nr_ver_squares = int(size.split("x")[1])
        self.size = (self.nr_hor_squares, self.nr_ver_squares)
        self.square_length = square_length
        self.marker_length = marker_length

        # Opencv stuff
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)

        self.objectPoints = np.zeros((np.prod(self.size), 3), np.float32)
        self.objectPoints[:, :2] = np.indices(self.size).T.reshape(-1, 2)

        self.objectPoints *= self.square_length

    def __resizeWithAspectRatio__(
        self, image, width=None, height=None, inter=cv.INTER_AREA
    ):
        """
        Resizes an image while respecting aspect ratio.

        Args:
          image: the image to resize
          width (default=None): the new width of the image.
          height (default=None): the new width of the image. This parameter has precedence over width (for when you supply both width and height)
          inter: interpolation method

        Returns:
          the resized image object, or the original if width == height == None
        """
        dim = None
        (h, w) = image.shape[:2]

        if width is None and height is None:
            return image
        if width is None:
            r = height / float(h)
            dim = (int(w * r), height)
        else:
            r = width / float(w)
            dim = (width, int(h * r))

        return cv.resize(image, dim, interpolation=inter)

    def draw(self, img, corners, imgpts):
        """
        Simple drawing function for the end result to check calibration

        Args:
          img: the input image
          corners: the detected corners of the chessboard
          imgpts: the projected points to draw

        Returns:
          the same image object, but now with a drawing on it!
        """

        corners = corners.astype(int)
        imgpts = imgpts.astype(int)

        corner = tuple(corners[0].ravel())
        img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img

    def chessboard_calibrate(self, imgPath: str):
        """
        Calibration using a chessboard pattern.

        Args:
          imgPath: the path to the calibration images. This can be a mask such as '*.jpg'

        Returns:
          mtx: the camera calibration matrix as defined in opencv
          dist: the distortion matrix as defined in opencv
        """

        imgPoints = []
        objPoints = []

        # Read images to fetch image points

        images = glob.glob(imgPath)
        for fname in images:
            img = cv.imread(fname)
            img = self.__resizeWithAspectRatio__(img, height=1024)

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, self.size, None)

            if ret == True:
                objPoints.append(self.objectPoints)
                refinedCorners = cv.cornerSubPix(
                    gray, corners, (5, 5), (-1, -1), self.criteria
                )
                imgPoints.append(refinedCorners.reshape(-1, 2))

                # Draw and display the corners
                cv.drawChessboardCorners(img, self.size, refinedCorners, ret)
                cv.imshow("Chessboard", img)
                cv.waitKey(50)

        # Calibrate
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objPoints, imgPoints, gray.shape[::-1], None, None
        )

        # Do a test projection for testing purposes
        for fname in images:
            img = cv.imread(fname)
            img = self.__resizeWithAspectRatio__(img, height=1024)

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, self.size, None)

            if ret == True:
                refinedCorners = cv.cornerSubPix(
                    gray, corners, (5, 5), (-1, -1), self.criteria
                )

                ret, rvecs, tvecs = cv.solvePnP(
                    self.objectPoints, refinedCorners.reshape(-1, 2), mtx, dist
                )

                projection, jacobian = cv.projectPoints(
                    np.float32(
                        [
                            [self.square_length, 0, 0],
                            [0, self.square_length, 0],
                            [0, 0, -self.square_length],
                        ]
                    ).reshape(-1, 3),
                    rvecs,
                    tvecs,
                    mtx,
                    dist,
                )

                img = self.draw(img, refinedCorners, projection)

                cv.imshow("Chessboard", img)
                break

        cv.waitKey(20000)
        cv.destroyAllWindows()

        return mtx, dist

    def create_charucoboard(self, aruco_type: dict):
        """Create a charucoboard object

        Args:
            aruco_type (dict): The type of the charuco board

        Returns:
            aruco.board: The generated charuco board
        """

        # Create a dictionary
        dictionary = aruco.getPredefinedDictionary(aruco_type)

        # board parameters
        board = aruco.CharucoBoard_create(
            self.nr_hor_squares,
            self.nr_ver_squares,
            self.square_length,
            self.marker_length,
            dictionary,
        )

        return board

    def charucoboard_calibrate(self, imgPath: str, aruco_type: dict):
        """
        Calibration using a charuco board

        Args:
          videoPath (str): the path to the calibration video.
          aruco_type (dict): charuco board type as defined in opencv

        Returns:
          ndarray: the camera calibration matrix as defined in opencv
          ndarray: the distortion matrix as defined in opencv
        """

        # Create a dictionary
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

        # create the board
        board = self.create_charucoboard(aruco_type)

        # Calibration

        allCorners = []
        allIds = []
        allImgs = []

        images = glob.glob(imgPath)

        if len(images) == 0:
            print("no images in selected path")
            return

        # Go over each image and detect the markers
        for fname in tqdm(images):
            frame = cv.imread(fname)
            allImgs.append(frame)

            corners, ids, rejected = aruco.detectMarkers(frame, dictionary)

            # if nothin was detected
            if corners == None or len(corners) == 0:
                continue

            ret, charucoCorners, charucoIds = aruco.interpolateCornersCharuco(
                corners, ids, frame, board
            )

            # The parameters depend on the initial value detected by detectMarkers
            if (
                corners is not None
                and charucoIds is not None
                and not len(charucoCorners) < 4
            ):
                allCorners.append(charucoCorners)
                allIds.append(charucoIds)

        w, h = allImgs[0].shape[1], allImgs[0].shape[0]
        print("start constructing calibration matrix, this may take a while")
        ret, cameraMatrix, distCoeff, rvecs, tvecs = aruco.calibrateCameraCharuco(
            allCorners, allIds, board, (w, h), None, None
        )

        print(f"analyzed a total of {len(images)} frames")
        print(f"Calculated error is: {ret}")

        ### Draw the pose on one image to see if it worked ###
        frame = allImgs[random.randint(0, len(images) - 1)]
        frame_copy = frame.copy()

        ids = []
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary)

        if type(ids).__name__ == "NoneType":
            pass
        elif len(ids) > 0:
            # Draw the boxes
            aruco.drawDetectedMarkers(frame_copy, corners, ids)

            # estimate pose of board
            retval, rvec, tvec = aruco.estimatePoseBoard(
                corners, ids, board, cameraMatrix, distCoeff, None, None
            )

            # Draw the axis on the board
            if retval > 0:
                frame_copy = aruco.drawAxis(
                    frame_copy, cameraMatrix, distCoeff, rvec, tvec, self.square_length
                )

        # Display the resulting frame
        frame_copy = cv.resize(frame_copy, (960, 540))
        cv.imshow("frame", frame_copy)

        # wait for user input
        cv.waitKey(0) & 0xFF == ord("q")

        cv.destroyAllWindows()

        return cameraMatrix, distCoeff
