import numpy as np
import cv2
from cv2 import aruco
import argparse


parser = argparse.ArgumentParser(description='Compute camera calibration.')
parser.add_argument(dest='img', type=str, help='Folder with stereo***.png images.')
args = parser.parse_args()

K1 = np.matrix([ [585.83359426984816, 0., 403.74376097657625], [0.,
          585.61393084845713, 298.70552945549412], [0., 0., 1. ]])
D1 = np.matrix([ [0.020482474422127164], [0.099803061978546243],
          [-0.00096699914335497813], [0.00043855596218342471],
          [-0.25304063135333554 ]])
R1 = np.matrix([[ 0.9999761299480604, 0.0059071805397527977,
          0.0035839576128242011],[-0.0059202429749446233,
          0.99997583169112325, 0.0036451003777757205],[
          -0.003562338728612573, -0.0036662312689201971,
          0.9999869341601747 ]])
P1 = np.matrix([[ 549.11607062795667, 0., 400.21700286865234, 0.], [0.,
          549.11607062795667, 295.17863845825195, 0.], [0., 0., 1., 0. ]])

K2 = np.matrix([[ 586.56935017574006, 0., 393.31731644973434], [0.,
          586.21969422080338, 297.80922246269984], [0., 0., 1. ]])
D2 = np.matrix([ [0.023157593810997388],[0.070899386092015917],
          [-0.0027964220274352184], [-0.00011480535972265989],
          [-0.22196244351067573 ]])
R2 = np.matrix([[ 0.99995410724235456, 0.0054207040394587099,
          -0.0078993276208926006], [-0.0054495456228810914,
          0.99997854727237079, -0.0036342052147914469],[
          0.0078794582078806508, 0.0036770861773524326,
          0.99996219587322133 ]])
P2 = np.matrix([ [549.11607062795667, 0., 400.21700286865234,
          -33.464463235805965],[0., 549.11607062795667,
          295.17863845825195, 0.], [0., 0., 1., 0. ]])

img = cv2.imread(args.img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

left = img[0:img.shape[0], 0:int(img.shape[1]/2)]
leftGray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
image_left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
image_size_left = image_left.shape[::-1]

right = img[0:img.shape[0], int(img.shape[1]/2):img.shape[1]]
rightGray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
image_right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
image_size_right = image_right.shape[::-1]

leftMapX, leftMapY = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size_left, cv2.CV_32FC1)
left_rectified = cv2.remap(image_left, leftMapX, leftMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
rightMapX, rightMapY = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size_right, cv2.CV_32FC1)
right_rectified = cv2.remap(image_right, rightMapX, rightMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

displayImg = cv2.vconcat([cv2.hconcat([image_left, image_right]), cv2.hconcat([left_rectified, right_rectified])])
proportion = max(displayImg.shape) / 1000.0
displayImg = cv2.resize(displayImg, (int(displayImg.shape[1]/proportion), int(displayImg.shape[0]/proportion)))
cv2.imshow('Original and stereo rectified image', displayImg)
cv2.waitKey(0)



CHARUCOBOARD_ROWCOUNT = 6
CHARUCOBOARD_COLCOUNT = 9
CHARUCOBOARD_SQUARE_LENGTH = 0.030
CHARUCOBOARD_MARKER_SIZE = 0.022
dictionary = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
board = cv2.aruco.CharucoBoard((CHARUCOBOARD_COLCOUNT, CHARUCOBOARD_ROWCOUNT), CHARUCOBOARD_SQUARE_LENGTH, CHARUCOBOARD_MARKER_SIZE, dictionary)
board.setLegacyPattern(True)
detctorparams = cv2.aruco.DetectorParameters()
charucoparams = cv2.aruco.CharucoParameters()
charucoparams.tryRefineMarkers = True
detector = cv2.aruco.CharucoDetector(board, charucoparams, detctorparams)
corners, ids, _ = aruco.detectMarkers(
            image=leftGray,
            dictionary=dictionary)

image_size_left = None
corners_all_left = []
ids_all_left = []
image_left = []

image_size_right = None
corners_all_right = []
ids_all_right = []
image_right = []

def is_opencv47():
    (major, minor, _) = cv2.__version__.split(".")
    return int(major) >= 4 and int(minor) >= 7

img = cv2.imread(args.img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

left = img[0:img.shape[0], 0:int(img.shape[1]/2)]
leftGray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
if len(image_left) < 1:
    image_left = left.copy()

if is_opencv47():
    charuco_corners, charuco_ids, corners, ids = detector.detectBoard(leftGray)
else:
    corners, ids, _ = aruco.detectMarkers(
        image=leftGray,
        dictionary=dictionary)
    response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=leftGray,
        board=board)
if len(charuco_ids) > 1:
    corners_all_left.append(charuco_corners)
    ids_all_left.append(charuco_ids)


if is_opencv47():
        charuco_corners, charuco_ids, corners, ids = detector.detectBoard(rightGray)
else:
    corners, ids, _ = aruco.detectMarkers(
        image=rightGray,
        dictionary=dictionary)
    response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=rightGray,
        board=board)
if len(charuco_ids) > 1:
    corners_all_right.append(charuco_corners)
    ids_all_right.append(charuco_ids)

obj_all = [];
pts_all_left = [];
pts_all_right = [];
for i in range(len(ids_all_left)):
    ids_left = ids_all_left[i]
    ids_right = ids_all_right[i]
    
    obj = None;
    pts_left = [];
    pts_right = [];
    
    for j in range(len(ids_left)):
        for k in range(len(ids_right)):
            if ids_right[k] == ids_left[j]:
                if obj is None:
                    if is_opencv47():
                        obj = board.getChessboardCorners()[ids_left[j][0]]
                    else:
                        obj = board.chessboardCorners[ids_left[j][0]]
                    pts_left = corners_all_left[i][j]
                    pts_right = corners_all_right[i][k]
                    
                else:
                    if is_opencv47():
                        obj = np.vstack((obj, board.getChessboardCorners()[ids_left[j][0]]))
                    else:
                        obj = np.vstack((obj, board.chessboardCorners[ids_left[j][0]]))
                    pts_left = np.vstack((pts_left, corners_all_left[i][j]))
                    pts_right = np.vstack((pts_right, corners_all_right[i][k]))
    if len(obj) > 4:
        obj_all.append(obj)
        pts_all_left.append(pts_left)
        pts_all_right.append(pts_right)


#https://temugeb.github.io/opencv/python/2021/02/02/stereo-camera-calibration-and-triangulation.html
def DLT(P1, P2, point1, point2):
    A = [point1[1]*P1[2,:] - P1[1,:],
         P1[0,:] - point1[0]*P1[2,:],
         point2[1]*P2[2,:] - P2[1,:],
         P2[0,:] - point2[0]*P2[2,:]
        ]
    A = np.array(A).reshape((4,4))
    #print('A: ')
    #print(A)
 
    B = A.transpose() @ A
    from scipy import linalg
    U, s, Vh = linalg.svd(B, full_matrices = False)
 
    #print('Triangulated point: ')
    #print(Vh[3,0:3]/Vh[3,3])
    return Vh[3,0:3]/Vh[3,3]

font = cv2.FONT_HERSHEY_SIMPLEX
img = cv2.imread(args.img)
distsum = 0
pts_all_left = pts_all_left[0]
pts_all_right = pts_all_right[0]
for j in range(len(pts_all_left)):
    distance = DLT(P1, P2, pts_all_left[j], pts_all_right[j])[2]
    distsum += distance
    cv2.putText(img, str(distance), (int(pts_all_left[j][0]), int(pts_all_left[j][1])), font, 0.2, (0, 0, 255), 1, cv2.LINE_AA)
cv2.imshow(args.img+" avg dist="+str(distsum/len(pts_all_left)), img)
cv2.waitKey(0)