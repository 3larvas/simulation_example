import socket
import cv2
import numpy as np
import math
import time
from lidar_util import get_lidar, visualize_lidar_img, transform_lidar2cam
from cam_util import get_img

def roi(equ_frame, vertices):
    mask = np.zeros_like(equ_frame)
    cv2.fillPoly(mask, vertices, 255)
    masked = cv2.bitwise_and(equ_frame, mask)
    return masked

def nothing(x):
    pass

def initializeTrackbars(intialTracbarVals):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],50, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], 100, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], 50, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], 100, nothing)

def valTrackbars():
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")

    src = np.float32([(widthTop/100,heightTop/100), (1-(widthTop/100), heightTop/100),
                      (widthBottom/100, heightBottom/100), (1-(widthBottom/100), heightBottom/100)])
    #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    return src


def initializeTrackbars_side(intialTracbarVals):
    cv2.namedWindow("Trackbars_side")
    cv2.resizeWindow("Trackbars_side", 360, 360)
    cv2.createTrackbar("a", "Trackbars_side", intialTracbarVals[0], 100, nothing)
    cv2.createTrackbar("b", "Trackbars_side", intialTracbarVals[1], 100, nothing)
    cv2.createTrackbar("c", "Trackbars_side", intialTracbarVals[2], 100, nothing)
    cv2.createTrackbar("d", "Trackbars_side", intialTracbarVals[3], 100, nothing)
    cv2.createTrackbar("e", "Trackbars_side", intialTracbarVals[4], 100, nothing)
    cv2.createTrackbar("f", "Trackbars_side", intialTracbarVals[5], 100, nothing)
    cv2.createTrackbar("g", "Trackbars_side", intialTracbarVals[6], 100, nothing)
    cv2.createTrackbar("h", "Trackbars_side", intialTracbarVals[7], 100, nothing)

def valTrackbars_side():
    a = cv2.getTrackbarPos("a", "Trackbars_side")
    b = cv2.getTrackbarPos("b", "Trackbars_side")
    c = cv2.getTrackbarPos("c", "Trackbars_side")
    d = cv2.getTrackbarPos("d", "Trackbars_side")
    e = cv2.getTrackbarPos("e", "Trackbars_side")
    f = cv2.getTrackbarPos("f", "Trackbars_side")
    g = cv2.getTrackbarPos("g", "Trackbars_side")
    h = cv2.getTrackbarPos("h", "Trackbars_side")

    src = np.float32([(a/100,b/100), ((c/100), d/100),
                      (e/100, f/100), ((g/100), h/100)])
    #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    return src

def initializeTrackbars_side_r(intialTracbarVals):
    cv2.namedWindow("Trackbars_side_r")
    cv2.resizeWindow("Trackbars_side_r", 360, 360)
    cv2.createTrackbar("a", "Trackbars_side_r", intialTracbarVals[0], 100, nothing)
    cv2.createTrackbar("b", "Trackbars_side_r", intialTracbarVals[1], 100, nothing)
    cv2.createTrackbar("c", "Trackbars_side_r", intialTracbarVals[2], 100, nothing)
    cv2.createTrackbar("d", "Trackbars_side_r", intialTracbarVals[3], 100, nothing)
    cv2.createTrackbar("e", "Trackbars_side_r", intialTracbarVals[4], 100, nothing)
    cv2.createTrackbar("f", "Trackbars_side_r", intialTracbarVals[5], 100, nothing)
    cv2.createTrackbar("g", "Trackbars_side_r", intialTracbarVals[6], 100, nothing)
    cv2.createTrackbar("h", "Trackbars_side_r", intialTracbarVals[7], 100, nothing)

def valTrackbars_side_r():
    a = cv2.getTrackbarPos("a", "Trackbars_side_r")
    b = cv2.getTrackbarPos("b", "Trackbars_side_r")
    c = cv2.getTrackbarPos("c", "Trackbars_side_r")
    d = cv2.getTrackbarPos("d", "Trackbars_side_r")
    e = cv2.getTrackbarPos("e", "Trackbars_side_r")
    f = cv2.getTrackbarPos("f", "Trackbars_side_r")
    g = cv2.getTrackbarPos("g", "Trackbars_side_r")
    h = cv2.getTrackbarPos("h", "Trackbars_side_r")

    src = np.float32([(a/100,b/100), ((c/100), d/100),
                      (e/100, f/100), ((g/100), h/100)])
    #src = np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)])
    return src

def perspective_warp(img,
                     dst_size=(1280, 720),
                     src=np.float32([(0.43,0.65),(0.58,0.65),(0.1,1),(1,1)]),
                     dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

intialTracbarVals = [39, 36, 0, 56]   #wT,hT,wB,hB
intialTracbarVals_side = [0, 0, 100, 0, 0, 100, 100, 25]   #wT,hT,wB,hB
intialTracbarVals_side_r = [0, 0, 100, 0, 0, 25, 100, 100]   #wT,hT,wB,hB

initializeTrackbars(intialTracbarVals)
initializeTrackbars_side(intialTracbarVals_side)
initializeTrackbars_side_r(intialTracbarVals_side_r)

params_cam_1 = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0.5, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": -10,
    "ROLL": 0
}

params_cam_2 = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1231,
    "Block_SIZE": int(65000),
    "X": 0.5, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": -10,
    "ROLL": 0
}

params_cam_3 = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1230,
    "Block_SIZE": int(65000),
    "X": 0.5, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": -10,
    "ROLL": 0
}

params_visual = {
    "USE_INTENSITY_MAP": True,
    "DISTANCE_MAX": 50, # the maximum distance shown on the distance map
    "COLORMAP": cv2.COLORMAP_RAINBOW, # a type of color maps
    "DILATION_KERNAL": cv2.MORPH_ELLIPSE, # cv2.MORPH_RECT, cv2.MORPH_ELLIPSE, cv2.MORPH_CROSS
    "DILATION_SIZE": 5 # a type of color maps
}
UDP_cam_1 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_cam_1.bind((params_cam_1["localIP"], params_cam_1["localPort"]))

UDP_cam_2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_cam_2.bind((params_cam_2["localIP"], params_cam_2["localPort"]))

UDP_cam_3 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_cam_3.bind((params_cam_3["localIP"], params_cam_3["localPort"]))

def main():
    while(1):
        img_cam_1 = get_img(UDP_cam_1, params_cam_1)
        img_cam_2 = get_img(UDP_cam_2, params_cam_2)
        img_cam_3 = get_img(UDP_cam_3, params_cam_3)
        perspective_val_1 = valTrackbars();
        perspective_val_2 = valTrackbars_side()
        perspective_val_3 = valTrackbars_side_r()

        #
        frame_1 = cv2.GaussianBlur(img_cam_1, (5, 5), 0)
        height, width = frame_1.shape[:2]
        frame_1 = perspective_warp(frame_1, dst_size=(width, height), src=perspective_val_1)
        frame_1 = cv2.pyrDown(frame_1)

        #
        frame_2 = cv2.GaussianBlur(img_cam_2, (5, 5), 0)
        height, width = frame_2.shape[:2]
        frame_2 = perspective_warp(frame_2, dst_size=(width, height), src=perspective_val_2)
        # frame_2 = cv2.pyrDown(frame_2)
        matrix_2 = cv2.getRotationMatrix2D((width / 2, height / 2), 90, 1);
        frame_2 = cv2.warpAffine(frame_2, matrix_2, (width, height))

        #
        frame_3 = cv2.GaussianBlur(img_cam_3, (5, 5), 0)
        height, width = frame_3.shape[:2]
        frame_3 = perspective_warp(frame_3, dst_size=(width, height), src=perspective_val_3)
        # frame_3 = cv2.pyrDown(frame_3)
        matrix_3 = cv2.getRotationMatrix2D((width / 2, height / 2), -90, 1);
        frame_3 = cv2.warpAffine(frame_3, matrix_3, (width, height))

        cv2.imshow('Result__1', cv2.resize(frame_1, (params_cam_1["WIDTH"] * 2, params_cam_1["HEIGHT"] * 2),interpolation=cv2.INTER_LINEAR))
        # cv2.imshow('Result_1', cv2.resize(img_cam_1, (params_cam_1["WIDTH"] * 2, params_cam_1["HEIGHT"] * 2),interpolation=cv2.INTER_LINEAR))
        cv2.imshow('Result__2', cv2.resize(frame_2, (params_cam_2["HEIGHT"] * 2, params_cam_2["WIDTH"] * 2),interpolation=cv2.INTER_LINEAR))
        # cv2.imshow('Result_2_', cv2.resize(frame_2, (params_cam_2["WIDTH"] * 2, params_cam_2["HEIGHT"] * 2),interpolation=cv2.INTER_LINEAR))
        # cv2.imshow('Result_2', cv2.resize(img_cam_2, (params_cam_2["WIDTH"] * 2, params_cam_2["HEIGHT"] * 2),interpolation=cv2.INTER_LINEAR))
        cv2.imshow('Result__3', cv2.resize(frame_3, (params_cam_3["HEIGHT"] * 2, params_cam_3["WIDTH"] * 2),interpolation=cv2.INTER_LINEAR))
        # cv2.imshow('Result_3', cv2.resize(img_cam_3, (params_cam_3["WIDTH"] * 2, params_cam_3["HEIGHT"] * 2),interpolation=cv2.INTER_LINEAR))
        cv2.waitKey(1)

    UDP_cam_1.close()
    UDP_cam_2.close()
    UDP_cam_3.close()

if __name__ == '__main__':

    main()